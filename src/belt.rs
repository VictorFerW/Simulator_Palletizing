//! Esteira de produtos: port of the conveyor FB (automatic mode). The motor turns on
//! when a product reaches the entry or exit sensor (each one is optional) and, with
//! accumulation on, stops with a product at the exit while the next conveyor cannot
//! take it, or after `TempoVaiPara` without products at the entry. `TempoOciosidade`
//! with both sensors free stops it. `EsteiraCheia` (accumulating, product at the exit,
//! motor stopped) is published; `EsteiraPosteriorOK` comes from the PLC when bound,
//! otherwise from the conveyor at the end: another esteira that is not full, a mesa
//! with the entry stop down, a conveyor with the motor on, or nothing there.

use bevy::prelude::*;

use crate::model::{Equipment, EquipmentKind, Io, SimControl};

/// On-delay timer (TON).
#[derive(Debug, Default, Clone, Copy)]
struct Ton {
    elapsed: f32,
}

impl Ton {
    fn run(&mut self, input: bool, preset: f32, dt: f32) -> bool {
        if input {
            self.elapsed += dt;
        } else {
            self.elapsed = 0.0;
        }
        input && self.elapsed >= preset
    }
}

#[derive(Component, Debug)]
pub struct BeltLogic {
    pub motor: bool,
    first_stop: bool,
    flag_ativar_acumulo: bool,
    trigger_vai_para: bool,
    ociosidade: Ton,
    vai_para: Ton,
    /// Downstream ready (EsteiraPosteriorOK) of the last scan, for the panel.
    pub posterior_ok: bool,
    /// EsteiraCheia of the last scan.
    pub cheia: bool,
}

impl Default for BeltLogic {
    fn default() -> Self {
        // FirstStop: the belt starts once when the automatic mode begins.
        Self {
            motor: false,
            first_stop: true,
            flag_ativar_acumulo: false,
            trigger_vai_para: false,
            ociosidade: Ton::default(),
            vai_para: Ton::default(),
            posterior_ok: true,
            cheia: false,
        }
    }
}

struct Inputs {
    sensor_entrada: bool,
    sensor_saida: bool,
    ativa_acumulo: bool,
    posterior_ok: bool,
    tempo_ociosidade: f32,
    tempo_vai_para: f32,
}

impl BeltLogic {
    /// One scan in automatic mode (ManAuto = 1, Emergencia ok, HabilitaEsteira = 1).
    fn scan(&mut self, i: &Inputs, dt: f32) {
        self.cheia = i.ativa_acumulo && i.sensor_saida && !i.posterior_ok && !self.motor;
        let ociosidade = self.ociosidade.run(self.motor && !i.sensor_entrada && !i.sensor_saida, i.tempo_ociosidade, dt);
        let vai_para = self.vai_para.run(self.trigger_vai_para, i.tempo_vai_para, dt);
        if !i.ativa_acumulo {
            if self.first_stop || i.sensor_entrada || i.sensor_saida || (self.flag_ativar_acumulo && !i.ativa_acumulo) {
                self.motor = true;
                self.first_stop = false;
            }
        } else {
            // Está acumulando
            if self.first_stop
                || i.sensor_entrada
                || (i.sensor_saida && i.posterior_ok)
                || (self.flag_ativar_acumulo && !i.ativa_acumulo)
            {
                self.motor = true;
                self.first_stop = false;
            }
            self.trigger_vai_para = self.motor && !i.sensor_entrada;
            if (vai_para && !i.posterior_ok) || (i.sensor_saida && !i.posterior_ok) {
                self.motor = false;
            }
        }
        self.flag_ativar_acumulo = i.ativa_acumulo;
        // Condição para desligar
        if ociosidade {
            self.motor = false;
        }
    }
}

/// Conveyor seen from the one feeding it: kind, frame, and whether it can take a
/// product now.
struct Next {
    entity: Entity,
    eq: Equipment,
    tf: Transform,
    ready: bool,
}

/// Ready to receive: what the conveyor at `point` (world) does with a product.
fn downstream_ok(point: Vec3, conveyors: &[Next], me: Entity) -> bool {
    conveyors
        .iter()
        .filter(|n| n.entity != me)
        .find(|n| {
            let local = n.tf.compute_affine().inverse().transform_point3(point);
            local.x.abs() <= n.eq.params.length / 2.0 && local.z.abs() <= n.eq.params.width / 2.0 + 0.05
        })
        .is_none_or(|n| n.ready)
}

#[allow(clippy::type_complexity)]
pub fn run_belts(
    time: Res<Time>,
    control: Res<SimControl>,
    mut set: ParamSet<(Query<(Entity, &Equipment, &Transform, &Io)>, Query<(Entity, &Equipment, &Transform, &mut Io, &mut BeltLogic)>)>,
) {
    let dt = if control.running { time.delta_secs() * control.time_scale } else { 0.0 };
    let conveyors: Vec<Next> = set
        .p0()
        .iter()
        .filter(|(_, eq, ..)| eq.kind.is_conveyor())
        .map(|(entity, eq, tf, io)| {
            let ready = if eq.kind.is_mesa() {
                !io.get_bool("Bloqueio_Entrada")
            } else if eq.kind == EquipmentKind::BeltConveyor && eq.params.belt_logic {
                !io.get_bool("EsteiraCheia")
            } else {
                io.get_bool("motor")
            };
            Next { entity, eq: eq.clone(), tf: *tf, ready }
        })
        .collect();
    for (entity, eq, tf, mut io, mut logic) in &mut set.p1() {
        if eq.kind != EquipmentKind::BeltConveyor || !eq.params.belt_logic {
            continue;
        }
        let p = &eq.params;
        let dir = if io.get_bool("reverse") { -1.0 } else { 1.0 };
        let end = tf.transform_point(Vec3::new(dir * (p.length / 2.0 + 0.1), p.height, 0.0));
        let posterior_ok = if io.from_plc.contains("EsteiraPosteriorOK") {
            io.get_bool("EsteiraPosteriorOK")
        } else {
            downstream_ok(end, &conveyors, entity)
        };
        let inputs = Inputs {
            sensor_entrada: p.belt_entry_sensor && io.get_bool("SensorEntrada"),
            sensor_saida: p.belt_exit_sensor && io.get_bool("SensorSaida"),
            ativa_acumulo: p.belt_accumulate,
            posterior_ok,
            tempo_ociosidade: p.belt_idle_time,
            tempo_vai_para: p.belt_run_on,
        };
        if dt > 0.0 {
            let before = (logic.motor, logic.cheia);
            logic.scan(&inputs, dt);
            if std::env::var("PDT_TRACE").is_ok() && before != (logic.motor, logic.cheia) {
                eprintln!(
                    "[{:.2}] {}: motor {} cheia {} (entrada {} saída {} posterior_ok {})",
                    time.elapsed_secs(),
                    eq.name,
                    u8::from(logic.motor),
                    u8::from(logic.cheia),
                    u8::from(inputs.sensor_entrada),
                    u8::from(inputs.sensor_saida),
                    u8::from(posterior_ok)
                );
            }
        }
        logic.posterior_ok = posterior_ok;
        io.set_bool("EsteiraCheia", logic.cheia);
        if !io.from_plc.contains("motor") {
            io.set_bool("motor", logic.motor);
        }
        if !io.from_plc.contains("EsteiraPosteriorOK") {
            io.set_bool("EsteiraPosteriorOK", posterior_ok);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inputs(entrada: bool, saida: bool, ok: bool) -> Inputs {
        Inputs {
            sensor_entrada: entrada,
            sensor_saida: saida,
            ativa_acumulo: true,
            posterior_ok: ok,
            tempo_ociosidade: 10.0,
            tempo_vai_para: 3.0,
        }
    }

    #[test]
    fn accumulates_at_the_exit() {
        let mut b = BeltLogic::default();
        b.scan(&inputs(true, false, false), 0.1);
        assert!(b.motor, "first stop and product at the entry start the belt");
        // Product at the exit and the next conveyor busy: stops.
        b.scan(&inputs(false, true, false), 0.1);
        assert!(!b.motor);
        // Next conveyor ready: runs again.
        b.scan(&inputs(false, true, true), 0.1);
        assert!(b.motor);
    }

    #[test]
    fn vai_para_and_idle_stop() {
        let mut b = BeltLogic::default();
        b.scan(&inputs(true, false, false), 0.1);
        // Entry free, next busy: runs TempoVaiPara then stops.
        for _ in 0..25 {
            b.scan(&inputs(false, false, false), 0.1);
        }
        assert!(b.motor);
        for _ in 0..10 {
            b.scan(&inputs(false, false, false), 0.1);
        }
        assert!(!b.motor);
        // Without accumulation: idle time with both sensors free stops it.
        let mut b = BeltLogic::default();
        let mut i = inputs(true, false, true);
        i.ativa_acumulo = false;
        b.scan(&i, 0.1);
        i.sensor_entrada = false;
        for _ in 0..99 {
            b.scan(&i, 0.1);
        }
        assert!(b.motor);
        for _ in 0..3 {
            b.scan(&i, 0.1);
        }
        assert!(!b.motor);
    }
}
