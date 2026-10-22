//! Elevador de pallets (alimentador), Selgron "MOD ALIMENTADOR DE PALETE": two
//! clamps (garras) on a vertical carriage over a TDP keep a stack of empty pallets
//! lifted. To feed one pallet the elevator lowers the stack onto the TDP (sensor
//! inferior), opens, rises one pallet height (sensor intermediário), closes on the
//! second pallet and lifts the stack again (sensor superior): the bottom pallet
//! stays on the TDP and leaves.
//!
//! The internal logic ports the automatic steps of the elevator FB (reset vector
//! 0..43, cycle 100..111, faults 400..402).

use bevy::prelude::*;

use crate::model::{Equipment, EquipmentKind, Io, Params, SimControl};
use crate::sim::{self, Load, LoadKind, LoadState};
use crate::visuals::Palette;

/// Sensor superior above the intermediário: the lifted stack clears the pallet on the TDP.
pub const CLEARANCE: f32 = 0.20;
/// Mechanical limit switches past the sensors.
const MEC_OVERTRAVEL: f32 = 0.02;
const MOTOR_SPEED: f32 = 0.12; // m/s
const CLAMP_TIME: f32 = 1.0; // s
const SENSOR_BAND: f32 = 0.006;
/// Most pallets drawn in the stack.
pub const MAX_STACK: u32 = 20;
const TEMPO_FALHA_SOBE_DESCE: f32 = 15.0;
const TEMPO_FALHA_ABRE_FECHA: f32 = 5.0;
/// Automatic refill: time to bring a new stack once the elevator waits empty at the bottom.
const REFILL_TIME: f32 = 3.0;

/// Clamp heights above the TDP top (bottom of the clamped pallet): inferior,
/// intermediário, superior.
pub fn levels(p: &Params) -> [f32; 3] {
    let h = p.pallet_size[1];
    [0.0, h, h + CLEARANCE]
}

#[derive(Component, Debug)]
pub struct ElevatorState {
    /// Clamp height above the TDP top.
    pub z: f32,
    /// 0 = open .. 1 = closed.
    pub clamp: f32,
    /// Pallets in the stack, not yet released.
    pub count: u32,
    /// The stack stands on the TDP instead of hanging from the clamps.
    pub resting: bool,
    /// Last released pallet, held under the stack until it is lifted.
    pub released: Option<Entity>,
    /// Time waiting empty at the bottom (automatic refill).
    refill_t: f32,
}

impl ElevatorState {
    /// Full stack lifted, clamps closed at the superior position.
    pub fn new(p: &Params) -> Self {
        Self {
            z: levels(p)[2],
            clamp: 1.0,
            count: p.stack_size.min(MAX_STACK),
            resting: false,
            released: None,
            refill_t: 0.0,
        }
    }

    /// Height of the stack bottom above the TDP top.
    pub fn stack_bottom(&self) -> f32 {
        if self.resting { 0.0 } else { self.z }
    }
}

/// Port of the elevator FB (automatic mode).
#[derive(Component, Debug, Default)]
pub struct ElevatorLogic {
    pub passo: u32,
    pub fim_do_ciclo: bool,
    pub luz_sem_pallet: bool,
    pub falha_sensor_central: bool,
    pub falha_abre_fecha: bool,
    pub falha_sobe_desce: bool,
    motor_sobe: bool,
    motor_desce: bool,
    garra_abrir: bool,
    garra_fechar: bool,
    timer_garra: f32,
    timer_motor: f32,
    /// CmdReiniciar from the property panel.
    pub reiniciar: bool,
    last_passo: u32,
}

/// Clamp bar of one side (`-1` / `+1` along local Z).
#[derive(Component)]
pub struct ElevatorClamp(pub f32);

/// Pallet `i` of the stack (from the bottom).
#[derive(Component)]
pub struct ElevatorPallet(pub u32);

/// Free pallet whose center is under the elevator (photo sensor of the TDP).
fn pallet_under<'a>(
    eq: &Equipment,
    tf: &Transform,
    mut loads: impl Iterator<Item = (Entity, &'a Load, &'a Transform)>,
) -> Option<Entity> {
    let inverse = tf.compute_affine().inverse();
    loads
        .find(|(_, l, ltf)| {
            let local = inverse.transform_point3(ltf.translation);
            l.kind == LoadKind::Pallet
                && l.state == LoadState::Free
                && local.x.abs() < l.size.x / 2.0
                && local.z.abs() < l.size.z / 2.0
                && (local.y - l.size.y / 2.0 - eq.params.height).abs() < 0.08
        })
        .map(|(e, ..)| e)
}

/// Mechanics: carriage motor, clamps, picking up and releasing the stack.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn elevator_mechanics(
    time: Res<Time>,
    control: Res<SimControl>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    palette: Res<Palette>,
    mut elevators: Query<(&Equipment, &Transform, &mut Io, &mut ElevatorState), Without<Load>>,
    mut loads: Query<(Entity, &mut Load, &Transform), Without<Equipment>>,
) {
    let dt = if control.running { time.delta_secs() * control.time_scale } else { 0.0 };
    for (eq, tf, mut io, mut state) in &mut elevators {
        if eq.kind != EquipmentKind::PalletElevator {
            continue;
        }
        let p = &eq.params;
        let h = p.pallet_size[1];
        let [inf, int, sup] = levels(p);
        let under = pallet_under(eq, tf, loads.iter());

        // Carriage: stops on the sensors it crosses (one frame), so the logic sees them.
        let up = io.get_bool("MotorSobe") && !io.get_bool("MotorDesce");
        let down = io.get_bool("MotorDesce") && !io.get_bool("MotorSobe");
        let holding = !state.resting && state.count > 0;
        // A lifted stack cannot go through a pallet already on the TDP.
        let floor = if holding && under.is_some() { int } else { inf };
        let old = state.z;
        if up {
            state.z = (state.z + MOTOR_SPEED * dt).min(sup + MEC_OVERTRAVEL);
        } else if down {
            state.z = (state.z - MOTOR_SPEED * dt).max(floor);
        }
        for level in [inf, int, sup] {
            let crossed = (old < level - 1e-4 && state.z > level) || (old > level + 1e-4 && state.z < level);
            if crossed {
                state.z = level;
            }
        }

        // Clamps (bistable valve: keeps its position with both outputs off).
        let was_closed = state.clamp >= 0.98;
        match (io.get_bool("Garra_Abrir"), io.get_bool("Garra_Fechar")) {
            (true, false) => state.clamp = (state.clamp - dt / CLAMP_TIME).max(0.0),
            (false, true) => state.clamp = (state.clamp + dt / CLAMP_TIME).min(1.0),
            _ => {}
        }
        let closed = state.clamp >= 0.98;

        // Opening: the stack drops onto the TDP (absorbing a pallet already there).
        if holding && state.clamp < 0.9 {
            if let Some(pallet) = under {
                commands.entity(pallet).despawn();
                state.count += 1;
            }
            state.resting = true;
        }
        // Closing on the stack standing on the TDP: the clamps take the pallet at
        // their height and everything above; the pallets below stay on the TDP.
        if closed && !was_closed && state.resting && state.count > 0 {
            let k = (state.z / h).round();
            if (state.z - k * h).abs() < 0.03 {
                // Above the last pallet the clamps close on nothing and it stays too.
                state.resting = false;
                if k >= 1.0 {
                    state.count -= 1;
                    let size = p.pallet_vec();
                    let at = tf.transform_point(Vec3::new(0.0, p.height + size.y / 2.0 + 0.002, 0.0));
                    let pallet = sim::spawn_load(
                        &mut commands,
                        &mut meshes,
                        &palette,
                        LoadKind::Pallet,
                        size,
                        Transform::from_translation(at).with_rotation(tf.rotation),
                    );
                    state.released = Some(pallet);
                }
            }
        }
        // The released pallet carries the stack until it is lifted clear (it is
        // spawned at the end of the frame, so it may not be queryable yet).
        if let Some(pallet) = state.released {
            let lifted = !state.resting && state.z >= int + 0.02;
            match loads.get_mut(pallet) {
                Ok((_, mut load, _)) if !lifted => load.hold = true,
                Ok(_) => state.released = None,
                Err(_) if lifted => state.released = None,
                Err(_) => {}
            }
        }

        // Automatic refill: a new stack arrives while the empty elevator waits at the
        // bottom with no pallet on the TDP (taken by the clamps if they are closed).
        let waiting_empty = p.auto_refill && state.count == 0 && state.z <= inf + SENSOR_BAND && under.is_none();
        state.refill_t = if waiting_empty { state.refill_t + dt } else { 0.0 };
        if state.refill_t >= REFILL_TIME {
            state.refill_t = 0.0;
            state.count = p.stack_size.min(MAX_STACK);
            state.resting = !closed;
        }

        let near = |level: f32| (state.z - level).abs() <= SENSOR_BAND;
        io.set_bool("SensorPosInferior", near(inf));
        io.set_bool("SensorPosInferiorMec", state.z <= inf + 1e-4);
        io.set_bool("SensorPosIntermediario", near(int));
        io.set_bool("SensorPosSuperior", near(sup));
        io.set_bool("SensorPosSuperiorMec", state.z >= sup + MEC_OVERTRAVEL - 1e-4);
        io.set_bool("SensorGarraAberta", state.clamp <= 0.02);
        io.set_bool("SensorGarraFechada", closed);
        io.set_bool("SensorPilhaPallet", state.count > 0);
        io.set_bool("SensorTDP", under.is_some() || (state.resting && state.count > 0));
    }
}

fn drive(io: &mut Io, key: &'static str, value: bool) {
    if !io.from_plc.contains(key) {
        io.set_bool(key, value);
    }
}

/// Internal logic: the elevator FB in automatic. TDPProntoReceber is derived from
/// the TDP under the elevator (no pallet on it) and TDPosteriorProntoReceber is
/// always on, unless the PLC drives them.
#[allow(clippy::type_complexity)]
pub fn elevator_logic(
    time: Res<Time>,
    control: Res<SimControl>,
    mut elevators: Query<(&Equipment, &Transform, &mut Io, &mut ElevatorLogic)>,
    conveyors: Query<(&Equipment, &Transform), Without<ElevatorLogic>>,
    loads: Query<(&Load, &Transform), Without<Equipment>>,
) {
    if !control.running {
        return;
    }
    let dt = time.delta_secs() * control.time_scale;
    for (eq, tf, mut io, mut l) in &mut elevators {
        if eq.kind != EquipmentKind::PalletElevator || !eq.params.auto_logic {
            continue;
        }
        // TDP under the elevator: ready to receive while it carries no pallet.
        let center = tf.translation;
        let tdp_empty = conveyors
            .iter()
            .filter(|(c, _)| c.kind.is_conveyor() && c.kind != EquipmentKind::ShuttleCar)
            .find(|(c, ctf)| {
                let local = ctf.compute_affine().inverse().transform_point3(center);
                local.x.abs() <= c.params.length / 2.0 && local.z.abs() <= c.params.width / 2.0
            })
            .is_none_or(|(c, ctf)| {
                let inverse = ctf.compute_affine().inverse();
                !loads.iter().any(|(load, ltf)| {
                    let local = inverse.transform_point3(ltf.translation);
                    load.kind == LoadKind::Pallet
                        && load.state == LoadState::Free
                        && local.x.abs() <= c.params.length / 2.0
                        && local.z.abs() <= c.params.width / 2.0
                        && (local.y - load.size.y / 2.0 - c.params.height).abs() < 0.08
                })
            });
        drive(&mut io, "TDPProntoReceber", tdp_empty);
        drive(&mut io, "TDPosteriorProntoReceber", true);

        let s = |k: &str| io.get_bool(k);
        let inferior = s("SensorPosInferior") || s("SensorPosInferiorMec");
        let intermediario = s("SensorPosIntermediario");
        let superior = s("SensorPosSuperior") || s("SensorPosSuperiorMec");
        let (aberta, fechada) = (s("SensorGarraAberta"), s("SensorGarraFechada"));
        let (sensor_tdp, pilha) = (s("SensorTDP"), s("SensorPilhaPallet"));
        let (tdp_pronto, posterior_pronto) = (s("TDPProntoReceber"), s("TDPosteriorProntoReceber"));

        // Timers de processo.
        let trig_garra = (!aberta && l.garra_abrir) || (!fechada && l.garra_fechar);
        l.timer_garra = if trig_garra { l.timer_garra + dt } else { 0.0 };
        let trig_motor = l.motor_desce || l.motor_sobe;
        l.timer_motor = if trig_motor { l.timer_motor + dt } else { 0.0 };

        if std::mem::take(&mut l.reiniciar) {
            // CmdReiniciar
            *l = ElevatorLogic { last_passo: l.last_passo, ..default() };
        }
        if l.timer_garra >= TEMPO_FALHA_ABRE_FECHA {
            l.passo = 401;
        }
        if l.timer_motor >= TEMPO_FALHA_SOBE_DESCE {
            l.passo = 402;
        }
        let open = |l: &mut ElevatorLogic| {
            l.garra_abrir = true;
            l.garra_fechar = false;
        };
        let close = |l: &mut ElevatorLogic| {
            l.garra_abrir = false;
            l.garra_fechar = true;
        };
        // Ciclo pronto no sensor superior: termina o ciclo ou aguarda o TDP.
        let at_top = |l: &mut ElevatorLogic| l.passo = if tdp_pronto && sensor_tdp { 110 } else { 100 };

        match l.passo {
            0 => {
                // Vetor de reset
                if fechada {
                    close(&mut l);
                    l.passo = 1;
                } else {
                    open(&mut l);
                    if aberta {
                        l.passo = 10;
                    }
                }
            }
            1 => {
                if superior {
                    at_top(&mut l);
                } else {
                    l.motor_sobe = true;
                    l.passo = 2;
                }
            }
            2 => {
                if superior {
                    l.motor_sobe = false;
                    at_top(&mut l);
                }
            }
            10 => {
                open(&mut l);
                l.passo = if aberta { 12 } else { 11 };
            }
            11 => {
                if aberta {
                    l.passo = 12;
                }
            }
            12 => {
                // Verifica em que posição está.
                if inferior {
                    l.motor_sobe = true;
                    l.passo = 13;
                } else if intermediario {
                    l.passo = 40;
                } else if superior {
                    l.motor_desce = true;
                    l.passo = 20;
                } else {
                    l.motor_sobe = true;
                    l.passo = 30;
                }
            }
            13 => {
                if intermediario {
                    l.motor_sobe = false;
                    l.passo = 12;
                }
                if s("SensorPosSuperior") && s("SensorPosSuperiorMec") {
                    l.motor_sobe = false;
                    l.passo = 400;
                }
            }
            20 => {
                if inferior || intermediario {
                    l.motor_desce = false;
                    l.passo = 12;
                }
            }
            30 => {
                if intermediario || superior {
                    l.motor_sobe = false;
                    l.passo = 12;
                }
            }
            40 => {
                close(&mut l);
                l.passo = 41;
            }
            41 => {
                if fechada {
                    l.garra_abrir = false;
                    l.passo = 42;
                }
            }
            42 => {
                l.motor_sobe = true;
                l.passo = 43;
            }
            43 => {
                if superior {
                    l.motor_sobe = false;
                    at_top(&mut l);
                }
            }
            100 => {
                // Aguarda TDP pronto para receber e sem pallet.
                if tdp_pronto && !sensor_tdp {
                    l.motor_desce = true;
                    l.passo = 101;
                    l.luz_sem_pallet = false;
                }
            }
            101 => {
                if inferior {
                    l.motor_desce = false;
                    l.passo = 102;
                }
            }
            102 => {
                // Abre a garra se tiver pallet na pilha.
                l.luz_sem_pallet = !pilha;
                if pilha {
                    open(&mut l);
                    l.passo = 103;
                }
            }
            103 => {
                if aberta {
                    l.garra_abrir = false;
                    l.garra_fechar = false;
                    l.passo = 104;
                }
            }
            104 => {
                l.motor_sobe = true;
                l.passo = 105;
            }
            105 => {
                if intermediario {
                    l.motor_sobe = false;
                    l.passo = 106;
                }
                if superior {
                    l.motor_sobe = false;
                    l.passo = 400;
                }
            }
            106 => {
                close(&mut l);
                l.passo = 107;
            }
            107 => {
                if fechada {
                    l.garra_abrir = false;
                    l.passo = 108;
                }
            }
            108 => {
                // Sobe a pilha quando o TDP posterior estiver pronto.
                if posterior_pronto {
                    l.motor_sobe = true;
                    l.passo = 109;
                }
            }
            109 => {
                if superior {
                    l.motor_sobe = false;
                    l.passo = 110;
                }
            }
            110 => {
                l.fim_do_ciclo = true;
                l.passo = 111;
            }
            111 => {
                // Aguarda o pallet sair do sensor e o TDP baixar o pronto para receber.
                if !sensor_tdp && !tdp_pronto {
                    l.fim_do_ciclo = false;
                    l.passo = 100;
                }
            }
            400 => {
                l.motor_desce = false;
                l.motor_sobe = false;
                l.falha_sensor_central = true;
            }
            401 => {
                l.garra_abrir = false;
                l.garra_fechar = false;
                l.falha_abre_fecha = true;
            }
            402 => {
                l.motor_desce = false;
                l.motor_sobe = false;
                l.falha_sobe_desce = true;
            }
            _ => l.passo = 0,
        }
        if std::env::var("PDT_TRACE").is_ok() && l.passo != l.last_passo {
            eprintln!(
                "[{:.2}] {}: passo {} -> {} (inf {} int {} sup {} aberta {} fechada {} tdp {} pronto {})",
                time.elapsed_secs(),
                eq.name,
                l.last_passo,
                l.passo,
                u8::from(inferior),
                u8::from(intermediario),
                u8::from(superior),
                u8::from(aberta),
                u8::from(fechada),
                u8::from(sensor_tdp),
                u8::from(tdp_pronto)
            );
            l.last_passo = l.passo;
        }
        drive(&mut io, "MotorSobe", l.motor_sobe);
        drive(&mut io, "MotorDesce", l.motor_desce);
        drive(&mut io, "Garra_Abrir", l.garra_abrir);
        drive(&mut io, "Garra_Fechar", l.garra_fechar);
    }
}

#[allow(clippy::type_complexity)]
pub fn animate_elevators(
    elevators: Query<(&Equipment, &ElevatorState, &Children)>,
    mut clamps: Query<(&ElevatorClamp, &mut Transform), Without<ElevatorPallet>>,
    mut pallets: Query<(&ElevatorPallet, &mut Transform, &mut Visibility), Without<ElevatorClamp>>,
) {
    for (eq, state, children) in &elevators {
        let p = &eq.params;
        let h = p.pallet_size[1];
        let half_z = p.pallet_size[2] / 2.0;
        for child in children.iter() {
            if let Ok((clamp, mut tf)) = clamps.get_mut(child) {
                tf.translation.y = p.height + state.z + h / 2.0;
                tf.translation.z = clamp.0 * (half_z + 0.05 + 0.08 * (1.0 - state.clamp));
            } else if let Ok((pallet, mut tf, mut vis)) = pallets.get_mut(child) {
                tf.translation.y = p.height + state.stack_bottom() + pallet.0 as f32 * h;
                *vis = if pallet.0 < state.count { Visibility::Inherited } else { Visibility::Hidden };
            }
        }
    }
}
