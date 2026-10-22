//! Carro cheio giratório (CCH), Selgron "MOD CARRO ALIMENT ROTATIVO": a car on a
//! rail (inverter + brake, distance sensor) carrying a TDP on a slewing ring. It
//! receives full pallets from the palletizing lines with the TDP across the rail,
//! turns the TDP (giro) and delivers them at the delivery position.
//!
//! Stops (`params.stops`): 0 = espera (ID 10), 1 = entrega (ID 20), 2.. = linhas
//! A, B, ... (IDs 100, 200, ...). The internal logic ports the two FBs:
//! - "Carro Cheio Movimento": go to the line whose ID is requested (`ID_Busca`),
//!   to the delivery with a pallet on the centre, else to the waiting position;
//!   positioning in steps 200..203 (fast/proportional/slow speed, tolerance,
//!   brake timers);
//! - "Carro Cheio TDP": turn to receive, receive until the pallet is centred
//!   (Sensor1..4), turn to deliver, transfer, turn back.
//!
//! `ID_Busca` comes from the pallet tracking of the PLC; internally it is the line
//! that has waited longest with a full pallet on the TDP discharging into the car.

use bevy::prelude::*;

use crate::conveyor;
use crate::model::{Equipment, EquipmentKind, Io, SimControl};
use crate::shuttle::{ShuttleState, travel_limits};
use crate::sim::{self, ConveyorState, Load, LoadKind, LoadState, PalletLoad, PalletizerState};
use crate::visuals::Roller;

pub const ID_ESPERA: i32 = 10;
pub const ID_ENTREGA: i32 = 20;

/// ID of line `k` (0 = A): 100, 200, ...
pub fn line_id(k: usize) -> i32 {
    100 * (k as i32 + 1)
}

/// Stop index of an ID.
fn stop_of(id: i32) -> Option<usize> {
    match id {
        ID_ESPERA => Some(0),
        ID_ENTREGA => Some(1),
        id if id >= 100 && id % 100 == 0 => Some(1 + id as usize / 100),
        _ => None,
    }
}

// Positioning parameters of the FB (mm, mm/s, s).
const TOLERANCIA_POSICAO: f32 = 5.0;
const DISTANCIA_REDUCAO_VELOCIDADE: f32 = 50.0;
const DISTANCIA_REDUCAO_METADE: f32 = 800.0;
const DISTANCIA_REDUCAO_METADE_VAZIO: f32 = 1200.0;
const VELOCIDADE_LENTA: f32 = 60.0;
const TEMPO_FREIO_ON: f32 = 0.3;
const TEMPO_MOTOR_OFF: f32 = 0.3;
const TEMPO_POSICAO_OK: f32 = 0.05;
const TEMPO_FREIO_TDP: f32 = 0.25;
const TEMPO_PRONTO_TRANSFERIR: f32 = 0.05;
const ACCELERATION: f32 = 0.5; // m/s², heavier than the CVZ
const BRAKE_DECELERATION: f32 = 2.0;
/// Giro position sensors (rad).
const GIRO_SENSOR: f32 = 0.01;

/// Radius of the round TDP (Ø2054 mm for the 1829 mm TDP): the rollers end inside
/// it, so the turning TDP does not hit the TDPs next to the rail.
pub fn turn_radius(p: &crate::model::Params) -> f32 {
    (p.car_length / 2.0 + 0.1125).max(p.car_width / 2.0)
}

/// Rotating TDP of the car (child of the car body).
#[derive(Component)]
pub struct FullCarTop;

/// Port of the "Carro Cheio Movimento" and "Carro Cheio TDP" FBs.
#[derive(Component, Debug, Default)]
pub struct FullCarLogic {
    pub passo: u32,
    pub tdp_passo: u32,
    pub id_busca: i32,
    pub id_destino: i32,
    posicao_destino: f32,
    pub inp: bool,
    motor: bool,
    liberar_freio: bool,
    velocidade: f32,
    motor_tdp: bool,
    liberar_freio_tdp: bool,
    velocidade_tdp: f32,
    motor_giro: bool,
    liberar_freio_giro: bool,
    velocidade_giro: f32,
    t_freio_on: f32,
    t_motor_off: f32,
    t_posicao_ok: f32,
    iniciar_posicao_ok: bool,
    t_freio_tdp: f32,
    t_transferir: f32,
    iniciar_transferir: bool,
    pub pronto_receber: bool,
    pub pronto_transferir: bool,
    /// Since when each line asks for the car (served first come, first served).
    requested_since: Vec<Option<f32>>,
    last: (u32, u32),
}

fn drive(io: &mut Io, key: &'static str, value: f64) {
    if !io.from_plc.contains(key) {
        io.set(key, value);
    }
}

/// Point of the car TDP where pallets enter (TDP across the rail) at rail position `x`.
fn receive_edge(eq: &Equipment, tf: &Transform, x: f32) -> Vec3 {
    let p = &eq.params;
    tf.transform_point(Vec3::new(x - p.length / 2.0, p.height, -p.car_length / 2.0))
}

/// Lines (index k) with a full pallet waiting at a TDP that discharges into the car at their stop.
#[allow(clippy::type_complexity)]
fn requesting_lines(
    eq: &Equipment,
    tf: &Transform,
    conveyors: &Query<(&Equipment, &Transform, Option<&PalletizerState>), Without<FullCarLogic>>,
    loads: &Query<(&Load, &Transform, Option<&PalletLoad>), Without<Equipment>>,
) -> Vec<usize> {
    let stops = &eq.params.stops;
    (2..stops.len())
        .filter(|&i| {
            let edge = receive_edge(eq, tf, stops[i]);
            conveyors.iter().any(|(c, ctf, palletizer)| {
                if !matches!(c.kind, EquipmentKind::PalletConveyor | EquipmentKind::PalletizingConveyor) {
                    return false;
                }
                let end = ctf.transform_point(Vec3::new(c.params.length / 2.0, c.params.height, 0.0));
                if Vec2::new(end.x - edge.x, end.z - edge.z).length() > 0.5 {
                    return false;
                }
                match palletizer {
                    Some(state) => state.full && state.pallet.is_some(),
                    None => loads.iter().any(|(l, ltf, content)| {
                        l.kind == LoadKind::Pallet
                            && l.state == LoadState::Free
                            && content.is_some_and(|p| p.placed > 0)
                            && sim::at_conveyor_end(c, ctf, l.size, ltf)
                    }),
                }
            })
        })
        .map(|i| i - 2)
        .collect()
}

#[allow(clippy::type_complexity)]
pub fn fullcar_logic(
    time: Res<Time>,
    control: Res<SimControl>,
    mut cars: Query<(&Equipment, &Transform, &mut Io, &mut FullCarLogic)>,
    conveyors: Query<(&Equipment, &Transform, Option<&PalletizerState>), Without<FullCarLogic>>,
    loads: Query<(&Load, &Transform, Option<&PalletLoad>), Without<Equipment>>,
) {
    if !control.running {
        return;
    }
    let dt = time.delta_secs() * control.time_scale;
    for (eq, tf, mut io, mut logic) in &mut cars {
        let l = &mut *logic;
        let p = &eq.params;
        if eq.kind != EquipmentKind::FullCar || !p.auto_logic || p.stops.len() < 3 {
            continue;
        }
        let mm = |stop: usize| p.stops[stop.min(p.stops.len() - 1)] * 1000.0;
        let (espera, entrega) = (mm(0), mm(1));
        let requesting = requesting_lines(eq, tf, &conveyors, &loads);
        let now = time.elapsed_secs();
        l.requested_since.resize(p.stops.len().saturating_sub(2), None);
        for (k, since) in l.requested_since.iter_mut().enumerate() {
            *since = if requesting.contains(&k) { Some(since.unwrap_or(now)) } else { None };
        }
        l.id_busca = l
            .requested_since
            .iter()
            .enumerate()
            .filter_map(|(k, since)| since.map(|t| (k, t)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map_or(0, |(k, _)| line_id(k));

        let s = |k: &str| io.get_bool(k);
        let (s1, s2, s3, s4) = (s("Sensor1"), s("Sensor2"), s("Sensor3"), s("Sensor4"));
        let sem_sensor = !s1 && !s2 && !s3 && !s4;
        let no_centro = (s2 || s3) && !s1;
        let posicao = io.get("PosicaoAtual") as f32;
        let driver_v = io.get("DriverVelocidade") as f32;
        let fim_de_curso = s("FimDeCurso");
        let (giro_receber, giro_entregar) = (s("Giro_Sensor_PosReceber"), s("Giro_Sensor_PosEntregar"));
        let tdp_v = io.get("VelocidadeAtualTDP") as f32;
        let giro_v = io.get("VelocidadeAtualGiro") as f32;

        // Timers.
        l.t_freio_on = if l.liberar_freio { l.t_freio_on + dt } else { 0.0 };
        l.t_motor_off = if !l.motor { l.t_motor_off + dt } else { 0.0 };
        l.t_posicao_ok = if l.iniciar_posicao_ok { l.t_posicao_ok + dt } else { 0.0 };
        l.t_freio_tdp = if l.liberar_freio_tdp { l.t_freio_tdp + dt } else { 0.0 };
        l.t_transferir = if l.iniciar_transferir { l.t_transferir + dt } else { 0.0 };
        let freio_on = l.t_freio_on >= TEMPO_FREIO_ON;
        let motor_off = l.t_motor_off >= TEMPO_MOTOR_OFF;
        let freio_tdp = l.t_freio_tdp >= TEMPO_FREIO_TDP;

        // CondicaoMovimentacao: the TDP is not receiving or transferring.
        let condicao_movimentacao = !matches!(l.tdp_passo, 2 | 3 | 100..=103);
        let err = l.posicao_destino - posicao;
        let cheio = no_centro;
        let scale = |metade: f32, rapida: f32| {
            let k = ((err.abs() - DISTANCIA_REDUCAO_VELOCIDADE) / (metade - DISTANCIA_REDUCAO_VELOCIDADE)).clamp(0.0, 1.0);
            VELOCIDADE_LENTA + (rapida - VELOCIDADE_LENTA) * k
        };
        let (rapida_cheio, rapida_vazio) = (p.car_speed * 1000.0, p.car_speed_empty * 1000.0);
        let go = |l: &mut FullCarLogic, destino: f32, id: i32| {
            l.posicao_destino = destino;
            l.id_destino = id;
            l.inp = false;
            l.passo = 200;
        };

        // FB "Carro Cheio Movimento".
        match l.passo {
            0 => {
                if sem_sensor {
                    l.passo = if l.id_busca == 0 { 100 } else { 1 };
                }
                if no_centro {
                    go(l, entrega, ID_ENTREGA);
                }
            }
            1 => {
                if driver_v == 0.0 && motor_off {
                    l.liberar_freio = false;
                }
                l.motor = false;
                if no_centro && (!l.inp || l.id_destino != ID_ENTREGA) {
                    go(l, entrega, ID_ENTREGA);
                } else if l.id_busca != 0 && sem_sensor && (l.id_destino != l.id_busca || !l.inp) {
                    let id = l.id_busca;
                    go(l, mm(stop_of(id).unwrap_or(0)), id);
                } else if l.id_busca == 0 && sem_sensor && (l.id_destino != ID_ESPERA || !l.inp) {
                    go(l, espera, ID_ESPERA);
                }
            }
            100 => go(l, espera, ID_ESPERA),
            200 => {
                l.velocidade = (if cheio { rapida_cheio } else { rapida_vazio }) * err.signum();
                if driver_v == 0.0 || motor_off {
                    l.liberar_freio = false;
                }
                if (!no_centro && sem_sensor) || (l.posicao_destino == entrega && no_centro) {
                    if condicao_movimentacao {
                        l.liberar_freio = true;
                        if err.abs() <= TOLERANCIA_POSICAO {
                            l.inp = true;
                            l.motor = false;
                            l.liberar_freio = false;
                            l.passo = 203;
                        } else {
                            l.passo = 201;
                        }
                    }
                } else {
                    l.passo = 1;
                }
            }
            201 => {
                if err.abs() > TOLERANCIA_POSICAO && freio_on {
                    l.motor = true;
                }
                let mut v = if cheio {
                    scale(DISTANCIA_REDUCAO_METADE, rapida_cheio)
                } else {
                    scale(DISTANCIA_REDUCAO_METADE_VAZIO, rapida_vazio)
                };
                if err.abs() < DISTANCIA_REDUCAO_VELOCIDADE {
                    v = VELOCIDADE_LENTA;
                    l.passo = 202;
                }
                l.velocidade = v * err.signum();
                // A line asks for the car while it goes to wait: go there instead.
                if l.posicao_destino == espera && l.id_destino == ID_ESPERA && l.id_busca != 0 {
                    l.motor = false;
                    l.passo = 1;
                }
                if !sem_sensor && !no_centro {
                    l.motor = false;
                    l.passo = 1;
                }
                l.iniciar_posicao_ok = false;
            }
            202 => {
                l.velocidade = VELOCIDADE_LENTA * err.signum();
                if err.abs() <= TOLERANCIA_POSICAO {
                    l.motor = false;
                    if driver_v == 0.0 && motor_off {
                        l.liberar_freio = false;
                    }
                    l.iniciar_posicao_ok = !l.liberar_freio;
                } else {
                    l.iniciar_posicao_ok = false;
                    l.liberar_freio = true;
                    if freio_on {
                        l.motor = true;
                    }
                }
                if l.t_posicao_ok >= TEMPO_POSICAO_OK && driver_v == 0.0 && !fim_de_curso {
                    l.inp = true;
                    l.motor = false;
                    l.liberar_freio = false;
                    l.iniciar_posicao_ok = false;
                    l.passo = 203;
                }
                if !sem_sensor && !no_centro && !l.inp {
                    l.motor = false;
                    l.passo = 1;
                }
            }
            203 => {
                l.motor = false;
                if driver_v == 0.0 || motor_off {
                    l.liberar_freio = false;
                }
                if l.id_busca == 0 && l.posicao_destino == entrega && sem_sensor {
                    l.passo = 100;
                }
                if l.id_busca != 0 && sem_sensor {
                    l.passo = 1;
                }
                if l.posicao_destino != entrega && l.posicao_destino != espera && no_centro {
                    l.passo = 1;
                }
                if l.posicao_destino == espera && sem_sensor {
                    l.passo = 1;
                }
            }
            402 => {
                l.motor = false;
                if driver_v == 0.0 || motor_off {
                    l.liberar_freio = false;
                }
            }
            _ => l.passo = 0,
        }
        if matches!(l.passo, 201 | 202) && (fim_de_curso) {
            l.motor = false;
            l.passo = 402;
        }

        // FB "Carro Cheio TDP".
        let posicionado_receber = l.inp && l.id_destino >= 100;
        let posicionado_transferir = l.inp && l.id_destino == ID_ENTREGA;
        let anterior_pronto = l.id_busca != 0 && l.id_busca == l.id_destino;
        let giro_parado = giro_v.abs() < 0.5;
        let giro_nom = p.giro_speed;
        let turn = |l: &mut FullCarLogic, toward_deliver: bool| {
            l.liberar_freio_giro = true;
            l.velocidade_giro = if toward_deliver { giro_nom } else { -giro_nom };
            l.motor_giro = true;
        };
        let stop_turn = |l: &mut FullCarLogic| {
            l.motor_giro = false;
            if giro_parado {
                l.liberar_freio_giro = false;
            }
        };
        match l.tdp_passo {
            0 => {
                if posicionado_transferir && (s4 || s3 || s2) {
                    l.pronto_transferir = true;
                    l.tdp_passo = 100;
                }
                if (posicionado_receber || (!s1 && !s2)) && !s4 && !s3 {
                    l.pronto_receber = true;
                    l.tdp_passo = 1;
                }
                if !s1 && s3 && s2 && !s4 {
                    l.tdp_passo = 5;
                }
            }
            1 => {
                // Gira para receber e aguarda o TDP anterior.
                l.motor_tdp = false;
                if tdp_v.abs() < 80.0 {
                    l.liberar_freio_tdp = false;
                }
                if sem_sensor && !giro_receber {
                    turn(l, false);
                } else {
                    stop_turn(l);
                }
                if anterior_pronto && giro_receber && posicionado_receber && !l.liberar_freio_giro {
                    l.velocidade_tdp = p.speed * 1000.0;
                    l.liberar_freio_tdp = true;
                    l.tdp_passo = 2;
                }
            }
            2 => {
                l.motor_tdp = freio_tdp;
                // TDPAnterior.Transferindo: the pallet reaches the entry sensor.
                if s1 {
                    l.tdp_passo = 3;
                }
                if !anterior_pronto && !s1 {
                    l.motor_tdp = false;
                    l.tdp_passo = 1;
                }
                if s4 || s3 {
                    l.motor_tdp = false;
                }
            }
            3 => {
                l.motor_tdp = freio_tdp;
                l.velocidade_tdp = p.speed * 1000.0 * if s3 { 0.7 } else { 1.0 };
                if (s3 && !s1 && s2) || s4 {
                    l.motor_tdp = false;
                    if tdp_v.abs() < 80.0 {
                        l.liberar_freio_tdp = false;
                        l.tdp_passo = 4;
                    }
                    l.iniciar_transferir = true;
                }
            }
            4 => {
                if l.t_transferir >= TEMPO_PRONTO_TRANSFERIR {
                    l.pronto_receber = false;
                    l.iniciar_transferir = false;
                    l.tdp_passo = 5;
                }
            }
            5 => {
                // Gira para entregar a caminho da entrega (ProximoEntrega) ou na entrega.
                let toward = posicionado_transferir || l.id_destino == ID_ENTREGA;
                if !s1 && s2 && s3 && !s4 && !giro_entregar && !l.liberar_freio_tdp && toward {
                    turn(l, true);
                } else {
                    stop_turn(l);
                }
                if giro_entregar {
                    l.motor_giro = false;
                    if giro_parado {
                        l.liberar_freio_giro = false;
                        l.tdp_passo = 6;
                    }
                }
            }
            6 => {
                if posicionado_transferir && giro_entregar {
                    l.pronto_transferir = true;
                    l.tdp_passo = 100;
                }
            }
            100 => {
                // TDP posterior pronto para receber: the simulation holds the pallet
                // when the next conveyor is busy.
                if tdp_v.abs() < 80.0 {
                    l.liberar_freio_tdp = false;
                }
                l.tdp_passo = 101;
            }
            101 => {
                l.velocidade_tdp = p.speed * 1000.0;
                l.liberar_freio_tdp = true;
                l.motor_tdp = freio_tdp;
                l.tdp_passo = 102;
            }
            102 => {
                l.motor_tdp = freio_tdp;
                if sem_sensor {
                    l.motor_tdp = false;
                    l.pronto_transferir = false;
                    l.iniciar_transferir = true;
                    l.tdp_passo = 103;
                }
            }
            103 => {
                if tdp_v.abs() < 80.0 {
                    l.liberar_freio_tdp = false;
                }
                if l.t_transferir >= TEMPO_PRONTO_TRANSFERIR && !l.liberar_freio_tdp {
                    l.iniciar_transferir = false;
                    l.tdp_passo = 104;
                }
            }
            104 => {
                if sem_sensor && !giro_receber {
                    turn(l, false);
                } else {
                    stop_turn(l);
                    if !l.liberar_freio_giro && posicionado_receber {
                        l.pronto_receber = true;
                        l.tdp_passo = 1;
                    }
                }
            }
            _ => l.tdp_passo = 0,
        }

        if std::env::var("PDT_TRACE").is_ok() && (l.passo, l.tdp_passo) != l.last {
            eprintln!(
                "[{:.2}] {}: movimento {} · TDP {} (busca {} destino {} INP {} pos {:.0} sensores {}{}{}{})",
                time.elapsed_secs(),
                eq.name,
                l.passo,
                l.tdp_passo,
                l.id_busca,
                l.id_destino,
                u8::from(l.inp),
                posicao,
                u8::from(s1),
                u8::from(s2),
                u8::from(s3),
                u8::from(s4)
            );
            l.last = (l.passo, l.tdp_passo);
        }
        drive(&mut io, "Motor", f64::from(u8::from(l.motor)));
        drive(&mut io, "LiberarFreio", f64::from(u8::from(l.liberar_freio)));
        drive(&mut io, "Velocidade", l.velocidade.round() as f64);
        drive(&mut io, "MotorTDP", f64::from(u8::from(l.motor_tdp)));
        drive(&mut io, "LiberarFreioTDP", f64::from(u8::from(l.liberar_freio_tdp)));
        drive(&mut io, "VelocidadeTDP", l.velocidade_tdp.round() as f64);
        drive(&mut io, "MotorGiro", f64::from(u8::from(l.motor_giro)));
        drive(&mut io, "LiberarFreioGiro", f64::from(u8::from(l.liberar_freio_giro)));
        drive(&mut io, "VelocidadeGiro", l.velocidade_giro.round() as f64);
    }
}

/// Car travel (inverter + brake), giro (slewing ring between mechanical stops),
/// loads on board and the feedback signals.
#[allow(clippy::type_complexity)]
pub fn fullcar_motion(
    time: Res<Time>,
    control: Res<SimControl>,
    mut cars: Query<(&Equipment, &Transform, &mut ShuttleState, &mut Io), Without<Load>>,
    mut loads: Query<(&Load, &mut Transform), Without<Equipment>>,
) {
    let dt = if control.running { time.delta_secs() * control.time_scale } else { 0.0 };
    for (eq, tf, mut state, mut io) in &mut cars {
        if eq.kind != EquipmentKind::FullCar {
            continue;
        }
        let p = &eq.params;
        let (min_x, max_x) = travel_limits(eq);
        if state.x == 0.0 {
            state.x = p.stops.first().copied().unwrap_or(min_x).clamp(min_x, max_x);
        }
        let before = conveyor::surface(eq, tf, Some(&*state));

        let brake_released = io.get_bool("LiberarFreio");
        let target_v = if brake_released && io.get_bool("Motor") { io.get("Velocidade") as f32 / 1000.0 } else { 0.0 };
        let accel = if brake_released { ACCELERATION } else { BRAKE_DECELERATION };
        state.v += (target_v - state.v).clamp(-accel * dt, accel * dt);
        let old_x = state.x;
        state.x = (state.x + state.v * dt).clamp(min_x, max_x);
        let at_limit = state.x <= min_x + 1e-4 || state.x >= max_x - 1e-4;
        if at_limit {
            state.v = 0.0;
        }
        let dx = state.x - old_x;

        // Giro between the mechanical stops at 0 (receive) and `giro_deg` (deliver).
        let end = p.giro_deg.to_radians();
        let (lo, hi) = (end.min(0.0), end.max(0.0));
        state.giro_v = if io.get_bool("LiberarFreioGiro") && io.get_bool("MotorGiro") {
            (io.get("VelocidadeGiro") as f32).to_radians()
        } else {
            0.0
        };
        let old_giro = state.giro;
        state.giro = (state.giro + state.giro_v * dt).clamp(lo, hi);
        if (state.giro - old_giro).abs() < 1e-7 {
            state.giro_v = 0.0;
        }
        let turn = Quat::from_rotation_y(state.giro - old_giro);

        // Loads on the TDP turn and travel with it.
        let pivot = before.transform.translation;
        let shift = tf.rotation * Vec3::X * dx;
        let inverse = before.transform.compute_affine().inverse();
        let mut loaded = false;
        for (load, mut ltf) in &mut loads {
            if load.state != LoadState::Free {
                continue;
            }
            let local = inverse.transform_point3(ltf.translation);
            let on_car = local.x.abs() <= before.length / 2.0
                && local.z.abs() <= before.width / 2.0
                && (local.y - load.size.y / 2.0 - before.top).abs() < 0.05;
            if on_car {
                loaded = true;
                ltf.translation = pivot + turn * (ltf.translation - pivot) + shift;
                ltf.rotation = turn * ltf.rotation;
            }
        }
        state.loaded = loaded;

        let tdp_v = if io.get_bool("MotorTDP") && io.get_bool("LiberarFreioTDP") { io.get("VelocidadeTDP") } else { 0.0 };
        io.set("PosicaoAtual", (state.x * 1000.0).round() as f64);
        io.set("DriverVelocidade", (state.v * 1000.0).round() as f64);
        io.set_bool("DriverFalha", false);
        io.set_bool("FimDeCurso", at_limit);
        io.set_bool("Giro_Sensor_PosReceber", state.giro.abs() <= GIRO_SENSOR);
        io.set_bool("Giro_Sensor_PosEntregar", (state.giro - end).abs() <= GIRO_SENSOR);
        io.set("VelocidadeAtualTDP", tdp_v);
        io.set("VelocidadeAtualGiro", f64::from(state.giro_v.to_degrees()));
    }
}

/// Turns the TDP visual and spins its rollers.
pub fn animate_fullcar(
    cars: Query<(&Equipment, &ShuttleState, &ConveyorState, &Children)>,
    bodies: Query<&Children, With<crate::shuttle::ShuttleBody>>,
    mut tops: Query<(&mut Transform, &Children), With<FullCarTop>>,
    mut rollers: Query<(&Roller, &mut Transform), Without<FullCarTop>>,
) {
    for (eq, state, conveyor, children) in &cars {
        if eq.kind != EquipmentKind::FullCar {
            continue;
        }
        for body in children.iter().filter_map(|c| bodies.get(c).ok()) {
            for top in body.iter() {
                let Ok((mut tf, top_children)) = tops.get_mut(top) else { continue };
                tf.rotation = Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2 + state.giro);
                for child in top_children.iter() {
                    if let Ok((roller, mut rtf)) = rollers.get_mut(child) {
                        let angle = -conveyor.travel[0] / roller.radius;
                        rtf.rotation = Quat::from_rotation_z(angle) * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
                    }
                }
            }
        }
    }
}
