//! Carro de pallet vazio (CVZ): car on a rail carrying a TDP that transports
//! across the rail. Its position is read by a distance sensor at the rail start
//! (`PosicaoAtual`, mm) and it is driven by an inverter (`LigarDrive`,
//! `LiberarFreio`, `Velocidade` in mm/s, signed).
//!
//! The internal logic ports the positioning of the CVZ FB (step 201: nominal
//! speed, proportional slow-down, precision speed, stop within tolerance, brake
//! once the speed feedback is zero) with a simple cycle: receive an empty pallet
//! at the first stop (coleta), take it to the next destination stop, unload it,
//! return. The FB's destination choice depends on pallet tracking IDs from the
//! PLC; with a PLC connected, the PLC decides.

use bevy::prelude::*;

use crate::conveyor;
use crate::model::{Equipment, EquipmentKind, Io, SimControl};
use crate::sim::{Load, LoadState};

/// Car motion state; `x` is the car center measured from the rail start.
#[derive(Component, Debug, Default)]
pub struct ShuttleState {
    pub x: f32,
    pub v: f32,
    /// A load rests on the car TDP.
    pub loaded: bool,
}

/// Child entity carrying the car meshes (moved along the rail).
#[derive(Component)]
pub struct ShuttleBody;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ShuttleStep {
    #[default]
    Reset,
    /// Moving to `target` stop.
    Travel,
    /// At the coleta: run the TDP until a pallet is on board.
    Load,
    /// At a destination: run the TDP until the car is empty.
    Unload,
}

#[derive(Component, Debug, Default)]
pub struct ShuttleLogic {
    pub step: ShuttleStep,
    pub target: usize,
    next_destination: usize,
    pub inp: bool,
    empty_timer: f32,
}

// Positioning parameters of the FB (mm, mm/s).
const TOLERANCIA_PARADA: f32 = 5.0;
const DISTANCIA_PRECISAO: f32 = 50.0;
const DISTANCIA_INICIO_REDUCAO: f32 = 500.0;
const VELOCIDADE_PRECISAO: f32 = 80.0;
const ACCELERATION: f32 = 0.8; // m/s²
const BRAKE_DECELERATION: f32 = 3.0; // m/s²

/// Car travel limits along the rail (center positions).
pub fn travel_limits(eq: &Equipment) -> (f32, f32) {
    let half = eq.params.car_width / 2.0;
    (half, (eq.params.length - half).max(half))
}

fn drive(io: &mut Io, key: &'static str, value: f64) {
    if !io.from_plc.contains(key) {
        io.set(key, value);
    }
}

pub fn shuttle_logic(time: Res<Time>, control: Res<SimControl>, mut cars: Query<(&Equipment, &ShuttleState, &mut Io, &mut ShuttleLogic)>) {
    if !control.running {
        return;
    }
    let dt = time.delta_secs() * control.time_scale;
    for (eq, state, mut io, mut logic) in &mut cars {
        if eq.kind != EquipmentKind::ShuttleCar || !eq.params.auto_logic || eq.params.stops.is_empty() {
            continue;
        }
        let stops = &eq.params.stops;
        let target_mm = stops[logic.target.min(stops.len() - 1)] * 1000.0;
        let pos_mm = io.get("PosicaoAtual") as f32;
        let err = target_mm - pos_mm;
        let mut motor_tdp = false;
        match logic.step {
            ShuttleStep::Reset => {
                logic.target = 0;
                logic.next_destination = 1;
                logic.step = ShuttleStep::Travel;
                logic.inp = false;
            }
            ShuttleStep::Travel => {
                logic.inp = false;
                if err.abs() > TOLERANCIA_PARADA {
                    // Speed profile of step 201.
                    let nominal = eq.params.car_speed * 1000.0;
                    let speed = if err.abs() > DISTANCIA_INICIO_REDUCAO {
                        nominal
                    } else if err.abs() > DISTANCIA_PRECISAO {
                        let k = (err.abs() - DISTANCIA_PRECISAO) / (DISTANCIA_INICIO_REDUCAO - DISTANCIA_PRECISAO);
                        VELOCIDADE_PRECISAO + (nominal - VELOCIDADE_PRECISAO) * k
                    } else {
                        VELOCIDADE_PRECISAO
                    };
                    drive(&mut io, "LiberarFreio", 1.0);
                    drive(&mut io, "LigarDrive", 1.0);
                    drive(&mut io, "Velocidade", (speed * err.signum()).round() as f64);
                } else {
                    drive(&mut io, "LigarDrive", 0.0);
                    if io.get("DriverFDBVelocidade") == 0.0 {
                        drive(&mut io, "LiberarFreio", 0.0);
                        logic.inp = true;
                        logic.empty_timer = 0.0;
                        logic.step = if logic.target == 0 { ShuttleStep::Load } else { ShuttleStep::Unload };
                    }
                }
            }
            ShuttleStep::Load => {
                if !io.get_bool("Sensor_Pallet_Carro") {
                    motor_tdp = true;
                } else if stops.len() > 1 {
                    // Destinations are served in turn (posição A, B, ...).
                    let destination = logic.next_destination.clamp(1, stops.len() - 1);
                    logic.target = destination;
                    logic.next_destination = if destination + 1 >= stops.len() { 1 } else { destination + 1 };
                    logic.step = ShuttleStep::Travel;
                }
            }
            ShuttleStep::Unload => {
                motor_tdp = true;
                if state.loaded {
                    logic.empty_timer = 0.0;
                } else {
                    logic.empty_timer += dt;
                    if logic.empty_timer > 0.5 {
                        logic.target = 0;
                        logic.step = ShuttleStep::Travel;
                    }
                }
            }
        }
        drive(&mut io, "Motor_TDP", if motor_tdp { 1.0 } else { 0.0 });
        drive(&mut io, "Reverso_TDP", 0.0);
    }
}

/// Inverter + brake dynamics, carries the loads on board, publishes the
/// distance sensor and the drive feedback.
#[allow(clippy::type_complexity)]
pub fn shuttle_motion(
    time: Res<Time>,
    control: Res<SimControl>,
    mut cars: Query<(&Equipment, &Transform, &mut ShuttleState, &mut Io), Without<Load>>,
    mut loads: Query<(&Load, &mut Transform), Without<Equipment>>,
) {
    let dt = if control.running { time.delta_secs() * control.time_scale } else { 0.0 };
    for (eq, tf, mut state, mut io) in &mut cars {
        if eq.kind != EquipmentKind::ShuttleCar {
            continue;
        }
        let (min_x, max_x) = travel_limits(eq);
        if state.x == 0.0 {
            state.x = eq.params.stops.first().copied().unwrap_or(min_x).clamp(min_x, max_x);
        }
        let brake_released = io.get_bool("LiberarFreio");
        let target_v = if brake_released && io.get_bool("LigarDrive") {
            io.get("Velocidade") as f32 / 1000.0
        } else {
            0.0
        };
        let accel = if brake_released { ACCELERATION } else { BRAKE_DECELERATION };
        let dv = (target_v - state.v).clamp(-accel * dt, accel * dt);
        state.v += dv;
        let old_x = state.x;
        state.x = (state.x + state.v * dt).clamp(min_x, max_x);
        if state.x == min_x || state.x == max_x {
            state.v = 0.0;
        }
        let dx = state.x - old_x;

        // Loads resting on the car TDP travel with it.
        let surface = conveyor::surface(eq, tf, Some(state.x));
        let inverse = surface.transform.compute_affine().inverse();
        let shift = tf.rotation * Vec3::X * dx;
        let mut loaded = false;
        for (load, mut ltf) in &mut loads {
            if load.state != LoadState::Free {
                continue;
            }
            let local = inverse.transform_point3(ltf.translation);
            let on_car = local.x.abs() <= surface.length / 2.0
                && local.z.abs() <= surface.width / 2.0
                && (local.y - load.size.y / 2.0 - surface.top).abs() < 0.05;
            if on_car {
                loaded = true;
                ltf.translation += shift;
            }
        }
        state.loaded = loaded;

        // Distance sensor at the rail start; 0 would mean signal lost.
        io.set("PosicaoAtual", (state.x * 1000.0).round() as f64);
        io.set("DriverFDBVelocidade", (state.v * 1000.0).round() as f64);
        io.set_bool("DriverEmFalha", false);
        io.set_bool("SensorExtremoAtivoTDP", state.x <= min_x + 0.005 || state.x >= max_x - 0.005);
    }
}

pub fn animate_shuttle(
    cars: Query<(&Equipment, &ShuttleState, &Children)>,
    mut bodies: Query<&mut Transform, With<ShuttleBody>>,
) {
    for (eq, state, children) in &cars {
        for child in children.iter() {
            if let Ok(mut tf) = bodies.get_mut(child) {
                tf.translation.x = state.x - eq.params.length / 2.0;
            }
        }
    }
}
