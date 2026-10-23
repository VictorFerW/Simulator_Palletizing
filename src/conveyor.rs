//! Geometry of everything that carries loads: driven segments, pneumatic stops
//! (bloqueios/batentes), photo sensors and lateral pushers, in the equipment's
//! local frame (transport along +X, width along Z, origin at the floor).
//!
//! The mesas follow the Selgron FBs: the signal keys are the FB variable names,
//! so binding them to the PLC tags is one-to-one.

use bevy::prelude::{Quat, Transform, Vec3};

use crate::model::{Equipment, EquipmentKind};

/// Load-carrying surface of a conveyor: its frame (transport along local +X,
/// origin under the surface center) and size. For the CVZ it is the TDP on the
/// car: it follows the car along the rail and transports across it.
pub struct Surface {
    pub transform: Transform,
    pub length: f32,
    pub width: f32,
    pub top: f32,
}

pub fn surface(eq: &Equipment, tf: &Transform, car: Option<&crate::shuttle::ShuttleState>) -> Surface {
    let p = &eq.params;
    if matches!(eq.kind, EquipmentKind::ShuttleCar | EquipmentKind::FullCar) {
        // The car TDP transports across the rail; the CCH turns it by `giro`.
        let (x, giro) = car.map_or((0.0, 0.0), |c| (c.x, c.giro));
        let car = Transform::from_xyz(x - p.length / 2.0, 0.0, 0.0)
            .with_rotation(Quat::from_rotation_y(-std::f32::consts::FRAC_PI_2 + giro));
        return Surface { transform: *tf * car, length: p.car_length, width: p.car_width, top: p.height };
    }
    Surface { transform: *tf, length: p.length, width: p.width, top: p.height }
}

impl Surface {
    pub fn point(&self, local: Vec3) -> Vec3 {
        self.transform.transform_point(local)
    }
}

pub struct Segment {
    pub x0: f32,
    pub x1: f32,
    pub motor: &'static str,
}

/// A stop across the conveyor at local `x`, raised while `key` is on.
pub struct Barrier {
    pub x: f32,
    pub key: &'static str,
}

/// Through-beam sensor across `z0..z1` at local `x`, just above the rollers.
pub struct PhotoSensor {
    pub key: &'static str,
    pub x: f32,
    pub z0: f32,
    pub z1: f32,
}

/// Lateral actuator acting on the loads between `x0..x1`. A plate pusher moves
/// its face from `face_start` (retracted, +Z side) to `face_end` (advanced). A comb
/// (garra under the rollers) moves its center from `face_start` to `face_end`
/// and, while raised, lifts the fardo above it and carries it across.
pub struct Pusher {
    pub x0: f32,
    pub x1: f32,
    pub face_start: f32,
    pub face_end: f32,
    pub advance: &'static str,
    pub retract: &'static str,
    pub sensor_advanced: &'static str,
    pub sensor_retracted: &'static str,
    /// Vertical motion of a comb: (raise, lower, sensor raised, sensor lowered).
    pub lift: Option<(&'static str, &'static str, &'static str, &'static str)>,
    pub comb: bool,
    /// The comb lifts the whole row ahead of it (receita 10x1: each push moves every
    /// fardo already pushed one more lane).
    pub train: bool,
}

#[derive(Default)]
pub struct ConveyorLayout {
    pub segments: Vec<Segment>,
    pub barriers: Vec<Barrier>,
    pub sensors: Vec<PhotoSensor>,
    pub pushers: Vec<Pusher>,
    /// Start of the pick area (coleta), where the robot takes the loads.
    pub pick_x0: Option<f32>,
}

/// Lateral position (local Z) of the two product lanes of a mesa: the entry lane
/// (posição 1, where fardos arrive) and the pushed lane (posição 2).
pub fn mesa_lanes(eq: &Equipment) -> (f32, f32) {
    if is_10x1(eq) {
        let lanes = lanes_10x1(eq);
        return (lanes[3], lanes[2]);
    }
    let bw = eq.params.box_size[2];
    (bw / 2.0 + 0.01, -bw / 2.0 - 0.01)
}

pub fn is_10x1(eq: &Equipment) -> bool {
    eq.kind == EquipmentKind::Table3Stage && eq.params.fardo_10kg
}

/// Receita 10x1 of the mesa 3 estágios: four lanes side by side, from the physical
/// stop of the pré-formadora (lane 0, far side) to the entry lane (lane 3).
pub fn lanes_10x1(eq: &Equipment) -> [f32; 4] {
    let bw = eq.params.box_size[2];
    let pitch = bw + 0.02;
    let stop = -eq.params.width / 2.0 + bw / 2.0 + 0.005;
    [stop, stop + pitch, stop + 2.0 * pitch, stop + 3.0 * pitch]
}

pub fn layout(eq: &Equipment) -> ConveyorLayout {
    let p = &eq.params;
    let (l, w) = (p.length, p.width);
    let (x_start, x_end) = (-l / 2.0, l / 2.0);
    let bl = p.box_size[0];
    let lane_sensor = |key: &'static str, x: f32, lane: f32| PhotoSensor { key, x, z0: lane - 0.1, z1: lane + 0.1 };
    match eq.kind {
        EquipmentKind::BeltConveyor | EquipmentKind::PalletConveyor => ConveyorLayout {
            segments: vec![Segment { x0: x_start, x1: x_end, motor: "motor" }],
            sensors: if eq.kind == EquipmentKind::PalletConveyor {
                vec![PhotoSensor { key: "presence", x: x_end - 0.1, z0: -w / 2.0, z1: w / 2.0 }]
            } else {
                Vec::new()
            },
            ..Default::default()
        },
        EquipmentKind::ShuttleCar => {
            let (cl, cw) = (p.car_length, p.car_width);
            ConveyorLayout {
                segments: vec![Segment { x0: -cl / 2.0, x1: cl / 2.0, motor: "Motor_TDP" }],
                sensors: vec![PhotoSensor { key: "Sensor_Pallet_Carro", x: cl / 2.0 - 0.1, z0: -cw / 2.0, z1: cw / 2.0 }],
                ..Default::default()
            }
        }
        EquipmentKind::FullCar => {
            // Sensor 1 at the entry; 2 and 3 covered by a centred pallet; 4 past the centre.
            let (cl, cw, half) = (p.car_length, p.car_width, p.pallet_size[0] / 2.0);
            let beam = |key, x| PhotoSensor { key, x, z0: -cw / 2.0, z1: cw / 2.0 };
            ConveyorLayout {
                segments: vec![Segment { x0: -cl / 2.0, x1: cl / 2.0, motor: "MotorTDP" }],
                sensors: vec![
                    beam("Sensor1", -cl / 2.0 + 0.1),
                    beam("Sensor2", -half + 0.15),
                    beam("Sensor3", half - 0.05),
                    beam("Sensor4", (half + 0.12).min(cl / 2.0 - 0.05)),
                ],
                ..Default::default()
            }
        }
        EquipmentKind::PalletizingConveyor => ConveyorLayout {
            segments: vec![Segment { x0: x_start, x1: x_end, motor: "motor" }],
            barriers: vec![Barrier { x: x_end, key: "stop" }],
            ..Default::default()
        },
        EquipmentKind::Table2Stage => {
            // Roletada (antecipada) + Coleta, each half of the length.
            let x_col = x_start + l / 2.0;
            let (lane_in, lane_out) = mesa_lanes(eq);
            let at_stop = x_col - 0.08;
            ConveyorLayout {
                segments: vec![
                    Segment { x0: x_start, x1: x_col, motor: "Motor" },
                    Segment { x0: x_col, x1: x_end, motor: "Motor_Coleta" },
                ],
                barriers: vec![
                    Barrier { x: x_start, key: "Bloqueio_Entrada" },
                    Barrier { x: x_col, key: "Bloqueio_Coleta" },
                    Barrier { x: x_end, key: "Batente_Coleta" },
                ],
                sensors: vec![
                    lane_sensor("Sensor_EsteiraFim", x_start - 0.5, lane_in),
                    lane_sensor("Sensor_AntesBloqueio", x_start - 0.08, lane_in),
                    lane_sensor("Sensor_AposBloqueio", x_start + 0.15, lane_in),
                    lane_sensor("Sensor_RoletadaPos1", at_stop, lane_in),
                    lane_sensor("Sensor_RoletadaPos2", at_stop, lane_out),
                    lane_sensor("Sensor_RoletadaPos3", at_stop - bl, lane_out),
                    lane_sensor("Sensor_Coleta1", x_end - 0.08, lane_in),
                    lane_sensor("Sensor_Coleta2", x_end - 0.08, lane_out),
                ],
                pushers: vec![Pusher {
                    x0: x_col - bl - 0.04,
                    x1: x_col - 0.01,
                    face_start: w / 2.0,
                    face_end: lane_out + p.box_size[2] / 2.0,
                    advance: "EmpurradorAv",
                    retract: "EmpurradorRc",
                    sensor_advanced: "Sensor_EmpurradorAv",
                    sensor_retracted: "Sensor_EmpurradorRc",
                    lift: None,
                    comb: false,
                    train: false,
                }],
                pick_x0: Some(x_col),
            }
        }
        EquipmentKind::Table3Stage if is_10x1(eq) => {
            // Receita 10x1: the garra pushes the row one lane per stroke (entry lane 3
            // to lanes 2 and 1); the empurrador takes a fardo from the entry lane to the
            // stop (lane 0); groups of 3 (lanes 0..2) or 4 (0..3) go to the coleta.
            let x_mid = x_start + l / 3.0;
            let x_col = x_start + 2.0 * l / 3.0;
            let lane = lanes_10x1(eq);
            let beam = |key: &'static str, x: f32, z: f32| PhotoSensor { key, x, z0: z - 0.05, z1: z + 0.05 };
            ConveyorLayout {
                segments: vec![
                    Segment { x0: x_start, x1: x_mid, motor: "Motor" },
                    Segment { x0: x_mid, x1: x_col, motor: "Motor" },
                    Segment { x0: x_col, x1: x_end, motor: "Motor_Coleta" },
                ],
                barriers: vec![
                    Barrier { x: x_start, key: "BloqueioEntrada" },
                    Barrier { x: x_mid, key: "BloqueioMeio" },
                    Barrier { x: x_col, key: "BloqueioColeta" },
                    Barrier { x: x_end, key: "Batente_Coleta" },
                ],
                sensors: vec![
                    beam("SensorEsteira", x_start - 0.5, lane[3]),
                    beam("SensorEtapa1Pac1ProxBloq", x_start + 0.15, lane[3]),
                    beam("SensorEtapa1Pac1", x_mid - 0.08, lane[3]),
                    beam("SensorEtapa1Pac2", x_mid - 0.08, lane[2]),
                    beam("SensorEtapa1Pac3", x_mid - 0.08, lane[1]),
                    beam("SensorEtapa2Pac1", x_col - 0.08, lane[3]),
                    beam("SensorEtapa2Pac2", x_col - 0.08, lane[0]),
                    beam("SensorEtapa2Pac3", x_col - 0.08, lane[1]),
                    beam("Sensor_Coleta1", x_end - 0.08, lane[1]),
                    beam("Sensor_Coleta2", x_end - 0.08, lane[0]),
                ],
                pushers: vec![
                    Pusher {
                        x0: x_mid - bl - 0.04,
                        x1: x_mid - 0.01,
                        face_start: lane[3],
                        face_end: lane[2],
                        advance: "GarraAvancar",
                        retract: "GarraRecuar",
                        sensor_advanced: "SensorGarraAv",
                        sensor_retracted: "SensorGarraRec",
                        lift: Some(("GarraSobe", "GarraDesce", "SensorGarraLevantada", "SensorGarraRecolhida")),
                        comb: true,
                        train: true,
                    },
                    Pusher {
                        x0: x_col - bl - 0.04,
                        x1: x_col - 0.01,
                        face_start: w / 2.0,
                        face_end: lane[0] + p.box_size[2] / 2.0,
                        advance: "EmpurradorAvancar",
                        retract: "EmpurradorRecuar",
                        sensor_advanced: "SensorEmpurradorAv",
                        sensor_retracted: "SensorEmpurradorRec",
                        lift: None,
                        comb: false,
                        train: true,
                    },
                ],
                pick_x0: Some(x_col),
            }
        }
        EquipmentKind::Table3Stage => {
            // Roletada (etapa 1) + Pré-formadora (etapa 2) + Coleta, a third each.
            let x_mid = x_start + l / 3.0;
            let x_col = x_start + 2.0 * l / 3.0;
            let (lane_in, lane_out) = mesa_lanes(eq);
            ConveyorLayout {
                segments: vec![
                    Segment { x0: x_start, x1: x_mid, motor: "Motor" },
                    Segment { x0: x_mid, x1: x_col, motor: "Motor" },
                    Segment { x0: x_col, x1: x_end, motor: "Motor_Coleta" },
                ],
                barriers: vec![
                    Barrier { x: x_start, key: "BloqueioEntrada" },
                    Barrier { x: x_mid, key: "BloqueioMeio" },
                    Barrier { x: x_col, key: "BloqueioColeta" },
                    Barrier { x: x_end, key: "Batente_Coleta" },
                ],
                sensors: vec![
                    lane_sensor("SensorEsteira", x_start - 0.5, lane_in),
                    lane_sensor("SensorEtapa1Pac1ProxBloq", x_start + 0.15, lane_in),
                    lane_sensor("SensorEtapa1Pac1", x_mid - 0.08, lane_in),
                    lane_sensor("SensorEtapa1Pac2", x_mid - 0.08, lane_out),
                    lane_sensor("SensorEtapa1Pac3", x_mid - 0.08 - bl, lane_out),
                    lane_sensor("SensorEtapa2Pac1", x_col - 0.08, lane_in),
                    PhotoSensor { key: "SensorEtapa2Pac2", x: x_col - 0.08, z0: -0.04, z1: 0.04 },
                    lane_sensor("SensorEtapa2Pac3", x_col - 0.08, lane_out),
                    lane_sensor("Sensor_Coleta1", x_end - 0.08, lane_in),
                    lane_sensor("Sensor_Coleta2", x_end - 0.08, lane_out),
                ],
                pushers: vec![
                    // Garra: comb between the rollers, from under posição 1 to posição 2.
                    Pusher {
                        x0: x_mid - bl - 0.04,
                        x1: x_mid - 0.01,
                        face_start: lane_in,
                        face_end: lane_out,
                        advance: "GarraAvancar",
                        retract: "GarraRecuar",
                        sensor_advanced: "SensorGarraAv",
                        sensor_retracted: "SensorGarraRec",
                        lift: Some(("GarraSobe", "GarraDesce", "SensorGarraLevantada", "SensorGarraRecolhida")),
                        comb: true,
                        train: false,
                    },
                    Pusher {
                        x0: x_col - bl - 0.04,
                        x1: x_col - 0.01,
                        face_start: w / 2.0,
                        face_end: p.box_size[2] / 2.0,
                        advance: "EmpurradorAvancar",
                        retract: "EmpurradorRecuar",
                        sensor_advanced: "SensorEmpurradorAv",
                        sensor_retracted: "SensorEmpurradorRec",
                        lift: None,
                        comb: false,
                        train: false,
                    },
                ],
                pick_x0: Some(x_col),
            }
        }
        _ => ConveyorLayout::default(),
    }
}
