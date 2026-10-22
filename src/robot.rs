//! FANUC M-410iC/185: 4-axis palletizing robot (J1 base rotation, J2/J3 arm
//! with parallelograms keeping the wrist level, J4 flange yaw).
//! Dimensions measured on the manufacturer's STEP model (robot on its pedestal):
//! horizontal reach to the flange 3.164 m (data sheet: 3143 mm). The program
//! below plays the role of the robot controller's palletizing routine.

use std::f32::consts::{FRAC_PI_2, PI};

use bevy::{math::Affine3A, prelude::*};

use crate::model::{Equipment, EquipmentKind, Io};
use crate::mesa_logic::MesaLogic;
use crate::sim::{self, Load, LoadKind, LoadState, PalletLoad, PalletizerState};

/// J2 axis height above the floor (robot on its pedestal).
pub const BASE_HEIGHT: f32 = 1.110;
/// J2 axis ahead of the J1 axis.
pub const J2_OFFSET: f32 = 0.394;
/// J2 -> J3.
pub const L1: f32 = 1.220;
/// J3 -> wrist pivot.
pub const L2: f32 = 1.306;
pub const UPPER_BACK: f32 = 0.45;
/// Flange (J4 axis) ahead of the wrist pivot.
pub const WRIST_FWD: f32 = 0.244;
/// Flange face below the wrist pivot.
pub const WRIST_DROP: f32 = 0.159;
/// Cabeçote Selgron "4 fardos" (0323P01-010000): flange coupling to the bottom of
/// the frame, where the fardos' top rests (TCP).
pub const TOOL_LENGTH: f32 = 0.435;
/// Clamp (pinça) half opening when open, and its height below the frame.
pub const JAW_OPEN: f32 = 0.60;
pub const JAW_HEIGHT: f32 = 0.295;
pub const TCP_DROP: f32 = WRIST_DROP + TOOL_LENGTH;
/// Horizontal reach limit used to pick the nearest conveyor/pallet automatically.
pub const REACH: f32 = J2_OFFSET + WRIST_FWD + L1 + L2 - 0.1;

/// Pivots of the STEP model in its rest pose (lower arm vertical, upper arm
/// horizontal), in the robot frame: (x forward, y up) of the arm plane.
mod rest {
    use bevy::math::Vec2;
    pub const J2: Vec2 = Vec2::new(0.394, 1.110);
    pub const J3: Vec2 = Vec2::new(0.394, 2.330);
    pub const WRIST: Vec2 = Vec2::new(1.700, 2.330);
    /// S303 lever end / S304 rod bottom.
    pub const LEVER: Vec2 = Vec2::new(-0.199, 1.135);
    /// S305 rod bottom, fixed on the turntable.
    pub const LEVEL_ROD: Vec2 = Vec2::new(-0.111, 1.3795);
    /// S401 bellcrank / S403 link rear end.
    pub const LEVEL_LINK: Vec2 = Vec2::new(0.888, 2.5978);
    /// J4 axis (flange center), lateral offset in z.
    pub const FLANGE_X: f32 = 1.944;
    pub const FLANGE_Z: f32 = 0.0188;
}

/// Parts of the manufacturer's model (`assets/models/fanuc_m410ic185/*.glb`).
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug)]
pub enum CadPart {
    Base,
    Turntable,
    LowerArm,
    Lever,
    LeverRod,
    LevelRod,
    Bellcrank,
    UpperArm,
    LevelLink,
    Wrist,
    Flange,
}

impl CadPart {
    pub const ALL: [(Self, &'static str); 12] = [
        (Self::Base, "S201"),
        (Self::Base, "Base_plate"),
        (Self::Turntable, "S301"),
        (Self::LowerArm, "S302"),
        (Self::Lever, "S303"),
        (Self::LeverRod, "S304"),
        (Self::LevelRod, "S305"),
        (Self::Bellcrank, "S401"),
        (Self::UpperArm, "S402"),
        (Self::LevelLink, "S403"),
        (Self::Wrist, "S501"),
        (Self::Flange, "S502"),
    ];

    /// Pose of the part in the robot frame for the given joints.
    fn transform(self, joints: &Joints, flange_yaw: f32) -> Transform {
        let a = joints.shoulder - FRAC_PI_2; // lower arm rotation from rest (vertical)
        let b = joints.upper_dir.y.atan2(joints.upper_dir.x); // upper arm from rest (horizontal)
        let j2 = rest::J2.extend(0.0);
        let j3 = (rest::J2 + joints.elbow).extend(0.0);
        let d3 = j3 - rest::J3.extend(0.0);
        let wrist = j3 + (joints.upper_dir * L2).extend(0.0);
        let dw = wrist - rest::WRIST.extend(0.0);
        let about = |pivot: Vec3, new_pivot: Vec3, angle: f32| {
            Affine3A::from_translation(new_pivot) * Affine3A::from_rotation_z(angle) * Affine3A::from_translation(-pivot)
        };
        let arm = match self {
            Self::Base | Self::Turntable => Affine3A::IDENTITY,
            Self::LowerArm => about(j2, j2, a),
            Self::Lever => about(j2, j2, b),
            Self::LeverRod => {
                let lever = rest::LEVER.extend(0.0);
                let end = j2 + Quat::from_rotation_z(b) * (lever - j2);
                about(lever, end, a)
            }
            Self::LevelRod => about(rest::LEVEL_ROD.extend(0.0), rest::LEVEL_ROD.extend(0.0), a),
            Self::Bellcrank | Self::Wrist => Affine3A::from_translation(if self == Self::Wrist { dw } else { d3 }),
            Self::UpperArm => about(rest::J3.extend(0.0), j3, b),
            Self::LevelLink => {
                let link = rest::LEVEL_LINK.extend(0.0);
                about(link, link + d3, b)
            }
            Self::Flange => {
                let axis = Vec3::new(rest::FLANGE_X, 0.0, rest::FLANGE_Z);
                Affine3A::from_translation(axis + dw) * Affine3A::from_rotation_y(flange_yaw) * Affine3A::from_translation(-axis)
            }
        };
        let full = if self == Self::Base { arm } else { Affine3A::from_rotation_y(joints.j1) * arm };
        Transform::from_matrix(Mat4::from(full))
    }
}

#[derive(Component, Clone, Copy, PartialEq, Eq, Debug)]
pub enum RobotPart {
    Turntable,
    LowerArm,
    UpperArm,
    Rod,
    Wrist,
    Gripper,
    /// Clamps of the cabeçote, on either side (`-Z` / `+Z` of the gripper).
    JawA,
    JawB,
}

/// TCP pose in the robot frame, cylindrical: angle around J1, radius, height,
/// and gripper yaw (robot frame).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub theta: f32,
    pub r: f32,
    pub y: f32,
    pub yaw: f32,
}

impl Pose {
    pub const HOME: Self = Self {
        theta: 0.0,
        r: 1.7,
        y: 1.7,
        yaw: 0.0,
    };

    pub fn local_point(&self) -> Vec3 {
        Vec3::new(self.r * self.theta.cos(), self.y, -self.r * self.theta.sin())
    }

    pub fn from_local(p: Vec3, yaw: f32) -> Self {
        Self {
            theta: (-p.z).atan2(p.x),
            r: Vec2::new(p.x, p.z).length(),
            y: p.y,
            yaw,
        }
    }

    fn lerp(&self, to: &Self, t: f32) -> Self {
        Self {
            theta: self.theta + angle_diff(self.theta, to.theta) * t,
            r: self.r + (to.r - self.r) * t,
            y: self.y + (to.y - self.y) * t,
            yaw: self.yaw + (to.yaw - self.yaw) * t,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    Idle,
    ToAbovePick,
    DownPick,
    Grip,
    UpPick,
    ToAbovePlace,
    DownPlace,
    Release,
    UpPlace,
}

/// A fardo carried by the gripper, relative to the picked group.
#[derive(Debug, Clone)]
pub struct HeldBox {
    pub entity: Entity,
    /// Horizontal offset from the group center, in the group frame.
    offset: Vec3,
    rel_yaw: f32,
    height: f32,
}

#[derive(Component, Debug)]
pub struct RobotState {
    pub pose: Pose,
    from: Pose,
    to: Pose,
    t: f32,
    duration: f32,
    pub step: Step,
    wait: f32,
    pub held: Vec<HeldBox>,
    held_plan: Vec<HeldBox>,
    /// Group yaw minus gripper yaw (world), captured when gripping.
    hold_yaw: f32,
    group_yaw: f32,
    /// Footprint of the picked group (one pallet placement).
    pub unit: Vec3,
    pick_top: Vec3,
    pick_conveyor: Option<Entity>,
    place: Option<(Entity, Vec3, f32)>,
    safe_y: f32,
    /// Robo_Fardos_Coletados.
    pub collected: bool,
    /// Clamp closure 0 (open) .. 1 (closed on the fardos).
    pub jaw: f32,
    pub status: String,
    pub cycles: u32,
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            pose: Pose::HOME,
            from: Pose::HOME,
            to: Pose::HOME,
            t: 0.0,
            duration: 0.0,
            step: Step::Idle,
            wait: 0.0,
            held: Vec::new(),
            held_plan: Vec::new(),
            hold_yaw: 0.0,
            group_yaw: 0.0,
            unit: Vec3::ONE,
            pick_top: Vec3::ZERO,
            pick_conveyor: None,
            place: None,
            safe_y: Pose::HOME.y,
            collected: false,
            jaw: 0.0,
            status: "Aguardando".into(),
            cycles: 0,
        }
    }
}

impl RobotState {
    fn moving(&self) -> bool {
        self.t < self.duration
    }

    fn move_to(&mut self, to: Pose, speed_override: f32, vertical: bool) {
        let ovr = speed_override.clamp(0.05, 1.0);
        let from = self.pose;
        let linear = if vertical { 1.2 } else { 3.0 } * ovr;
        let angular = 2.4 * ovr;
        let a = from.local_point();
        let b = to.local_point();
        let duration = [
            angle_diff(from.theta, to.theta).abs() / angular,
            (from.r - to.r).abs() / linear,
            (from.y - to.y).abs() / linear,
            (a - b).length() / (linear * 1.4),
            (from.yaw - to.yaw).abs() / (4.0 * ovr),
        ]
        .into_iter()
        .fold(0.12, f32::max);
        self.from = from;
        self.to = to;
        self.t = 0.0;
        self.duration = duration;
    }
}

pub struct Joints {
    pub j1: f32,
    pub shoulder: f32,
    pub elbow: Vec2,
    pub upper_dir: Vec2,
    pub reachable: bool,
}

/// Inverse kinematics in the arm plane (elbow-up solution). `elbow` and the
/// wrist pivot are relative to the J2 axis; the TCP is `WRIST_FWD` ahead of and
/// `TCP_DROP` below the wrist pivot.
pub fn solve(pose: &Pose) -> Joints {
    let r = pose.r - J2_OFFSET - WRIST_FWD;
    let h = pose.y + TCP_DROP - BASE_HEIGHT;
    let target = Vec2::new(r, h);
    let d = target.length();
    let reachable = d <= L1 + L2 - 1e-3 && d >= (L1 - L2).abs() + 1e-3 && pose.y >= -0.05;
    let dc = d.clamp((L1 - L2).abs() + 1e-3, L1 + L2 - 1e-3);
    let cos_inner = ((L1 * L1 + dc * dc - L2 * L2) / (2.0 * L1 * dc)).clamp(-1.0, 1.0);
    let shoulder = h.atan2(r) + cos_inner.acos();
    let elbow = Vec2::new(L1 * shoulder.cos(), L1 * shoulder.sin());
    let wrist = if reachable {
        target
    } else {
        elbow + (target - elbow).normalize_or_zero() * L2
    };
    Joints {
        j1: pose.theta,
        shoulder,
        elbow,
        upper_dir: (wrist - elbow).normalize_or_zero(),
        reachable,
    }
}

fn plane_to_local(j1: f32, p: Vec2) -> Vec3 {
    let radial = J2_OFFSET + p.x;
    Vec3::new(radial * j1.cos(), BASE_HEIGHT + p.y, -radial * j1.sin())
}

fn link_rotation(j1: f32, plane_angle: f32) -> Quat {
    Quat::from_rotation_y(j1) * Quat::from_rotation_z(plane_angle - FRAC_PI_2)
}

/// Shortest signed difference `to - from` in radians.
fn angle_diff(from: f32, to: f32) -> f32 {
    (to - from + PI).rem_euclid(2.0 * PI) - PI
}

/// Picks `target + k*PI` closest to `current` (boxes are symmetric under 180°).
fn nearest_half_turn(current: f32, target: f32) -> f32 {
    let k = ((current - target) / PI).round();
    target + k * PI
}

fn smooth(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

pub fn yaw_of(rotation: Quat) -> f32 {
    rotation.to_euler(EulerRot::YXZ).0
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn robot_program(
    time: Res<Time>,
    control: Res<crate::model::SimControl>,
    mut commands: Commands,
    mut robots: Query<(&Equipment, &Transform, &mut RobotState, &mut Io), Without<Load>>,
    equipment: Query<(Entity, &Equipment, &Transform), (Without<Load>, Without<RobotState>)>,
    palletizers: Query<&PalletizerState>,
    mut mesas: Query<&mut MesaLogic>,
    mut loads: Query<(Entity, &mut Load, &mut Transform), Without<Equipment>>,
    mut pallet_loads: Query<&mut PalletLoad>,
) {
    let dt = if control.running { time.delta_secs() * control.time_scale } else { 0.0 };
    for (robot_eq, robot_tf, mut state, mut io) in &mut robots {
        let params = &robot_eq.params;
        let world_to_local = robot_tf.compute_affine().inverse();
        let robot_yaw = yaw_of(robot_tf.rotation);
        let in_reach = |p: Vec3, margin: f32| {
            let local = world_to_local.transform_point3(p);
            Vec2::new(local.x, local.z).length() <= REACH + margin
        };

        // Pick conveyor (mesa coleta or belt end) and palletizing TDP, by name or nearest in reach.
        let pick = equipment
            .iter()
            .filter(|(_, e, _)| e.kind.is_product_conveyor())
            .filter(|(_, e, _)| params.pick_from.is_empty() || e.name == params.pick_from)
            .map(|(entity, e, tf)| {
                let end = tf.transform_point(Vec3::new(e.params.length / 2.0 - 0.3, e.params.height, 0.0));
                (entity, e, tf, end)
            })
            .filter(|(.., end)| in_reach(*end, 0.0))
            .min_by(|a, b| {
                a.3.distance_squared(robot_tf.translation)
                    .total_cmp(&b.3.distance_squared(robot_tf.translation))
            });
        let palletizer = equipment
            .iter()
            .filter(|(_, e, _)| e.kind == EquipmentKind::PalletizingConveyor)
            .filter(|(_, e, _)| params.place_on.is_empty() || e.name == params.place_on)
            .filter(|(_, _, tf)| in_reach(tf.translation, 0.8))
            .min_by(|a, b| {
                a.2.translation
                    .distance_squared(robot_tf.translation)
                    .total_cmp(&b.2.translation.distance_squared(robot_tf.translation))
            });
        let tdp_state = palletizer.and_then(|(e, ..)| palletizers.get(e).ok());

        'logic: {
            if dt <= 0.0 {
                break 'logic;
            }
            if state.moving() {
                state.t += dt;
                let k = smooth(state.t / state.duration);
                state.pose = state.from.lerp(&state.to, k);
                break 'logic;
            }
            if state.wait > 0.0 {
                state.wait -= dt;
                break 'logic;
            }
            match state.step {
                Step::Idle => {
                    let Some((pick_entity, pick_eq, pick_tf, _)) = pick else {
                        state.status = "Sem mesa/esteira de coleta ao alcance".into();
                        break 'logic;
                    };
                    let Some((_, tdp_eq, _)) = palletizer else {
                        state.status = "Sem TDP de paletização ao alcance".into();
                        break 'logic;
                    };
                    if !io.get_bool("start") {
                        state.status = "Aguardando liberação (start)".into();
                        break 'logic;
                    }
                    let Some(tdp) = tdp_state.filter(|s| s.pallet.is_some()) else {
                        state.status = "Aguardando pallet em posição".into();
                        break 'logic;
                    };
                    if tdp.full {
                        state.status = "Pallet completo".into();
                        break 'logic;
                    }
                    if loads.get(tdp.pallet.expect("checked")).is_ok_and(|(_, l, _)| l.hold) {
                        state.status = "Aguardando colocador de papelão".into();
                        break 'logic;
                    }
                    // Autoriza_Pick: from the PLC, else from the mesa's internal logic.
                    let mesa_logic = mesas.get(pick_entity).ok().filter(|_| pick_eq.params.auto_logic);
                    let authorized = if io.from_plc.contains("Autoriza_Pick") {
                        io.get_bool("Autoriza_Pick")
                    } else if let Some(logic) = mesa_logic {
                        logic.coleta.autoriza_pick
                    } else {
                        io.get_bool("Autoriza_Pick")
                    };
                    if !authorized {
                        state.status = "Aguardando Autoriza_Pick da coleta".into();
                        break 'logic;
                    }
                    // Everything resting in the pick area is taken in one pick (a pair of fardos on a mesa).
                    let mut group: Vec<(Entity, Vec3, Quat, Vec3)> = loads
                        .iter()
                        .filter(|(_, l, tf)| {
                            l.kind == LoadKind::Box && l.state == LoadState::Free && sim::in_pick_area(pick_eq, pick_tf, l.size, tf)
                        })
                        .map(|(e, l, tf)| (e, tf.translation, tf.rotation, l.size))
                        .collect();
                    if !pick_eq.kind.is_mesa() {
                        group.truncate(1);
                    }
                    if group.is_empty() {
                        state.status = "Aguardando produto na coleta".into();
                        break 'logic;
                    }
                    let Ok((_, pallet, pallet_tf)) = loads.get(tdp.pallet.expect("checked")) else { break 'logic };
                    let (pallet_entity, pallet_size, pallet_tf) = (tdp.pallet.expect("checked"), pallet.size, *pallet_tf);

                    // Group frame = pick conveyor frame; offsets of each box from the group center.
                    let group_yaw = yaw_of(pick_tf.rotation);
                    let to_group = Quat::from_rotation_y(-group_yaw);
                    let (mut min, mut max) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
                    for (_, pos, rot, size) in &group {
                        let local = to_group * *pos;
                        let rel = to_group * *rot;
                        let hx = half_extent_along(rel, *size, Vec3::X);
                        let hz = half_extent_along(rel, *size, Vec3::Z);
                        min = min.min(Vec3::new(local.x - hx, 0.0, local.z - hz));
                        max = max.max(Vec3::new(local.x + hx, 0.0, local.z + hz));
                    }
                    let height = group.iter().map(|g| g.3.y).fold(0.0, f32::max);
                    let center_local = (min + max) / 2.0;
                    let center = Quat::from_rotation_y(group_yaw) * center_local;
                    let top = group.iter().map(|g| g.1.y + g.3.y / 2.0).fold(f32::MIN, f32::max);
                    let unit = Vec3::new(max.x - min.x, height, max.z - min.z);
                    state.held_plan = group
                        .iter()
                        .map(|(e, pos, rot, size)| HeldBox {
                            entity: *e,
                            offset: to_group * (*pos - Vec3::new(center.x, pos.y, center.z)),
                            rel_yaw: yaw_of(*rot) - group_yaw,
                            height: size.y,
                        })
                        .collect();
                    state.unit = unit;
                    state.group_yaw = group_yaw;
                    state.pick_top = Vec3::new(center.x, top, center.z);

                    // The pallet pattern keeps the footprint of its first placement.
                    let (units_done, unit) = pallet_loads
                        .get(pallet_entity)
                        .map_or((0, unit), |p| (p.units, p.unit_size.unwrap_or(unit)));
                    state.unit = unit;
                    let Some((slot_local, slot_yaw)) = sim::pallet_slot(&tdp_eq.params, pallet_size, unit, units_done) else {
                        state.status = "Pallet completo".into();
                        break 'logic;
                    };
                    // Stack on top of the cardboard sheets.
                    let sheets = pallet_loads.get(pallet_entity).map_or(0, |p| p.sheets);
                    let slot_local = slot_local + Vec3::Y * sheets as f32 * crate::paper::SHEET_THICKNESS;
                    let place_top = pallet_tf.transform_point(slot_local + Vec3::Y * unit.y / 2.0);
                    let local_pick = world_to_local.transform_point3(state.pick_top);
                    let local_place = world_to_local.transform_point3(place_top);
                    let pick_pose = Pose::from_local(local_pick, 0.0);
                    let place_pose = Pose::from_local(local_place, 0.0);
                    if !solve(&pick_pose).reachable || !solve(&place_pose).reachable {
                        state.status = "Coleta ou posição no pallet fora do alcance".into();
                        break 'logic;
                    }
                    state.safe_y = local_pick.y.max(local_place.y) + 0.35;
                    let grip_yaw = nearest_half_turn(state.pose.yaw, group_yaw - robot_yaw);
                    let above = Pose { y: state.safe_y, yaw: grip_yaw, ..pick_pose };
                    state.pick_conveyor = Some(pick_entity);
                    state.place = Some((pallet_entity, slot_local, slot_yaw));
                    state.status = "Indo para coleta".into();
                    state.step = Step::ToAbovePick;
                    state.move_to(above, params.speed_override, false);
                }
                Step::ToAbovePick => {
                    let local = world_to_local.transform_point3(state.pick_top);
                    let target = Pose { yaw: state.pose.yaw, ..Pose::from_local(local, 0.0) };
                    state.step = Step::DownPick;
                    state.status = "Descendo para coleta".into();
                    state.move_to(target, params.speed_override, true);
                }
                Step::DownPick => {
                    state.step = Step::Grip;
                    state.wait = 0.25;
                    state.collected = false;
                    state.status = "Fechando garra".into();
                }
                Step::Grip => {
                    let plan = std::mem::take(&mut state.held_plan);
                    let mut held = Vec::new();
                    for item in plan {
                        if let Ok((_, mut b, _)) = loads.get_mut(item.entity)
                            && b.state == LoadState::Free
                        {
                            b.state = LoadState::Held;
                            held.push(item);
                        }
                    }
                    if held.is_empty() {
                        state.step = Step::Idle;
                        break 'logic;
                    }
                    let gripper_world_yaw = state.pose.yaw + robot_yaw;
                    state.hold_yaw = state.group_yaw - gripper_world_yaw;
                    state.held = held;
                    state.step = Step::UpPick;
                    let up = Pose { y: state.safe_y, ..state.pose };
                    state.move_to(up, params.speed_override, true);
                }
                Step::UpPick | Step::ToAbovePlace => {
                    if state.step == Step::UpPick {
                        // Fardos lifted: Robo_Fardos_Coletados.
                        state.collected = true;
                        if let Some(mut logic) = state.pick_conveyor.and_then(|e| mesas.get_mut(e).ok()) {
                            logic.robot_collected = true;
                        }
                    }
                    let Some((pallet_entity, slot_local, slot_yaw)) = state.place else {
                        state.step = Step::Idle;
                        break 'logic;
                    };
                    let Ok((_, _, pallet_tf)) = loads.get(pallet_entity) else {
                        state.status = "Pallet saiu durante o ciclo".into();
                        state.step = Step::Idle;
                        break 'logic;
                    };
                    let pallet_tf = *pallet_tf;
                    let place_top = pallet_tf.transform_point(slot_local + Vec3::Y * state.unit.y / 2.0);
                    let local = world_to_local.transform_point3(place_top);
                    let unit_yaw = yaw_of(pallet_tf.rotation) + slot_yaw;
                    if state.step == Step::UpPick {
                        let yaw = nearest_half_turn(state.pose.yaw, unit_yaw - state.hold_yaw - robot_yaw);
                        let target = Pose { y: state.safe_y, yaw, ..Pose::from_local(local, 0.0) };
                        state.step = Step::ToAbovePlace;
                        state.status = "Transportando para o pallet".into();
                        state.move_to(target, params.speed_override, false);
                    } else {
                        let target = Pose { yaw: state.pose.yaw, ..Pose::from_local(local, 0.0) };
                        state.step = Step::DownPlace;
                        state.status = "Depositando".into();
                        state.move_to(target, params.speed_override, true);
                    }
                }
                Step::DownPlace => {
                    state.step = Step::Release;
                    state.wait = 0.25;
                    state.status = "Abrindo garra".into();
                }
                Step::Release => {
                    let held = std::mem::take(&mut state.held);
                    if let Some((pallet_entity, slot_local, slot_yaw)) = state.place {
                        // The fardos become part of the pallet load and travel with it.
                        let slot_rot = Quat::from_rotation_y(slot_yaw);
                        let bottom = slot_local.y - state.unit.y / 2.0;
                        let count = held.len() as u32;
                        for item in held {
                            if let Ok((_, mut b, mut box_tf)) = loads.get_mut(item.entity) {
                                b.state = LoadState::Placed(pallet_entity);
                                let offset = slot_rot * item.offset;
                                *box_tf = Transform::from_translation(Vec3::new(
                                    slot_local.x + offset.x,
                                    bottom + item.height / 2.0,
                                    slot_local.z + offset.z,
                                ))
                                .with_rotation(Quat::from_rotation_y(slot_yaw + item.rel_yaw));
                                commands.entity(item.entity).insert(ChildOf(pallet_entity));
                            }
                        }
                        if let Ok(mut pallet) = pallet_loads.get_mut(pallet_entity) {
                            pallet.placed += count;
                            pallet.units += 1;
                            pallet.unit_size.get_or_insert(state.unit);
                        }
                        state.cycles += 1;
                    }
                    state.step = Step::UpPlace;
                    let up = Pose { y: state.safe_y, ..state.pose };
                    state.move_to(up, params.speed_override, true);
                }
                Step::UpPlace => {
                    state.step = Step::Idle;
                    state.status = "Ciclo concluído".into();
                }
            }
        }

        // Carried fardos follow the TCP.
        if !state.held.is_empty() {
            let group_yaw = robot_yaw + state.pose.yaw + state.hold_yaw;
            let rot = Quat::from_rotation_y(group_yaw);
            let tcp = robot_tf.transform_point(state.pose.local_point());
            for item in &state.held {
                if let Ok((_, _, mut box_tf)) = loads.get_mut(item.entity) {
                    box_tf.translation = tcp - Vec3::Y * item.height / 2.0 + rot * item.offset;
                    box_tf.rotation = Quat::from_rotation_y(group_yaw + item.rel_yaw);
                }
            }
        }

        // Pinças close while gripping and stay closed while carrying.
        let closed = !state.held.is_empty() || state.step == Step::Grip;
        let step = dt / 0.25;
        state.jaw += ((if closed { 1.0 } else { 0.0 }) - state.jaw).clamp(-step, step);

        io.set_bool("busy", state.step != Step::Idle);
        io.set_bool("Robo_Fardos_Coletados", state.collected);
        io.set_bool("pallet_full", tdp_state.is_some_and(|p| p.full));
        io.set("count", tdp_state.map_or(0.0, |p| p.placed as f64));
    }
}

fn half_extent_along(rot: Quat, size: Vec3, dir: Vec3) -> f32 {
    let x = rot * Vec3::X;
    let z = rot * Vec3::Z;
    x.dot(dir).abs() * size.x / 2.0 + z.dot(dir).abs() * size.z / 2.0
}

pub fn animate_robots(
    robots: Query<&RobotState>,
    mut parts: Query<(&RobotPart, &ChildOf, &mut Transform), Without<CadPart>>,
    mut cad: Query<(&CadPart, &ChildOf, &mut Transform), Without<RobotPart>>,
) {
    for (part, child_of, mut tf) in &mut cad {
        let Ok(state) = robots.get(child_of.parent()) else { continue };
        let joints = solve(&state.pose);
        *tf = part.transform(&joints, state.pose.yaw - joints.j1);
    }
    for (part, child_of, mut tf) in &mut parts {
        let Ok(state) = robots.get(child_of.parent()) else { continue };
        let pose = state.pose;
        let joints = solve(&pose);
        let j1 = joints.j1;
        let u = joints.upper_dir;
        let upper_angle = u.y.atan2(u.x);
        *tf = match part {
            RobotPart::Turntable => Transform::from_rotation(Quat::from_rotation_y(j1)),
            RobotPart::LowerArm => Transform {
                translation: plane_to_local(j1, joints.elbow / 2.0),
                rotation: link_rotation(j1, joints.shoulder),
                ..default()
            },
            RobotPart::UpperArm => Transform {
                translation: plane_to_local(j1, joints.elbow + u * (L2 - UPPER_BACK) / 2.0),
                rotation: link_rotation(j1, upper_angle),
                ..default()
            },
            RobotPart::Rod => Transform {
                translation: plane_to_local(j1, joints.elbow / 2.0 - u * 0.35),
                rotation: link_rotation(j1, joints.shoulder),
                ..default()
            },
            RobotPart::Wrist => Transform {
                translation: plane_to_local(j1, joints.elbow + u * L2 + Vec2::X * WRIST_FWD),
                rotation: Quat::from_rotation_y(pose.yaw),
                ..default()
            },
            RobotPart::Gripper | RobotPart::JawA | RobotPart::JawB => {
                let gripper = Transform {
                    translation: plane_to_local(j1, joints.elbow + u * L2 + Vec2::X * WRIST_FWD) - Vec3::Y * TCP_DROP,
                    rotation: Quat::from_rotation_y(pose.yaw),
                    ..default()
                };
                // Clamps close onto the picked group (its width along the gripper Z).
                let closed = state.unit.z / 2.0 + 0.008;
                let half = JAW_OPEN + (closed.min(JAW_OPEN) - JAW_OPEN) * state.jaw;
                match part {
                    RobotPart::JawA => gripper * Transform::from_xyz(0.0, -JAW_HEIGHT / 2.0, -half),
                    RobotPart::JawB => gripper * Transform::from_xyz(0.0, -JAW_HEIGHT / 2.0, half),
                    _ => gripper,
                }
            }
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ik_reaches_target() {
        let pose = Pose { theta: 0.7, r: 2.2, y: 1.0, yaw: 0.0 };
        let j = solve(&pose);
        assert!(j.reachable);
        let wrist = plane_to_local(j.j1, j.elbow + j.upper_dir * L2 + Vec2::X * WRIST_FWD);
        let tcp = wrist - Vec3::Y * TCP_DROP;
        assert!((tcp - pose.local_point()).length() < 1e-3, "{tcp:?} vs {:?}", pose.local_point());
    }

    #[test]
    fn cad_rest_pose_is_identity() {
        // Pose whose IK gives the STEP rest pose: lower arm vertical, upper arm horizontal.
        let tcp_plane = rest::WRIST + Vec2::new(WRIST_FWD, -TCP_DROP);
        let pose = Pose { theta: 0.0, r: tcp_plane.x, y: tcp_plane.y, yaw: 0.0 };
        let joints = solve(&pose);
        assert!(joints.reachable);
        for (part, _) in CadPart::ALL {
            let tf = part.transform(&joints, 0.0);
            assert!(tf.translation.length() < 1e-3, "{part:?} {:?}", tf.translation);
            assert!(tf.rotation.angle_between(Quat::IDENTITY) < 1e-3, "{part:?}");
        }
    }

    #[test]
    fn half_turn_choice() {
        assert!((nearest_half_turn(3.0, 0.1) - (0.1 + PI)).abs() < 1e-5);
        assert!((nearest_half_turn(0.0, 0.1) - 0.1).abs() < 1e-5);
    }
}
