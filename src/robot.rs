//! FANUC M-410iC/185: 4-axis palletizing robot (J1 base rotation, J2/J3 arm
//! with parallelograms keeping the wrist level, J4 flange yaw).
//! Dimensions measured on the manufacturer's STEP model (robot on its pedestal);
//! axis ranges, speeds and the operating space follow the FANUC data sheet (see
//! [`spec`]). Moves are joint (J) or linear (L) with a trapezoidal profile, like
//! the controller. The program below plays the role of the Selgron TP routines
//! (PICK, PLACE, MOVE_PERCH).

use std::collections::HashMap;
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
/// Maximum reach of the flange center (data sheet), used to find conveyors in reach.
pub const REACH: f32 = spec::REACH;

/// FANUC data sheet, M-410iC/185 on the pedestal type base.
pub mod spec {
    const DEG: f32 = std::f32::consts::PI / 180.0;
    pub const PAYLOAD_KG: f32 = 185.0;
    pub const MASS_KG: f32 = 1600.0;
    pub const REPEATABILITY_MM: f32 = 0.05;
    /// Operating space of the wrist face plate center: reach, top and bottom
    /// (from the floor), inner radius at floor level and rear interference radius.
    pub const REACH: f32 = 3.143;
    pub const MAX_HEIGHT: f32 = 2.397;
    pub const MIN_HEIGHT: f32 = -0.561;
    pub const MIN_RADIUS: f32 = 1.100;
    pub const REAR_RADIUS: f32 = 0.725;
    /// Maximum speeds J1..J4 (rad/s): 140, 140, 140, 305 °/s.
    pub const SPEED: [f32; 4] = [140.0 * DEG, 140.0 * DEG, 140.0 * DEG, 305.0 * DEG];
    /// Motion ranges J1..J4 (rad), totals 360°, 144°, 136°, 720°. J2 is the lower
    /// arm from the vertical (forward positive), J3 the upper arm from the
    /// horizontal (kept by the parallelogram), J4 the flange relative to J1. The
    /// J2/J3 split reproduces the data sheet envelope: top 2397 mm, bottom -561 mm
    /// and 1100 mm inner radius at the floor.
    pub const MIN: [f32; 4] = [-180.0 * DEG, -44.0 * DEG, -126.0 * DEG, -360.0 * DEG];
    pub const MAX: [f32; 4] = [180.0 * DEG, 100.0 * DEG, 10.0 * DEG, 360.0 * DEG];
    /// Acceleration time of the trapezoidal speed profile (ACC 100).
    pub const ACC_TIME: f32 = 0.35;
}

/// Axis positions J1..J4 (rad), see [`spec::MIN`] for their conventions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Axes(pub [f32; 4]);

impl Axes {
    pub fn within_limits(&self) -> bool {
        (0..4).all(|i| self.0[i] >= spec::MIN[i] - 1e-4 && self.0[i] <= spec::MAX[i] + 1e-4)
    }

    /// Forward kinematics: the TCP pose for these axes.
    pub fn pose(&self) -> Pose {
        let [j1, j2, j3, j4] = self.0;
        let shoulder = FRAC_PI_2 - j2;
        let wrist = Vec2::new(shoulder.cos(), shoulder.sin()) * L1 + Vec2::new(j3.cos(), j3.sin()) * L2;
        Pose {
            theta: j1,
            r: J2_OFFSET + wrist.x + WRIST_FWD,
            y: BASE_HEIGHT + wrist.y - TCP_DROP,
            yaw: j4 + j1,
        }
    }
}

/// Outline of the operating space of the flange center in the arm plane:
/// (radius from J1, height above the floor), around the J2/J3 limit rectangle.
pub fn envelope_outline(steps: usize) -> Vec<Vec2> {
    let (lo, hi) = (spec::MIN, spec::MAX);
    let lerp = |a: f32, b: f32, k: usize| a + (b - a) * k as f32 / steps as f32;
    let mut axes = Vec::new();
    for k in 0..=steps {
        axes.push((lerp(lo[1], hi[1], k), hi[2]));
    }
    for k in 0..=steps {
        axes.push((hi[1], lerp(hi[2], lo[2], k)));
    }
    for k in 0..=steps {
        axes.push((lerp(hi[1], lo[1], k), lo[2]));
    }
    for k in 0..=steps {
        axes.push((lo[1], lerp(lo[2], hi[2], k)));
    }
    axes.into_iter()
        .map(|(j2, j3)| {
            let p = Axes([0.0, j2, j3, 0.0]).pose();
            Vec2::new(p.r, p.y + TOOL_LENGTH)
        })
        .collect()
}

/// Kind of move, as in the TP routines: J (joint interpolated, % of the axis
/// speeds) or L (straight line at a TCP speed in m/s).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Move {
    Joint,
    Linear(f32),
}

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

    /// Axis positions of this pose (J1 within ±180°, never wrapped).
    pub fn axes(&self) -> Axes {
        let joints = solve(self);
        Axes([
            self.theta,
            FRAC_PI_2 - joints.shoulder,
            joints.upper_dir.y.atan2(joints.upper_dir.x),
            self.yaw - self.theta,
        ])
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
    from_axes: Axes,
    to_axes: Axes,
    motion: Move,
    t: f32,
    duration: f32,
    /// Acceleration time of the current move.
    ramp: f32,
    /// Axis speeds of the last frame (rad/s), for the property panel.
    pub axis_speed: [f32; 4],
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
    /// Last pick conveyor served (mesas take turns).
    last_pick: Option<Entity>,
    /// TDP of the current (or last) placement.
    pub tdp: Option<Entity>,
    place: Option<(Entity, Vec3, f32)>,
    safe_y: f32,
    /// Robo_Fardos_Coletados.
    pub collected: bool,
    /// Waiting at the perch (MOVE_PERCH) over the next coleta.
    pub perched: bool,
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
            from_axes: Pose::HOME.axes(),
            to_axes: Pose::HOME.axes(),
            motion: Move::Joint,
            t: 0.0,
            duration: 0.0,
            ramp: 0.0,
            axis_speed: [0.0; 4],
            step: Step::Idle,
            wait: 0.0,
            held: Vec::new(),
            held_plan: Vec::new(),
            hold_yaw: 0.0,
            group_yaw: 0.0,
            unit: Vec3::ONE,
            pick_top: Vec3::ZERO,
            pick_conveyor: None,
            last_pick: None,
            tdp: None,
            place: None,
            safe_y: Pose::HOME.y,
            collected: false,
            perched: false,
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

    /// Starts a move. Its duration is set by the slowest axis at its maximum speed
    /// (and, for L moves, by the TCP speed), times the override (OVERRIDE=R[104]),
    /// plus the acceleration ramps.
    fn move_to(&mut self, to: Pose, speed_override: f32, motion: Move) {
        let ovr = speed_override.clamp(0.05, 1.0);
        let (a, b) = (self.pose.axes(), to.axes());
        let axes_time = (0..4).map(|i| (b.0[i] - a.0[i]).abs() / (spec::SPEED[i] * ovr)).fold(0.0, f32::max);
        let travel = match motion {
            Move::Joint => axes_time,
            // On a line the axes do not all move at a constant rate: keep some margin.
            Move::Linear(v) => ((to.local_point() - self.pose.local_point()).length() / (v * ovr)).max(axes_time * 1.15),
        };
        // Trapezoid: cruise time + one ramp; short moves never reach full speed.
        let ramp = spec::ACC_TIME;
        let duration = if travel >= ramp { travel + ramp } else { 2.0 * (travel * ramp).sqrt() };
        self.from = self.pose;
        self.to = to;
        self.from_axes = a;
        self.to_axes = b;
        self.motion = motion;
        self.t = 0.0;
        self.duration = duration.max(0.05);
        self.ramp = ramp.min(self.duration / 2.0);
    }

    /// Pose along the current move at time `t`.
    fn pose_at(&self, t: f32) -> Pose {
        let k = trapezoid(t, self.duration, self.ramp);
        match self.motion {
            Move::Joint => {
                let mut q = [0.0; 4];
                for (i, v) in q.iter_mut().enumerate() {
                    *v = self.from_axes.0[i] + (self.to_axes.0[i] - self.from_axes.0[i]) * k;
                }
                Axes(q).pose()
            }
            Move::Linear(_) => {
                let p = self.from.local_point().lerp(self.to.local_point(), k);
                Pose::from_local(p, self.from.yaw + (self.to.yaw - self.from.yaw) * k)
            }
        }
    }
}

/// Fraction of the path covered at `t` with a trapezoidal speed profile.
fn trapezoid(t: f32, total: f32, ramp: f32) -> f32 {
    if total <= 0.0 {
        return 1.0;
    }
    let t = t.clamp(0.0, total);
    let cruise = total - ramp; // peak speed = 1 / cruise
    let s = if t < ramp {
        t * t / (2.0 * ramp)
    } else if t <= total - ramp {
        ramp / 2.0 + (t - ramp)
    } else {
        let left = total - t;
        cruise - left * left / (2.0 * ramp)
    };
    (s / cruise).clamp(0.0, 1.0)
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
    let in_plane = d <= L1 + L2 - 1e-3 && d >= (L1 - L2).abs() + 1e-3 && pose.y >= -0.05;
    let dc = d.clamp((L1 - L2).abs() + 1e-3, L1 + L2 - 1e-3);
    let cos_inner = ((L1 * L1 + dc * dc - L2 * L2) / (2.0 * L1 * dc)).clamp(-1.0, 1.0);
    let shoulder = h.atan2(r) + cos_inner.acos();
    let elbow = Vec2::new(L1 * shoulder.cos(), L1 * shoulder.sin());
    let wrist = if in_plane {
        target
    } else {
        elbow + (target - elbow).normalize_or_zero() * L2
    };
    let upper_dir = (wrist - elbow).normalize_or_zero();
    // Axis limits of the data sheet (J2 from the vertical, J3 from the horizontal).
    let j2 = FRAC_PI_2 - shoulder;
    let j3 = upper_dir.y.atan2(upper_dir.x);
    let axes = Axes([pose.theta, j2, j3, pose.yaw - pose.theta]);
    Joints {
        j1: pose.theta,
        shoulder,
        elbow,
        upper_dir,
        reachable: in_plane && axes.within_limits(),
    }
}

fn plane_to_local(j1: f32, p: Vec2) -> Vec3 {
    let radial = J2_OFFSET + p.x;
    Vec3::new(radial * j1.cos(), BASE_HEIGHT + p.y, -radial * j1.sin())
}

fn link_rotation(j1: f32, plane_angle: f32) -> Quat {
    Quat::from_rotation_y(j1) * Quat::from_rotation_z(plane_angle - FRAC_PI_2)
}

/// Picks `target + k*PI` closest to `current` (boxes are symmetric under 180°).
fn nearest_half_turn(current: f32, target: f32) -> f32 {
    let k = ((current - target) / PI).round();
    target + k * PI
}

/// Gripper yaw for `target` (any half turn) closest to `current`, keeping J4
/// (yaw relative to J1 at `theta`) inside its ±360° range.
fn wrist_yaw(current: f32, target: f32, theta: f32) -> f32 {
    (-8..=8)
        .map(|k| target + k as f32 * PI)
        .filter(|y| (y - theta).abs() <= spec::MAX[3])
        .min_by(|a, b| (a - current).abs().total_cmp(&(b - current).abs()))
        .unwrap_or_else(|| nearest_half_turn(current, target))
}

pub fn yaw_of(rotation: Quat) -> f32 {
    rotation.to_euler(EulerRot::YXZ).0
}

/// Equipment robots are working on, so robots sharing mesas or TDPs take turns:
/// a pick conveyor is reserved from planning until the fardos are lifted, a TDP
/// until the placement is done.
#[derive(Default)]
pub struct Claims {
    owner: HashMap<Entity, Entity>,
    /// Robot waiting for equipment another robot holds: it is next in line.
    waiting: HashMap<Entity, Entity>,
}

impl Claims {
    fn available(&self, target: Entity, robot: Entity) -> bool {
        self.owner.get(&target).is_none_or(|r| *r == robot) && self.waiting.get(&target).is_none_or(|r| *r == robot)
    }
}

/// One planned pick-and-place.
struct Plan {
    held: Vec<HeldBox>,
    unit: Vec3,
    group_yaw: f32,
    pick_top: Vec3,
    place: (Entity, Vec3, f32),
    safe_y: f32,
    above: Pose,
}

type LoadQuery<'w, 's> = Query<'w, 's, (Entity, &'static mut Load, &'static mut Transform), Without<Equipment>>;

/// Plans taking everything resting in the pick area of `pick` (a pair of fardos
/// on a mesa, one box on a belt) to the next slot of the pallet on `tdp`.
#[allow(clippy::too_many_arguments)]
fn plan_cycle(
    state: &RobotState,
    robot_tf: &Transform,
    world_to_local: &Affine3A,
    (pick_eq, pick_tf): (&Equipment, &Transform),
    (tdp_eq, pallet_entity): (&Equipment, Entity),
    loads: &LoadQuery,
    pallet_loads: &Query<&mut PalletLoad>,
) -> Result<Plan, &'static str> {
    let robot_yaw = yaw_of(robot_tf.rotation);
    let mut group: Vec<(Entity, Vec3, Quat, Vec3)> = loads
        .iter()
        .filter(|(_, l, tf)| l.kind == LoadKind::Box && l.state == LoadState::Free && sim::in_pick_area(pick_eq, pick_tf, l.size, tf))
        .map(|(e, l, tf)| (e, tf.translation, tf.rotation, l.size))
        .collect();
    if !pick_eq.kind.is_mesa() {
        group.truncate(1);
    }
    if group.is_empty() {
        return Err("Aguardando produto na coleta");
    }
    let Ok((_, pallet, pallet_tf)) = loads.get(pallet_entity) else {
        return Err("Aguardando pallet em posição");
    };
    let (pallet_size, pallet_tf) = (pallet.size, *pallet_tf);

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
    let center = Quat::from_rotation_y(group_yaw) * ((min + max) / 2.0);
    let top = group.iter().map(|g| g.1.y + g.3.y / 2.0).fold(f32::MIN, f32::max);
    let held = group
        .iter()
        .map(|(e, pos, rot, size)| HeldBox {
            entity: *e,
            offset: to_group * (*pos - Vec3::new(center.x, pos.y, center.z)),
            rel_yaw: yaw_of(*rot) - group_yaw,
            height: size.y,
        })
        .collect();
    let pick_top = Vec3::new(center.x, top, center.z);

    // The pallet pattern keeps the footprint of its first placement.
    let unit = Vec3::new(max.x - min.x, height, max.z - min.z);
    let content = pallet_loads.get(pallet_entity).ok();
    let units_done = content.map_or(0, |p| p.units);
    let unit = content.and_then(|p| p.unit_size).unwrap_or(unit);
    let Some((slot_local, slot_yaw)) = sim::pallet_slot(&tdp_eq.params, pallet_size, unit, units_done) else {
        return Err("Pallet completo");
    };
    // Stack on top of the cardboard sheets.
    let sheets = content.map_or(0, |p| p.sheets);
    let slot_local = slot_local + Vec3::Y * sheets as f32 * crate::paper::SHEET_THICKNESS;
    let place_top = pallet_tf.transform_point(slot_local + Vec3::Y * unit.y / 2.0);
    let local_pick = world_to_local.transform_point3(pick_top);
    let local_place = world_to_local.transform_point3(place_top);
    let pick_pose = Pose::from_local(local_pick, 0.0);
    let place_pose = Pose::from_local(local_place, 0.0);
    if !solve(&pick_pose).reachable || !solve(&place_pose).reachable {
        return Err("Coleta ou posição no pallet fora do alcance");
    }
    let safe_y = local_pick.y.max(local_place.y) + 0.35;
    let grip_yaw = wrist_yaw(state.pose.yaw, group_yaw - robot_yaw, pick_pose.theta);
    Ok(Plan {
        held,
        unit,
        group_yaw,
        pick_top,
        place: (pallet_entity, slot_local, slot_yaw),
        safe_y,
        above: Pose { y: safe_y, yaw: grip_yaw, ..pick_pose },
    })
}

/// Perch (wait) pose of MOVE_PERCH: over the middle of the pick area, the
/// gripper aligned with the conveyor, high enough to clear the incoming fardos.
fn perch_pose(state: &RobotState, world_to_local: &Affine3A, robot_yaw: f32, pick_eq: &Equipment, pick_tf: &Transform) -> Option<Pose> {
    let p = &pick_eq.params;
    let end = p.length / 2.0;
    let x = crate::conveyor::layout(pick_eq).pick_x0.map_or(end - 0.3, |x0| (x0 + end) / 2.0);
    let world = pick_tf.transform_point(Vec3::new(x, p.height + p.box_size[1] + 0.35, 0.0));
    let local = world_to_local.transform_point3(world);
    let theta = Pose::from_local(local, 0.0).theta;
    let yaw = wrist_yaw(state.pose.yaw, yaw_of(pick_tf.rotation) - robot_yaw, theta);
    let pose = Pose::from_local(local, yaw);
    solve(&pose).reachable.then_some(pose)
}

/// Listed names first, in list order, then the rest by name.
fn list_order(names: &[String], name: &str) -> (usize, String) {
    (names.iter().position(|n| n == name).unwrap_or(usize::MAX), name.to_owned())
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn robot_program(
    time: Res<Time>,
    control: Res<crate::model::SimControl>,
    mut commands: Commands,
    mut claims: Local<Claims>,
    mut robots: Query<(Entity, &Equipment, &Transform, &mut RobotState, &mut Io), Without<Load>>,
    equipment: Query<(Entity, &Equipment, &Transform), (Without<Load>, Without<RobotState>)>,
    palletizers: Query<&PalletizerState>,
    mut mesas: Query<&mut MesaLogic>,
    mut loads: LoadQuery,
    mut pallet_loads: Query<&mut PalletLoad>,
) {
    let dt = if control.running { time.delta_secs() * control.time_scale } else { 0.0 };
    // Forget robots that were deleted.
    let alive: Vec<Entity> = robots.iter().map(|r| r.0).collect();
    claims.owner.retain(|_, r| alive.contains(r));
    claims.waiting.retain(|_, r| alive.contains(r));
    for (robot_entity, robot_eq, robot_tf, mut state, mut io) in &mut robots {
        let params = &robot_eq.params;
        let world_to_local = robot_tf.compute_affine().inverse();
        let robot_yaw = yaw_of(robot_tf.rotation);
        let in_reach = |p: Vec3, margin: f32| {
            let local = world_to_local.transform_point3(p);
            Vec2::new(local.x, local.z).length() <= REACH + margin
        };

        // Pick conveyors (mesa coletas, belt ends) and palletizing TDPs in reach: the
        // listed ones, or every one. Mesas take turns (round robin); the TDP with the
        // most advanced pallet comes first, so pallets are completed one at a time.
        let mut picks: Vec<(Entity, &Equipment, &Transform)> = equipment
            .iter()
            .filter(|(_, e, _)| e.kind.is_product_conveyor())
            .filter(|(_, e, _)| params.pick_from.is_empty() || params.pick_from.contains(&e.name))
            .filter(|(_, e, tf)| in_reach(tf.transform_point(Vec3::new(e.params.length / 2.0 - 0.3, e.params.height, 0.0)), 0.0))
            .collect();
        picks.sort_by_cached_key(|(_, e, _)| list_order(&params.pick_from, &e.name));
        if let Some(i) = picks.iter().position(|p| Some(p.0) == state.last_pick) {
            picks.rotate_left(i + 1);
        }
        let mut tdps: Vec<(Entity, &Equipment, Option<&PalletizerState>)> = equipment
            .iter()
            .filter(|(_, e, _)| e.kind == EquipmentKind::PalletizingConveyor)
            .filter(|(_, e, _)| params.place_on.is_empty() || params.place_on.contains(&e.name))
            .filter(|(_, _, tf)| in_reach(tf.translation, 0.8))
            .map(|(entity, e, _)| (entity, e, palletizers.get(entity).ok()))
            .collect();
        tdps.sort_by_cached_key(|(_, e, _)| list_order(&params.place_on, &e.name));
        tdps.sort_by_key(|(.., s)| std::cmp::Reverse(s.filter(|s| s.pallet.is_some()).map_or(0, |s| s.units)));
        let tdp_state = state.tdp.or(tdps.first().map(|t| t.0)).and_then(|e| palletizers.get(e).ok());

        'logic: {
            if dt <= 0.0 {
                break 'logic;
            }
            if state.moving() {
                let before = state.pose.axes();
                state.t += dt;
                state.pose = state.pose_at(state.t);
                let after = state.pose.axes();
                state.axis_speed = std::array::from_fn(|i| (after.0[i] - before.0[i]) / dt);
                break 'logic;
            }
            state.axis_speed = [0.0; 4];
            if state.wait > 0.0 {
                state.wait -= dt;
                break 'logic;
            }
            match state.step {
                Step::Idle => {
                    // Release what this robot used in the last cycle.
                    claims.owner.retain(|_, r| *r != robot_entity);
                    claims.waiting.retain(|_, r| *r != robot_entity);
                    if picks.is_empty() {
                        state.status = "Sem mesa/esteira de coleta ao alcance".into();
                        break 'logic;
                    }
                    if tdps.is_empty() {
                        state.status = "Sem TDP de paletização ao alcance".into();
                        break 'logic;
                    }
                    if !io.get_bool("start") {
                        state.status = "Aguardando liberação (start)".into();
                        break 'logic;
                    }
                    let mut status: Option<&'static str> = None;
                    let mut wait_for = Vec::new();
                    let mut chosen = None;
                    'search: for &(tdp_entity, tdp_eq, tdp) in &tdps {
                        let Some(pallet_entity) = tdp.and_then(|t| t.pallet) else {
                            status.get_or_insert("Aguardando pallet em posição");
                            continue;
                        };
                        if tdp.is_some_and(|t| t.full) {
                            status.get_or_insert("Pallet completo");
                            continue;
                        }
                        if loads.get(pallet_entity).is_ok_and(|(_, l, _)| l.hold) {
                            status.get_or_insert("Aguardando colocador de papelão");
                            continue;
                        }
                        if !claims.available(tdp_entity, robot_entity) {
                            wait_for.push(tdp_entity);
                            status.get_or_insert("TDP em uso por outro robô");
                            continue;
                        }
                        for &(pick_entity, pick_eq, pick_tf) in &picks {
                            if !claims.available(pick_entity, robot_entity) {
                                wait_for.push(pick_entity);
                                status.get_or_insert("Coleta em uso por outro robô");
                                continue;
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
                                status.get_or_insert("Aguardando Autoriza_Pick da coleta");
                                continue;
                            }
                            match plan_cycle(
                                &state,
                                robot_tf,
                                &world_to_local,
                                (pick_eq, pick_tf),
                                (tdp_eq, pallet_entity),
                                &loads,
                                &pallet_loads,
                            ) {
                                Ok(plan) => {
                                    chosen = Some((pick_entity, tdp_entity, plan));
                                    break 'search;
                                }
                                Err(reason) => {
                                    status.get_or_insert(reason);
                                }
                            }
                        }
                    }
                    let Some((pick_entity, tdp_entity, plan)) = chosen else {
                        // Next in line for what another robot is using.
                        for target in wait_for {
                            claims.waiting.entry(target).or_insert(robot_entity);
                        }
                        state.status = status.unwrap_or("Aguardando").into();
                        // MOVE_PERCH: wait over the coleta of the next mesa, at the infeed height.
                        if !state.perched
                            && let Some(&(_, pick_eq, pick_tf)) = picks.first()
                            && let Some(perch) = perch_pose(&state, &world_to_local, robot_yaw, pick_eq, pick_tf)
                        {
                            state.perched = true;
                            state.status = format!("Perch: {}", state.status);
                            state.move_to(perch, params.speed_override, Move::Joint);
                        }
                        break 'logic;
                    };
                    state.perched = false;
                    if std::env::var("PDT_TRACE").is_ok() {
                        let name = |e: Entity| equipment.get(e).map_or("?", |(_, eq, _)| eq.name.as_str());
                        eprintln!(
                            "[{:.2}] {}: coleta {} -> {}",
                            time.elapsed_secs(),
                            robot_eq.name,
                            name(pick_entity),
                            name(tdp_entity)
                        );
                    }
                    claims.owner.insert(pick_entity, robot_entity);
                    claims.owner.insert(tdp_entity, robot_entity);
                    state.held_plan = plan.held;
                    state.unit = plan.unit;
                    state.group_yaw = plan.group_yaw;
                    state.pick_top = plan.pick_top;
                    state.safe_y = plan.safe_y;
                    state.pick_conveyor = Some(pick_entity);
                    state.last_pick = Some(pick_entity);
                    state.tdp = Some(tdp_entity);
                    state.place = Some(plan.place);
                    state.status = "Indo para coleta".into();
                    state.step = Step::ToAbovePick;
                    state.move_to(plan.above, params.speed_override, Move::Joint);
                }
                Step::ToAbovePick => {
                    let local = world_to_local.transform_point3(state.pick_top);
                    let target = Pose { yaw: state.pose.yaw, ..Pose::from_local(local, 0.0) };
                    state.step = Step::DownPick;
                    state.status = "Descendo para coleta".into();
                    state.move_to(target, params.speed_override, Move::Linear(3.0)); // PICK: L PR[3] 3000mm/sec
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
                    state.move_to(up, params.speed_override, Move::Linear(1.8)); // PICK: L PR[11] 1800mm/sec
                }
                Step::UpPick | Step::ToAbovePlace => {
                    if state.step == Step::UpPick {
                        // Fardos lifted: Robo_Fardos_Coletados.
                        state.collected = true;
                        if let Some(mut logic) = state.pick_conveyor.and_then(|e| mesas.get_mut(e).ok()) {
                            logic.robot_collected = true;
                        }
                        // The coleta is free for the other robots.
                        if let Some(pick) = state.pick_conveyor
                            && claims.owner.get(&pick) == Some(&robot_entity)
                        {
                            claims.owner.remove(&pick);
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
                        let above = Pose::from_local(local, 0.0);
                        let yaw = wrist_yaw(state.pose.yaw, unit_yaw - state.hold_yaw - robot_yaw, above.theta);
                        let target = Pose { y: state.safe_y, yaw, ..above };
                        state.step = Step::ToAbovePlace;
                        state.status = "Transportando para o pallet".into();
                        state.move_to(target, params.speed_override, Move::Joint); // PLACE: J PR[11] 100%
                    } else {
                        let target = Pose { yaw: state.pose.yaw, ..Pose::from_local(local, 0.0) };
                        state.step = Step::DownPlace;
                        state.status = "Depositando".into();
                        state.move_to(target, params.speed_override, Move::Linear(2.0)); // PLACE: L PR[5] 2000mm/sec
                    }
                }
                Step::DownPlace => {
                    state.step = Step::Release;
                    state.wait = 0.25;
                    state.status = "Abrindo garra".into();
                }
                Step::Release => {
                    if std::env::var("PDT_TRACE").is_ok() {
                        eprintln!("[{:.2}] {}: depositado", time.elapsed_secs(), robot_eq.name);
                    }
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
                    state.move_to(up, params.speed_override, Move::Linear(4.0)); // PLACE: L PR[11] 4000mm/sec
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
    fn envelope_matches_data_sheet() {
        // Top of the operating space: J2 vertical, J3 at its upper limit.
        let top = Axes([0.0, 0.0, spec::MAX[2], 0.0]).pose().y + TOOL_LENGTH;
        assert!((top - spec::MAX_HEIGHT).abs() < 0.01, "{top}");
        // Inner radius where the envelope crosses the floor, and its bottom.
        let outline = envelope_outline(400);
        let inner = outline
            .windows(2)
            .filter(|w| (w[0].y >= 0.0) != (w[1].y >= 0.0))
            .map(|w| w[0].x)
            .fold(f32::MAX, f32::min);
        assert!((inner - spec::MIN_RADIUS).abs() < 0.03, "{inner}");
        let bottom = outline.iter().map(|p| p.y).fold(f32::MAX, f32::min);
        assert!((bottom - spec::MIN_HEIGHT).abs() < 0.03, "{bottom}");
    }

    #[test]
    fn axes_round_trip() {
        let pose = Pose { theta: -1.2, r: 2.4, y: 0.8, yaw: 0.5 };
        let back = pose.axes().pose();
        assert!((back.local_point() - pose.local_point()).length() < 1e-3);
        assert!((back.yaw - pose.yaw).abs() < 1e-5);
    }

    #[test]
    fn trapezoid_profile() {
        assert_eq!(trapezoid(0.0, 2.0, 0.35), 0.0);
        assert!((trapezoid(2.0, 2.0, 0.35) - 1.0).abs() < 1e-6);
        assert!((trapezoid(1.0, 2.0, 0.35) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn half_turn_choice() {
        assert!((nearest_half_turn(3.0, 0.1) - (0.1 + PI)).abs() < 1e-5);
        assert!((nearest_half_turn(0.0, 0.1) - 0.1).abs() < 1e-5);
    }
}
