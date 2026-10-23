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

use crate::model::{Equipment, EquipmentKind, Io, RobotModel};
use crate::mesa_logic::MesaLogic;
use crate::sim::{self, Load, LoadKind, LoadState, PalletLoad, PalletizerState};

/// Cabeçote Selgron "4 fardos" (0323P01-010000): flange coupling to the bottom of
/// the frame, where the fardos' top rests (TCP).
pub const TOOL_LENGTH: f32 = 0.435;
/// Clamp (pinça) half opening when open, and its height below the frame.
pub const JAW_OPEN: f32 = 0.60;
/// Clamp opening while releasing in the layer: just enough to let the fardo go,
/// without going into the fardos beside it; it opens fully once lifted.
pub const JAW_RELEASE: f32 = 0.025;
/// Clamp plate thickness and its gap to the fardo when closed.
pub const JAW_THICKNESS: f32 = 0.012;
pub const JAW_GAP: f32 = 0.008;
pub const JAW_HEIGHT: f32 = 0.295;

const DEG: f32 = PI / 180.0;

/// Kinematics and data sheet of a palletizing robot model. Both models move as a
/// 4-axis palletizer: base rotation, lower arm, upper arm with the flange kept level
/// (FANUC by parallelograms, KUKA by A5) and flange yaw. Internally the arm axes
/// are the lower arm from the vertical (forward positive) and the upper arm
/// elevation from the horizontal; [`Kin::to_motor`] gives the manufacturer's axes,
/// where the data sheet ranges and speeds apply.
#[derive(Clone)]
pub struct Kin {
    pub model: RobotModel,
    /// Axis names J1..J4 (FANUC) or A1, A2, A3, A6 (KUKA; A5 keeps the flange level).
    pub axis_names: [&'static str; 4],
    /// J2 axis height above the floor and ahead of the J1 axis.
    pub base_height: f32,
    pub j2_offset: f32,
    /// J2 -> J3 and J3 -> wrist pivot.
    pub l1: f32,
    pub l2: f32,
    /// Parametric model: upper arm length behind J3.
    pub upper_back: f32,
    /// Flange center ahead of and flange face below the wrist pivot.
    pub wrist_fwd: f32,
    pub wrist_drop: f32,
    pub payload_kg: f32,
    pub mass_kg: f32,
    pub repeatability_mm: f32,
    /// Operating space of the flange center: reach, top and bottom (from the floor),
    /// inner radius at floor level and rear interference radius.
    pub reach: f32,
    pub max_height: f32,
    pub min_height: f32,
    pub min_radius: f32,
    pub rear_radius: f32,
    /// Maximum speeds (rad/s) and motion ranges (rad) of the data sheet, in the
    /// manufacturer's axes.
    pub speed: [f32; 4],
    pub min: [f32; 4],
    pub max: [f32; 4],
    /// Highest upper arm elevation from the horizontal.
    pub max_elevation: f32,
    /// A3 measured between the arms (KUKA: 90° with the lower arm vertical and the
    /// upper arm horizontal) instead of from the horizontal (FANUC J3).
    pub relative_a3: bool,
    /// Acceleration time of the trapezoidal speed profile (ACC 100).
    pub acc_time: f32,
    /// Rest position (MOVE_PERCH start).
    pub home: Pose,
    /// Speeds of the robot program (m/s, linear moves).
    pub program: Program,
    /// Body turning with J1 (column, J2/A2 housing), in the J1 direction: from
    /// `body_back` behind to `body_front` ahead of the axis, `body_half_width` to each
    /// side, up to `body_top` above the robot base.
    pub body_front: f32,
    pub body_back: f32,
    pub body_half_width: f32,
    pub body_top: f32,
}

/// Linear speeds and limits of the robot program: FANUC TP routines PICK/PLACE
/// (Selgron), KUKA KRL MovePick/MovePlace/MovePerch.
#[derive(Clone)]
pub struct Program {
    pub pick: &'static str,
    pub place: &'static str,
    /// Down to the pick, up from it, down to the place, up from it.
    pub pick_down: f32,
    pub pick_up: f32,
    pub place_down: f32,
    pub place_up: f32,
    /// Highest TCP height of the transfers (KUKA: PosAux.Z <= 2000).
    pub max_travel_y: f32,
}

/// FANUC data sheet, M-410iC/185. Dimensions measured on the manufacturer's STEP
/// model: J2 1110 mm above the floor (pedestal type base), offset 394 mm, arms
/// 1220/1306 mm, flange 244 mm ahead of and 159 mm below the wrist pin. The J2/J3
/// split of the ranges (totals 144° and 136°) reproduces the data sheet envelope:
/// top 2397 mm, bottom -561 mm and 1100 mm inner radius at the floor.
pub static FANUC: Kin = Kin {
    model: RobotModel::FanucM410iC185,
    axis_names: ["J1", "J2", "J3", "J4"],
    base_height: 1.110,
    j2_offset: 0.394,
    l1: 1.220,
    l2: 1.306,
    upper_back: 0.45,
    wrist_fwd: 0.244,
    wrist_drop: 0.159,
    payload_kg: 185.0,
    mass_kg: 1600.0,
    repeatability_mm: 0.05,
    reach: 3.143,
    max_height: 2.397,
    min_height: -0.561,
    min_radius: 1.100,
    rear_radius: 0.725,
    speed: [140.0 * DEG, 140.0 * DEG, 140.0 * DEG, 305.0 * DEG],
    min: [-180.0 * DEG, -44.0 * DEG, -126.0 * DEG, -360.0 * DEG],
    max: [180.0 * DEG, 100.0 * DEG, 10.0 * DEG, 360.0 * DEG],
    max_elevation: 10.0 * DEG,
    relative_a3: false,
    acc_time: 0.35,
    home: Pose { theta: 0.0, r: 1.7, y: 1.405, yaw: 0.0 },
    // PICK: L PR[3] 3000 mm/s, L PR[11] 1800 mm/s; PLACE: L PR[5] 2000 mm/s, L PR[11] 4000 mm/s.
    program: Program {
        pick: "PICK",
        place: "PLACE",
        pick_down: 3.0,
        pick_up: 1.8,
        place_down: 2.0,
        place_up: 4.0,
        max_travel_y: f32::INFINITY,
    },
    // Turntable and J2 housing of the STEP (the pedestal is not drawn).
    body_front: 0.49,
    body_back: 0.45,
    body_half_width: 0.40,
    body_top: 1.36,
};

/// KUKA data sheet, KR 180 R3200-2 PA (0000-361-137 / V5.1): 180 kg, reach 3195 mm,
/// 5 axes (A4 absent), A1 ±185° 137 °/s, A2 -140°/-5° 111 °/s, A3 0°/165° 119 °/s,
/// A5 -55°/150° 225 °/s, A6 ±350° 321 °/s, ±0.07 mm, 1017 kg. Dimensions from the
/// skeleton of the manufacturer's STEP: A2 645 mm above the floor and 330 mm ahead
/// of A1, link arm 1350 mm, arm 1255 mm to A5, flange 265 mm ahead of and 250 mm
/// below A5. With the arm at most +60° above the horizontal (workspace graphic)
/// these give the envelope of the STEP: reach 3195 mm at 513 mm, top 2832 mm,
/// bottom -742 mm.
pub static KUKA: Kin = Kin {
    model: RobotModel::KukaKr180R3200_2Pa,
    axis_names: ["A1", "A2", "A3", "A6"],
    base_height: 0.645,
    j2_offset: 0.330,
    l1: 1.350,
    l2: 1.255,
    upper_back: 0.35,
    wrist_fwd: 0.265,
    wrist_drop: 0.250,
    payload_kg: 180.0,
    mass_kg: 1017.0,
    repeatability_mm: 0.07,
    reach: 3.195,
    max_height: 2.832,
    min_height: -0.742,
    min_radius: 0.940,
    rear_radius: 0.816,
    speed: [137.0 * DEG, 111.0 * DEG, 119.0 * DEG, 321.0 * DEG],
    min: [-185.0 * DEG, -140.0 * DEG, 0.0, -350.0 * DEG],
    max: [185.0 * DEG, -5.0 * DEG, 165.0 * DEG, 350.0 * DEG],
    max_elevation: 60.0 * DEG,
    relative_a3: true,
    acc_time: 0.35,
    home: Pose { theta: 0.0, r: 1.85, y: 1.0, yaw: 0.0 },
    // MovePick: SLIN up $VEL.CP=3.0, SPTP 100 % above the pick, SLIN down 3.0 to the
    // approach point and 2.5 to the pick. MovePlace: SLIN up 3.0, SPTP 100 %, SLIN
    // down 2.5 to the approach point and 3.0 to the place. Transfers at most Z 2000.
    program: Program {
        pick: "MovePick",
        place: "MovePlace",
        pick_down: 2.5,
        pick_up: 3.0,
        place_down: 2.5,
        place_up: 3.0,
        max_travel_y: 2.0,
    },
    // Rotating column with the A2 gearbox and motors (STEP): 0.53 m ahead and
    // 0.75 m behind A1, up to the A2 hub (0.645 + 0.25 m).
    body_front: 0.53,
    body_back: 0.75,
    body_half_width: 0.45,
    body_top: 0.90,
};

pub fn kin(model: RobotModel) -> &'static Kin {
    match model {
        RobotModel::FanucM410iC185 => &FANUC,
        RobotModel::KukaKr180R3200_2Pa => &KUKA,
    }
}

impl Kin {
    /// This model with the robot's software limits (degrees, manufacturer's axes,
    /// `[min, max]` per axis), each kept inside the data sheet range. Empty or a
    /// missing axis: the data sheet range.
    pub fn with_limits(&self, limits: &[[f32; 2]]) -> Kin {
        let mut kin = self.clone();
        for (i, [lo, hi]) in limits.iter().take(4).enumerate() {
            let (lo, hi) = (lo.to_radians().max(self.min[i]), hi.to_radians().min(self.max[i]));
            if lo < hi {
                kin.min[i] = lo;
                kin.max[i] = hi;
            }
        }
        kin
    }

    /// J1 for the direction `theta`: `theta` or a full turn away, inside the J1 range
    /// (the KUKA goes to ±185°), the one closest to `near`.
    pub fn j1_near(&self, theta: f32, near: f32) -> f32 {
        [theta, theta + 2.0 * PI, theta - 2.0 * PI]
            .into_iter()
            .filter(|j| *j >= self.min[0] - 1e-4 && *j <= self.max[0] + 1e-4)
            .min_by(|a, b| (a - near).abs().total_cmp(&(b - near).abs()))
            .unwrap_or(theta)
    }

    /// Flange face to the bottom of the cabeçote frame.
    pub fn frame_drop(&self) -> f32 {
        self.wrist_drop + TOOL_LENGTH
    }

    /// TCP: tips of the clamps, where the bottom of the held fardos is (the fardos
    /// sit at the bottom of the gripper, between the clamps).
    pub fn tcp_drop(&self) -> f32 {
        self.frame_drop() + JAW_HEIGHT
    }

    /// Manufacturer's axes for these axes (KUKA: A2 = J2 - 90°, A3 between the arms).
    pub fn to_motor(&self, a: Axes) -> [f32; 4] {
        let [j1, j2, j3, j4] = a.0;
        if self.relative_a3 {
            [j1, j2 - FRAC_PI_2, FRAC_PI_2 - j2 - j3, j4]
        } else {
            a.0
        }
    }

    pub fn within_limits(&self, a: Axes) -> bool {
        let m = self.to_motor(a);
        (0..4).all(|i| m[i] >= self.min[i] - 1e-4 && m[i] <= self.max[i] + 1e-4) && a.0[2] <= self.max_elevation + 1e-4
    }

    /// Forward kinematics: the TCP pose for these axes.
    pub fn pose(&self, a: Axes) -> Pose {
        let [j1, j2, j3, j4] = a.0;
        let shoulder = FRAC_PI_2 - j2;
        let wrist = Vec2::new(shoulder.cos(), shoulder.sin()) * self.l1 + Vec2::new(j3.cos(), j3.sin()) * self.l2;
        Pose {
            theta: j1,
            r: self.j2_offset + wrist.x + self.wrist_fwd,
            y: self.base_height + wrist.y - self.tcp_drop(),
            yaw: j4 + j1,
        }
    }

    /// Axis positions of this pose (J1 within its range, never wrapped).
    pub fn axes(&self, pose: &Pose) -> Axes {
        let joints = self.solve(pose);
        Axes([
            self.j1_near(pose.theta, pose.theta),
            FRAC_PI_2 - joints.shoulder,
            joints.upper_dir.y.atan2(joints.upper_dir.x),
            pose.yaw - pose.theta,
        ])
    }

    /// Outline of the operating space of the flange center in the arm plane
    /// (radius from J1, height above the floor), along the edges of the arm axis
    /// ranges and of the upper arm elevation limit, as separate polylines.
    pub fn envelope_outline(&self, steps: usize) -> Vec<Vec<Vec2>> {
        let (lo, hi) = (self.min, self.max);
        let lerp = |a: f32, b: f32, k: usize| a + (b - a) * k as f32 / steps as f32;
        // Manufacturer's (A2, A3) -> internal (J2, J3).
        let internal = |m2: f32, m3: f32| if self.relative_a3 { (m2 + FRAC_PI_2, -m2 - m3) } else { (m2, m3) };
        let j2_lo = internal(lo[1], 0.0).0;
        let j2_hi = internal(hi[1], 0.0).0;
        let edges: Vec<Vec<(f32, f32)>> = vec![
            (0..=steps).map(|k| internal(lerp(lo[1], hi[1], k), hi[2])).collect(),
            (0..=steps).map(|k| internal(hi[1], lerp(hi[2], lo[2], k))).collect(),
            (0..=steps).map(|k| internal(lerp(hi[1], lo[1], k), lo[2])).collect(),
            (0..=steps).map(|k| internal(lo[1], lerp(lo[2], hi[2], k))).collect(),
            (0..=steps).map(|k| (lerp(j2_lo, j2_hi, k), self.max_elevation)).collect(),
        ];
        let mut lines = Vec::new();
        for edge in edges {
            let mut line = Vec::new();
            for (j2, j3) in edge {
                let a = Axes([0.0, j2, j3, 0.0]);
                if self.within_limits(a) {
                    let p = self.pose(a);
                    line.push(Vec2::new(p.r, p.y + TOOL_LENGTH + JAW_HEIGHT));
                } else if line.len() > 1 {
                    lines.push(std::mem::take(&mut line));
                } else {
                    line.clear();
                }
            }
            if line.len() > 1 {
                lines.push(line);
            }
        }
        lines
    }

    /// Inverse kinematics in the arm plane (elbow-up solution). `elbow` and the
    /// wrist pivot are relative to the J2 axis; the TCP is `wrist_fwd` ahead of and
    /// `tcp_drop` below the wrist pivot.
    pub fn solve(&self, pose: &Pose) -> Joints {
        let (l1, l2) = (self.l1, self.l2);
        let r = pose.r - self.j2_offset - self.wrist_fwd;
        let h = pose.y + self.tcp_drop() - self.base_height;
        let target = Vec2::new(r, h);
        let d = target.length();
        let in_plane = d <= l1 + l2 - 1e-3 && d >= (l1 - l2).abs() + 1e-3;
        let dc = d.clamp((l1 - l2).abs() + 1e-3, l1 + l2 - 1e-3);
        let cos_inner = ((l1 * l1 + dc * dc - l2 * l2) / (2.0 * l1 * dc)).clamp(-1.0, 1.0);
        let shoulder = h.atan2(r) + cos_inner.acos();
        let elbow = Vec2::new(l1 * shoulder.cos(), l1 * shoulder.sin());
        let wrist = if in_plane {
            target
        } else {
            elbow + (target - elbow).normalize_or_zero() * l2
        };
        let upper_dir = (wrist - elbow).normalize_or_zero();
        let j2 = FRAC_PI_2 - shoulder;
        let j3 = upper_dir.y.atan2(upper_dir.x);
        let axes = Axes([self.j1_near(pose.theta, pose.theta), j2, j3, pose.yaw - pose.theta]);
        Joints {
            j1: pose.theta,
            shoulder,
            elbow,
            upper_dir,
            reachable: in_plane && self.within_limits(axes),
        }
    }

    fn plane_to_local(&self, j1: f32, p: Vec2) -> Vec3 {
        let radial = self.j2_offset + p.x;
        Vec3::new(radial * j1.cos(), self.base_height + p.y, -radial * j1.sin())
    }

    /// Highest TCP height the arm reaches at radius `r` from J1 within the axis limits
    /// (the top of the operating space at that radius); -inf when out of reach.
    pub fn max_tcp_height(&self, r: f32) -> f32 {
        let reachable = |y: f32| self.solve(&Pose { theta: 0.0, r, y, yaw: 0.0 }).reachable;
        let (mut lo, mut hi) = (f32::NEG_INFINITY, 3.5);
        let mut y = hi;
        while y > -1.0 {
            if reachable(y) {
                lo = y;
                break;
            }
            hi = y;
            y -= 0.05;
        }
        if lo == f32::NEG_INFINITY {
            return lo;
        }
        for _ in 0..12 {
            let mid = (lo + hi) / 2.0;
            if reachable(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    /// J4 inside its range for the gripper yaw `y` with J1 at `theta`.
    fn j4_ok(&self, y: f32, theta: f32) -> bool {
        let j4 = y - theta;
        j4 >= self.min[3] - 1e-4 && j4 <= self.max[3] + 1e-4
    }

    /// Gripper yaw for exactly `target` (full turns only) with J1 at `theta`: the one
    /// that turns the J4/A6 motor the least from the current pose (the flange turns
    /// with J1 too, so the yaw closest to the current one can cost almost a full turn
    /// of the wrist after a half turn of J1).
    fn wrist_yaw_exact(&self, current: &Pose, target: f32, theta: f32) -> f32 {
        let j4 = current.yaw - current.theta;
        let cost = |y: f32| ((y - theta) - j4).powi(2) + 0.25 * (y - theta).powi(2);
        (-4..=4)
            .map(|k| target + k as f32 * 2.0 * PI)
            .filter(|y| self.j4_ok(*y, theta))
            .min_by(|a, b| cost(*a).total_cmp(&cost(*b)))
            .unwrap_or(target)
    }

    /// Gripper yaw for `target` (any half turn) with J1 at `theta`, turning the J4/A6
    /// motor the least from the current pose (see [`Kin::wrist_yaw_exact`]) and keeping
    /// it toward the middle of its range: a wrist that drifted to one end needs almost
    /// a full turn back at the next exact placement.
    fn wrist_yaw(&self, current: &Pose, target: f32, theta: f32) -> f32 {
        let j4 = current.yaw - current.theta;
        let cost = |y: f32| ((y - theta) - j4).powi(2) + 0.5 * (y - theta).powi(2);
        (-8..=8)
            .map(|k| target + k as f32 * PI)
            .filter(|y| self.j4_ok(*y, theta))
            .min_by(|a, b| cost(*a).total_cmp(&cost(*b)))
            .unwrap_or_else(|| nearest_half_turn(current.yaw, target))
    }

    /// Why the TCP point `p` (robot frame) is out of reach: distance of the arm, axis
    /// limits (data sheet), floor (`floor_y` in the robot frame: minus the height the
    /// robot is mounted at), maximum height at that radius. `None` when reachable.
    pub fn reach_report(&self, p: Vec3, floor_y: f32) -> Option<String> {
        let pose = Pose::from_local(p, 0.0);
        if self.solve(&pose).reachable && pose.y >= floor_y - 0.01 {
            return None;
        }
        let (l1, l2) = (self.l1, self.l2);
        let mut why = Vec::new();
        let plane = Vec2::new(pose.r - self.j2_offset - self.wrist_fwd, pose.y + self.tcp_drop() - self.base_height).length();
        if plane > l1 + l2 - 1e-3 {
            why.push(format!(
                "longe demais: {:.0} mm do eixo {} até o punho, o braço estica até {:.0} mm",
                plane * 1000.0,
                self.axis_names[1],
                (l1 + l2) * 1000.0
            ));
        }
        if plane < (l1 - l2).abs() + 1e-3 {
            why.push("perto demais da base".to_owned());
        }
        if pose.y < floor_y - 0.01 {
            why.push(format!("abaixo do piso ({:.2} m abaixo da base do robô)", -floor_y));
        }
        let axes = self.axes(&pose);
        let motor = self.to_motor(axes);
        for (i, m) in motor.iter().enumerate().take(3) {
            let (q, lo, hi) = (m.to_degrees(), self.min[i].to_degrees(), self.max[i].to_degrees());
            if q < lo - 0.01 || q > hi + 0.01 {
                why.push(format!("{} precisaria de {q:.0}° (faixa {lo:.0}° a {hi:.0}°)", self.axis_names[i]));
            }
        }
        if axes.0[2] > self.max_elevation + 1e-3 {
            why.push(format!(
                "o braço precisaria subir {:.0}° acima da horizontal (máximo {:.0}°)",
                axes.0[2].to_degrees(),
                self.max_elevation.to_degrees()
            ));
        }
        let top = self.max_tcp_height(pose.r);
        if top.is_finite() && pose.y > top {
            why.push(format!("nesse raio o TCP sobe só até {top:.2} m"));
        } else if !top.is_finite() {
            why.push(format!("raio além do alcance ({:.2} m)", self.reach));
        }
        Some(format!("TCP a {:.2} m de raio e {:.2} m de altura: {}", pose.r, pose.y, why.join("; ")))
    }
}

/// Axis positions J1..J4 (rad): J1 base, J2 lower arm from the vertical, J3 upper arm
/// from the horizontal, J4 flange relative to J1 (see [`Kin`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Axes(pub [f32; 4]);

/// Kind of move, as in the TP routines: J (joint interpolated, % of the axis
/// speeds) or L (straight line at a TCP speed in m/s).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Move {
    Joint,
    Linear(f32),
}

/// Pivots of the FANUC STEP model in its rest pose (lower arm vertical, upper arm
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

/// Pivots of the KUKA STEP model in its rest pose (A2 -90°, A3 +90°), from the
/// skeleton sketch of the model.
mod kuka_rest {
    use bevy::math::Vec2;
    pub const A2: Vec2 = Vec2::new(0.330, 0.645);
    pub const A3: Vec2 = Vec2::new(0.330, 1.995);
    pub const A5: Vec2 = Vec2::new(1.585, 1.995);
    /// A6 axis (flange center).
    pub const FLANGE_X: f32 = 1.850;
}

/// Parts of the manufacturer's models (`assets/models/<model>/*.glb`). FANUC: the
/// pedestal (S201) and its floor plate are left out, only the robot is drawn from
/// the J1 turntable up (the kinematics keep J2 at 1110 mm). KUKA: floor mounted,
/// base frame, rotating column, link arm (A2), arm (A3), in-line wrist (A5, kept
/// level) and mounting flange (A6).
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug)]
pub enum CadPart {
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
    KukaBase,
    KukaColumn,
    KukaLinkArm,
    KukaArm,
    KukaWrist,
    KukaFlange,
}

impl CadPart {
    pub const FANUC: [(Self, &'static str); 10] = [
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
    pub const KUKA: [(Self, &'static str); 6] = [
        (Self::KukaBase, "base"),
        (Self::KukaColumn, "column"),
        (Self::KukaLinkArm, "link_arm"),
        (Self::KukaArm, "arm"),
        (Self::KukaWrist, "wrist"),
        (Self::KukaFlange, "flange"),
    ];

    /// Parts of a model and the folder of their files under `assets/`.
    pub fn of(model: RobotModel) -> (&'static [(Self, &'static str)], &'static str) {
        match model {
            RobotModel::FanucM410iC185 => (&Self::FANUC, "models/fanuc_m410ic185"),
            RobotModel::KukaKr180R3200_2Pa => (&Self::KUKA, "models/kuka_kr180_r3200_2_pa"),
        }
    }

    /// Pose of the part in the robot frame for the given joints.
    fn transform(self, joints: &Joints, flange_yaw: f32) -> Transform {
        let a = joints.shoulder - FRAC_PI_2; // lower arm rotation from rest (vertical)
        let b = joints.upper_dir.y.atan2(joints.upper_dir.x); // upper arm from rest (horizontal)
        let about = |pivot: Vec3, new_pivot: Vec3, angle: f32| {
            Affine3A::from_translation(new_pivot) * Affine3A::from_rotation_z(angle) * Affine3A::from_translation(-pivot)
        };
        let flange = |x: f32, z: f32, dw: Vec3| {
            let axis = Vec3::new(x, 0.0, z);
            Affine3A::from_translation(axis + dw) * Affine3A::from_rotation_y(flange_yaw) * Affine3A::from_translation(-axis)
        };
        let arm = match self {
            Self::KukaBase => return Transform::IDENTITY,
            Self::KukaColumn => Affine3A::IDENTITY,
            Self::KukaLinkArm | Self::KukaArm | Self::KukaWrist | Self::KukaFlange => {
                let a2 = kuka_rest::A2.extend(0.0);
                let a3 = (kuka_rest::A2 + joints.elbow).extend(0.0);
                let dw = a3 + (joints.upper_dir * KUKA.l2).extend(0.0) - kuka_rest::A5.extend(0.0);
                match self {
                    Self::KukaLinkArm => about(a2, a2, a),
                    Self::KukaArm => about(kuka_rest::A3.extend(0.0), a3, b),
                    Self::KukaWrist => Affine3A::from_translation(dw),
                    _ => flange(kuka_rest::FLANGE_X, 0.0, dw),
                }
            }
            _ => {
                let j2 = rest::J2.extend(0.0);
                let j3 = (rest::J2 + joints.elbow).extend(0.0);
                let d3 = j3 - rest::J3.extend(0.0);
                let dw = j3 + (joints.upper_dir * FANUC.l2).extend(0.0) - rest::WRIST.extend(0.0);
                match self {
                    Self::LowerArm => about(j2, j2, a),
                    Self::Lever => about(j2, j2, b),
                    Self::LeverRod => {
                        let lever = rest::LEVER.extend(0.0);
                        let end = j2 + Quat::from_rotation_z(b) * (lever - j2);
                        about(lever, end, a)
                    }
                    Self::LevelRod => about(rest::LEVEL_ROD.extend(0.0), rest::LEVEL_ROD.extend(0.0), a),
                    Self::Bellcrank => Affine3A::from_translation(d3),
                    Self::Wrist => Affine3A::from_translation(dw),
                    Self::UpperArm => about(rest::J3.extend(0.0), j3, b),
                    Self::LevelLink => {
                        let link = rest::LEVEL_LINK.extend(0.0);
                        about(link, link + d3, b)
                    }
                    Self::Flange => flange(rest::FLANGE_X, rest::FLANGE_Z, dw),
                    _ => Affine3A::IDENTITY,
                }
            }
        };
        let full = Affine3A::from_rotation_y(joints.j1) * arm;
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
    /// One clamp opened: lift a little before the next drop of the same pick.
    Lift,
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

/// One release on the pallet within a pick: the boxes let go (with their final
/// pose in the pallet frame), the TCP point (top of the group) and group yaw there.
#[derive(Debug, Clone)]
pub struct Drop {
    boxes: Vec<(Entity, Vec3, f32)>,
    top: Vec3,
    yaw: f32,
    /// Pattern slots filled (empty with the automatic grid).
    slots: Vec<u32>,
    /// Placements counted on the pallet.
    units: u32,
    /// One clamp only: the group yaw is exact (a half turn would put the other
    /// fardo on the slot), so the wrist turns only by full turns.
    exact: bool,
}

#[derive(Component)]
pub struct RobotState {
    /// Model of this robot (kinematics, limits, speeds).
    pub kin: Kin,
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
    /// Axis speeds of the last frame (rad/s, manufacturer's axes), for the property panel.
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
    pallet: Option<Entity>,
    drops: Vec<Drop>,
    drop_i: usize,
    safe_y: f32,
    /// Robo_Fardos_Coletados.
    pub collected: bool,
    /// Waiting at the perch (MOVE_PERCH) over the next coleta.
    pub perched: bool,
    /// Clamp closure 0 (open) .. 1 (closed on the fardos).
    pub jaw: [f32; 2],
    pub status: String,
    pub cycles: u32,
    /// Log de falhas: (time, details), newest last.
    pub faults: Vec<(f32, String)>,
    /// Faults of the last planning attempt: a fault is logged when it appears.
    active_faults: Vec<String>,
    /// Cavalete under the robot (from its parameters).
    pub base: Option<RobotBase>,
}

impl Default for RobotState {
    fn default() -> Self {
        Self::new(FANUC.clone())
    }
}

impl RobotState {
    /// Clamp position from the gripper center along its Z (`jaw` 1 = closed): open
    /// only a little while down at the place, fully elsewhere.
    fn jaw_half(&self, closed: f32, jaw: f32) -> f32 {
        let open = if matches!(self.step, Step::DownPlace | Step::Release | Step::Lift) {
            (closed + JAW_RELEASE).min(JAW_OPEN)
        } else {
            JAW_OPEN
        };
        open + (closed.min(JAW_OPEN) - open) * jaw
    }

    pub fn new(kin: Kin) -> Self {
        let home = kin.home;
        let home_axes = kin.axes(&home);
        Self {
            kin,
            pose: home,
            from: home,
            to: home,
            from_axes: home_axes,
            to_axes: home_axes,
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
            pallet: None,
            drops: Vec::new(),
            drop_i: 0,
            safe_y: home.y,
            collected: false,
            perched: false,
            jaw: [0.0; 2],
            status: "Aguardando".into(),
            cycles: 0,
            faults: Vec::new(),
            active_faults: Vec::new(),
            base: None,
        }
    }

    fn moving(&self) -> bool {
        self.t < self.duration
    }

    /// Starts a move. Its duration is set by the slowest axis at its maximum speed
    /// (and, for L moves, by the TCP speed), times the override (OVERRIDE=R[104]),
    /// plus the acceleration ramps.
    fn move_to(&mut self, to: Pose, speed_override: f32, motion: Move) {
        let ovr = speed_override.clamp(0.05, 1.0);
        let (a, mut b) = (self.kin.axes(&self.pose), self.kin.axes(&to));
        // J1: the way allowed by its range that turns the least (a full turn away
        // when the range passes ±180°).
        b.0[0] = self.kin.j1_near(b.0[0], a.0[0]);
        if std::env::var("PDT_TRACE_J1").is_ok() {
            eprintln!("J1 {:?} {:.1} -> {:.1} (giro {:.1}) J4 {:.1} -> {:.1}", motion, a.0[0].to_degrees(), b.0[0].to_degrees(), (b.0[0] - a.0[0]).to_degrees(), a.0[3].to_degrees(), b.0[3].to_degrees());
        }
        // Data sheet speeds apply to the manufacturer's axes (KUKA A3 between the arms).
        let (ma, mb) = (self.kin.to_motor(a), self.kin.to_motor(b));
        let axes_time = (0..4).map(|i| (mb[i] - ma[i]).abs() / (self.kin.speed[i] * ovr)).fold(0.0, f32::max);
        let travel = match motion {
            Move::Joint => axes_time,
            // On a line the axes do not all move at a constant rate: keep some margin.
            Move::Linear(v) => ((to.local_point() - self.pose.local_point()).length() / (v * ovr)).max(axes_time * 1.15),
        };
        // Trapezoid: cruise time + one ramp; short moves never reach full speed.
        let ramp = self.kin.acc_time;
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
                self.kin.pose(Axes(q))
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

fn link_rotation(j1: f32, plane_angle: f32) -> Quat {
    Quat::from_rotation_y(j1) * Quat::from_rotation_z(plane_angle - FRAC_PI_2)
}

/// Picks `target + k*PI` closest to `current` (boxes are symmetric under 180°).
fn nearest_half_turn(current: f32, target: f32) -> f32 {
    let k = ((current - target) / PI).round();
    target + k * PI
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

/// Why a cycle could not be planned: waiting for something, or a fault (a point out
/// of the robot's reach), with the details for the log.
enum Failure {
    Wait(&'static str),
    Fault(String),
}

impl From<&'static str> for Failure {
    fn from(reason: &'static str) -> Self {
        Self::Wait(reason)
    }
}

/// One planned pick-and-place.
struct Plan {
    held: Vec<HeldBox>,
    unit: Vec3,
    group_yaw: f32,
    pick_top: Vec3,
    pallet: Entity,
    drops: Vec<Drop>,
    safe_y: f32,
    above: Pose,
    /// Products still missing on the pallet after this pick (with a pattern).
    /// Products of the next pick (with a pattern): the mesa prepares that group.
    next_size: Option<usize>,
}

/// Footprint overlap area of two products (yaw multiples of 90°).
/// Penetration of two boxes turned around Y (center, half size, yaw): the smallest
/// overlap over the separating axes (the two pairs of sides and the vertical), or None
/// when they do not touch.
fn obb_depth(a: Vec3, ha: Vec3, a_yaw: f32, b: Vec3, hb: Vec3, b_yaw: f32) -> Option<f32> {
    let vertical = ha.y + hb.y - (a.y - b.y).abs();
    if vertical <= 0.0 {
        return None;
    }
    let axes = |yaw: f32| [Vec2::new(yaw.cos(), -yaw.sin()), Vec2::new(yaw.sin(), yaw.cos())];
    let (aa, ab) = (axes(a_yaw), axes(b_yaw));
    let d = Vec2::new(b.x - a.x, b.z - a.z);
    let mut depth = vertical;
    for n in aa.iter().chain(ab.iter()) {
        let ra = ha.x * aa[0].dot(*n).abs() + ha.z * aa[1].dot(*n).abs();
        let rb = hb.x * ab[0].dot(*n).abs() + hb.z * ab[1].dot(*n).abs();
        let overlap = ra + rb - d.dot(*n).abs();
        if overlap <= 0.0 {
            return None;
        }
        depth = depth.min(overlap);
    }
    Some(depth)
}

fn overlap(a: Vec3, a_yaw: f32, b: Vec3, b_yaw: f32, size: Vec3) -> f32 {
    let half = |yaw: f32| {
        if yaw.sin().abs() > 0.7 { Vec2::new(size.z, size.x) / 2.0 } else { Vec2::new(size.x, size.z) / 2.0 }
    };
    let (ha, hb) = (half(a_yaw), half(b_yaw));
    let dx = (ha.x + hb.x - (a.x - b.x).abs()).max(0.0);
    let dz = (ha.y + hb.y - (a.z - b.z).abs()).max(0.0);
    dx * dz
}

/// True when `a` and `b` are the same orientation for a product (180° symmetric).
fn aligned(a: f32, b: f32) -> bool {
    let d = (a - b).rem_euclid(PI);
    !(0.05..=PI - 0.05).contains(&d)
}

/// Current pick of the pattern: its drops with the slots still to fill, and the
/// number of products of the pick after it (the next pallet starts over).
fn current_pick(p: &crate::pattern::Pattern, filled: &[u32]) -> Option<(Vec<Vec<u32>>, usize)> {
    let fallback: Vec<Vec<Vec<u32>>>;
    let picks = if p.picks.is_empty() {
        let seq = p.sequence();
        fallback = seq.chunks(p.per_pick.max(1) as usize).map(|c| c.iter().map(|&s| vec![s]).collect()).collect();
        &fallback
    } else {
        &p.picks
    };
    let i = picks.iter().position(|k| k.iter().flatten().any(|s| !filled.contains(s)))?;
    let drops: Vec<Vec<u32>> = picks[i]
        .iter()
        .map(|d| d.iter().copied().filter(|s| !filled.contains(s)).collect::<Vec<_>>())
        .filter(|d| !d.is_empty())
        .collect();
    let next = picks.get(i + 1).unwrap_or(&picks[0]).iter().flatten().count();
    Some((drops, next))
}

/// Drops of the current pick: each drop lets go of the fardos on one side of the
/// cabeçote (the lowest or highest along the group Z: a pair, a run of 3, one
/// fardo...) onto a run of side-by-side slots. The group turns (exactly, a half
/// turn would swap the sides) so the fardos still held do not land on products
/// already placed. With a different number of fardos than the pick, they go one by
/// one onto the next slots.
fn pattern_drops(p: &crate::pattern::Pattern, pallet_h: f32, held: &[HeldBox], spec: &[Vec<u32>], filled: &[u32], dy: f32) -> Vec<Drop> {
    let h = held.iter().map(|b| b.height).fold(0.0, f32::max);
    let place = |slot: u32| p.slot(pallet_h, h, slot).map(|(c, yaw)| (c + Vec3::Y * dy, yaw));
    let needed: usize = spec.iter().map(|d| d.len()).sum();
    let spec: Vec<Vec<u32>> = if needed == held.len() {
        spec.to_vec()
    } else {
        spec.iter().flatten().take(held.len()).map(|&s| vec![s]).collect()
    };
    let size = Vec3::from_array(p.unit);
    let mut occupied: Vec<(Vec3, f32)> = filled.iter().filter_map(|&s| place(s)).collect();
    let mut remaining: Vec<usize> = (0..held.len()).collect();
    remaining.sort_by(|&a, &b| held[a].offset.z.total_cmp(&held[b].offset.z));
    // Clamp plates: on both sides of the held group along its Z, closed against it.
    let half_z = |b: &HeldBox| if b.rel_yaw.sin().abs() > 0.7 { size.x / 2.0 } else { size.z / 2.0 };
    let group_lo = held.iter().map(|b| b.offset.z - half_z(b)).fold(f32::MAX, f32::min);
    let group_hi = held.iter().map(|b| b.offset.z + half_z(b)).fold(f32::MIN, f32::max);
    let jaw_off = JAW_GAP + JAW_THICKNESS / 2.0;
    let jaws = [group_lo - jaw_off, group_hi + jaw_off];
    let jaw_half = Vec3::new(0.25, 0.1, JAW_THICKNESS / 2.0);
    let mut drops = Vec::new();
    for slots in &spec {
        let k = slots.len();
        let Some(spots) = slots.iter().map(|&s| place(s)).collect::<Option<Vec<_>>>() else { break };
        if k > remaining.len() {
            break;
        }
        let mut subsets = vec![remaining[..k].to_vec()];
        if k < remaining.len() {
            subsets.push(remaining[remaining.len() - k..].to_vec());
        }
        let mut best: Option<(f32, Vec<usize>, Vec<usize>, f32)> = None;
        for subset in &subsets {
            for reverse in [false, true] {
                // Slot order for the boxes of the subset (ascending along the group Z).
                let order: Vec<usize> = if reverse { (0..k).rev().collect() } else { (0..k).collect() };
                let gamma = if k >= 2 {
                    let d = spots[order[k - 1]].0 - spots[order[0]].0;
                    d.x.atan2(d.z)
                } else {
                    spots[0].1 - held[subset[0]].rel_yaw + if reverse { PI } else { 0.0 }
                };
                let rot = Quat::from_rotation_y(gamma);
                let mean_spot = order.iter().map(|&j| spots[j].0).sum::<Vec3>() / k as f32;
                let mean_off = subset.iter().map(|&b| Vec3::new(held[b].offset.x, 0.0, held[b].offset.z)).sum::<Vec3>() / k as f32;
                let centre = mean_spot - rot * mean_off;
                let mut score = 0.0;
                for (n, &b) in subset.iter().enumerate() {
                    if !aligned(gamma + held[b].rel_yaw, spots[order[n]].1) {
                        score += 100.0;
                    }
                }
                // The clamps come down beside the group: not into a fardo of the layer.
                let layer: Vec<&(Vec3, f32)> = occupied.iter().filter(|(o, _)| (o.y - spots[0].0.y).abs() < 0.02).collect();
                for z in jaws {
                    let jaw = centre + rot * Vec3::new(0.0, 0.0, z);
                    for (o, oy) in &layer {
                        if let Some(depth) = obb_depth(Vec3::new(jaw.x, o.y, jaw.z), jaw_half, gamma, *o, size / 2.0, *oy) {
                            score += depth * 0.5;
                        }
                    }
                }
                for &j in remaining.iter().filter(|j| !subset.contains(j)) {
                    let other = centre + rot * Vec3::new(held[j].offset.x, 0.0, held[j].offset.z);
                    let other_yaw = gamma + held[j].rel_yaw;
                    score += occupied
                        .iter()
                        .filter(|(o, _)| (o.y - spots[0].0.y).abs() < 0.02)
                        .map(|(o, oy)| overlap(other, other_yaw, *o, *oy, size))
                        .sum::<f32>();
                }
                if best.as_ref().is_none_or(|b| score < b.0 - 1e-6) {
                    best = Some((score, subset.clone(), order, gamma));
                }
            }
        }
        let Some((_, subset, order, gamma)) = best else { break };
        let rot = Quat::from_rotation_y(gamma);
        let mean_spot = order.iter().map(|&j| spots[j].0).sum::<Vec3>() / k as f32;
        let mean_off = subset.iter().map(|&b| Vec3::new(held[b].offset.x, 0.0, held[b].offset.z)).sum::<Vec3>() / k as f32;
        let centre = mean_spot - rot * mean_off;
        drops.push(Drop {
            boxes: subset.iter().zip(&order).map(|(&b, &j)| (held[b].entity, spots[j].0, spots[j].1)).collect(),
            top: Vec3::new(centre.x, spots[0].0.y - h / 2.0, centre.z),
            yaw: gamma,
            slots: order.iter().map(|&j| slots[j]).collect(),
            units: k as u32,
            exact: true,
        });
        occupied.extend(spots);
        remaining.retain(|j| !subset.contains(j));
    }
    drops
}

type LoadQuery<'w, 's> = Query<'w, 's, (Entity, &'static mut Load, &'static mut Transform), Without<Equipment>>;

/// A pallet (with what is stacked on it) the gripper may pass over, in the robot
/// frame: direction (J1 angle), radius, half of its diagonal and top height.
#[derive(Clone, Copy)]
struct Stack {
    theta: f32,
    r: f32,
    half: f32,
    top: f32,
    /// How close the gripper center may come: its half size seen from the stack.
    reach: f32,
}

/// Every pallet on the floor or on a conveyor, with the top of its stack.
fn stacks(loads: &LoadQuery, world_to_local: &Affine3A) -> Vec<Stack> {
    let mut out: Vec<(Entity, Stack)> = loads
        .iter()
        .filter(|(_, l, _)| l.kind == LoadKind::Pallet && l.state == LoadState::Free)
        .map(|(e, l, tf)| {
            let c = world_to_local.transform_point3(tf.translation);
            let stack = Stack {
                theta: (-c.z).atan2(c.x),
                r: Vec2::new(c.x, c.z).length(),
                half: Vec2::new(l.size.x, l.size.z).length() / 2.0,
                top: c.y + l.size.y / 2.0,
                reach: GRIPPER_REACH,
            };
            (e, stack)
        })
        .collect();
    // Fardos placed on a pallet are its children (pallet frame).
    for (_, l, tf) in loads.iter() {
        if let LoadState::Placed(pallet) = l.state
            && let Some((_, stack)) = out.iter_mut().find(|(e, _)| *e == pallet)
            && let Ok((_, _, ptf)) = loads.get(pallet)
        {
            let top = world_to_local.transform_point3(ptf.transform_point(tf.translation)).y + l.size.y / 2.0;
            stack.top = stack.top.max(top);
        }
    }
    out.into_iter().map(|(_, s)| s).collect()
}

/// Gripper (cabeçote) half length plus the carried fardos beyond it.
const GRIPPER_REACH: f32 = 0.70;

/// Highest stack under the path of the TCP turning J1 from `a` to `b` (angle, radius;
/// J1 unwrapped as the robot turns) with the gripper, or None.
fn highest_on_path(stacks: &[Stack], a: (f32, f32), b: (f32, f32)) -> Option<&Stack> {
    let (lo, hi) = (a.0.min(b.0), a.0.max(b.0));
    let (r_lo, r_hi) = (a.1.min(b.1), a.1.max(b.1));
    stacks
        .iter()
        .filter(|s| {
            let reach = s.half + s.reach;
            let in_band = s.r + reach >= r_lo && s.r - reach <= r_hi;
            let w = if s.r > reach { (reach / s.r).asin() } else { PI };
            in_band && (-1..=1).any(|k| {
                let t = s.theta + k as f32 * 2.0 * PI;
                t >= lo - w && t <= hi + w
            })
        })
        .max_by(|x, y| x.top.total_cmp(&y.top))
}

/// Cabeçote frame half sizes (500 x 1234 mm): along its X and Z.
const FRAME_HALF: Vec2 = Vec2::new(0.25, 0.617);

/// The robot itself as obstacles for the path: the cavalete under it (top at the
/// robot base) and its body, which always faces the arm (so only its front counts).
fn self_stacks(kin: &Kin, base: Option<RobotBase>) -> Vec<Stack> {
    let mut out = vec![Stack { theta: 0.0, r: 0.0, half: kin.body_front, top: kin.body_top, reach: FRAME_HALF.y }];
    if let Some(b) = base {
        out.push(Stack { theta: 0.0, r: 0.0, half: Vec2::new(b.depth, b.width).length() / 2.0, top: 0.0, reach: FRAME_HALF.y });
    }
    out
}

/// Cavalete under the robot, in the robot frame.
#[derive(Clone, Copy, Debug)]
pub struct RobotBase {
    pub height: f32,
    pub width: f32,
    pub depth: f32,
    /// Turn relative to the robot (rad).
    pub yaw: f32,
}

/// What the cabeçote (frame and carried fardos, TCP at its bottom) would hit with the
/// TCP at `tcp` (robot frame) and gripper yaw `yaw`: the cavalete or the robot body
/// (turned to `theta`, the J1 direction). None when clear.
fn self_hit(kin: &Kin, base: Option<RobotBase>, tcp: Vec3, yaw: f32, theta: f32) -> Option<&'static str> {
    let frame = Vec3::new(FRAME_HALF.x, 0.0, FRAME_HALF.y);
    let flat = |p: Vec3| Vec3::new(p.x, 0.0, p.z);
    if let Some(b) = base
        && tcp.y < 0.02
        && obb_depth(flat(tcp), frame + Vec3::Y, yaw, Vec3::ZERO, Vec3::new(b.depth / 2.0, 1.0, b.width / 2.0), b.yaw).is_some_and(|d| d > 0.005)
    {
        return Some("o cavalete");
    }
    let along = (kin.body_front - kin.body_back) / 2.0;
    let center = Vec3::new(along * theta.cos(), 0.0, -along * theta.sin());
    let half = Vec3::new((kin.body_front + kin.body_back) / 2.0, 1.0, kin.body_half_width);
    if tcp.y < kin.body_top && obb_depth(flat(tcp), frame + Vec3::Y, yaw, center, half, theta).is_some_and(|d| d > 0.005) {
        return Some("o corpo do robô (coluna)");
    }
    None
}

/// Clearance of the TCP (bottom of the gripper and of the carried fardos) over a stack.
const STACK_CLEARANCE: f32 = 0.10;

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
) -> Result<Plan, Failure> {
    let kin = &state.kin;
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
        return Err("Aguardando produto na coleta".into());
    }
    let Ok((_, pallet, pallet_tf)) = loads.get(pallet_entity) else {
        return Err("Aguardando pallet em posição".into());
    };
    let (pallet_size, pallet_tf) = (pallet.size, *pallet_tf);
    let content = pallet_loads.get(pallet_entity).ok();
    let pattern = tdp_eq.params.pattern.as_ref();
    let filled: &[u32] = content.map_or(&[], |c| &c.filled);
    let pick = pattern.and_then(|p| current_pick(p, filled));
    if pattern.is_some() {
        let Some((spec, _)) = &pick else {
            return Err("Pallet completo".into());
        };
        // Take only what the pick needs; one product is the one the empurrador
        // pushed (lane out, lowest Z on the mesa).
        let need: usize = spec.iter().map(|d| d.len()).sum();
        if group.len() > need {
            let to_pick = pick_tf.compute_affine().inverse();
            group.sort_by(|a, b| to_pick.transform_point3(a.1).z.total_cmp(&to_pick.transform_point3(b.1).z));
            group.truncate(need);
        }
    }

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
    if pick_eq.kind.is_mesa() && group.len() == 1 {
        // A single fardo on one lane: the cabeçote stays centred on the mesa and holds
        // it with the clamp on that side.
        let mid = (to_group * pick_tf.translation).z;
        let half = (min.z - mid).abs().max((max.z - mid).abs());
        min.z = mid - half;
        max.z = mid + half;
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
        .collect::<Vec<_>>();
    let mut held = held;
    if pick_eq.kind.is_mesa() && held.len() >= 2 {
        // The clamps close the fardos against each other (no gap between them).
        let mut order: Vec<usize> = (0..held.len()).collect();
        order.sort_by(|&a, &b| held[a].offset.z.total_cmp(&held[b].offset.z));
        let widths: Vec<f32> = group.iter().map(|(_, _, rot, size)| 2.0 * half_extent_along(to_group * *rot, *size, Vec3::Z)).collect();
        let mut z = -widths.iter().sum::<f32>() / 2.0;
        for i in order {
            held[i].offset.z = z + widths[i] / 2.0;
            z += widths[i];
        }
    }
    // TCP at the bottom of the fardos (tips of the clamps).
    let pick_top = Vec3::new(center.x, top - height, center.z);

    // Stack on top of the cardboard sheets.
    let sheets_dy = content.map_or(0, |p| p.sheets) as f32 * crate::paper::SHEET_THICKNESS;
    let unit = Vec3::new(max.x - min.x, height, max.z - min.z);
    let (drops, next_size) = match (pattern, &pick) {
        (Some(p), Some((spec, next))) => (pattern_drops(p, pallet_size.y, &held, spec, filled, sheets_dy), Some(*next)),
        (Some(_), None) => return Err("Pallet completo".into()),
        (None, _) => {
            // Automatic grid: the pallet keeps the footprint of its first placement.
            let units_done = content.map_or(0, |p| p.units);
            let group = unit;
            let unit = content.and_then(|p| p.unit_size).unwrap_or(unit);
            let Some((slot_local, slot_yaw)) = sim::pallet_slot(&tdp_eq.params, pallet_size, unit, units_done) else {
                return Err("Pallet completo".into());
            };
            // A group that came turned 90° (e.g. fardos fed from a side belt) is turned
            // back so its footprint matches the slot.
            let swapped = (group.x - unit.z).abs() < 0.03 && (group.z - unit.x).abs() < 0.03;
            let same = (group.x - unit.x).abs() < 0.03 && (group.z - unit.z).abs() < 0.03;
            let slot_yaw = if swapped && !same { slot_yaw + FRAC_PI_2 } else { slot_yaw };
            let slot_local = slot_local + Vec3::Y * sheets_dy;
            let slot_rot = Quat::from_rotation_y(slot_yaw);
            let bottom = slot_local.y - unit.y / 2.0;
            let boxes = held
                .iter()
                .map(|b| {
                    let o = slot_rot * b.offset;
                    (b.entity, Vec3::new(slot_local.x + o.x, bottom + b.height / 2.0, slot_local.z + o.z), slot_yaw + b.rel_yaw)
                })
                .collect();
            let top = slot_local - Vec3::Y * unit.y / 2.0;
            (vec![Drop { boxes, top, yaw: slot_yaw, slots: Vec::new(), units: 1, exact: true }], None)
        }
    };
    if drops.is_empty() {
        return Err("Pallet completo".into());
    }
    let local_pick = world_to_local.transform_point3(pick_top);
    let pick_pose = Pose::from_local(local_pick, 0.0);
    let tops: Vec<Vec3> = drops.iter().map(|d| world_to_local.transform_point3(pallet_tf.transform_point(d.top))).collect();
    // Each point the arm cannot reach, with the reason (log de falhas). The floor is
    // below the robot base by the height the robot is mounted at (pedestal).
    let floor_y = -robot_tf.translation.y;
    let mut faults = Vec::new();
    if let Some(why) = kin.reach_report(local_pick, floor_y) {
        faults.push(format!("coleta em {}: {why}", pick_eq.name));
    }
    for (d, t) in drops.iter().zip(&tops) {
        if let Some(why) = kin.reach_report(*t, floor_y) {
            let what = if d.slots.is_empty() {
                format!("depósito no {}", tdp_eq.name)
            } else {
                let slots: Vec<String> = d.slots.iter().map(|s| (s + 1).to_string()).collect();
                format!("depósito no {} (posição {} do padrão)", tdp_eq.name, slots.join(", "))
            };
            faults.push(format!("{what}: {why}"));
        }
    }
    if !faults.is_empty() {
        return Err(Failure::Fault(faults.join(" | ")));
    }
    // Safe height for the transfer, never above what the arm reaches at the pick and
    // place radii (the operating space narrows near its top).
    // The carried fardos (TCP at their bottom) must clear the fardos already standing
    // in the layer being built, one fardo height above each drop point.
    let highest = tops.iter().map(|t| t.y + height).fold(f32::MIN, f32::max);
    // Over the pallet the arm must clear the layer; at the pick (often close to the base,
    // where the arm cannot go as high) it rises only as far as it reaches there.
    let ceiling = tops.iter().map(|t| kin.max_tcp_height(radial(*t))).fold(f32::INFINITY, f32::min) - 0.02;
    let pick_ceiling = kin.max_tcp_height(radial(local_pick)) - 0.02;
    let safe_y = (highest + 0.15).min(ceiling).min(kin.program.max_travel_y.max(highest + 0.05));
    if safe_y < highest + 0.05 {
        return Err(Failure::Fault(format!(
            "altura sobre o {} (coleta em {}): precisa passar a {:.2} m (topo da camada {:.2} m + 0,05) e o braço só sobe até {:.2} m a {:.2} m de raio",
            tdp_eq.name,
            pick_eq.name,
            highest + 0.05,
            highest,
            ceiling + 0.02,
            tops.iter().map(|t| radial(*t)).fold(0.0, f32::max),
        )));
    }
    // Other pallets on the way (a tall one next to the pallet being built): the arm
    // passes above them, from where it is to the pick and from the pick to the place.
    let mut obstacles = stacks(loads, world_to_local);
    obstacles.extend(self_stacks(kin, state.base));
    let j1_now = kin.axes(&state.pose).0[0];
    let j1_at_pick = kin.j1_near(pick_pose.theta, j1_now);
    let j1_at_place = kin.j1_near(Pose::from_local(tops[0], 0.0).theta, j1_at_pick);
    let place_r = tops.iter().map(|t| radial(*t)).fold(0.0, f32::max);
    let over = |stack: Option<&Stack>, limit: f32, what: &str| -> Result<f32, Failure> {
        let Some(s) = stack else { return Ok(f32::MIN) };
        let need = s.top + STACK_CLEARANCE;
        if need > limit {
            return Err(Failure::Fault(format!(
                "{what}: precisa passar sobre o pallet a {:.0}° e {:.2} m de raio (topo a {:.2} m) a {:.2} m, e o braço só sobe até {:.2} m",
                s.theta.to_degrees(),
                s.r,
                s.top,
                need,
                limit + 0.02
            )));
        }
        Ok(need)
    };
    // The stack the robot is above now (it just placed there) is left upward.
    let here = state.pose.local_point();
    let leaving: Vec<Stack> = obstacles
        .iter()
        .filter(|s| {
            let center = Vec2::new(s.r * s.theta.cos(), -s.r * s.theta.sin());
            !(s.r > 0.0 && center.distance(Vec2::new(here.x, here.z)) < s.half && here.y >= s.top)
        })
        .map(|s| Stack { ..*s })
        .collect();
    let to_pick = over(highest_on_path(&leaving, (j1_now, state.pose.r), (j1_at_pick, radial(local_pick))), pick_ceiling, &format!("caminho até a coleta em {}", pick_eq.name))?;
    let to_place = over(highest_on_path(&obstacles, (j1_at_pick, radial(local_pick)), (j1_at_place, place_r)), ceiling, &format!("caminho até o {}", tdp_eq.name))?;
    if std::env::var("PDT_TRACE").is_ok() && to_place > safe_y {
        eprintln!("{}: transferência sobe para {to_place:.2} m (outro pallet no caminho; camada pedia {safe_y:.2} m)", tdp_eq.name);
    }
    let safe_y = safe_y.max(to_place);
    // Gripper yaw at the pick (any half turn of the group): the one whose J4/A6 turns
    // the least now and at the first placement (exact or half turn there), kept
    // toward the middle of its range.
    let j4_now = state.pose.yaw - state.pose.theta;
    let place_theta = Pose::from_local(tops[0], 0.0).theta;
    // Turn of the group from the pick to the first placement (world).
    let delta = yaw_of(pallet_tf.rotation) + drops[0].yaw - group_yaw;
    let place_step = if drops[0].exact { 2.0 * PI } else { PI };
    // J1 at the pick and at the place along the way the robot will turn: the flange
    // (and the fardos) turn by J1 + J4 in the world, so of two equal wrist turns the
    // one that does not spin the fardos a full turn is taken.
    let j1_pick = kin.j1_near(pick_pose.theta, kin.axes(&state.pose).0[0]);
    let j1_place = kin.j1_near(place_theta, j1_pick);
    let grip_yaw = (-8..=8)
        .map(|k| group_yaw - robot_yaw + k as f32 * PI)
        .filter(|g| kin.j4_ok(*g, pick_pose.theta))
        .filter_map(|g| {
            let j4_pick = g - pick_pose.theta;
            let place_cost = |j4: f32| {
                let spin = (j1_place + j4) - (j1_pick + j4_pick);
                (j4 - j4_pick).powi(2) + 0.25 * j4.powi(2) + 0.1 * spin.powi(2)
            };
            let j4_place = (-8..=8)
                .map(|k| g + delta + k as f32 * place_step)
                .filter(|y| kin.j4_ok(*y, place_theta))
                .map(|y| y - place_theta)
                .min_by(|a, b| place_cost(*a).total_cmp(&place_cost(*b)))?;
            // Squares: one large turn costs more than two medium ones.
            let cost = (j4_pick - j4_now).powi(2) + (j4_place - j4_pick).powi(2) + 0.5 * (j4_pick.powi(2) + j4_place.powi(2));
            Some((g, cost))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map_or_else(|| kin.wrist_yaw(&state.pose, group_yaw - robot_yaw, pick_pose.theta), |(g, _)| g);
    // The cabeçote at the pick and at each place: not into the cavalete nor the body.
    if let Some(what) = self_hit(kin, state.base, local_pick, grip_yaw, pick_pose.theta) {
        return Err(Failure::Fault(format!(
            "coleta em {}: o cabeçote bateria n{} (TCP a {:.2} m de raio e {:.2} m de altura)",
            pick_eq.name,
            what,
            radial(local_pick),
            local_pick.y
        )));
    }
    for (d, t) in drops.iter().zip(&tops) {
        let yaw = grip_yaw + yaw_of(pallet_tf.rotation) + d.yaw - group_yaw;
        if let Some(what) = self_hit(kin, state.base, *t, yaw, Pose::from_local(*t, 0.0).theta) {
            let slots: Vec<String> = d.slots.iter().map(|s| (s + 1).to_string()).collect();
            return Err(Failure::Fault(format!(
                "depósito no {}{}: o cabeçote bateria n{} (TCP a {:.2} m de raio e {:.2} m de altura): afaste o TDP do robô ou suba o robô",
                tdp_eq.name,
                if slots.is_empty() { String::new() } else { format!(" (posição {} do padrão)", slots.join(", ")) },
                what,
                radial(*t),
                t.y
            )));
        }
    }
    Ok(Plan {
        held,
        unit,
        group_yaw,
        pick_top,
        pallet: pallet_entity,
        drops,
        safe_y,
        above: Pose { y: safe_y.min(pick_ceiling).max(pick_pose.y).max(to_pick), yaw: grip_yaw, ..pick_pose },
        next_size,
    })
}

/// Perch (wait) pose of MOVE_PERCH: over the middle of the pick area, the
/// gripper aligned with the conveyor, high enough to clear the incoming fardos.
fn perch_pose(state: &RobotState, world_to_local: &Affine3A, robot_yaw: f32, (pick_eq, pick_tf): (&Equipment, &Transform), loads: &LoadQuery) -> Option<Pose> {
    let kin = &state.kin;
    let p = &pick_eq.params;
    let end = p.length / 2.0;
    let x = crate::conveyor::layout(pick_eq).pick_x0.map_or(end - 0.3, |x0| (x0 + end) / 2.0);
    let world = pick_tf.transform_point(Vec3::new(x, p.height + p.box_size[1] + 0.35, 0.0));
    let mut local = world_to_local.transform_point3(world);
    // Above the pallets on the way there.
    let theta = Pose::from_local(local, 0.0).theta;
    let j1_now = kin.axes(&state.pose).0[0];
    let mut all = stacks(loads, world_to_local);
    all.extend(self_stacks(kin, state.base));
    let on_path = highest_on_path(&all, (j1_now, state.pose.r), (kin.j1_near(theta, j1_now), radial(local)));
    if let Some(s) = on_path {
        local.y = local.y.max(s.top + STACK_CLEARANCE);
    }
    local.y = local.y.min(kin.max_tcp_height(radial(local)) - 0.02);
    let yaw = kin.wrist_yaw(&state.pose, yaw_of(pick_tf.rotation) - robot_yaw, theta);
    let pose = Pose::from_local(local, yaw);
    kin.solve(&pose).reachable.then_some(pose)
}

fn radial(local: Vec3) -> f32 {
    Vec2::new(local.x, local.z).length()
}

/// The robot reaches the pick point at the end of this conveyor (mesa coleta, belt end).
pub fn reaches_pick(kin: &Kin, robot_tf: &Transform, eq: &Equipment, tf: &Transform) -> bool {
    let end = tf.transform_point(Vec3::new(eq.params.length / 2.0 - 0.3, eq.params.height, 0.0));
    radial(robot_tf.compute_affine().inverse().transform_point3(end)) <= kin.reach
}

/// The robot reaches the pallet on this TDP.
pub fn reaches_tdp(kin: &Kin, robot_tf: &Transform, tf: &Transform) -> bool {
    radial(robot_tf.compute_affine().inverse().transform_point3(tf.translation)) <= kin.reach + 0.8
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
    mut traced: Local<HashMap<Entity, String>>,
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
        // Model with this robot's software limits.
        let kin = &kin(params.robot_model).with_limits(&params.axis_limits);
        if state.kin.model != kin.model {
            // Model changed: start over from its home position.
            *state = RobotState::new(kin.clone());
        } else {
            state.kin.min = kin.min;
            state.kin.max = kin.max;
        }
        state.base = params.robot_base.then(|| RobotBase {
            height: params.base_height,
            width: params.base_width,
            depth: params.base_depth,
            yaw: params.base_yaw.to_radians(),
        });
        let world_to_local = robot_tf.compute_affine().inverse();
        let robot_yaw = yaw_of(robot_tf.rotation);
        let in_reach = |p: Vec3, margin: f32| {
            let local = world_to_local.transform_point3(p);
            Vec2::new(local.x, local.z).length() <= kin.reach + margin
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
                let before = kin.to_motor(kin.axes(&state.pose));
                state.t += dt;
                state.pose = state.pose_at(state.t);
                let after = kin.to_motor(kin.axes(&state.pose));
                state.axis_speed = std::array::from_fn(|i| (after[i] - before[i]) / dt);
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
                    // Give the mesas the sizes of the picks of a pallet (receita 10x1: 4, 3, 3...):
                    // each mesa forms its groups in that order.
                    if let Some(&(_, tdp_eq, _)) = tdps.first()
                        && let Some(pattern) = tdp_eq.params.pattern.as_ref()
                        && !pattern.groups.is_empty()
                    {
                        let sizes: Vec<u32> = pattern.picks.iter().map(|k| k.iter().map(|d| d.len() as u32).sum()).collect();
                        for &(mesa, ..) in &picks {
                            if let Ok(mut logic) = mesas.get_mut(mesa)
                                && logic.size_cycle != sizes
                            {
                                logic.size_cycle = sizes.clone();
                                logic.group_index = 0;
                            }
                        }
                    }
                    let mut status: Option<&'static str> = None;
                    let mut faults: Vec<String> = Vec::new();
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
                            // Rotas: this mesa does not feed this TDP.
                            if !params.routed(&tdp_eq.name, &pick_eq.name) {
                                status.get_or_insert("Aguardando coleta da mesa roteada para o TDP");
                                continue;
                            }
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
                                Err(Failure::Wait(reason)) => {
                                    status.get_or_insert(reason);
                                }
                                Err(Failure::Fault(details)) => {
                                    status.get_or_insert("Falha: posição fora do alcance (ver log de falhas)");
                                    faults.push(details);
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
                        let active = std::mem::take(&mut state.active_faults);
                        for details in &faults {
                            // A fault is logged once while it lasts.
                            if !active.contains(details) {
                                if std::env::var("PDT_TRACE").is_ok() {
                                    eprintln!("[{:.2}] {}: FALHA {details}", time.elapsed_secs(), robot_eq.name);
                                }
                                state.faults.push((time.elapsed_secs(), details.clone()));
                                if state.faults.len() > 50 {
                                    state.faults.remove(0);
                                }
                            }
                        }
                        state.active_faults = faults;
                        // MOVE_PERCH: wait over the coleta of the next mesa, at the infeed height.
                        if !state.perched
                            && let Some(&(_, pick_eq, pick_tf)) = picks.first()
                            && let Some(perch) = perch_pose(&state, &world_to_local, robot_yaw, (pick_eq, pick_tf), &loads)
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
                        let drops: Vec<String> = plan
                            .drops
                            .iter()
                            .map(|d| if d.slots.is_empty() { "grade".to_owned() } else { format!("{:?}", d.slots) })
                            .collect();
                        eprintln!(
                            "[{:.2}] {}: coleta {} ({} fardo(s)) -> {} depósitos {}",
                            time.elapsed_secs(),
                            robot_eq.name,
                            name(pick_entity),
                            plan.held.len(),
                            name(tdp_entity),
                            drops.join(" ")
                        );
                    }
                    // Última camada ímpar: the next mesa group is a single fardo.
                    if plan.next_size == Some(1)
                        && let Some(i) = picks.iter().position(|p| p.0 == pick_entity)
                        && let Ok(mut next) = mesas.get_mut(picks[(i + 1) % picks.len()].0)
                    {
                        next.single_request = true;
                    }
                    claims.owner.insert(pick_entity, robot_entity);
                    claims.owner.insert(tdp_entity, robot_entity);
                    state.held_plan = plan.held;
                    state.unit = plan.unit;
                    state.group_yaw = plan.group_yaw;
                    state.pick_top = plan.pick_top;
                    state.safe_y = plan.safe_y;
                    state.active_faults.clear();
                    state.pick_conveyor = Some(pick_entity);
                    state.last_pick = Some(pick_entity);
                    state.tdp = Some(tdp_entity);
                    state.pallet = Some(plan.pallet);
                    state.drops = plan.drops;
                    state.drop_i = 0;
                    state.status = format!("{}: indo para a coleta", kin.program.pick);
                    state.step = Step::ToAbovePick;
                    state.move_to(plan.above, params.speed_override, Move::Joint);
                }
                Step::ToAbovePick => {
                    let local = world_to_local.transform_point3(state.pick_top);
                    let target = Pose { yaw: state.pose.yaw, ..Pose::from_local(local, 0.0) };
                    state.step = Step::DownPick;
                    state.status = format!("{}: descendo para a coleta", kin.program.pick);
                    state.move_to(target, params.speed_override, Move::Linear(kin.program.pick_down)); // PICK: L PR[3] 3000mm/sec / MovePick SLIN 2.5
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
                    let y = state.safe_y.min(kin.max_tcp_height(state.pose.r) - 0.02).max(state.pose.y);
                    let up = Pose { y, ..state.pose };
                    state.move_to(up, params.speed_override, Move::Linear(kin.program.pick_up)); // PICK: L PR[11] 1800mm/sec / MovePlace SLIN 3.0
                }
                Step::UpPick | Step::ToAbovePlace | Step::Lift => {
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
                    let (Some(pallet_entity), Some(drop)) = (state.pallet, state.drops.get(state.drop_i).cloned()) else {
                        state.step = Step::Idle;
                        break 'logic;
                    };
                    let Ok((_, _, pallet_tf)) = loads.get(pallet_entity) else {
                        state.status = "Pallet saiu durante o ciclo".into();
                        state.step = Step::Idle;
                        break 'logic;
                    };
                    let pallet_tf = *pallet_tf;
                    let local = world_to_local.transform_point3(pallet_tf.transform_point(drop.top));
                    let group_world_yaw = yaw_of(pallet_tf.rotation) + drop.yaw;
                    if matches!(state.step, Step::UpPick | Step::Lift) {
                        let above = Pose::from_local(local, 0.0);
                        let target_yaw = group_world_yaw - state.hold_yaw - robot_yaw;
                        let yaw = if drop.exact {
                            kin.wrist_yaw_exact(&state.pose, target_yaw, above.theta)
                        } else {
                            kin.wrist_yaw(&state.pose, target_yaw, above.theta)
                        };
                        if state.step == Step::UpPick {
                            let target = Pose { y: state.safe_y, yaw, ..above };
                            state.status = format!("{}: transportando para o pallet", kin.program.place);
                            state.move_to(target, params.speed_override, Move::Joint); // PLACE: J PR[11] 100%
                        } else {
                            // Next drop of the same pick: turn and move just above it.
                            let y = state.pose.y.max(local.y + 0.06).min(kin.max_tcp_height(above.r) - 0.02);
                            state.status = "Girando para o outro lado da garra".into();
                            state.move_to(Pose { y, yaw, ..above }, params.speed_override, Move::Linear(1.0));
                        }
                        state.step = Step::ToAbovePlace;
                    } else {
                        let target = Pose { yaw: state.pose.yaw, ..Pose::from_local(local, 0.0) };
                        state.step = Step::DownPlace;
                        state.status = format!("{}: depositando", kin.program.place);
                        state.move_to(target, params.speed_override, Move::Linear(kin.program.place_down)); // PLACE: L PR[5] 2000mm/sec / MovePlace SLIN 2.5
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
                    if std::env::var("PDT_TRACE").is_ok()
                        && let Some(drop) = state.drops.get(state.drop_i)
                    {
                        // Diagnostic: the clamp that opens must be the one over the slot.
                        let hold_rot = Quat::from_rotation_y(state.hold_yaw);
                        let tcp = robot_tf.transform_point(state.pose.local_point());
                        let sides: Vec<String> = drop
                            .boxes
                            .iter()
                            .filter_map(|(e, ..)| state.held.iter().find(|h| h.entity == *e))
                            .map(|h| {
                                let z = (hold_rot * h.offset).z;
                                let visual = loads.get(h.entity).map_or(Vec3::ZERO, |(_, _, t)| t.translation);
                                let target = drop.boxes.iter().find(|b| b.0 == h.entity).map(|b| b.1).unwrap_or_default();
                                let target = state.pallet.and_then(|e| loads.get(e).ok()).map_or(target, |(_, _, ptf)| ptf.transform_point(target));
                                let miss = Vec2::new(visual.x - target.x, visual.z - target.z).length();
                                format!("garra {} (erro {:.0} mm)", if z < 0.0 { "A" } else { "B" }, miss * 1000.0)
                            })
                            .collect();
                        let _ = tcp;
                        eprintln!("[{:.2}] {}: abre {}", time.elapsed_secs(), robot_eq.name, sides.join(", "));
                    }
                    if let (Some(pallet_entity), Some(drop)) = (state.pallet, state.drops.get(state.drop_i).cloned()) {
                        // The fardos let go become part of the pallet load and travel with it.
                        for &(entity, pos, yaw) in &drop.boxes {
                            if let Ok((_, mut b, mut box_tf)) = loads.get_mut(entity) {
                                b.state = LoadState::Placed(pallet_entity);
                                *box_tf = Transform::from_translation(pos).with_rotation(Quat::from_rotation_y(yaw));
                                commands.entity(entity).insert(ChildOf(pallet_entity));
                            }
                        }
                        state.held.retain(|h| !drop.boxes.iter().any(|(e, ..)| *e == h.entity));
                        if let Ok(mut pallet) = pallet_loads.get_mut(pallet_entity) {
                            pallet.placed += drop.boxes.len() as u32;
                            pallet.units += drop.units;
                            pallet.filled.extend(&drop.slots);
                            if drop.slots.is_empty() {
                                pallet.unit_size.get_or_insert(state.unit);
                            }
                        }
                    }
                    state.drop_i += 1;
                    if state.drop_i < state.drops.len() && !state.held.is_empty() {
                        // One clamp opened: lift a little, then the other product.
                        state.step = Step::Lift;
                        state.status = "Abriu um lado da garra".into();
                        // Up until the clamps (and the fardo still held) clear the products
                        // already placed and the next slot, before moving sideways.
                        let next_top = match (state.pallet.and_then(|e| loads.get(e).ok()), state.drops.get(state.drop_i)) {
                            (Some((_, _, ptf)), Some(next)) => world_to_local.transform_point3(ptf.transform_point(next.top)),
                            _ => state.pose.local_point(),
                        };
                        let held_h = state.held.iter().map(|b| b.height).fold(0.0, f32::max);
                        // The TCP is the lowest point (clamp tips, bottom of the held fardo):
                        // it must pass above the fardo just placed and the next slot.
                        let clear = (state.pose.y + held_h).max(next_top.y) + 0.05;
                        let ceiling = kin.max_tcp_height(state.pose.r).min(kin.max_tcp_height(radial(next_top))) - 0.02;
                        let y = clear.min(ceiling).max(state.pose.y);
                        let up = Pose { y, ..state.pose };
                        state.move_to(up, params.speed_override, Move::Linear(1.0));
                    } else {
                        state.cycles += 1;
                        state.step = Step::UpPlace;
                        let up = Pose { y: state.safe_y.max(state.pose.y), ..state.pose };
                        state.move_to(up, params.speed_override, Move::Linear(kin.program.place_up)); // PLACE: L PR[11] 4000mm/sec / MovePick SLIN 3.0
                    }
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
                    box_tf.translation = tcp + Vec3::Y * item.height / 2.0 + rot * item.offset;
                    box_tf.rotation = Quat::from_rotation_y(group_yaw + item.rel_yaw);
                }
            }
            // Diagnostic (PDT_TRACE): the carried fardos or the cabeçote (frame, clamps)
            // going through pallets or fardos already placed on any pallet.
            if std::env::var("PDT_TRACE").is_ok() && state.step != Step::Idle {
                let mut solids: Vec<(&str, Vec3, Vec3, f32)> = state
                    .held
                    .iter()
                    .filter_map(|h| loads.get(h.entity).ok().map(|(_, l, t)| ("fardo da garra", t.translation, l.size / 2.0, yaw_of(t.rotation))))
                    .collect();
                let grip_yaw = robot_yaw + state.pose.yaw;
                let rot = Quat::from_rotation_y(grip_yaw);
                let frame = robot_tf.transform_point(state.pose.local_point() + Vec3::Y * JAW_HEIGHT);
                solids.push(("quadro do cabeçote", frame + Vec3::Y * 0.04, Vec3::new(0.25, 0.04, 0.617), grip_yaw));
                let closed = state.unit.z / 2.0 + JAW_GAP;
                let half = |jaw: f32| state.jaw_half(closed, jaw);
                for (name, side) in [("pinça A", -half(state.jaw[0])), ("pinça B", half(state.jaw[1]))] {
                    solids.push((name, frame + rot * Vec3::new(0.0, -JAW_HEIGHT / 2.0, side), Vec3::new(0.25, JAW_HEIGHT / 2.0, 0.006), grip_yaw));
                }
                // The robot's own cavalete and body (turned with J1).
                let mut own: Vec<(&str, Vec3, Vec3, f32)> = Vec::new();
                if let Some(b) = state.base {
                    own.push(("o cavalete", robot_tf.transform_point(Vec3::new(0.0, -b.height / 2.0, 0.0)), Vec3::new(b.depth / 2.0, b.height / 2.0, b.width / 2.0), robot_yaw + b.yaw));
                }
                let j1 = state.pose.theta;
                let along = (state.kin.body_front - state.kin.body_back) / 2.0;
                own.push((
                    "o corpo do robô",
                    robot_tf.transform_point(Vec3::new(along * j1.cos(), state.kin.body_top / 2.0, -along * j1.sin())),
                    Vec3::new((state.kin.body_front + state.kin.body_back) / 2.0, state.kin.body_top / 2.0, state.kin.body_half_width),
                    robot_yaw + j1,
                ));
                for (name, c, h, y) in solids.iter().filter(|s| s.0 != "pinça A" && s.0 != "pinça B") {
                    for (what, oc, oh, oy) in &own {
                        if let Some(depth) = obb_depth(*c, *h, *y, *oc, *oh, *oy)
                            && depth > 0.005
                        {
                            eprintln!("[{:.2}] {}: COLISÃO do {name} com {what} ({:.0} mm, {:?})", time.elapsed_secs(), robot_eq.name, depth * 1000.0, state.step);
                        }
                    }
                }
                let held: Vec<Entity> = state.held.iter().map(|h| h.entity).collect();
                for (entity, l, t) in loads.iter() {
                    let (pos, yaw) = match l.state {
                        LoadState::Placed(pallet) => match loads.get(pallet) {
                            Ok((_, _, ptf)) => (ptf.transform_point(t.translation), yaw_of(ptf.rotation) + yaw_of(t.rotation)),
                            Err(_) => continue,
                        },
                        LoadState::Free if l.kind == LoadKind::Pallet => (t.translation, yaw_of(t.rotation)),
                        _ => continue,
                    };
                    if held.contains(&entity) {
                        continue;
                    }
                    for (name, c, h, y) in &solids {
                        if let Some(depth) = obb_depth(*c, *h, *y, pos, l.size / 2.0, yaw)
                            && depth > 0.005
                        {
                            let what = if l.kind == LoadKind::Pallet { "o pallet" } else { "um fardo do pallet" };
                            eprintln!("[{:.2}] {}: COLISÃO do {name} com {what} ({:.0} mm, {:?})", time.elapsed_secs(), robot_eq.name, depth * 1000.0, state.step);
                            if std::env::var("PDT_DBG").is_ok() {
                                eprintln!("DBG solid {c:.3} half {h:.3} yaw {:.1} | fardo {pos:.3} half {:.3} yaw {:.1} | unit {:.3} jaw {:?} held {}", y.to_degrees(), l.size / 2.0, yaw.to_degrees(), state.unit, state.jaw, state.held.len());
                            }
                        }
                    }
                }
            }
        }

        // Pinças: each side closes on its fardo while gripping and opens on its own drop.
        let hold_rot = Quat::from_rotation_y(state.hold_yaw);
        let gripping = state.step == Step::Grip;
        let boxes = if gripping { &state.held_plan } else { &state.held };
        let on_side = |side: f32| {
            boxes.iter().any(|b| {
                let z = (if gripping { b.offset } else { hold_rot * b.offset }).z;
                z.abs() < 0.02 || z.signum() == side
            })
        };
        let closed = [on_side(-1.0), on_side(1.0)];
        let step = dt / 0.25;
        for (jaw, closed) in state.jaw.iter_mut().zip(closed) {
            *jaw += ((if closed { 1.0 } else { 0.0 }) - *jaw).clamp(-step, step);
        }

        // PDT_TRACE: the status, when it changes.
        if std::env::var("PDT_TRACE").is_ok() && traced.get(&robot_entity) != Some(&state.status) {
            eprintln!("[{:.2}] {}: estado {}", time.elapsed_secs(), robot_eq.name, state.status);
            traced.insert(robot_entity, state.status.clone());
        }
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

/// A robot on the cavalete stands on its top: the robot base height is the cavalete
/// height (the robot program uses it as the mounting height above the floor).
pub fn mount_on_base(mut robots: Query<(&Equipment, &mut Transform), With<RobotState>>) {
    for (eq, mut tf) in &mut robots {
        if eq.params.robot_base && (tf.translation.y - eq.params.base_height).abs() > 1e-4 {
            tf.translation.y = eq.params.base_height;
        }
    }
}

pub fn animate_robots(
    robots: Query<&RobotState>,
    mut parts: Query<(&RobotPart, &ChildOf, &mut Transform), Without<CadPart>>,
    mut cad: Query<(&CadPart, &ChildOf, &mut Transform), Without<RobotPart>>,
) {
    for (part, child_of, mut tf) in &mut cad {
        let Ok(state) = robots.get(child_of.parent()) else { continue };
        let joints = state.kin.solve(&state.pose);
        *tf = part.transform(&joints, state.pose.yaw - joints.j1);
    }
    for (part, child_of, mut tf) in &mut parts {
        let Ok(state) = robots.get(child_of.parent()) else { continue };
        let (kin, pose) = (&state.kin, state.pose);
        let joints = kin.solve(&pose);
        let j1 = joints.j1;
        let u = joints.upper_dir;
        let upper_angle = u.y.atan2(u.x);
        *tf = match part {
            RobotPart::Turntable => Transform::from_rotation(Quat::from_rotation_y(j1)),
            RobotPart::LowerArm => Transform {
                translation: kin.plane_to_local(j1, joints.elbow / 2.0),
                rotation: link_rotation(j1, joints.shoulder),
                ..default()
            },
            RobotPart::UpperArm => Transform {
                translation: kin.plane_to_local(j1, joints.elbow + u * (kin.l2 - kin.upper_back) / 2.0),
                rotation: link_rotation(j1, upper_angle),
                ..default()
            },
            RobotPart::Rod => Transform {
                translation: kin.plane_to_local(j1, joints.elbow / 2.0 - u * 0.35),
                rotation: link_rotation(j1, joints.shoulder),
                ..default()
            },
            RobotPart::Wrist => Transform {
                translation: kin.plane_to_local(j1, joints.elbow + u * kin.l2 + Vec2::X * kin.wrist_fwd),
                rotation: Quat::from_rotation_y(pose.yaw),
                ..default()
            },
            RobotPart::Gripper | RobotPart::JawA | RobotPart::JawB => {
                let gripper = Transform {
                    translation: kin.plane_to_local(j1, joints.elbow + u * kin.l2 + Vec2::X * kin.wrist_fwd) - Vec3::Y * kin.frame_drop(),
                    rotation: Quat::from_rotation_y(pose.yaw),
                    ..default()
                };
                // Clamps close onto the picked group (its width along the gripper Z).
                let closed = state.unit.z / 2.0 + JAW_GAP;
                let half = |jaw: f32| state.jaw_half(closed, jaw);
                match part {
                    RobotPart::JawA => gripper * Transform::from_xyz(0.0, -JAW_HEIGHT / 2.0, -half(state.jaw[0])),
                    RobotPart::JawB => gripper * Transform::from_xyz(0.0, -JAW_HEIGHT / 2.0, half(state.jaw[1])),
                    _ => gripper,
                }
            }
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MODELS: [&Kin; 2] = [&FANUC, &KUKA];

    #[test]
    fn ik_reaches_target() {
        for kin in MODELS {
            let pose = Pose { theta: 0.7, r: 2.2, y: 1.0, yaw: 0.0 };
            let j = kin.solve(&pose);
            assert!(j.reachable, "{:?}", kin.model);
            let wrist = kin.plane_to_local(j.j1, j.elbow + j.upper_dir * kin.l2 + Vec2::X * kin.wrist_fwd);
            let tcp = wrist - Vec3::Y * kin.tcp_drop();
            assert!((tcp - pose.local_point()).length() < 1e-3, "{tcp:?} vs {:?}", pose.local_point());
        }
    }

    #[test]
    fn cad_rest_pose_is_identity() {
        // Poses whose IK gives the STEP rest pose: lower arm vertical, upper arm horizontal.
        for (kin, wrist) in [(&FANUC, rest::WRIST), (&KUKA, kuka_rest::A5)] {
            let tcp_plane = wrist + Vec2::new(kin.wrist_fwd, -kin.tcp_drop());
            let pose = Pose { theta: 0.0, r: tcp_plane.x, y: tcp_plane.y, yaw: 0.0 };
            let joints = kin.solve(&pose);
            assert!(joints.reachable, "{:?}", kin.model);
            for (part, _) in CadPart::of(kin.model).0 {
                let tf = part.transform(&joints, 0.0);
                assert!(tf.translation.length() < 1e-3, "{part:?} {:?}", tf.translation);
                assert!(tf.rotation.angle_between(Quat::IDENTITY) < 1e-3, "{part:?}");
            }
        }
    }

    #[test]
    fn kuka_rest_pivots_match_kinematics() {
        assert!((kuka_rest::A2 - Vec2::new(KUKA.j2_offset, KUKA.base_height)).length() < 1e-6);
        assert!((kuka_rest::A3 - kuka_rest::A2 - Vec2::Y * KUKA.l1).length() < 1e-6);
        assert!((kuka_rest::A5 - kuka_rest::A3 - Vec2::X * KUKA.l2).length() < 1e-6);
        assert!((kuka_rest::FLANGE_X - kuka_rest::A5.x - KUKA.wrist_fwd).abs() < 1e-6);
        // Rest pose: A2 -90°, A3 +90° (data sheet convention).
        let m = KUKA.to_motor(Axes([0.0, 0.0, 0.0, 0.0]));
        assert!((m[1].to_degrees() + 90.0).abs() < 1e-4 && (m[2].to_degrees() - 90.0).abs() < 1e-4, "{m:?}");
    }

    /// Envelope of the flange center (top, bottom, reach) from the outline.
    fn envelope(kin: &Kin) -> (f32, f32, f32) {
        let points: Vec<Vec2> = kin.envelope_outline(800).into_iter().flatten().collect();
        let top = points.iter().map(|p| p.y).fold(f32::MIN, f32::max);
        let bottom = points.iter().map(|p| p.y).fold(f32::MAX, f32::min);
        let reach = points.iter().map(|p| p.x).fold(f32::MIN, f32::max);
        (top, bottom, reach)
    }

    #[test]
    fn envelope_matches_data_sheet() {
        for kin in MODELS {
            let (top, bottom, reach) = envelope(kin);
            assert!((top - kin.max_height).abs() < 0.01, "{:?} top {top}", kin.model);
            assert!((bottom - kin.min_height).abs() < 0.03, "{:?} bottom {bottom}", kin.model);
            assert!((reach - kin.reach).abs() < 0.08, "{:?} reach {reach}", kin.model);
        }
        // FANUC: inner radius where the envelope crosses the floor.
        let inner = FANUC
            .envelope_outline(400)
            .iter()
            .flat_map(|l| l.windows(2).filter(|w| (w[0].y >= 0.0) != (w[1].y >= 0.0)).map(|w| w[0].x).collect::<Vec<_>>())
            .fold(f32::MAX, f32::min);
        assert!((inner - FANUC.min_radius).abs() < 0.03, "{inner}");
        // KUKA: maximum reach 3195 mm with the flange 513 mm above the floor (STEP).
        let far = KUKA.pose(Axes([0.0, 90.0f32.to_radians() - 0.0, 0.0, 0.0]));
        let _ = far;
        let stretched = KUKA.j2_offset + KUKA.l1 + KUKA.l2 + KUKA.wrist_fwd;
        assert!((stretched - KUKA.reach).abs() < 0.01, "{stretched}");
        assert!((KUKA.base_height - KUKA.wrist_drop - 0.395).abs() < 0.01);
    }

    #[test]
    fn height_limit_follows_envelope() {
        for kin in MODELS {
            // At the radius of the top of the envelope the flange reaches it.
            let r = (0..=400)
                .map(|k| 0.5 + 3.0 * k as f32 / 400.0)
                .max_by(|a, b| kin.max_tcp_height(*a).total_cmp(&kin.max_tcp_height(*b)))
                .unwrap();
            let top = kin.max_tcp_height(r) + TOOL_LENGTH + JAW_HEIGHT;
            assert!((top - kin.max_height).abs() < 0.02, "{:?} {top}", kin.model);
            // Far out the arm cannot rise as much.
            assert!(kin.max_tcp_height(3.0) < kin.max_tcp_height(2.0));
            assert!(kin.max_tcp_height(4.0) == f32::NEG_INFINITY);
        }
    }

    #[test]
    fn kuka_limits_follow_data_sheet() {
        // A2 -140° (leaning back 50°) and A3 165° are the ends of their ranges.
        assert!(KUKA.within_limits(Axes([0.0, -50f32.to_radians(), -10f32.to_radians(), 0.0])));
        assert!(!KUKA.within_limits(Axes([0.0, -55f32.to_radians(), 0.0, 0.0])));
        // The arm rises at most 60° above the horizontal.
        assert!(!KUKA.within_limits(Axes([0.0, 0.0, 65f32.to_radians(), 0.0])));
        // A6 ±350°.
        assert!(!KUKA.within_limits(Axes([0.0, 0.0, 0.0, 355f32.to_radians()])));
        assert!(KUKA.reach_report(Vec3::new(4.0, 1.0, 0.0), 0.0).is_some_and(|r| r.contains("longe demais")));
        // On a 0.8 m pedestal the arm works below its base (data sheet: down to -742 mm).
        let below_base = Vec3::new(2.0, -0.15, 0.0);
        assert!(KUKA.reach_report(below_base, -0.8).is_none());
        assert!(KUKA.reach_report(below_base, 0.0).is_some_and(|r| r.contains("abaixo do piso")));
    }

    #[test]
    fn software_limits() {
        // A1 limited to ±120°: a point behind the robot is out of reach.
        let kin = KUKA.with_limits(&[[-120.0, 120.0]]);
        let behind = Vec3::new(-2.0, 0.5, 0.5);
        assert!(KUKA.solve(&Pose::from_local(behind, 0.0)).reachable);
        assert!(!kin.solve(&Pose::from_local(behind, 0.0)).reachable);
        assert!(kin.reach_report(behind, -1.0).is_some_and(|r| r.contains("A1 precisaria")));
        // Never wider than the data sheet.
        let wide = KUKA.with_limits(&[[-300.0, 300.0]]);
        assert!((wide.max[0] - KUKA.max[0]).abs() < 1e-6);
        // KUKA A1 ±185°: at 178° the arm may come from -182° (short way from -170°).
        let j1 = KUKA.j1_near(178f32.to_radians(), -170f32.to_radians());
        assert!((j1.to_degrees() + 182.0).abs() < 0.01, "{}", j1.to_degrees());
        // FANUC J1 ±180°: no overlap, the long way.
        let j1 = FANUC.j1_near(178f32.to_radians(), -170f32.to_radians());
        assert!((j1.to_degrees() - 178.0).abs() < 0.01);
    }

    #[test]
    fn cabecote_against_cavalete_and_body() {
        let base = Some(RobotBase { height: 1.15, width: 1.3, depth: 1.85, yaw: 0.0 });
        // Long side of the cabeçote toward the robot (yaw 90°).
        let radial_yaw = FRAC_PI_2;
        // Below the base and right next to it: hits the cavalete.
        assert_eq!(self_hit(&KUKA, base, Vec3::new(1.21, -0.5, 0.0), radial_yaw, 0.0), Some("o cavalete"));
        // The narrow side toward the robot clears it at the same point.
        assert_eq!(self_hit(&KUKA, base, Vec3::new(1.21, -0.5, 0.0), 0.0, 0.0), None);
        // Far enough out, or above the base: clear.
        assert_eq!(self_hit(&KUKA, base, Vec3::new(1.8, -0.5, 0.0), radial_yaw, 0.0), None);
        assert_eq!(self_hit(&KUKA, base, Vec3::new(1.21, 1.0, 0.0), radial_yaw, 0.0), None);
        // Close to the column, below the A2 hub: hits the body.
        assert_eq!(self_hit(&KUKA, None, Vec3::new(1.0, 0.5, 0.0), radial_yaw, 0.0), Some("o corpo do robô (coluna)"));
        // The path near the robot passes above the cavalete and the body.
        let own = self_stacks(&KUKA, base);
        // From 1.0 m out the column is in the way (the cavalete is lower).
        let top = highest_on_path(&own, (0.0, 1.0), (1.0, 2.0)).map(|s| s.top);
        assert_eq!(top, Some(KUKA.body_top));
        assert!(highest_on_path(&own, (0.0, 2.6), (1.0, 2.6)).is_none());
    }

    #[test]
    fn home_is_reachable() {
        for kin in MODELS {
            assert!(kin.solve(&kin.home).reachable, "{:?}", kin.model);
        }
    }

    #[test]
    fn axes_round_trip() {
        for kin in MODELS {
            let pose = Pose { theta: -1.2, r: 2.4, y: 0.8, yaw: 0.5 };
            let back = kin.pose(kin.axes(&pose));
            assert!((back.local_point() - pose.local_point()).length() < 1e-3);
            assert!((back.yaw - pose.yaw).abs() < 1e-5);
        }
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
