//! Kinematic material flow. Loads (boxes and pallets) ride the driven segments
//! of conveyors, queue without overlapping, stop at raised stops (bloqueios,
//! batentes) and at line ends, are pushed sideways by mesa pushers/grippers and
//! fall when unsupported. Boxes placed by the robot become children of their
//! pallet, so a full pallet carries its load along the TDPs.

use std::f32::consts::FRAC_PI_2;

use bevy::{math::Affine3A, prelude::*};

use crate::conveyor::{self, ConveyorLayout, Surface};
use crate::shuttle::{ShuttleBody, ShuttleState};
use crate::model::{Equipment, EquipmentKind, Io, Params, SimControl};
use crate::visuals::{self, ActuatorPlate, BeltMark, Palette, Roller, StopBar, BELT_MARK_SPACING};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadKind {
    Box,
    Pallet,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadState {
    Free,
    Held,
    /// Stacked on the given pallet (and child of it).
    Placed(Entity),
}

#[derive(Component, Debug)]
pub struct Load {
    /// [length X, height Y, width Z] in the load's own frame.
    pub size: Vec3,
    pub kind: LoadKind,
    pub state: LoadState,
    vy: f32,
    /// Lifted by a mesa comb (garra): the rollers do not move it meanwhile.
    carried: bool,
    /// Held in place by a colocador de papelão while it works.
    pub hold: bool,
}

/// What is stacked on a pallet load.
#[derive(Component, Debug, Default)]
pub struct PalletLoad {
    /// Boxes on the pallet.
    pub placed: u32,
    /// Placements: picks with the automatic grid, products with a pattern.
    pub units: u32,
    /// Pattern slots already filled.
    pub filled: Vec<u32>,
    /// Footprint of one placement, fixed by the first one.
    pub unit_size: Option<Vec3>,
    /// Cardboard sheets laid on the pallet floor.
    pub sheets: u32,
    /// Released by the colocador de papelão (FimDoCiclo).
    pub papered: bool,
}

/// State of a TDP de paletização: the pallet held at its stop.
#[derive(Component, Debug, Default)]
pub struct PalletizerState {
    pub pallet: Option<Entity>,
    pub placed: u32,
    pub units: u32,
    pub capacity: u32,
    pub full: bool,
}

#[derive(Component, Default)]
pub struct EmitterState {
    timer: f32,
}

#[derive(Component, Default)]
pub struct ConveyorState {
    /// Accumulated travel of each segment; drives the belt/roller animation.
    pub travel: [f32; 3],
}

/// Position of each mesa pusher/gripper: `stroke` 0 = retracted .. 1 = advanced,
/// `lift` 0 = lowered .. 1 = raised (always 1 for plain pushers).
#[derive(Component, Default, Debug)]
pub struct MesaMech {
    pub stroke: [f32; 2],
    pub lift: [f32; 2],
}

const GRAVITY: f32 = 9.81;
const PUSHER_TIME: f32 = 0.8;
const LIFT_TIME: f32 = 0.4;

fn scaled_dt(time: &Time, control: &SimControl) -> f32 {
    if control.running {
        time.delta_secs() * control.time_scale
    } else {
        0.0
    }
}

/// Half extent of a footprint along a horizontal world direction.
fn half_extent_along(rot: Quat, size: Vec3, dir: Vec3) -> f32 {
    let x = rot * Vec3::X;
    let z = rot * Vec3::Z;
    x.dot(dir).abs() * size.x / 2.0 + z.dot(dir).abs() * size.z / 2.0
}

struct Conveyor {
    entity: Entity,
    affine: Affine3A,
    inverse: Affine3A,
    rotation: Quat,
    length: f32,
    width: f32,
    top: f32,
    direction: f32,
    /// Pallet conveyors (TDP, CVZ) hold one pallet: a load only crosses onto
    /// them when they are empty.
    single_load: bool,
    /// CVZ travelling: no transfer onto or off the car until it stops.
    moving: bool,
    /// (x0, x1, speed) of each driven segment.
    segments: Vec<(f32, f32, f32)>,
    /// Local x of raised stops.
    barriers: Vec<f32>,
}

impl Conveyor {
    fn contains(&self, world: Vec3, margin: f32) -> bool {
        let l = self.inverse.transform_point3(world);
        l.x.abs() <= self.length / 2.0 + margin && l.z.abs() <= self.width / 2.0 + margin
    }

    fn top_world_y(&self) -> f32 {
        self.affine.transform_point3(Vec3::new(0.0, self.top, 0.0)).y
    }

    fn segment_at(&self, world: Vec3) -> Option<usize> {
        let x = self.inverse.transform_point3(world).x;
        self.segments.iter().position(|s| x >= s.0 && x < s.1).or_else(|| {
            // Within the gap tolerance past either end: the end roller drives.
            if x >= self.length / 2.0 {
                Some(self.segments.len() - 1)
            } else if x <= -self.length / 2.0 {
                Some(0)
            } else {
                None
            }
        })
    }
}

/// True if the load rests at the discharge end (+X) of the conveyor.
pub fn at_conveyor_end(conveyor: &Equipment, conveyor_tf: &Transform, size: Vec3, load_tf: &Transform) -> bool {
    let inverse = conveyor_tf.compute_affine().inverse();
    let local = inverse.transform_point3(load_tf.translation);
    let dir = conveyor_tf.rotation * Vec3::X;
    let half = half_extent_along(load_tf.rotation, size, dir);
    let bottom = local.y - size.y / 2.0;
    local.x >= conveyor.params.length / 2.0 - half - 0.04
        && local.x <= conveyor.params.length / 2.0 + 0.02
        && local.z.abs() <= conveyor.params.width / 2.0
        && (bottom - conveyor.params.height).abs() < 0.05
}

/// True if the load rests in the pick area of a mesa (coleta), or at the end of a belt.
pub fn in_pick_area(conveyor: &Equipment, conveyor_tf: &Transform, size: Vec3, load_tf: &Transform) -> bool {
    match conveyor::layout(conveyor).pick_x0 {
        Some(x0) => {
            // Resting against the batente at the end of the coleta.
            let local = conveyor_tf.compute_affine().inverse().transform_point3(load_tf.translation);
            let half = half_extent_along(load_tf.rotation, size, conveyor_tf.rotation * Vec3::X);
            local.x >= x0
                && local.x + half >= conveyor.params.length / 2.0 - 0.05
                && local.x <= conveyor.params.length / 2.0
                && local.z.abs() <= conveyor.params.width / 2.0
                && (local.y - size.y / 2.0 - conveyor.params.height).abs() < 0.05
        }
        None => at_conveyor_end(conveyor, conveyor_tf, size, load_tf),
    }
}

#[allow(clippy::type_complexity)]
pub fn move_loads(
    time: Res<Time>,
    control: Res<SimControl>,
    mut conveyors: Query<(Entity, &Equipment, &Transform, &Io, &mut ConveyorState, Option<&ShuttleState>), Without<Load>>,
    mut loads: Query<(Entity, &mut Load, &mut Transform), Without<Equipment>>,
) {
    let dt = scaled_dt(&time, &control);
    if dt <= 0.0 {
        return;
    }
    let list: Vec<Conveyor> = conveyors
        .iter_mut()
        .filter(|(_, e, ..)| e.kind.is_conveyor())
        .map(|(entity, e, tf, io, mut state, car)| {
            let layout: ConveyorLayout = conveyor::layout(e);
            let surface = conveyor::surface(e, tf, car);
            let direction = match e.kind {
                EquipmentKind::BeltConveyor | EquipmentKind::PalletConveyor if io.get_bool("reverse") => -1.0,
                EquipmentKind::ShuttleCar if io.get_bool("Reverso_TDP") => -1.0,
                EquipmentKind::FullCar if io.get("VelocidadeTDP") < 0.0 => -1.0,
                _ => 1.0,
            };
            // The CCH TDP runs at its inverter speed, with the brake released.
            let speed = if e.kind == EquipmentKind::FullCar {
                if io.get_bool("LiberarFreioTDP") { io.get("VelocidadeTDP").abs() as f32 / 1000.0 } else { 0.0 }
            } else {
                e.params.speed
            };
            let segments: Vec<(f32, f32, f32)> = layout
                .segments
                .iter()
                .map(|s| (s.x0, s.x1, if io.get_bool(s.motor) { speed } else { 0.0 }))
                .collect();
            for (i, s) in segments.iter().enumerate().take(3) {
                state.travel[i] += s.2 * direction * dt;
            }
            let affine = surface.transform.compute_affine();
            Conveyor {
                entity,
                inverse: affine.inverse(),
                affine,
                rotation: surface.transform.rotation,
                length: surface.length,
                width: surface.width,
                top: surface.top,
                direction,
                moving: car.is_some_and(|c| c.v.abs() > 1e-3 || c.giro_v != 0.0),
                single_load: matches!(
                    e.kind,
                    EquipmentKind::PalletConveyor
                        | EquipmentKind::PalletizingConveyor
                        | EquipmentKind::ShuttleCar
                        | EquipmentKind::FullCar
                ),
                segments,
                barriers: layout.barriers.iter().filter(|b| io.get_bool(b.key)).map(|b| b.x).collect(),
            }
        })
        .collect();

    // Snapshot of free loads, processed front-most first so queues advance together.
    let mut snapshot: Vec<(Entity, Vec3, Quat, Vec3)> = loads
        .iter()
        .filter(|(_, l, _)| l.state == LoadState::Free && !l.carried)
        .map(|(e, l, tf)| (e, tf.translation, tf.rotation, l.size))
        .collect();
    let support_of = |pos: Vec3, size: Vec3| -> Option<usize> {
        let bottom = pos.y - size.y / 2.0;
        // The conveyor under the load center; a 5 cm tolerance bridges small gaps
        // between conveyors placed end to end.
        let best = |margin: f32| {
            list.iter()
                .enumerate()
                .filter(|(_, c)| c.contains(pos, margin) && c.top_world_y() <= bottom + 0.04)
                .max_by(|a, b| a.1.top_world_y().total_cmp(&b.1.top_world_y()))
                .map(|(i, _)| i)
        };
        best(0.0).or_else(|| best(0.05))
    };
    let key = |pos: Vec3, size: Vec3| -> f32 {
        support_of(pos, size)
            .map(|i| (list[i].rotation * Vec3::X * list[i].direction).dot(pos))
            .unwrap_or(f32::MIN)
    };
    snapshot.sort_by(|a, b| key(b.1, b.3).total_cmp(&key(a.1, a.3)));

    for i in 0..snapshot.len() {
        let (entity, mut pos, rot, size) = snapshot[i];
        let support = support_of(pos, size);
        let floor = support.map_or(0.0, |s| list[s].top_world_y());
        let bottom = pos.y - size.y / 2.0;
        let Ok((_, mut load, mut tf)) = loads.get_mut(entity) else { continue };
        if load.hold {
            // Stopped under the colocador de papelão; still blocks the queue behind.
            continue;
        }
        if bottom > floor + 0.001 {
            load.vy -= GRAVITY * dt;
            pos.y = (pos.y + load.vy * dt).max(floor + size.y / 2.0);
        } else {
            load.vy = 0.0;
            pos.y = floor + size.y / 2.0;
            if let Some(s) = support {
                let c = &list[s];
                let dir = c.rotation * Vec3::X * c.direction;
                let half = half_extent_along(rot, size, dir);
                let seg = c.segment_at(pos);
                let mut step = seg.map_or(0.0, |k| c.segments[k].2) * dt;
                // End of line unless another conveyor continues at the same height.
                let end = c.affine.transform_point3(Vec3::new(c.direction * c.length / 2.0, c.top, 0.0));
                let continues = !c.moving
                    && list.iter().any(|o| {
                    o.entity != c.entity
                        && !o.moving
                        && o.contains(end + dir * 0.06, 0.0)
                        && (o.top_world_y() - c.top_world_y()).abs() < 0.06
                        && !(o.single_load
                            && snapshot.iter().any(|&(other, opos, ..)| other != entity && o.contains(opos, 0.0)))
                });
                if !continues {
                    step = step.min(((end - pos).dot(dir) - half).max(0.0));
                }
                // Raised stops of any conveyor ahead (e.g. the mesa entry stop holds
                // fardos that are still on the incoming belt).
                for b in &list {
                    if b.barriers.is_empty() {
                        continue;
                    }
                    let local = b.inverse.transform_point3(pos);
                    let along = b.inverse.transform_vector3(dir).x;
                    if along < 0.5
                        || local.z.abs() > b.width / 2.0 + 0.05
                        || (local.y - size.y / 2.0 - b.top).abs() > 0.1
                    {
                        continue;
                    }
                    for &x in &b.barriers {
                        // Holds every load whose center has not crossed the stop yet. The mesa
                        // rollers run faster than the infeed belt, so a gap opens between fardos.
                        if local.x < x {
                            step = step.min(((x - local.x - half) / along).max(0.0));
                        }
                    }
                }
                // Queue behind loads ahead.
                let side = Vec3::Y.cross(dir);
                let half_side = half_extent_along(rot, size, side);
                for (j, &(other, opos, orot, osize)) in snapshot.iter().enumerate() {
                    if j == i || other == entity {
                        continue;
                    }
                    let d = opos - pos;
                    let along = d.dot(dir);
                    if along <= 0.0 || d.y.abs() > (size.y + osize.y) / 2.0 {
                        continue;
                    }
                    let lateral = d.dot(side).abs();
                    if lateral >= half_side + half_extent_along(orot, osize, side) - 0.01 {
                        continue;
                    }
                    let gap = along - half - half_extent_along(orot, osize, dir);
                    step = step.min(gap.max(0.0));
                }
                pos += dir * step;
                // Side guides keep the load inside the conveyor width.
                let mut local = c.inverse.transform_point3(pos);
                let lateral_half = half_extent_along(rot, size, c.rotation * Vec3::Z);
                let limit = (c.width / 2.0 - lateral_half).max(0.0);
                local.z = local.z.clamp(-limit, limit);
                pos = c.affine.transform_point3(local);
            }
        }
        tf.translation = pos;
        snapshot[i].1 = pos;
    }
}

/// Mesa pushers (empurrador) and gripper (garra): move with their solenoid
/// inputs, publish their end-position sensors and move fardos sideways. The garra
/// is a comb under the rollers: raised and advancing, it lifts the fardo and
/// carries it to the other lane; lowered, it sets it back on the rollers.
#[allow(clippy::type_complexity)]
pub fn mesa_mechanics(
    time: Res<Time>,
    control: Res<SimControl>,
    mut mesas: Query<(&Equipment, &Transform, &mut Io, &mut MesaMech), Without<Load>>,
    mut loads: Query<(&mut Load, &mut Transform), Without<Equipment>>,
) {
    let dt = scaled_dt(&time, &control);
    for (eq, tf, mut io, mut mech) in &mut mesas {
        let layout = conveyor::layout(eq);
        let affine = tf.compute_affine();
        let inverse = affine.inverse();
        for (i, pusher) in layout.pushers.iter().enumerate().take(2) {
            let (adv, ret) = (io.get_bool(pusher.advance), io.get_bool(pusher.retract));
            let target = match (adv, ret) {
                (true, false) => Some(1.0),
                (false, true) => Some(0.0),
                _ => None,
            };
            let old_stroke = mech.stroke[i];
            if let Some(t) = target {
                let step = dt / PUSHER_TIME;
                mech.stroke[i] += (t - mech.stroke[i]).clamp(-step, step);
            }
            // Both valves off (cilindro aliviado): the plate stays where it is but does
            // not push; the fardos arriving push it back.
            let relieved = !pusher.comb && !adv && !ret;
            if relieved {
                mech.stroke[i] = (mech.stroke[i] - dt / PUSHER_TIME).max(0.0);
            }
            match pusher.lift {
                Some((up, down, sensor_up, sensor_down)) => {
                    let target = match (io.get_bool(up), io.get_bool(down)) {
                        (true, false) => Some(1.0),
                        (false, true) => Some(0.0),
                        _ => None,
                    };
                    if let Some(t) = target {
                        let step = dt / LIFT_TIME;
                        mech.lift[i] += (t - mech.lift[i]).clamp(-step, step);
                    }
                    io.set_bool(sensor_up, mech.lift[i] >= 0.98);
                    io.set_bool(sensor_down, mech.lift[i] <= 0.02);
                }
                None => mech.lift[i] = 1.0,
            }
            io.set_bool(pusher.sensor_advanced, mech.stroke[i] >= 0.98);
            io.set_bool(pusher.sensor_retracted, mech.stroke[i] <= 0.02);
            if pusher.comb {
                let span = pusher.face_end - pusher.face_start;
                let center_old = pusher.face_start + span * old_stroke;
                let delta = span * (mech.stroke[i] - old_stroke);
                let carrying = mech.lift[i] >= 0.9 && mech.stroke[i] > old_stroke;
                let reach = eq.params.box_size[2] / 2.0;
                let ahead = (pusher.face_end - pusher.face_start).signum();
                for (mut load, mut ltf) in &mut loads {
                    if load.state != LoadState::Free {
                        continue;
                    }
                    let mut local = inverse.transform_point3(ltf.translation);
                    let bottom = local.y - load.size.y / 2.0 - eq.params.height;
                    let in_row = (local.z - center_old).abs() < reach
                        || (pusher.train && (local.z - center_old) * ahead > 0.0 && local.z.abs() <= eq.params.width / 2.0);
                    let over = local.x >= pusher.x0 && local.x <= pusher.x1 && (-0.06..0.1).contains(&bottom) && in_row;
                    if carrying && over {
                        local.z += delta;
                        local.y = eq.params.height + load.size.y / 2.0 + 0.04;
                        ltf.translation = affine.transform_point3(local);
                        load.carried = true;
                    } else if load.carried && local.x >= pusher.x0 && local.x <= pusher.x1 {
                        // Comb lowered or stopped: back onto the rollers (gravity does the rest).
                        load.carried = false;
                    }
                }
                continue;
            }
            if mech.lift[i] < 0.9 || dt <= 0.0 || relieved {
                continue;
            }
            let face = pusher.face_start + (pusher.face_end - pusher.face_start) * mech.stroke[i];
            for (load, mut ltf) in &mut loads {
                if load.state != LoadState::Free {
                    continue;
                }
                let mut local = inverse.transform_point3(ltf.translation);
                // Only fardos on this mesa (another mesa may run alongside at the same x).
                if local.x < pusher.x0
                    || local.x > pusher.x1
                    || local.z.abs() > eq.params.width / 2.0 + 0.05
                    || (local.y - load.size.y / 2.0 - eq.params.height).abs() > 0.06
                {
                    continue;
                }
                let half_z = half_extent_along(ltf.rotation, load.size, tf.rotation * Vec3::Z);
                if local.z + half_z > face {
                    local.z = (face - half_z).max(-eq.params.width / 2.0 + half_z);
                    ltf.translation = affine.transform_point3(local);
                }
            }
        }
    }
}

/// Photo sensors of every conveyor layout (mesa positions, TDP end of line).
#[allow(clippy::type_complexity)]
pub fn conveyor_sensors(
    mut conveyors: Query<(&Equipment, &Transform, &mut Io, Option<&ShuttleState>), Without<Load>>,
    loads: Query<(&Load, &GlobalTransform)>,
) {
    for (eq, tf, mut io, car) in &mut conveyors {
        if !eq.kind.is_conveyor() {
            continue;
        }
        let surface = conveyor::surface(eq, tf, car);
        for sensor in conveyor::layout(eq).sensors {
            let (a, b) = photo_beam(&surface, &sensor);
            let hit = loads.iter().any(|(l, gt)| segment_hits_box(a, b, l.size, gt));
            io.set_bool(sensor.key, hit);
        }
    }
}

pub fn photo_beam(surface: &Surface, sensor: &conveyor::PhotoSensor) -> (Vec3, Vec3) {
    let y = surface.top + 0.08;
    (
        surface.point(Vec3::new(sensor.x, y, sensor.z0)),
        surface.point(Vec3::new(sensor.x, y, sensor.z1)),
    )
}

fn area_occupied<'a>(at: Vec3, footprint: f32, height: f32, loads: impl Iterator<Item = (&'a Load, &'a Transform)>) -> bool {
    loads.into_iter().any(|(l, tf)| {
        let d = tf.translation - at;
        l.state == LoadState::Free
            && Vec2::new(d.x, d.z).length() < (l.size.x.max(l.size.z) + footprint) / 2.0
            && d.y.abs() < (height + l.size.y) / 2.0
    })
}

#[allow(clippy::type_complexity)]
pub fn emitters(
    time: Res<Time>,
    control: Res<SimControl>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    palette: Res<Palette>,
    mut emitters: Query<(&Equipment, &Transform, &Io, &mut EmitterState)>,
    loads: Query<(&Load, &Transform), Without<Equipment>>,
) {
    let dt = scaled_dt(&time, &control);
    if dt <= 0.0 {
        return;
    }
    for (eq, tf, io, mut state) in &mut emitters {
        let kind = match eq.kind {
            EquipmentKind::BoxEmitter => LoadKind::Box,
            EquipmentKind::PalletDispenser => LoadKind::Pallet,
            _ => continue,
        };
        if !io.get_bool("enable") {
            continue;
        }
        state.timer += dt;
        if state.timer < eq.params.interval.max(0.2) {
            continue;
        }
        let size = match kind {
            LoadKind::Box => eq.params.box_vec(),
            LoadKind::Pallet => eq.params.pallet_vec(),
        };
        let spawn = tf.transform_point(Vec3::new(0.0, eq.params.height + size.y / 2.0 + 0.002, 0.0));
        if area_occupied(spawn, size.x.max(size.z), size.y, loads.iter()) {
            continue;
        }
        state.timer = 0.0;
        spawn_load(&mut commands, &mut meshes, &palette, kind, size, Transform::from_translation(spawn).with_rotation(tf.rotation));
    }
}

pub fn spawn_load(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    palette: &Palette,
    kind: LoadKind,
    size: Vec3,
    transform: Transform,
) -> Entity {
    let entity = commands
        .spawn((
            transform,
            Visibility::default(),
            Load {
                size,
                kind,
                state: LoadState::Free,
                vy: 0.0,
                carried: false,
                hold: false,
            },
        ))
        .id();
    match kind {
        LoadKind::Box => {
            commands
                .entity(entity)
                .with_children(|parent| visuals::box_mesh(parent, meshes, palette, size));
        }
        LoadKind::Pallet => {
            commands.entity(entity).insert(PalletLoad::default()).with_children(|parent| {
                parent
                    .spawn((Transform::from_xyz(0.0, -size.y / 2.0, 0.0), Visibility::default()))
                    .with_children(|mesh| visuals::pallet_mesh(mesh, meshes, palette, size.x, size.z));
            });
        }
    }
    entity
}

pub fn removers(
    mut commands: Commands,
    control: Res<SimControl>,
    removers: Query<(&Equipment, &Transform, &Io)>,
    loads: Query<(Entity, &Load, &Transform), Without<Equipment>>,
) {
    if !control.running {
        return;
    }
    for (eq, tf, io) in &removers {
        if eq.kind != EquipmentKind::BoxRemover || !io.get_bool("enable") {
            continue;
        }
        let inverse = tf.compute_affine().inverse();
        for (entity, load, ltf) in &loads {
            if load.state != LoadState::Free {
                continue;
            }
            let l = inverse.transform_point3(ltf.translation);
            if l.x.abs() <= eq.params.length / 2.0 && l.z.abs() <= eq.params.width / 2.0 && l.y < eq.params.height + 2.0 {
                commands.entity(entity).despawn();
            }
        }
    }
}

pub fn sensors(mut sensors: Query<(&Equipment, &Transform, &mut Io)>, loads: Query<(&Load, &GlobalTransform)>) {
    for (eq, tf, mut io) in &mut sensors {
        if eq.kind != EquipmentKind::Sensor {
            continue;
        }
        let (a, b) = sensor_beam(eq, tf);
        let hit = loads.iter().any(|(l, gt)| segment_hits_box(a, b, l.size, gt));
        io.set_bool("detected", hit);
    }
}

pub fn sensor_beam(eq: &Equipment, tf: &Transform) -> (Vec3, Vec3) {
    (
        tf.transform_point(Vec3::new(0.0, eq.params.height, 0.0)),
        tf.transform_point(Vec3::new(0.0, eq.params.height, eq.params.width)),
    )
}

fn segment_hits_box(a: Vec3, b: Vec3, size: Vec3, gt: &GlobalTransform) -> bool {
    // Slab test in the box frame.
    let inv = gt.affine().inverse();
    let (la, lb) = (inv.transform_point3(a), inv.transform_point3(b));
    let half = size / 2.0;
    let d = lb - la;
    let (mut t0, mut t1) = (0.0f32, 1.0f32);
    for axis in 0..3 {
        let (o, dir, h) = (la[axis], d[axis], half[axis]);
        if dir.abs() < 1e-6 {
            if o.abs() > h {
                return false;
            }
        } else {
            let (mut ta, mut tb) = ((-h - o) / dir, (h - o) / dir);
            if ta > tb {
                std::mem::swap(&mut ta, &mut tb);
            }
            t0 = t0.max(ta);
            t1 = t1.min(tb);
            if t0 > t1 {
                return false;
            }
        }
    }
    true
}

/// Placement `index` on a pallet: center in the pallet frame (origin at the
/// pallet center) and yaw, for a unit of footprint `unit`. Layers alternate
/// orientation when `interlock` is set. `None` when the pallet is complete.
pub fn pallet_slot(pattern: &Params, pallet: Vec3, unit: Vec3, index: u32) -> Option<(Vec3, f32)> {
    if let Some(chosen) = &pattern.pattern {
        return chosen.slot(pallet.y, unit.y, index);
    }
    let mut remaining = index;
    for layer in 0..pattern.layers {
        let rotated = pattern.interlock && layer % 2 == 1;
        let (fx, fz, yaw) = if rotated {
            (unit.z, unit.x, FRAC_PI_2)
        } else {
            (unit.x, unit.z, 0.0)
        };
        let cols = ((pallet.x + 0.01) / fx).floor().max(1.0) as u32;
        let rows = ((pallet.z + 0.01) / fz).floor().max(1.0) as u32;
        let per_layer = cols * rows;
        if remaining < per_layer {
            let (c, r) = (remaining % cols, remaining / cols);
            let x = -(cols as f32) * fx / 2.0 + fx / 2.0 + c as f32 * fx;
            let z = -(rows as f32) * fz / 2.0 + fz / 2.0 + r as f32 * fz;
            let y = pallet.y / 2.0 + layer as f32 * unit.y + unit.y / 2.0;
            return Some((Vec3::new(x, y, z), yaw));
        }
        remaining -= per_layer;
    }
    None
}

pub fn pallet_capacity(pattern: &Params, pallet: Vec3, unit: Vec3) -> u32 {
    (0..)
        .take_while(|i| pallet_slot(pattern, pallet, unit, *i).is_some())
        .count() as u32
}

/// Box size used before the first placement: the first emitter's box.
pub fn product_box_size<'a>(equipment: impl Iterator<Item = &'a Equipment>) -> Vec3 {
    equipment
        .into_iter()
        .find(|e| e.kind == EquipmentKind::BoxEmitter)
        .map(|e| e.params.box_vec())
        .unwrap_or_else(|| Params::default().box_vec())
}

/// TDP de paletização: finds the pallet at the stop, publishes present/full/count
/// and, with internal logic on, drives motor and stop for inputs the PLC is not driving:
/// bring a pallet, hold it while palletizing, release it when full.
#[allow(clippy::type_complexity)]
pub fn palletizers(
    mut tdps: Query<(&Equipment, &Transform, &mut Io, &mut PalletizerState), Without<Load>>,
    loads: Query<(Entity, &Load, &Transform, Option<&PalletLoad>), Without<Equipment>>,
    all_equipment: Query<&Equipment>,
) {
    let default_unit = product_box_size(all_equipment.iter());
    for (eq, tf, mut io, mut state) in &mut tdps {
        if eq.kind != EquipmentKind::PalletizingConveyor {
            continue;
        }
        let pallet = loads.iter().find(|(_, l, ltf, p)| {
            l.kind == LoadKind::Pallet && l.state == LoadState::Free && p.is_some() && at_conveyor_end(eq, tf, l.size, ltf)
        });
        state.pallet = pallet.map(|(e, ..)| e);
        let content = pallet.and_then(|(.., p)| p);
        state.placed = content.map_or(0, |p| p.placed);
        state.units = content.map_or(0, |p| p.units);
        let unit = content.and_then(|p| p.unit_size).unwrap_or(default_unit);
        state.capacity = pallet.map_or(0, |(_, l, ..)| pallet_capacity(&eq.params, l.size, unit));
        state.full = state.pallet.is_some() && state.units >= state.capacity;
        let present = state.pallet.is_some();
        if eq.params.auto_logic {
            if !io.from_plc.contains("stop") {
                io.set_bool("stop", !(present && state.full));
            }
            if !io.from_plc.contains("motor") {
                io.set_bool("motor", !present || state.full);
            }
        }
        io.set_bool("present", present);
        io.set_bool("full", state.full);
        io.set("count", state.placed as f64);
    }
}

#[allow(clippy::type_complexity)]
pub fn animate_conveyors(
    conveyors: Query<(&Equipment, &ConveyorState, &Io, &Children, Option<&MesaMech>)>,
    mut marks: Query<(&BeltMark, &mut Transform), (Without<Roller>, Without<StopBar>, Without<ActuatorPlate>)>,
    mut rollers: Query<(&Roller, &mut Transform), (Without<BeltMark>, Without<StopBar>, Without<ActuatorPlate>)>,
    mut stops: Query<(&StopBar, &mut Transform), (Without<BeltMark>, Without<Roller>, Without<ActuatorPlate>)>,
    mut plates: Query<(&ActuatorPlate, &mut Transform), (Without<BeltMark>, Without<Roller>, Without<StopBar>)>,
    car_bodies: Query<&Children, With<ShuttleBody>>,
) {
    for (eq, state, io, children, mech) in &conveyors {
        // Rollers of the CVZ live under the moving car body.
        for body in children.iter().filter_map(|c| car_bodies.get(c).ok()) {
            for child in body.iter() {
                if let Ok((roller, mut tf)) = rollers.get_mut(child) {
                    let angle = -state.travel[roller.stage.min(2)] / roller.radius;
                    tf.rotation = Quat::from_rotation_z(angle) * Quat::from_rotation_x(FRAC_PI_2);
                }
            }
        }
        let half = eq.params.length / 2.0;
        let span = ((eq.params.length / BELT_MARK_SPACING).floor() * BELT_MARK_SPACING).max(BELT_MARK_SPACING);
        let pushers = mech.map(|_| conveyor::layout(eq).pushers);
        for child in children.iter() {
            if let Ok((mark, mut tf)) = marks.get_mut(child) {
                tf.translation.x = (mark.base_x + half + state.travel[0]).rem_euclid(span) - half;
            } else if let Ok((roller, mut tf)) = rollers.get_mut(child) {
                let angle = -state.travel[roller.stage.min(2)] / roller.radius;
                tf.rotation = Quat::from_rotation_z(angle) * Quat::from_rotation_x(FRAC_PI_2);
            } else if let Ok((stop, mut tf)) = stops.get_mut(child) {
                let target = if io.get_bool(stop.signal) { stop.raised } else { stop.lowered };
                tf.translation.y += (target - tf.translation.y).clamp(-0.01, 0.01);
            } else if let (Ok((plate, mut tf)), Some(mech), Some(pushers)) = (plates.get_mut(child), mech, &pushers)
                && let Some(p) = pushers.get(plate.index) {
                    let at = p.face_start + (p.face_end - p.face_start) * mech.stroke[plate.index];
                    if p.comb {
                        // Fingers below the roller tops when lowered, above them when raised.
                        tf.translation.z = at;
                        tf.translation.y = eq.params.height - 0.07 + 0.09 * mech.lift[plate.index];
                    } else {
                        tf.translation.z = at + 0.02;
                        tf.translation.y = eq.params.height - 0.12 + 0.24 * mech.lift[plate.index];
                    }
                }
        }
    }
}

/// Removes every load and resets robots and mesas (toolbar "Reiniciar").
pub fn reset_simulation(
    mut control: ResMut<SimControl>,
    mut commands: Commands,
    loads: Query<(Entity, &Load)>,
    mut robots: Query<&mut crate::robot::RobotState>,
    mut emitters: Query<&mut EmitterState>,
    mut mesas: Query<(&Equipment, &mut crate::mesa_logic::MesaLogic, &mut MesaMech)>,
    mut elevators: Query<(&Equipment, &mut crate::elevator::ElevatorState, &mut crate::elevator::ElevatorLogic)>,
) {
    if !control.reset_requested {
        return;
    }
    control.reset_requested = false;
    for (entity, load) in &loads {
        // Boxes on pallets go with their pallet.
        if !matches!(load.state, LoadState::Placed(_)) {
            commands.entity(entity).despawn();
        }
    }
    for mut r in &mut robots {
        *r = crate::robot::RobotState::default();
    }
    for mut e in &mut emitters {
        e.timer = 0.0;
    }
    for (eq, mut logic, mut mech) in &mut mesas {
        if let Some(fresh) = crate::mesa_logic::MesaLogic::for_kind(eq.kind) {
            *logic = fresh;
        }
        *mech = MesaMech::default();
    }
    for (eq, mut state, mut logic) in &mut elevators {
        *state = crate::elevator::ElevatorState::new(&eq.params);
        *logic = default();
    }
}
