//! Scene editing: orbit camera, mouse picking, multiple selection (Ctrl+click,
//! rubber band), drag to move with grid snap, keyboard rotate/delete/duplicate,
//! and helper gizmos.

use std::f32::consts::FRAC_PI_2;

use bevy::{
    input::mouse::{MouseMotion, MouseWheel},
    prelude::*,
    window::PrimaryWindow,
};

use crate::model::{Equipment, EquipmentKind, Io, NeedsRebuild, Params, Selection, TagBindings, unique_name};
use crate::robot::RobotState;
use crate::mesa_logic::MesaLogic;
use crate::sim::{self, ConveyorState, EmitterState, Load, MesaMech, PalletizerState};
use crate::conveyor;
use crate::shuttle::{ShuttleLogic, ShuttleState};
use crate::visuals::Palette;

#[derive(Component)]
pub struct OrbitCamera {
    pub target: Vec3,
    pub radius: f32,
    pub yaw: f32,
    pub pitch: f32,
}

/// Written by the UI each frame so the 3D view ignores clicks/keys meant for egui.
#[derive(Resource, Default)]
pub struct UiFocus {
    pub pointer_over_ui: bool,
    pub keyboard_ui: bool,
}

#[derive(Resource)]
pub struct EditorSettings {
    pub snap: bool,
    pub grid: f32,
    pub rotation_step_deg: f32,
    pub show_grid: bool,
}

impl Default for EditorSettings {
    fn default() -> Self {
        Self {
            snap: true,
            grid: 0.1,
            rotation_step_deg: 90.0,
            show_grid: true,
        }
    }
}

/// Requests from the UI/keyboard handled where `Commands` and assets are available.
#[derive(Resource, Default)]
pub struct EditRequests {
    pub add: Vec<EquipmentKind>,
    /// New robots of these models.
    pub add_robot: Vec<crate::model::RobotModel>,
    /// Catalog items to add: at the center of the view, or where they were dropped
    /// (screen position over the 3D view).
    pub place: Vec<(usize, Option<Vec2>)>,
    pub delete: Vec<Entity>,
    pub duplicate: Vec<Entity>,
    pub focus: Option<Entity>,
}

pub fn spawn_equipment(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    palette: &Palette,
    equipment: Equipment,
    transform: Transform,
    bindings: TagBindings,
) -> Entity {
    let kind = equipment.kind;
    let mut root = commands.spawn((transform, Visibility::default(), bindings, Io::default()));
    match kind {
        EquipmentKind::PalletizingConveyor => {
            root.insert((ConveyorState::default(), PalletizerState::default()));
        }
        EquipmentKind::ShuttleCar => {
            root.insert((ConveyorState::default(), ShuttleState::default(), ShuttleLogic::default()));
        }
        EquipmentKind::FullCar => {
            root.insert((ConveyorState::default(), ShuttleState::default(), crate::fullcar::FullCarLogic::default()));
        }
        EquipmentKind::CardboardPlacer => {
            root.insert((crate::paper::PaperPlacerState::default(), crate::paper::PaperPlacerLogic::default()));
        }
        EquipmentKind::PalletElevator => {
            root.insert((
                crate::elevator::ElevatorState::new(&equipment.params),
                crate::elevator::ElevatorLogic::default(),
            ));
        }
        k if k.is_mesa() => {
            let logic = MesaLogic::for_kind(k).expect("mesa kind");
            root.insert((ConveyorState::default(), MesaMech::default(), logic));
        }
        EquipmentKind::BeltConveyor => {
            root.insert((ConveyorState::default(), crate::belt::BeltLogic::default()));
        }
        k if k.is_conveyor() => {
            root.insert(ConveyorState::default());
        }
        EquipmentKind::Robot => {
            root.insert(RobotState::new(crate::robot::kin(equipment.params.robot_model).with_limits(&equipment.params.axis_limits)));
        }
        EquipmentKind::BoxEmitter | EquipmentKind::PalletDispenser => {
            root.insert(EmitterState::default());
        }
        _ => {}
    }
    let entity = root.id();
    crate::visuals::build_visual(commands, entity, &equipment, meshes, palette);
    commands.entity(entity).insert(equipment);
    entity
}

pub fn rebuild_visuals(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    palette: Res<Palette>,
    changed: Query<(Entity, &Equipment), With<NeedsRebuild>>,
) {
    for (entity, equipment) in &changed {
        commands.entity(entity).despawn_children().remove::<NeedsRebuild>();
        crate::visuals::build_visual(&mut commands, entity, equipment, &mut meshes, &palette);
    }
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
pub fn process_edit_requests(
    mut requests: ResMut<EditRequests>,
    mut selection: ResMut<Selection>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    palette: Res<Palette>,
    equipment: Query<(Entity, &Equipment, &Transform, &TagBindings)>,
    robots: Query<&RobotState>,
    mut cameras: Query<&mut OrbitCamera>,
    (views, settings): (Query<(&Camera, &GlobalTransform), With<OrbitCamera>>, Res<EditorSettings>),
) {
    let names: Vec<String> = equipment.iter().map(|(_, e, ..)| e.name.clone()).collect();
    let mut taken = names.clone();
    let target = cameras.single().map(|c| c.target).unwrap_or(Vec3::ZERO);
    // Catalog: at the center of the view, or on the floor where the card was dropped.
    for (index, screen) in std::mem::take(&mut requests.place) {
        let Some(item) = crate::catalog::ITEMS.get(index) else { continue };
        let floor = screen
            .and_then(|pos| {
                let (camera, at) = views.single().ok()?;
                ground_point(&camera.viewport_to_world(at, pos).ok()?, 0.0)
            })
            .unwrap_or(target);
        let step = if settings.snap { settings.grid.max(0.01) } else { 0.01 };
        let at = Vec3::new((floor.x / step).round() * step, 0.0, (floor.z / step).round() * step);
        let name = unique_name(item.prefix(), taken.iter().map(String::as_str));
        taken.push(name.clone());
        let entity = spawn_equipment(
            &mut commands,
            &mut meshes,
            &palette,
            Equipment { name, kind: item.kind, params: item.params() },
            Transform::from_translation(at),
            TagBindings::default(),
        );
        selection.0 = Some(entity);
        selection.1 = vec![entity];
    }
    let robots_to_add: Vec<(EquipmentKind, Option<crate::model::RobotModel>)> =
        std::mem::take(&mut requests.add_robot).into_iter().map(|m| (EquipmentKind::Robot, Some(m))).collect();
    for (kind, model) in std::mem::take(&mut requests.add).into_iter().map(|k| (k, None)).chain(robots_to_add) {
        let mut params = Params::for_kind(kind);
        if let Some(model) = model {
            params.robot_model = model;
        }
        let prefix = model.map_or(kind.prefix(), |m| m.prefix());
        let name = unique_name(prefix, taken.iter().map(String::as_str));
        taken.push(name.clone());
        let at = Vec3::new((target.x / 0.5).round() * 0.5, 0.0, (target.z / 0.5).round() * 0.5);
        let entity = spawn_equipment(
            &mut commands,
            &mut meshes,
            &palette,
            Equipment { name, kind, params },
            Transform::from_translation(at),
            TagBindings::default(),
        );
        selection.0 = Some(entity);
    }
    let duplicate = std::mem::take(&mut requests.duplicate);
    if !duplicate.is_empty() {
        // Copies keep their arrangement, shifted beside the group.
        let shift = duplicate
            .iter()
            .filter_map(|e| equipment.get(*e).ok())
            .map(|(_, eq, _, _)| eq.params.width.max(0.5))
            .fold(0.0, f32::max)
            + 0.3;
        let mut copies = Vec::new();
        for entity in duplicate {
            let Ok((_, eq, tf, bindings)) = equipment.get(entity) else { continue };
            let name = unique_name(eq.kind.prefix(), taken.iter().map(String::as_str));
            taken.push(name.clone());
            let mut copy = eq.clone();
            copy.name = name;
            copies.push(spawn_equipment(
                &mut commands,
                &mut meshes,
                &palette,
                copy,
                tf.with_translation(tf.translation + Vec3::Z * shift),
                bindings.clone(),
            ));
        }
        selection.0 = copies.first().copied();
        selection.1 = copies;
    }
    for entity in std::mem::take(&mut requests.delete) {
        if !equipment.contains(entity) {
            continue;
        }
        if let Ok(robot) = robots.get(entity) {
            for item in &robot.held {
                commands.entity(item.entity).despawn();
            }
        }
        commands.entity(entity).despawn();
        selection.1.retain(|e| *e != entity);
        if selection.0 == Some(entity) {
            selection.0 = selection.1.first().copied();
        }
    }
    if let Some(entity) = requests.focus.take()
        && let (Ok((_, _, tf, _)), Ok(mut cam)) = (equipment.get(entity), cameras.single_mut()) {
            cam.target = tf.translation + Vec3::Y * 0.8;
            cam.radius = cam.radius.min(10.0);
        }
}

pub fn camera_controls(
    mouse: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut motion: MessageReader<MouseMotion>,
    mut wheel: MessageReader<MouseWheel>,
    focus: Res<UiFocus>,
    mut cameras: Query<(&mut Transform, &mut OrbitCamera)>,
) {
    let Ok((mut transform, mut orbit)) = cameras.single_mut() else {
        return;
    };
    let delta: Vec2 = motion.read().map(|m| m.delta).sum();
    let pan = mouse.pressed(MouseButton::Middle)
        || (mouse.pressed(MouseButton::Right) && keyboard.pressed(KeyCode::ShiftLeft));
    if pan {
        let right = transform.right();
        let forward = Vec3::new(transform.forward().x, 0.0, transform.forward().z).normalize_or_zero();
        let scale = orbit.radius * 0.0015;
        orbit.target += (-right * delta.x + forward * delta.y) * scale;
        orbit.target.y = orbit.target.y.max(0.0);
    } else if mouse.pressed(MouseButton::Right) {
        orbit.yaw -= delta.x * 0.006;
        orbit.pitch = (orbit.pitch + delta.y * 0.006).clamp(0.05, 1.5);
    }
    let scroll: f32 = wheel.read().map(|w| w.y).sum();
    if !focus.pointer_over_ui && scroll != 0.0 {
        orbit.radius = (orbit.radius * (1.0 - scroll.signum() * 0.1)).clamp(1.5, 60.0);
    }
    let direction = Vec3::new(
        orbit.yaw.cos() * orbit.pitch.cos(),
        orbit.pitch.sin(),
        orbit.yaw.sin() * orbit.pitch.cos(),
    );
    transform.translation = orbit.target + direction * orbit.radius;
    transform.look_at(orbit.target, Vec3::Y);
}

fn cursor_ray(
    windows: &Query<&Window, With<PrimaryWindow>>,
    cameras: &Query<(&Camera, &GlobalTransform), With<OrbitCamera>>,
) -> Option<Ray3d> {
    let window = windows.single().ok()?;
    let cursor = window.cursor_position()?;
    let (camera, camera_tf) = cameras.single().ok()?;
    camera.viewport_to_world(camera_tf, cursor).ok()
}

fn ray_hits_obb(ray: &Ray3d, tf: &Transform, min: Vec3, max: Vec3) -> Option<f32> {
    let inv = tf.compute_affine().inverse();
    let origin = inv.transform_point3(ray.origin);
    let dir = inv.transform_vector3(*ray.direction);
    let (mut t0, mut t1) = (0.0f32, f32::MAX);
    for axis in 0..3 {
        if dir[axis].abs() < 1e-8 {
            if origin[axis] < min[axis] || origin[axis] > max[axis] {
                return None;
            }
            continue;
        }
        let mut ta = (min[axis] - origin[axis]) / dir[axis];
        let mut tb = (max[axis] - origin[axis]) / dir[axis];
        if ta > tb {
            std::mem::swap(&mut ta, &mut tb);
        }
        t0 = t0.max(ta);
        t1 = t1.min(tb);
        if t0 > t1 {
            return None;
        }
    }
    Some(t0)
}

fn ground_point(ray: &Ray3d, y: f32) -> Option<Vec3> {
    let t = ray.intersect_plane(Vec3::new(0.0, y, 0.0), InfinitePlane3d::new(Vec3::Y))?;
    Some(ray.get_point(t))
}

#[derive(Resource, Default)]
pub struct DragState {
    /// Dragging the selection: offset of each selected item from the grab point.
    group: Vec<(Entity, Vec3)>,
    grab: Option<Entity>,
    plane_y: f32,
    /// Rubber-band selection: start and current cursor positions (logical pixels).
    pub band: Option<(Vec2, Vec2)>,
    band_additive: bool,
}

/// Keeps the selection list consistent with the primary selection set elsewhere
/// (scene list, newly added items) and with deleted entities.
pub fn sync_selection(
    mut selection: ResMut<Selection>,
    mut last_primary: Local<Option<Entity>>,
    equipment: Query<(), With<Equipment>>,
) {
    let sel = &mut *selection;
    sel.1.retain(|e| equipment.contains(*e));
    if sel.0.is_some_and(|e| !equipment.contains(e)) {
        sel.0 = sel.1.first().copied();
    }
    if sel.0 != *last_primary {
        match sel.0 {
            Some(e) if !sel.1.contains(&e) => sel.1 = vec![e],
            None => sel.1.clear(),
            _ => {}
        }
        *last_primary = sel.0;
    }
}

#[allow(clippy::too_many_arguments)]
pub fn pick_and_drag(
    mouse: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cameras: Query<(&Camera, &GlobalTransform), With<OrbitCamera>>,
    focus: Res<UiFocus>,
    settings: Res<EditorSettings>,
    mut selection: ResMut<Selection>,
    mut drag: ResMut<DragState>,
    mut equipment: Query<(Entity, &Equipment, &mut Transform), Without<Load>>,
) {
    let ctrl = keyboard.pressed(KeyCode::ControlLeft) || keyboard.pressed(KeyCode::ControlRight);
    let cursor = windows.single().ok().and_then(|w| w.cursor_position());

    if mouse.just_released(MouseButton::Left) {
        drag.group.clear();
        drag.grab = None;
        if let Some((start, end)) = drag.band.take()
            && start.distance(end) > 4.0 {
                let (min, max) = (start.min(end), start.max(end));
                let Ok((camera, camera_tf)) = cameras.single() else { return };
                let inside: Vec<Entity> = equipment
                    .iter()
                    .filter(|(_, eq, tf)| {
                        let (lo, hi) = eq.local_bounds();
                        let center = tf.transform_point((lo + hi) / 2.0);
                        camera
                            .world_to_viewport(camera_tf, center)
                            .is_ok_and(|p| p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y)
                    })
                    .map(|(e, ..)| e)
                    .collect();
                if !drag.band_additive {
                    selection.1.clear();
                }
                for e in inside {
                    if !selection.1.contains(&e) {
                        selection.1.push(e);
                    }
                }
                selection.0 = selection.1.first().copied();
            }
    }
    let Some(ray) = cursor_ray(&windows, &cameras) else { return };

    if mouse.just_pressed(MouseButton::Left) && !focus.pointer_over_ui {
        let hit = equipment
            .iter()
            .filter_map(|(entity, eq, tf)| {
                let (min, max) = eq.local_bounds();
                ray_hits_obb(&ray, tf, min, max).map(|t| (entity, t))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(e, _)| e);
        match hit {
            Some(entity) if ctrl => {
                // Ctrl+click toggles the item in the selection.
                if let Some(i) = selection.1.iter().position(|e| *e == entity) {
                    selection.1.remove(i);
                    selection.0 = selection.1.first().copied();
                } else {
                    selection.1.push(entity);
                    selection.0 = Some(entity);
                }
            }
            Some(entity) => {
                if !selection.1.contains(&entity) {
                    selection.1 = vec![entity];
                }
                selection.0 = Some(entity);
                // Grab the whole selection.
                let plane_y = equipment.get(entity).map(|(_, _, tf)| tf.translation.y).unwrap_or(0.0);
                if let Some(point) = ground_point(&ray, plane_y) {
                    drag.plane_y = plane_y;
                    drag.grab = Some(entity);
                    drag.group = selection
                        .1
                        .iter()
                        .filter_map(|e| equipment.get(*e).ok().map(|(_, _, tf)| (*e, tf.translation - point)))
                        .collect();
                }
            }
            None => {
                if let Some(c) = cursor {
                    drag.band = Some((c, c));
                    drag.band_additive = ctrl;
                }
            }
        }
    }

    if mouse.pressed(MouseButton::Left) {
        if keyboard.pressed(KeyCode::KeyV) {
            // V + drag changes the height (keyboard_edit), not the position on the floor.
            return;
        }
        if let (Some((start, _)), Some(c)) = (drag.band, cursor) {
            drag.band = Some((start, c));
            return;
        }
        let Some(grab) = drag.grab else { return };
        let Some(point) = ground_point(&ray, drag.plane_y) else { return };
        let Some(&(_, grab_offset)) = drag.group.iter().find(|(e, _)| *e == grab) else { return };
        // Snap the grabbed item to the grid; the others keep their relative position.
        let mut target = point + grab_offset;
        let snap = settings.snap != keyboard.pressed(KeyCode::AltLeft);
        if snap && settings.grid > 0.0 {
            target.x = (target.x / settings.grid).round() * settings.grid;
            target.z = (target.z / settings.grid).round() * settings.grid;
        }
        let shift = target - (point + grab_offset);
        for &(entity, offset) in &drag.group {
            if let Ok((_, _, mut tf)) = equipment.get_mut(entity) {
                let new = point + offset + shift;
                if (tf.translation.x - new.x).abs() > 1e-4 || (tf.translation.z - new.z).abs() > 1e-4 {
                    tf.translation.x = new.x;
                    tf.translation.z = new.z;
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn keyboard_edit(
    keyboard: Res<ButtonInput<KeyCode>>,
    focus: Res<UiFocus>,
    settings: Res<EditorSettings>,
    mut selection: ResMut<Selection>,
    mut requests: ResMut<EditRequests>,
    mut equipment: Query<&mut Transform, With<Equipment>>,
    mut motion: MessageReader<MouseMotion>,
    mut height_drag: Local<f32>,
) {
    let mouse_dy: f32 = motion.read().map(|m| m.delta.y).sum();
    if focus.keyboard_ui || selection.1.is_empty() {
        *height_drag = 0.0;
        return;
    }
    // V held: moving the mouse up/down raises/lowers the selection (grid steps with snap).
    if keyboard.pressed(KeyCode::KeyV) {
        *height_drag -= mouse_dy * 0.004;
        let snap = settings.snap != keyboard.pressed(KeyCode::AltLeft) && settings.grid > 0.0;
        let dy = if snap {
            let steps = (*height_drag / settings.grid).trunc();
            steps * settings.grid
        } else {
            *height_drag
        };
        if dy != 0.0 {
            *height_drag -= dy;
            for e in &selection.1 {
                if let Ok(mut tf) = equipment.get_mut(*e) {
                    tf.translation.y = (tf.translation.y + dy).max(0.0);
                    if snap {
                        tf.translation.y = (tf.translation.y / settings.grid).round() * settings.grid;
                    }
                }
            }
        }
    } else {
        *height_drag = 0.0;
    }
    let shift = keyboard.pressed(KeyCode::ShiftLeft) || keyboard.pressed(KeyCode::ShiftRight);
    let ctrl = keyboard.pressed(KeyCode::ControlLeft) || keyboard.pressed(KeyCode::ControlRight);
    let items = selection.1.clone();
    if keyboard.just_pressed(KeyCode::Delete) {
        requests.delete.extend(items);
        return;
    }
    if ctrl && keyboard.just_pressed(KeyCode::KeyD) {
        requests.duplicate.extend(items);
        return;
    }
    if keyboard.just_pressed(KeyCode::Escape) {
        selection.0 = None;
        selection.1.clear();
        return;
    }
    if keyboard.just_pressed(KeyCode::KeyF) {
        requests.focus = selection.0;
    }
    let step = settings.rotation_step_deg.to_radians();
    let rotation = if keyboard.just_pressed(KeyCode::KeyR) {
        Some(if shift { -step } else { step })
    } else if keyboard.just_pressed(KeyCode::KeyQ) {
        Some(15f32.to_radians())
    } else if keyboard.just_pressed(KeyCode::KeyE) {
        Some(-15f32.to_radians())
    } else {
        None
    };
    if let Some(angle) = rotation {
        // Rotate the group about its center (a single item turns in place).
        let positions: Vec<Vec3> = items.iter().filter_map(|e| equipment.get(*e).ok().map(|t| t.translation)).collect();
        let center = positions.iter().copied().sum::<Vec3>() / positions.len().max(1) as f32;
        let turn = Quat::from_rotation_y(angle);
        for e in &items {
            if let Ok(mut tf) = equipment.get_mut(*e) {
                let rel = tf.translation - center;
                tf.translation = center + turn * rel;
                tf.rotate_y(angle);
                if keyboard.just_pressed(KeyCode::KeyR) {
                    snap_yaw(&mut tf);
                }
            }
        }
    }
    let nudge = if shift { 0.5 } else { 0.1 };
    let moves = [
        (KeyCode::ArrowLeft, Vec3::NEG_X),
        (KeyCode::ArrowRight, Vec3::X),
        (KeyCode::ArrowUp, Vec3::NEG_Z),
        (KeyCode::ArrowDown, Vec3::Z),
        (KeyCode::PageUp, Vec3::Y * 0.5),
        (KeyCode::PageDown, Vec3::NEG_Y * 0.5),
    ];
    for (key, dir) in moves {
        if keyboard.just_pressed(key) {
            for e in &items {
                if let Ok(mut tf) = equipment.get_mut(*e) {
                    tf.translation += dir * nudge;
                    tf.translation.y = tf.translation.y.max(0.0);
                }
            }
        }
    }
}

/// Removes float drift after repeated 90° rotations.
fn snap_yaw(tf: &mut Transform) {
    let yaw = crate::robot::yaw_of(tf.rotation);
    let snapped = (yaw.to_degrees() * 100.0).round() / 100.0;
    tf.rotation = Quat::from_rotation_y(snapped.to_radians());
}

pub fn draw_gizmos(
    mut gizmos: Gizmos,
    settings: Res<EditorSettings>,
    selection: Res<Selection>,
    equipment: Query<(Entity, &Equipment, &Transform, &Io, Option<&ShuttleState>)>,
    robots: Query<&RobotState>,
) {
    if settings.show_grid {
        gizmos
            .grid(
                Isometry3d::new(Vec3::new(0.0, 0.002, 0.0), Quat::from_rotation_x(FRAC_PI_2)),
                UVec2::splat(40),
                Vec2::splat(1.0),
                Color::srgba(1.0, 1.0, 1.0, 0.07),
            )
            .outer_edges();
    }
    for (entity, eq, tf, io, car) in &equipment {
        let selected = selection.0 == Some(entity);
        if selected || selection.1.contains(&entity) {
            let (min, max) = eq.local_bounds();
            let center = (min + max) / 2.0;
            gizmos.cube(
                Transform {
                    translation: tf.transform_point(center),
                    rotation: tf.rotation,
                    scale: max - min + Vec3::splat(0.04),
                },
                if selected { Color::srgb(0.2, 0.9, 1.0) } else { Color::srgb(1.0, 0.7, 0.2) },
            );
        }
        match eq.kind {
            k if k.is_conveyor() => {
                // One arrow per driven stage: green running, grey stopped.
                let p = &eq.params;
                let layout = conveyor::layout(eq);
                let surface = conveyor::surface(eq, tf, car);
                let reverse = io.get_bool("reverse") || io.get_bool("Reverso_TDP") || io.get("VelocidadeTDP") < 0.0;
                let sign = if reverse { -1.0 } else { 1.0 };
                if let Some(car) = car {
                    // Laser of the distance sensor to the car.
                    let y = 0.25;
                    gizmos.line(
                        tf.transform_point(Vec3::new(-p.length / 2.0, y, 0.0)),
                        tf.transform_point(Vec3::new(car.x - p.length / 2.0 - p.car_width / 2.0, y, 0.0)),
                        Color::srgb(1.0, 0.1, 0.1),
                    );
                    for stop in &p.stops {
                        let at = tf.transform_point(Vec3::new(stop - p.length / 2.0, 0.01, 0.0));
                        gizmos.line(at - tf.rotation * Vec3::Z * 0.7, at + tf.rotation * Vec3::Z * 0.7, Color::srgba(1.0, 1.0, 0.2, 0.6));
                    }
                }
                for sensor in &layout.sensors {
                    let (a, b) = sim::photo_beam(&surface, sensor);
                    let color = if io.get_bool(sensor.key) {
                        Color::srgb(1.0, 0.15, 0.1)
                    } else {
                        Color::srgba(0.1, 1.0, 0.3, 0.6)
                    };
                    gizmos.line(a, b, color);
                }
                for seg in &layout.segments {
                    let (x0, x1) = (seg.x0, seg.x1);
                    let motor = io.get_bool(seg.motor);
                    if !(selected || !motor) {
                        continue;
                    }
                    let (from, to) = if sign > 0.0 { (x0 + 0.12, x1 - 0.12) } else { (x1 - 0.12, x0 + 0.12) };
                    let color = if motor { Color::srgb(0.2, 0.9, 0.3) } else { Color::srgb(0.6, 0.6, 0.6) };
                    gizmos.arrow(
                        surface.point(Vec3::new(from, surface.top + 0.03, 0.0)),
                        surface.point(Vec3::new(to, surface.top + 0.03, 0.0)),
                        color,
                    );
                }
                if k == EquipmentKind::PalletizingConveyor && selected {
                    // Preview of the first layer pattern on a pallet at the stop.
                    let pallet = p.pallet_vec();
                    let size = crate::model::Params::default().box_vec();
                    let center = Vec3::new(p.length / 2.0 - pallet.x / 2.0, p.height + pallet.y / 2.0, 0.0);
                    let mut i = 0;
                    while let Some((pos, yaw)) = sim::pallet_slot(p, pallet, size, i) {
                        if pos.y > pallet.y / 2.0 + size.y {
                            break;
                        }
                        gizmos.rect(
                            Isometry3d::new(
                                tf.transform_point(center + pos - Vec3::Y * (size.y / 2.0 - 0.01)),
                                tf.rotation * Quat::from_rotation_y(yaw) * Quat::from_rotation_x(FRAC_PI_2),
                            ),
                            Vec2::new(size.x, size.z) * 0.96,
                            Color::srgba(1.0, 1.0, 1.0, 0.5),
                        );
                        i += 1;
                    }
                }
            }
            EquipmentKind::Sensor => {
                let (a, b) = sim::sensor_beam(eq, tf);
                let color = if io.get_bool("detected") {
                    Color::srgb(1.0, 0.15, 0.1)
                } else {
                    Color::srgb(0.1, 1.0, 0.3)
                };
                gizmos.line(a, b, color);
            }
            EquipmentKind::CardboardPlacer if selected => {
                // Delivery point: the pallet center must stop here.
                let at = crate::paper::delivery_point(eq, tf) + Vec3::Y * 0.02;
                gizmos.circle(Isometry3d::new(at, Quat::from_rotation_x(FRAC_PI_2)), 0.15, Color::srgb(1.0, 1.0, 0.2));
            }
            EquipmentKind::Robot if selected => {
                // Data sheet envelope of the model: reach, inner radius at the floor and
                // rear interference, plus the operating space of the flange center in the
                // current arm plane.
                let kin = crate::robot::kin(eq.params.robot_model).with_limits(&eq.params.axis_limits);
                let floor = Isometry3d::new(tf.translation + Vec3::Y * 0.01, Quat::from_rotation_x(FRAC_PI_2));
                // J1 range around the base: green arc of the allowed directions (a
                // spiral, so the part past ±180° shows twice), red rays at the limits,
                // red arc over the forbidden sector and the white current direction.
                let dir = |a: f32, r: f32| tf.transform_point(Vec3::new(r * a.cos(), 0.02, -r * a.sin()));
                let (lo, hi) = (kin.min[0], kin.max[0]);
                let steps = 96;
                let arc = (0..=steps).map(|k| {
                    let t = k as f32 / steps as f32;
                    dir(lo + (hi - lo) * t, 1.05 + 0.25 * t)
                });
                gizmos.linestrip(arc, Color::srgb(0.2, 0.95, 0.35));
                let red = Color::srgb(1.0, 0.15, 0.1);
                for (a, r_end) in [(lo, 1.05), (hi, 1.30)] {
                    gizmos.line(dir(a, 0.4), dir(a, kin.reach), red);
                    gizmos.line(dir(a, r_end - 0.08), dir(a, r_end + 0.08), red);
                }
                if hi - lo < 2.0 * std::f32::consts::PI {
                    let gap = 2.0 * std::f32::consts::PI - (hi - lo);
                    let forbidden = (0..=32).map(|k| dir(hi + gap * k as f32 / 32.0, 1.6));
                    gizmos.linestrip(forbidden, red);
                }
                // J1 = 0 and the current J1.
                gizmos.line(dir(0.0, 0.4), dir(0.0, 1.6), Color::srgba(0.2, 0.95, 0.35, 0.6));
                if let Ok(state) = robots.get(entity) {
                    let j1 = state.kin.axes(&state.pose).0[0];
                    gizmos.line(dir(j1, 0.4), dir(j1, 1.9), Color::WHITE);
                }
                gizmos.circle(floor, kin.reach, Color::srgba(1.0, 0.8, 0.0, 0.6));
                gizmos.circle(floor, kin.min_radius, Color::srgba(1.0, 0.5, 0.0, 0.5));
                gizmos.circle(floor, kin.rear_radius, Color::srgba(1.0, 0.2, 0.1, 0.6));
                let theta = robots.get(entity).map_or(0.0, |r| r.pose.theta);
                for line in kin.envelope_outline(24) {
                    let points = line
                        .into_iter()
                        .map(|p| tf.transform_point(Vec3::new(p.x * theta.cos(), p.y, -p.x * theta.sin())));
                    gizmos.linestrip(points, Color::srgba(0.3, 0.9, 1.0, 0.8));
                }
            }
            _ => {}
        }
    }
}

/// One equipment in an undo step: what the editor can change.
#[derive(Clone)]
struct Snap {
    name: String,
    kind: EquipmentKind,
    params: Params,
    transform: Transform,
    bindings: TagBindings,
}

/// The scene and a text key to compare scenes.
#[derive(Clone)]
struct Scene {
    items: Vec<Snap>,
    key: String,
}

impl Scene {
    fn new(mut items: Vec<Snap>) -> Self {
        items.sort_by(|a, b| a.name.cmp(&b.name));
        let key = serde_json::to_string(
            &items
                .iter()
                .map(|s| (&s.name, s.kind, &s.params, s.transform.translation.to_array(), s.transform.rotation.to_array(), &s.bindings))
                .collect::<Vec<_>>(),
        )
        .unwrap_or_default();
        Self { items, key }
    }
}

const UNDO_MAX: usize = 100;

/// Ctrl+Z / Ctrl+Y: edits of the scene (position, rotation, height, values in the
/// property panel, added, duplicated and deleted equipment). A step is recorded when
/// the edit ends (mouse released, text field left), so a drag is one step.
#[derive(Resource, Default)]
pub struct UndoHistory {
    undo: Vec<Scene>,
    redo: Vec<Scene>,
    last: Option<Scene>,
    /// A new scene was loaded: start a new history.
    pub reset: bool,
    /// Undo (true) or redo (false) requested outside the keyboard (tests).
    pub request: Option<bool>,
}

#[allow(clippy::too_many_arguments)]
pub fn undo_redo(
    mut history: ResMut<UndoHistory>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    palette: Res<Palette>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    focus: Res<UiFocus>,
    mut equipment: Query<(Entity, &mut Equipment, &mut Transform, &mut TagBindings)>,
    mut removed: RemovedComponents<Equipment>,
) {
    if history.reset {
        *history = UndoHistory::default();
    }
    let ctrl = keys.any_pressed([KeyCode::ControlLeft, KeyCode::ControlRight]);
    let shift = keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]);
    let request = history.request.take();
    let undo = (ctrl && !shift && keys.just_pressed(KeyCode::KeyZ)) || request == Some(true);
    let redo = (ctrl && (keys.just_pressed(KeyCode::KeyY) || (shift && keys.just_pressed(KeyCode::KeyZ)))) || request == Some(false);
    if !focus.keyboard_ui && (undo || redo) {
        let target = if undo { history.undo.pop() } else { history.redo.pop() };
        let Some(target) = target else { return };
        if let Some(current) = history.last.take() {
            if undo {
                history.redo.push(current);
            } else {
                history.undo.push(current);
            }
        }
        // Apply: same names are updated in place, the others spawned or removed.
        for snap in &target.items {
            match equipment.iter_mut().find(|(_, eq, ..)| eq.name == snap.name && eq.kind == snap.kind) {
                Some((entity, mut eq, mut tf, mut bindings)) => {
                    *tf = snap.transform;
                    let before = serde_json::to_string(&eq.params).unwrap_or_default();
                    if before != serde_json::to_string(&snap.params).unwrap_or_default() {
                        eq.params = snap.params.clone();
                        commands.entity(entity).insert(NeedsRebuild);
                    }
                    bindings.0 = snap.bindings.0.clone();
                }
                None => {
                    spawn_equipment(
                        &mut commands,
                        &mut meshes,
                        &palette,
                        Equipment { name: snap.name.clone(), kind: snap.kind, params: snap.params.clone() },
                        snap.transform,
                        snap.bindings.clone(),
                    );
                }
            }
        }
        for (entity, eq, ..) in &equipment {
            if !target.items.iter().any(|s| s.name == eq.name && s.kind == eq.kind) {
                commands.entity(entity).despawn();
            }
        }
        history.last = Some(target);
        removed.clear();
        return;
    }
    // Record when an edit ended.
    let changed = equipment.iter_mut().any(|(_, eq, tf, bindings)| eq.is_changed() || tf.is_changed() || bindings.is_changed());
    let touched = changed || removed.read().count() > 0 || history.last.is_none();
    if !touched || mouse.get_pressed().next().is_some() || focus.keyboard_ui {
        return;
    }
    let scene = Scene::new(
        equipment
            .iter()
            .map(|(_, eq, tf, bindings)| Snap {
                name: eq.name.clone(),
                kind: eq.kind,
                params: eq.params.clone(),
                transform: *tf,
                bindings: bindings.clone(),
            })
            .collect(),
    );
    match history.last.take() {
        Some(last) if last.key != scene.key => {
            if std::env::var("PDT_TRACE").is_ok() {
                let differs: Vec<&str> = scene
                    .items
                    .iter()
                    .filter(|s| {
                        !last.items.iter().any(|o| {
                            o.name == s.name
                                && o.transform == s.transform
                                && serde_json::to_string(&o.params).ok() == serde_json::to_string(&s.params).ok()
                        })
                    })
                    .map(|s| s.name.as_str())
                    .collect();
                eprintln!("desfazer: passo {} gravado ({})", history.undo.len() + 1, differs.join(", "));
            }
            history.undo.push(last);
            if history.undo.len() > UNDO_MAX {
                history.undo.remove(0);
            }
            history.redo.clear();
            history.last = Some(scene);
        }
        Some(last) => history.last = Some(last),
        None => history.last = Some(scene),
    }
}
