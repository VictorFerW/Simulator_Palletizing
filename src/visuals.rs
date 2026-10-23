//! Parametric meshes for the catalog. Every equipment is a root entity with
//! `Equipment` + `Transform`; its meshes are children so they can be rebuilt
//! when parameters change. Replace a visual with a licensed GLB later without
//! touching the root components.

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;

use crate::conveyor;
use crate::model::{Equipment, EquipmentKind, RobotModel};
use crate::robot::{self, RobotPart};

#[derive(Resource)]
pub struct Palette {
    pub fanuc_yellow: Handle<StandardMaterial>,
    pub dark: Handle<StandardMaterial>,
    pub steel: Handle<StandardMaterial>,
    pub frame_blue: Handle<StandardMaterial>,
    pub frame_grey: Handle<StandardMaterial>,
    pub belt: Handle<StandardMaterial>,
    pub belt_mark: Handle<StandardMaterial>,
    pub roller: Handle<StandardMaterial>,
    pub poly_roller: Handle<StandardMaterial>,
    pub wood: Handle<StandardMaterial>,
    pub emitter: Handle<StandardMaterial>,
    pub remover: Handle<StandardMaterial>,
    pub cardboard: Handle<StandardMaterial>,
    pub tape: Handle<StandardMaterial>,
    pub floor: Handle<StandardMaterial>,
    pub kuka_orange: Handle<StandardMaterial>,
    /// Manufacturer models of the robots whose GLBs are in `assets/`.
    pub robot_cad: Vec<(RobotModel, Vec<RobotCadPart>)>,
}

/// One part of the manufacturer's robot model: which link it follows, mesh, material.
pub type RobotCadPart = (robot::CadPart, Handle<Mesh>, Handle<StandardMaterial>);

/// Same lookup as Bevy's file asset reader: BEVY_ASSET_ROOT, CARGO_MANIFEST_DIR or the exe folder.
pub fn asset_root() -> std::path::PathBuf {
    std::env::var_os("BEVY_ASSET_ROOT")
        .or_else(|| std::env::var_os("CARGO_MANIFEST_DIR"))
        .map(std::path::PathBuf::from)
        .or_else(|| std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.to_path_buf())))
        .unwrap_or_default()
        .join("assets")
}

impl FromWorld for Palette {
    fn from_world(world: &mut World) -> Self {
        let mut materials = world.resource_mut::<Assets<StandardMaterial>>();
        let mut solid = |r: f32, g: f32, b: f32, metallic: f32, roughness: f32| {
            materials.add(StandardMaterial {
                base_color: Color::srgb(r, g, b),
                metallic,
                perceptual_roughness: roughness,
                ..default()
            })
        };
        let fanuc_yellow = solid(0.98, 0.78, 0.02, 0.1, 0.45);
        let kuka_orange = solid(1.0, 0.40, 0.0, 0.1, 0.45);
        let dark = solid(0.10, 0.11, 0.12, 0.2, 0.6);
        let steel = solid(0.62, 0.64, 0.67, 0.8, 0.35);
        let frame_blue = solid(0.12, 0.33, 0.60, 0.3, 0.5);
        let frame_grey = solid(0.42, 0.45, 0.48, 0.5, 0.45);
        let belt = solid(0.08, 0.08, 0.09, 0.0, 0.9);
        let belt_mark = solid(0.30, 0.30, 0.32, 0.0, 0.9);
        let roller = solid(0.75, 0.77, 0.80, 0.9, 0.25);
        let poly_roller = solid(0.92, 0.92, 0.88, 0.0, 0.6);
        let wood = solid(0.72, 0.55, 0.34, 0.0, 0.85);
        let cardboard = solid(0.70, 0.52, 0.32, 0.0, 0.9);
        let tape = solid(0.80, 0.70, 0.50, 0.0, 0.5);
        let floor = solid(0.30, 0.32, 0.34, 0.0, 0.95);
        let emitter = materials.add(StandardMaterial {
            base_color: Color::srgba(0.1, 0.8, 0.3, 0.55),
            alpha_mode: AlphaMode::Blend,
            ..default()
        });
        let remover = materials.add(StandardMaterial {
            base_color: Color::srgba(0.9, 0.15, 0.1, 0.55),
            alpha_mode: AlphaMode::Blend,
            ..default()
        });
        let mut robot_cad = Vec::new();
        for model in RobotModel::ALL {
            let (parts, dir) = robot::CadPart::of(model);
            if !parts.iter().all(|(_, file)| asset_root().join(dir).join(format!("{file}.glb")).exists()) {
                continue;
            }
            let assets = world.resource::<AssetServer>();
            let list = parts
                .iter()
                .map(|(part, file)| {
                    use robot::CadPart as P;
                    let material = match part {
                        P::LevelRod | P::LevelLink | P::LeverRod | P::KukaFlange => steel.clone(),
                        P::Flange | P::KukaBase => dark.clone(),
                        P::KukaColumn | P::KukaLinkArm | P::KukaArm | P::KukaWrist => kuka_orange.clone(),
                        _ => fanuc_yellow.clone(),
                    };
                    let mesh = assets.load(GltfAssetLabel::Primitive { mesh: 0, primitive: 0 }.from_asset(format!("{dir}/{file}.glb")));
                    (*part, mesh, material)
                })
                .collect();
            robot_cad.push((model, list));
        }
        Self {
            fanuc_yellow,
            dark,
            steel,
            frame_blue,
            frame_grey,
            belt,
            belt_mark,
            roller,
            poly_roller,
            wood,
            emitter,
            remover,
            cardboard,
            tape,
            floor,
            kuka_orange,
            robot_cad,
        }
    }
}

/// Moving cleat on a belt conveyor, offset along local X while the motor runs.
#[derive(Component)]
pub struct BeltMark {
    pub base_x: f32,
}

/// Roller that spins around local Z while its stage motor runs.
#[derive(Component)]
pub struct Roller {
    pub stage: usize,
    pub radius: f32,
}

/// Plate of a mesa pusher/gripper; index into `conveyor::layout(..).pushers`.
#[derive(Component)]
pub struct ActuatorPlate {
    pub index: usize,
}

/// Stop bar (batente) that rises when its input signal is on.
#[derive(Component)]
pub struct StopBar {
    pub signal: &'static str,
    pub lowered: f32,
    pub raised: f32,
}

pub const BELT_MARK_SPACING: f32 = 0.5;
/// TDP 1560 x 1280 x 120: rolo Ø76,2 a cada 120 mm.
const TDP_ROLLER_RADIUS: f32 = 0.0381;
const TDP_ROLLER_PITCH: f32 = 0.12;
const TABLE_ROLLER_RADIUS: f32 = 0.025;
const TABLE_ROLLER_PITCH: f32 = 0.075;

pub fn build_visual(
    commands: &mut Commands,
    entity: Entity,
    equipment: &Equipment,
    meshes: &mut Assets<Mesh>,
    palette: &Palette,
) {
    let p = equipment.params.clone();
    let kind = equipment.kind;
    commands.entity(entity).with_children(|parent| match kind {
        EquipmentKind::BeltConveyor => {
            conveyor_frame(parent, meshes, palette, p.length, p.width, p.height);
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(p.length, 0.04, p.width))),
                MeshMaterial3d(palette.belt.clone()),
                Transform::from_xyz(0.0, p.height - 0.02, 0.0),
            ));
            for end in [-1.0, 1.0] {
                parent.spawn((
                    Mesh3d(meshes.add(Cylinder::new(0.05, p.width))),
                    MeshMaterial3d(palette.steel.clone()),
                    Transform::from_xyz(end * p.length / 2.0, p.height - 0.045, 0.0)
                        .with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
                ));
            }
            let mark = meshes.add(Cuboid::new(0.03, 0.004, p.width * 0.96));
            let count = (p.length / BELT_MARK_SPACING).floor() as i32;
            for i in 0..count {
                let base_x = -p.length / 2.0 + (i as f32 + 0.5) * BELT_MARK_SPACING;
                parent.spawn((
                    Mesh3d(mark.clone()),
                    MeshMaterial3d(palette.belt_mark.clone()),
                    Transform::from_xyz(base_x, p.height + 0.001, 0.0),
                    BeltMark { base_x },
                ));
            }
        }
        EquipmentKind::Table2Stage | EquipmentKind::Table3Stage => table_visual(parent, meshes, palette, equipment),
        EquipmentKind::PalletConveyor | EquipmentKind::PalletizingConveyor => {
            tdp_visual(parent, meshes, palette, p.length, p.width, p.height);
            if kind == EquipmentKind::PalletizingConveyor {
                // Positioning stop at the discharge end and centering guides.
                parent.spawn((
                    Mesh3d(meshes.add(Cuboid::new(0.05, 0.14, p.width * 0.8))),
                    MeshMaterial3d(palette.fanuc_yellow.clone()),
                    Transform::from_xyz(p.length / 2.0 + 0.03, p.height, 0.0),
                    StopBar { signal: "stop", lowered: p.height - 0.1, raised: p.height + 0.05 },
                ));
                let guide = meshes.add(Cuboid::new(0.5, 0.12, 0.03));
                for side in [-1.0, 1.0] {
                    parent.spawn((
                        Mesh3d(guide.clone()),
                        MeshMaterial3d(palette.fanuc_yellow.clone()),
                        Transform::from_xyz(0.0, p.height + 0.08, side * (p.width / 2.0 + 0.02)),
                    ));
                }
            }
        }
        EquipmentKind::Robot => {
            if p.robot_base {
                robot_base_visual(parent, meshes, palette, &p);
            }
            robot_visual(parent, meshes, palette, p.robot_model)
        }
        EquipmentKind::CardboardPlacer => paper_visual(parent, meshes, palette, &p),
        EquipmentKind::PalletElevator => elevator_visual(parent, meshes, palette, &p),
        EquipmentKind::ShuttleCar => {
            // Rail, cable chain and the distance sensor at the rail start.
            let rail = meshes.add(Cuboid::new(p.length, 0.05, 0.07));
            for z in [-0.45, 0.45] {
                parent.spawn((
                    Mesh3d(rail.clone()),
                    MeshMaterial3d(palette.steel.clone()),
                    Transform::from_xyz(0.0, 0.025, z),
                ));
            }
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(p.length, 0.1, 0.12))),
                MeshMaterial3d(palette.dark.clone()),
                Transform::from_xyz(0.0, 0.05, p.car_length / 2.0 + 0.2),
            ));
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.08, 0.1, 0.1))),
                MeshMaterial3d(palette.fanuc_yellow.clone()),
                Transform::from_xyz(-p.length / 2.0 - 0.06, 0.25, 0.0),
            ));
            // Car: chassis + TDP, transporting across the rail.
            parent
                .spawn((
                    Transform::from_xyz(p.stops.first().copied().unwrap_or(0.0) - p.length / 2.0, 0.0, 0.0),
                    Visibility::default(),
                    crate::shuttle::ShuttleBody,
                ))
                .with_children(|car| {
                    car.spawn((
                        Mesh3d(meshes.add(Cuboid::new(p.car_width + 0.1, 0.12, p.car_length * 0.8))),
                        MeshMaterial3d(palette.frame_blue.clone()),
                        Transform::from_xyz(0.0, 0.11, 0.0),
                    ));
                    car.spawn((Transform::from_rotation(Quat::from_rotation_y(-FRAC_PI_2)), Visibility::default()))
                        .with_children(|tdp| tdp_visual(tdp, meshes, palette, p.car_length, p.car_width, p.height));
                });
        }
        EquipmentKind::FullCar => fullcar_visual(parent, meshes, palette, &p),
        EquipmentKind::PalletDispenser => {
            // Magazine: four posts holding a stack of empty pallets above the TDP.
            let post_h = 1.6;
            let post = meshes.add(Cuboid::new(0.08, post_h, 0.08));
            let pallet = p.pallet_vec();
            for (sx, sz) in [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)] {
                parent.spawn((
                    Mesh3d(post.clone()),
                    MeshMaterial3d(palette.frame_blue.clone()),
                    Transform::from_xyz(sx * (pallet.x / 2.0 + 0.08), post_h / 2.0, sz * (pallet.z / 2.0 + 0.08)),
                ));
            }
            for i in 0..4 {
                parent
                    .spawn((
                        Transform::from_xyz(0.0, p.height + 0.45 + i as f32 * pallet.y, 0.0),
                        Visibility::default(),
                    ))
                    .with_children(|stack| pallet_mesh(stack, meshes, palette, pallet.x, pallet.z));
            }
        }
        EquipmentKind::BoxEmitter | EquipmentKind::BoxRemover => {
            let material = if kind == EquipmentKind::BoxEmitter {
                palette.emitter.clone()
            } else {
                palette.remover.clone()
            };
            let post = meshes.add(Cuboid::new(0.05, 0.9, 0.05));
            for (sx, sz) in [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)] {
                parent.spawn((
                    Mesh3d(post.clone()),
                    MeshMaterial3d(material.clone()),
                    Transform::from_xyz(sx * p.length / 2.0, p.height + 0.45, sz * (p.width / 2.0 + 0.05)),
                ));
            }
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(p.length + 0.05, 0.05, p.width + 0.15))),
                MeshMaterial3d(material),
                Transform::from_xyz(0.0, p.height + 0.9, 0.0),
            ));
        }
        EquipmentKind::Sensor => {
            let post_h = p.height;
            parent.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.02, post_h))),
                MeshMaterial3d(palette.steel.clone()),
                Transform::from_xyz(0.0, post_h / 2.0, -0.05),
            ));
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.06, 0.09, 0.05))),
                MeshMaterial3d(palette.dark.clone()),
                Transform::from_xyz(0.0, post_h, -0.02),
            ));
            parent.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.02, post_h))),
                MeshMaterial3d(palette.steel.clone()),
                Transform::from_xyz(0.0, post_h / 2.0, p.width + 0.05),
            ));
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.07, 0.07, 0.01))),
                MeshMaterial3d(palette.remover.clone()),
                Transform::from_xyz(0.0, post_h, p.width + 0.02),
            ));
        }
    });
}

fn conveyor_frame(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    palette: &Palette,
    length: f32,
    width: f32,
    height: f32,
) {
    let rail = meshes.add(Cuboid::new(length, 0.12, 0.04));
    for side in [-1.0, 1.0] {
        parent.spawn((
            Mesh3d(rail.clone()),
            MeshMaterial3d(palette.frame_blue.clone()),
            Transform::from_xyz(0.0, height - 0.04, side * (width / 2.0 + 0.02)),
        ));
    }
    legs(parent, meshes, palette, length, width, height - 0.10, 1.5);
}

fn legs(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, length: f32, width: f32, leg_h: f32, span: f32) {
    let leg_h = leg_h.max(0.05);
    let leg = meshes.add(Cuboid::new(0.06, leg_h, 0.06));
    let foot = meshes.add(Cylinder::new(0.06, 0.012));
    let spans = (length / span).ceil().max(1.0) as i32;
    for i in 0..=spans {
        let x = -length / 2.0 + 0.1 + (length - 0.2) * i as f32 / spans as f32;
        for side in [-1.0, 1.0] {
            let z = side * (width / 2.0 + 0.02);
            parent.spawn((
                Mesh3d(leg.clone()),
                MeshMaterial3d(palette.steel.clone()),
                Transform::from_xyz(x, leg_h / 2.0, z),
            ));
            parent.spawn((
                Mesh3d(foot.clone()),
                MeshMaterial3d(palette.dark.clone()),
                Transform::from_xyz(x, 0.006, z),
            ));
        }
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.04, 0.04, width))),
            MeshMaterial3d(palette.steel.clone()),
            Transform::from_xyz(x, 0.2, 0.0),
        ));
    }
}

#[allow(clippy::too_many_arguments)]
fn spawn_rollers(
    parent: &mut ChildSpawnerCommands,
    meshes: &mut Assets<Mesh>,
    material: &Handle<StandardMaterial>,
    (x0, x1): (f32, f32),
    stage: usize,
    radius: f32,
    pitch: f32,
    width: f32,
    top: f32,
) {
    let mesh = meshes.add(Cylinder::new(radius, width));
    let count = ((x1 - x0) / pitch).floor().max(1.0) as i32;
    let start = (x0 + x1) / 2.0 - (count as f32 - 1.0) * pitch / 2.0;
    for i in 0..count {
        parent.spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_xyz(start + i as f32 * pitch, top - radius, 0.0)
                .with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
            Roller { stage, radius },
        ));
    }
}

/// Mesa Selgron: perforated side plates, one roller bed per segment, pneumatic
/// stops (bloqueios) and the coleta batente, lateral pusher/gripper plates and
/// the photocells, all positioned from `conveyor::layout`.
fn table_visual(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, equipment: &Equipment) {
    let p = &equipment.params;
    let layout = conveyor::layout(equipment);
    let (l, w, h) = (p.length, p.width, p.height);
    let plate = meshes.add(Cuboid::new(l, 0.36, 0.012));
    for side in [-1.0, 1.0] {
        parent.spawn((
            Mesh3d(plate.clone()),
            MeshMaterial3d(palette.frame_grey.clone()),
            Transform::from_xyz(0.0, h - 0.16, side * (w / 2.0 + 0.04)),
        ));
    }
    legs(parent, meshes, palette, l, w + 0.06, h - 0.34, 1.2);
    for (i, seg) in layout.segments.iter().enumerate() {
        let material = if equipment.kind == EquipmentKind::Table3Stage && i == 1 {
            &palette.poly_roller
        } else {
            &palette.roller
        };
        spawn_rollers(parent, meshes, material, (seg.x0, seg.x1), i, TABLE_ROLLER_RADIUS, TABLE_ROLLER_PITCH, w, h);
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.3, 0.2, 0.22))),
            MeshMaterial3d(palette.dark.clone()),
            Transform::from_xyz((seg.x0 + seg.x1) / 2.0, h - 0.45, w / 2.0 - 0.1),
        ));
    }
    for barrier in &layout.barriers {
        // Pneumatic stop rising between the rollers.
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.03, 0.14, w * 0.95))),
            MeshMaterial3d(palette.fanuc_yellow.clone()),
            Transform::from_xyz(barrier.x, h - 0.1, 0.0),
            StopBar { signal: barrier.key, lowered: h - 0.1, raised: h + 0.06 },
        ));
    }
    for (index, pusher) in layout.pushers.iter().enumerate() {
        if pusher.comb {
            // Garra: comb whose fingers sit in the gaps between the rollers.
            let seg = layout
                .segments
                .iter()
                .find(|s| s.x0 <= pusher.x0 && pusher.x1 <= s.x1)
                .map_or((pusher.x0, pusher.x1), |s| (s.x0, s.x1));
            let count = ((seg.1 - seg.0) / TABLE_ROLLER_PITCH).floor().max(1.0) as i32;
            let first = (seg.0 + seg.1) / 2.0 - (count as f32 - 1.0) * TABLE_ROLLER_PITCH / 2.0;
            let finger = meshes.add(Cuboid::new(0.022, 0.03, p.box_size[2] * 0.9));
            let carriage = meshes.add(Cuboid::new(pusher.x1 - pusher.x0, 0.03, 0.08));
            parent
                .spawn((
                    Transform::from_xyz(0.0, h - 0.07, pusher.face_start),
                    Visibility::default(),
                    ActuatorPlate { index },
                ))
                .with_children(|comb| {
                    comb.spawn((
                        Mesh3d(carriage),
                        MeshMaterial3d(palette.dark.clone()),
                        Transform::from_xyz((pusher.x0 + pusher.x1) / 2.0, -0.06, 0.0),
                    ));
                    for k in 0..count - 1 {
                        let x = first + (k as f32 + 0.5) * TABLE_ROLLER_PITCH;
                        if x < pusher.x0 || x > pusher.x1 {
                            continue;
                        }
                        comb.spawn((
                            Mesh3d(finger.clone()),
                            MeshMaterial3d(palette.fanuc_yellow.clone()),
                            Transform::from_xyz(x, 0.0, 0.0),
                        ));
                    }
                });
            continue;
        }
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(pusher.x1 - pusher.x0, 0.2, 0.04))),
            MeshMaterial3d(palette.frame_blue.clone()),
            Transform::from_xyz((pusher.x0 + pusher.x1) / 2.0, h + 0.12, pusher.face_start + 0.02),
            ActuatorPlate { index },
        ));
        if pusher.train {
            // Physical guide on the far side: the row is pushed against it.
            let dir = (pusher.face_end - pusher.face_start).signum();
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(pusher.x1 - pusher.x0 + 0.3, 0.14, 0.02))),
                MeshMaterial3d(palette.frame_grey.clone()),
                Transform::from_xyz((pusher.x0 + pusher.x1) / 2.0, h + 0.09, dir * (w / 2.0 + 0.005)),
            ));
        }
        // Cylinder body outside the frame.
        parent.spawn((
            Mesh3d(meshes.add(Cylinder::new(0.04, 0.5))),
            MeshMaterial3d(palette.steel.clone()),
            Transform::from_xyz((pusher.x0 + pusher.x1) / 2.0, h + 0.12, w / 2.0 + 0.3)
                .with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
        ));
    }
    let cell = meshes.add(Cuboid::new(0.04, 0.05, 0.04));
    for sensor in &layout.sensors {
        // Photocell on the far side plate (beam drawn as a gizmo).
        parent.spawn((
            Mesh3d(cell.clone()),
            MeshMaterial3d(palette.dark.clone()),
            Transform::from_xyz(sensor.x, h + 0.08, -(w / 2.0 + 0.06)),
        ));
    }
}

/// Colocador de papelão C2370: overhead beam on three legs, cardboard deposit
/// under the pick end, carriage with the vertical guide and the vacuum head.
fn paper_visual(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, p: &crate::model::Params) {
    use crate::paper::{DEPOSIT_FLOOR, HEAD_RETRACTED_Y, PaperCarriage, PaperHead, PaperStack};
    let beam_y = HEAD_RETRACTED_Y + 0.95;
    let (pick, delivery) = (-p.length / 2.0, p.length / 2.0);
    // The last leg stands clear of the pallet (and of the CVZ) at the delivery end.
    let (beam_start, beam_end) = (pick - 0.6, delivery + 0.95);
    let beam_len = beam_end - beam_start + 0.2;
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(beam_len, 0.16, 0.18))),
        MeshMaterial3d(palette.frame_grey.clone()),
        Transform::from_xyz((beam_start + beam_end) / 2.0, beam_y, -0.55),
    ));
    let leg = meshes.add(Cuboid::new(0.14, beam_y, 0.14));
    for x in [beam_start, (pick + delivery) / 2.0, beam_end] {
        parent.spawn((
            Mesh3d(leg.clone()),
            MeshMaterial3d(palette.frame_grey.clone()),
            Transform::from_xyz(x, beam_y / 2.0, -0.55),
        ));
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.4, 0.02, 0.4))),
            MeshMaterial3d(palette.dark.clone()),
            Transform::from_xyz(x, 0.01, -0.55),
        ));
    }
    // Motor (SA37) and cable chain on the beam.
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.25, 0.3, 0.25))),
        MeshMaterial3d(palette.dark.clone()),
        Transform::from_xyz(beam_end, beam_y - 0.1, -0.35),
    ));
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(beam_len * 0.6, 0.08, 0.1))),
        MeshMaterial3d(palette.dark.clone()),
        Transform::from_xyz(pick + beam_len * 0.25, beam_y + 0.12, -0.55),
    ));
    // Cardboard deposit with its stack.
    let (sx, sz) = (p.pallet_size[0], p.pallet_size[2]);
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(sx + 0.12, 0.03, sz + 0.12))),
        MeshMaterial3d(palette.dark.clone()),
        Transform::from_xyz(pick, DEPOSIT_FLOOR - 0.015, 0.0),
    ));
    let wall_x = meshes.add(Cuboid::new(0.02, 0.9, sz + 0.12));
    let wall_z = meshes.add(Cuboid::new(sx + 0.12, 0.9, 0.02));
    for side in [-1.0, 1.0] {
        parent.spawn((
            Mesh3d(wall_x.clone()),
            MeshMaterial3d(palette.frame_grey.clone()),
            Transform::from_xyz(pick + side * (sx / 2.0 + 0.06), 0.45, 0.0),
        ));
    }
    parent.spawn((
        Mesh3d(wall_z.clone()),
        MeshMaterial3d(palette.frame_grey.clone()),
        Transform::from_xyz(pick, 0.45, -(sz / 2.0 + 0.06)),
    ));
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(sx - 0.02, 1.0, sz - 0.02))),
        MeshMaterial3d(palette.cardboard.clone()),
        Transform::from_xyz(pick, DEPOSIT_FLOOR + 0.25, 0.0).with_scale(Vec3::new(1.0, 0.5, 1.0)),
        PaperStack,
    ));
    // Carriage running on the beam, with the head hanging from its guide.
    parent
        .spawn((Transform::from_xyz(pick, 0.0, 0.0), Visibility::default(), PaperCarriage))
        .with_children(|carriage| {
            carriage.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.45, 0.3, 0.6))),
                MeshMaterial3d(palette.frame_blue.clone()),
                Transform::from_xyz(0.0, beam_y, -0.3),
            ));
            carriage.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.12, 0.9, 0.12))),
                MeshMaterial3d(palette.frame_grey.clone()),
                Transform::from_xyz(0.0, beam_y + 0.45, 0.0),
            ));
            carriage
                .spawn((Transform::from_xyz(0.0, HEAD_RETRACTED_Y, 0.0), Visibility::default(), PaperHead))
                .with_children(|head| {
                    head.spawn((
                        Mesh3d(meshes.add(Cylinder::new(0.035, 1.3))),
                        MeshMaterial3d(palette.steel.clone()),
                        Transform::from_xyz(0.0, 0.75, 0.0),
                    ));
                    head.spawn((
                        Mesh3d(meshes.add(Cuboid::new(0.9, 0.05, 0.7))),
                        MeshMaterial3d(palette.frame_grey.clone()),
                        Transform::from_xyz(0.0, 0.11, 0.0),
                    ));
                    let cup = meshes.add(Cylinder::new(0.045, 0.06));
                    for x in [-0.35, 0.0, 0.35] {
                        for z in [-0.25, 0.25] {
                            head.spawn((
                                Mesh3d(cup.clone()),
                                MeshMaterial3d(palette.dark.clone()),
                                Transform::from_xyz(x, 0.03, z),
                            ));
                        }
                    }
                });
        });
}

/// TDP 1560 x 1280 x 120 (Selgron): C-profile sides, rollers Ø76,2 every 120 mm,
/// chain guard on the drive side, gearmotor underneath, adjustable feet.
fn tdp_visual(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, length: f32, width: f32, height: f32) {
    let profile = meshes.add(Cuboid::new(length, 0.15, 0.05));
    for side in [-1.0, 1.0] {
        parent.spawn((
            Mesh3d(profile.clone()),
            MeshMaterial3d(palette.frame_grey.clone()),
            Transform::from_xyz(0.0, height + 0.018 - 0.075, side * (width / 2.0 + 0.025)),
        ));
    }
    // Chain guard (proteção da corrente) on the drive side.
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(length, 0.2, 0.02))),
        MeshMaterial3d(palette.frame_blue.clone()),
        Transform::from_xyz(0.0, height - 0.1, -(width / 2.0 + 0.06)),
    ));
    spawn_rollers(
        parent,
        meshes,
        &palette.roller,
        (-length / 2.0, length / 2.0),
        0,
        TDP_ROLLER_RADIUS,
        TDP_ROLLER_PITCH,
        width,
        height,
    );
    let leg_h = height - 0.12;
    let leg = meshes.add(Cuboid::new(0.12, leg_h, 0.1));
    let foot = meshes.add(Cylinder::new(0.07, 0.015));
    for sx in [-1.0, 1.0] {
        for sz in [-1.0, 1.0] {
            let (x, z) = (sx * (length / 2.0 - 0.13), sz * (width / 2.0 + 0.02));
            parent.spawn((
                Mesh3d(leg.clone()),
                MeshMaterial3d(palette.frame_grey.clone()),
                Transform::from_xyz(x, leg_h / 2.0, z),
            ));
            parent.spawn((
                Mesh3d(foot.clone()),
                MeshMaterial3d(palette.dark.clone()),
                Transform::from_xyz(x, 0.008, z),
            ));
        }
    }
    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.035, width))),
        MeshMaterial3d(palette.steel.clone()),
        Transform::from_xyz(0.0, height - 0.32, 0.0).with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
    ));
    // Gearmotor (15 m/min).
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.2, 0.18, 0.4))),
        MeshMaterial3d(palette.dark.clone()),
        Transform::from_xyz(-length / 2.0 + 0.45, height - 0.3, -width / 2.0 + 0.25),
    ));
}

/// Carro cheio giratório: rails, cable chain, distance sensor, the car chassis
/// with its slewing ring and the rotating TDP (1829 x 1380, rollers on top).
fn fullcar_visual(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, p: &crate::model::Params) {
    let rail = meshes.add(Cuboid::new(p.length, 0.05, 0.07));
    for z in [-0.5, 0.5] {
        parent.spawn((
            Mesh3d(rail.clone()),
            MeshMaterial3d(palette.steel.clone()),
            Transform::from_xyz(0.0, 0.025, z),
        ));
    }
    let reach = crate::fullcar::turn_radius(p);
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(p.length, 0.1, 0.12))),
        MeshMaterial3d(palette.dark.clone()),
        Transform::from_xyz(0.0, 0.05, reach + 0.2),
    ));
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.08, 0.1, 0.1))),
        MeshMaterial3d(palette.fanuc_yellow.clone()),
        Transform::from_xyz(-p.length / 2.0 - 0.06, 0.25, 0.0),
    ));
    parent
        .spawn((
            Transform::from_xyz(p.stops.first().copied().unwrap_or(0.0) - p.length / 2.0, 0.0, 0.0),
            Visibility::default(),
            crate::shuttle::ShuttleBody,
        ))
        .with_children(|car| {
            // Chassis with wheels and drive, slewing ring.
            car.spawn((
                Mesh3d(meshes.add(Cuboid::new(1.3, 0.22, 1.2))),
                MeshMaterial3d(palette.frame_grey.clone()),
                Transform::from_xyz(0.0, 0.17, 0.0),
            ));
            let wheel = meshes.add(Cylinder::new(0.1, 0.06));
            for (x, z) in [(-0.55, -0.5), (0.55, -0.5), (-0.55, 0.5), (0.55, 0.5)] {
                car.spawn((
                    Mesh3d(wheel.clone()),
                    MeshMaterial3d(palette.dark.clone()),
                    Transform::from_xyz(x, 0.1, z).with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
                ));
            }
            car.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.45, 0.18, 0.2))),
                MeshMaterial3d(palette.dark.clone()),
                Transform::from_xyz(0.3, 0.2, 0.62),
            ));
            car.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.48, 0.06))),
                MeshMaterial3d(palette.steel.clone()),
                Transform::from_xyz(0.0, 0.31, 0.0),
            ));
            car.spawn((
                Transform::from_rotation(Quat::from_rotation_y(-FRAC_PI_2)),
                Visibility::default(),
                crate::fullcar::FullCarTop,
            ))
            .with_children(|top| {
                // Round TDP: checker plate disc, rim, and rollers cut to the circle so the
                // turning TDP clears the neighbouring TDPs.
                let (cl, cw) = (p.car_length, p.car_width);
                let radius = crate::fullcar::turn_radius(p);
                top.spawn((
                    Mesh3d(meshes.add(Cylinder::new(radius, 0.03))),
                    MeshMaterial3d(palette.steel.clone()),
                    Transform::from_xyz(0.0, p.height - 0.085, 0.0),
                ));
                top.spawn((
                    Mesh3d(meshes.add(Cylinder::new(radius + 0.01, 0.1))),
                    MeshMaterial3d(palette.dark.clone()),
                    Transform::from_xyz(0.0, p.height - 0.15, 0.0),
                ));
                // Side guides along the transport direction, inside the circle.
                let guide_half = (radius * radius - (cw / 2.0 + 0.04).powi(2)).max(0.0).sqrt();
                let guide = meshes.add(Cuboid::new(2.0 * guide_half, 0.08, 0.04));
                for side in [-1.0, 1.0] {
                    top.spawn((
                        Mesh3d(guide.clone()),
                        MeshMaterial3d(palette.frame_grey.clone()),
                        Transform::from_xyz(0.0, p.height - 0.03, side * (cw / 2.0 + 0.04)),
                    ));
                }
                let count = (cl / TDP_ROLLER_PITCH).floor().max(1.0) as i32;
                let start = -(count as f32 - 1.0) * TDP_ROLLER_PITCH / 2.0;
                for i in 0..count {
                    let x = start + i as f32 * TDP_ROLLER_PITCH;
                    let half = ((radius - 0.05).powi(2) - x * x).max(0.0).sqrt().min(cw / 2.0);
                    if half < 0.1 {
                        continue;
                    }
                    top.spawn((
                        Mesh3d(meshes.add(Cylinder::new(TDP_ROLLER_RADIUS, 2.0 * half))),
                        MeshMaterial3d(palette.roller.clone()),
                        Transform::from_xyz(x, p.height - TDP_ROLLER_RADIUS, 0.0).with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
                        Roller { stage: 0, radius: TDP_ROLLER_RADIUS },
                    ));
                }
            });
        });
}

/// Elevador de pallets: four columns around the TDP, top rails and the drive
/// beam, the clamp bars with their forks and the stack of empty pallets.
fn elevator_visual(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, p: &crate::model::Params) {
    use crate::elevator::{ElevatorClamp, ElevatorPallet, MAX_STACK, levels};
    let top = p.height + 2.0;
    let (hx, hz) = (p.length / 2.0, p.width / 2.0);
    let post = meshes.add(Cuboid::new(0.12, top, 0.12));
    for x in [-hx, hx] {
        for z in [-hz, hz] {
            parent.spawn((
                Mesh3d(post.clone()),
                MeshMaterial3d(palette.frame_grey.clone()),
                Transform::from_xyz(x, top / 2.0, z),
            ));
        }
    }
    let rail = meshes.add(Cuboid::new(p.length + 0.12, 0.14, 0.12));
    for z in [-hz, hz] {
        parent.spawn((
            Mesh3d(rail.clone()),
            MeshMaterial3d(palette.frame_grey.clone()),
            Transform::from_xyz(0.0, top - 0.07, z),
        ));
    }
    // Drive beam (yellow) with the motor-reducer, at the back.
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.16, 0.28, p.width + 0.12))),
        MeshMaterial3d(palette.fanuc_yellow.clone()),
        Transform::from_xyz(-hx, top - 0.14, 0.0),
    ));
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.3, 0.3, 0.35))),
        MeshMaterial3d(palette.dark.clone()),
        Transform::from_xyz(-hx, top + 0.15, 0.25),
    ));
    // Vertical guides on the side columns.
    let guide = meshes.add(Cuboid::new(0.05, top - p.height, 0.05));
    for z in [-hz, hz] {
        parent.spawn((
            Mesh3d(guide.clone()),
            MeshMaterial3d(palette.steel.clone()),
            Transform::from_xyz(0.0, (top + p.height) / 2.0, z * 0.97),
        ));
    }
    // Clamp bars: the bar outside, two forks entering the pallet side openings.
    let (px, pz) = (p.pallet_size[0], p.pallet_size[2]);
    let bar = meshes.add(Cuboid::new(px * 0.9, 0.12, 0.06));
    let fork = meshes.add(Cuboid::new(0.16, 0.06, 0.14));
    let arm = meshes.add(Cuboid::new(0.08, 0.08, hz - pz / 2.0));
    let z0 = levels(p)[2];
    for side in [-1.0f32, 1.0] {
        parent
            .spawn((
                Transform::from_xyz(0.0, p.height + z0 + p.pallet_size[1] / 2.0, side * (pz / 2.0 + 0.05)),
                Visibility::default(),
                ElevatorClamp(side),
            ))
            .with_children(|clamp| {
                clamp.spawn((
                    Mesh3d(bar.clone()),
                    MeshMaterial3d(palette.dark.clone()),
                    Transform::from_xyz(0.0, 0.0, side * 0.03),
                ));
                for x in [-1.0, 1.0] {
                    clamp.spawn((
                        Mesh3d(fork.clone()),
                        MeshMaterial3d(palette.fanuc_yellow.clone()),
                        Transform::from_xyz(x * px * 0.27, -0.01, -side * 0.06),
                    ));
                }
                // Arm to the carriage on the guide.
                clamp.spawn((
                    Mesh3d(arm.clone()),
                    MeshMaterial3d(palette.frame_grey.clone()),
                    Transform::from_xyz(0.0, 0.0, side * (0.06 + (hz - pz / 2.0) / 2.0)),
                ));
            });
    }
    for i in 0..p.stack_size.min(MAX_STACK) {
        parent
            .spawn((
                Transform::from_xyz(0.0, p.height + i as f32 * p.pallet_size[1], 0.0),
                Visibility::default(),
                ElevatorPallet(i),
            ))
            .with_children(|pallet| pallet_mesh(pallet, meshes, palette, px, pz));
    }
}

/// Wooden pallet with its bottom at local y = 0; `length` along X, `width` along Z.
pub fn pallet_mesh(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, length: f32, width: f32) {
    let thickness = 0.144;
    let boards = 7;
    let board_w = width / boards as f32 * 0.8;
    let board = meshes.add(Cuboid::new(length, 0.022, board_w));
    for i in 0..boards {
        let z = -width / 2.0 + board_w / 2.0 + i as f32 * (width - board_w) / (boards - 1) as f32;
        parent.spawn((
            Mesh3d(board.clone()),
            MeshMaterial3d(palette.wood.clone()),
            Transform::from_xyz(0.0, thickness - 0.011, z),
        ));
    }
    let block = meshes.add(Cuboid::new(0.145, 0.078, 0.145));
    let bottom = meshes.add(Cuboid::new(length, 0.022, 0.145));
    for zi in [-1.0, 0.0, 1.0] {
        let z = zi * (width / 2.0 - 0.0725);
        for xi in [-1.0, 0.0, 1.0] {
            parent.spawn((
                Mesh3d(block.clone()),
                MeshMaterial3d(palette.wood.clone()),
                Transform::from_xyz(xi * (length / 2.0 - 0.0725), 0.022 + 0.039, z),
            ));
        }
        parent.spawn((
            Mesh3d(bottom.clone()),
            MeshMaterial3d(palette.wood.clone()),
            Transform::from_xyz(0.0, 0.011, z),
        ));
    }
}

fn robot_visual(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, model: RobotModel) {
    if let Some((_, parts)) = palette.robot_cad.iter().find(|(m, _)| *m == model) {
        for (part, mesh, material) in parts {
            parent.spawn((Mesh3d(mesh.clone()), MeshMaterial3d(material.clone()), Transform::default(), *part));
        }
        robot_gripper(parent, meshes, palette);
        return;
    }
    // Parametric model.
    let kin = robot::kin(model);
    let paint = if model == RobotModel::KukaKr180R3200_2Pa { palette.kuka_orange.clone() } else { palette.fanuc_yellow.clone() };
    // Fixed base (J1 housing).
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.05, 0.12, 1.05))),
        MeshMaterial3d(palette.dark.clone()),
        Transform::from_xyz(0.0, 0.06, 0.0),
    ));
    parent.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.48, 0.22))),
        MeshMaterial3d(paint.clone()),
        Transform::from_xyz(0.0, 0.23, 0.0),
    ));
    // J1 turntable carrying the J2 housing; transform set by robot::animate.
    parent
        .spawn((Transform::default(), Visibility::default(), RobotPart::Turntable))
        .with_children(|turntable| {
            turntable.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.44, 0.16))),
                MeshMaterial3d(paint.clone()),
                Transform::from_xyz(0.0, 0.42, 0.0),
            ));
            turntable.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.55, 0.42, 0.62))),
                MeshMaterial3d(paint.clone()),
                Transform::from_xyz(kin.j2_offset * 0.6, kin.base_height - 0.08, 0.0),
            ));
            for side in [-1.0, 1.0] {
                turntable.spawn((
                    Mesh3d(meshes.add(Cylinder::new(0.2, 0.08))),
                    MeshMaterial3d(palette.dark.clone()),
                    Transform::from_xyz(kin.j2_offset, kin.base_height, side * 0.35)
                        .with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
                ));
            }
            turntable.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.09, 0.7))),
                MeshMaterial3d(palette.steel.clone()),
                Transform::from_xyz(-0.25, 0.85, 0.0).with_rotation(Quat::from_rotation_z(0.5)),
            ));
        });
    parent
        .spawn((Transform::default(), Visibility::default(), RobotPart::LowerArm))
        .with_children(|arm| {
            arm.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.30, kin.l1 + 0.1, 0.26))),
                MeshMaterial3d(paint.clone()),
                Transform::default(),
            ));
        });
    parent
        .spawn((Transform::default(), Visibility::default(), RobotPart::UpperArm))
        .with_children(|arm| {
            arm.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.22, kin.l2 + kin.upper_back, 0.22))),
                MeshMaterial3d(paint.clone()),
                Transform::default(),
            ));
            arm.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.16, 0.32))),
                MeshMaterial3d(palette.dark.clone()),
                Transform::from_xyz(0.0, -(kin.l2 + kin.upper_back) / 2.0 + 0.12, 0.0)
                    .with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
            ));
        });
    parent
        .spawn((Transform::default(), Visibility::default(), RobotPart::Rod))
        .with_children(|rod| {
            rod.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.07, kin.l1, 0.07))),
                MeshMaterial3d(palette.steel.clone()),
                Transform::default(),
            ));
        });
    parent
        .spawn((Transform::default(), Visibility::default(), RobotPart::Wrist))
        .with_children(|wrist| {
            wrist.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.24, kin.wrist_drop, 0.24))),
                MeshMaterial3d(paint.clone()),
                Transform::from_xyz(0.0, -kin.wrist_drop / 2.0, 0.0),
            ));
        });
    robot_gripper(parent, meshes, palette);
}

/// Cavalete base robô (Selgron 01F11M000520): welded tubular frame the robot stands
/// on, 1150 mm wide and 1594.5 mm tall in the drawing (both editable), with four
/// columns on anchored base plates, two top beams along the depth carrying the robot
/// plate, a ring of beams at mid height and diagonal braces on the long sides. Built
/// below the robot origin (the top of the plate), turned by `base_yaw`.
fn robot_base_visual(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, p: &crate::model::Params) {
    const TUBE: f32 = 0.15;
    const PLATE: f32 = 0.025;
    let (h, w, d) = (p.base_height.max(0.2), p.base_width.max(0.4), p.base_depth.max(0.4));
    let turn = Quat::from_rotation_y(p.base_yaw.to_radians());
    let mut part = |size: Vec3, at: Vec3, rot: Quat, material: &Handle<StandardMaterial>| {
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(size.x, size.y, size.z))),
            MeshMaterial3d(material.clone()),
            Transform::from_translation(turn * at).with_rotation(turn * rot),
        ));
    };
    let (cx, cz) = (d / 2.0 - TUBE / 2.0, w / 2.0 - TUBE / 2.0);
    let floor = -h;
    let beam_y = -PLATE - TUBE / 2.0;
    let column_h = h - PLATE - TUBE - PLATE;
    let ring_y = floor + 0.46 * h;
    // Robot mounting plate on the top beams.
    part(Vec3::new(0.95f32.min(d), PLATE, w - 0.05), Vec3::new(0.0, -PLATE / 2.0, 0.0), Quat::IDENTITY, &palette.steel);
    for sz in [-1.0, 1.0] {
        // Top beam along the depth.
        part(Vec3::new(d, TUBE, TUBE), Vec3::new(0.0, beam_y, sz * cz), Quat::IDENTITY, &palette.frame_grey);
        // Mid-height beam along the depth.
        part(Vec3::new(d - 2.0 * TUBE, 0.10, 0.10), Vec3::new(0.0, ring_y, sz * cz), Quat::IDENTITY, &palette.frame_grey);
        // Diagonal braces from the mid-height beam up to the top beam.
        for sx in [-1.0f32, 1.0] {
            let a = Vec2::new(sx * (cx - TUBE / 2.0), ring_y);
            let b = Vec2::new(sx * 0.12, beam_y - TUBE / 2.0);
            let len = (b - a).length();
            let angle = (b.y - a.y).atan2(b.x - a.x);
            let mid = (a + b) / 2.0;
            part(Vec3::new(len, 0.07, 0.07), Vec3::new(mid.x, mid.y, sz * cz), Quat::from_rotation_z(angle), &palette.frame_grey);
        }
        for sx in [-1.0, 1.0] {
            // Column on its base plate (anchored with chemical anchors and M20 rods).
            part(Vec3::new(TUBE, column_h, TUBE), Vec3::new(sx * cx, floor + PLATE + column_h / 2.0, sz * cz), Quat::IDENTITY, &palette.frame_grey);
            part(Vec3::new(0.36, PLATE, 0.36), Vec3::new(sx * cx, floor + PLATE / 2.0, sz * cz), Quat::IDENTITY, &palette.steel);
        }
    }
    for sx in [-1.0, 1.0] {
        // Mid-height beam across the width, at each end.
        part(Vec3::new(0.10, 0.10, w - 2.0 * TUBE), Vec3::new(sx * cx, ring_y, 0.0), Quat::IDENTITY, &palette.frame_grey);
    }
}

/// Cabeçote Selgron "4 fardos" (0323P01-010000): 1234 x 500 mm frame with the
/// flange coupling 435 mm above its bottom (origin of this part), central divider
/// and two pneumatic clamps (pinças) 295 mm tall that close along the gripper Z.
/// The fardos sit at the bottom, between the clamp tips (the TCP).
fn robot_gripper(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette) {
    let tool = robot::TOOL_LENGTH;
    parent
        .spawn((Transform::default(), Visibility::default(), RobotPart::Gripper))
        .with_children(|gripper| {
            // Coupling flange and upper housing (valves, guides).
            gripper.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.11, 0.04))),
                MeshMaterial3d(palette.dark.clone()),
                Transform::from_xyz(0.0, tool - 0.02, 0.0),
            ));
            gripper.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.42, 0.16, 0.56))),
                MeshMaterial3d(palette.frame_grey.clone()),
                Transform::from_xyz(0.0, tool - 0.12, 0.0),
            ));
            // Springs of the anti-impact pins.
            for z in [-0.2, 0.2] {
                gripper.spawn((
                    Mesh3d(meshes.add(Cylinder::new(0.03, 0.14))),
                    MeshMaterial3d(palette.steel.clone()),
                    Transform::from_xyz(0.0, 0.15, z),
                ));
            }
            // Main frame with the slanted guide plates at the ends.
            gripper.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.50, 0.08, 1.234))),
                MeshMaterial3d(palette.frame_grey.clone()),
                Transform::from_xyz(0.0, 0.04, 0.0),
            ));
            for side in [-1.0, 1.0] {
                gripper.spawn((
                    Mesh3d(meshes.add(Cuboid::new(0.48, 0.01, 0.30))),
                    MeshMaterial3d(palette.steel.clone()),
                    Transform::from_xyz(0.0, 0.11, side * 0.46).with_rotation(Quat::from_rotation_x(side * 0.25)),
                ));
                // Cylinder CIL 40-420 driving each clamp.
                gripper.spawn((
                    Mesh3d(meshes.add(Cylinder::new(0.025, 0.42))),
                    MeshMaterial3d(palette.steel.clone()),
                    Transform::from_xyz(0.18, 0.04, side * 0.32).with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
                ));
            }
            // Central divider between the two compartments.
            gripper.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.44, 0.2286, 0.008))),
                MeshMaterial3d(palette.dark.clone()),
                Transform::from_xyz(0.0, -0.1143, 0.0),
            ));
        });
    let jaw = meshes.add(Cuboid::new(0.50, robot::JAW_HEIGHT, 0.012));
    for part in [RobotPart::JawA, RobotPart::JawB] {
        parent.spawn((Mesh3d(jaw.clone()), MeshMaterial3d(palette.steel.clone()), Transform::default(), part));
    }
}

pub fn box_mesh(parent: &mut ChildSpawnerCommands, meshes: &mut Assets<Mesh>, palette: &Palette, size: Vec3) {
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(size.x, size.y, size.z))),
        MeshMaterial3d(palette.cardboard.clone()),
        Transform::default(),
    ));
    parent.spawn((
        Mesh3d(meshes.add(Cuboid::new(size.x + 0.002, 0.004, 0.06))),
        MeshMaterial3d(palette.tape.clone()),
        Transform::from_xyz(0.0, size.y / 2.0, 0.0),
    ));
}
