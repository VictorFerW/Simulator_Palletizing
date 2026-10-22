mod communication;
mod conveyor;
mod editor;
mod elevator;
mod fullcar;
mod io;
mod layout;
mod mesa_logic;
mod model;
mod paper;
mod pattern;
mod robot;
mod s7;
mod shuttle;
mod sim;
mod tia_importer;
mod ui;
mod visuals;

use bevy::prelude::*;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

use editor::{DragState, EditRequests, EditorSettings, OrbitCamera, UiFocus};
use layout::LayoutRequests;
use model::{Selection, SimControl};
use tia_importer::TiaProject;
use visuals::Palette;

#[derive(Resource, Default)]
pub struct ProjectState(pub TiaProject);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Paletização FANUC M-410iC/185 · Gêmeo digital".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(ClearColor(Color::srgb(0.55, 0.62, 0.70)))
        .insert_resource(GlobalAmbientLight {
            color: Color::WHITE,
            brightness: 350.0,
            ..default()
        })
        .init_resource::<Palette>()
        .init_resource::<ProjectState>()
        .init_resource::<Selection>()
        .init_resource::<SimControl>()
        .init_resource::<UiFocus>()
        .init_resource::<EditorSettings>()
        .init_resource::<DragState>()
        .init_resource::<EditRequests>()
        .init_resource::<LayoutRequests>()
        .init_resource::<io::PlcLink>()
        .init_resource::<ui::UiState>()
        .add_systems(Startup, (setup_scene, layout::setup_default_cell))
        .add_systems(
            Update,
            (
                (
                    layout::process_layout_requests,
                    editor::process_edit_requests,
                    editor::sync_selection,
                    editor::rebuild_visuals,
                )
                    .chain(),
                (editor::camera_controls, editor::pick_and_drag, editor::keyboard_edit),
                (
                    // Controllers: PLC values or internal logic -> actuators.
                    (
                        io::plc_events,
                        io::apply_inputs,
                        sim::reset_simulation,
                        sim::palletizers,
                        mesa_logic::run_mesas,
                        shuttle::shuttle_logic,
                        fullcar::fullcar_logic,
                        paper::paper_logic,
                        elevator::elevator_logic,
                    )
                        .chain(),
                    // Physics, sensors and outputs.
                    (
                        shuttle::shuttle_motion,
                        fullcar::fullcar_motion,
                        paper::paper_hold,
                        elevator::elevator_mechanics,
                        sim::emitters,
                        sim::move_loads,
                        paper::paper_mechanics,
                        sim::mesa_mechanics,
                        robot::robot_program,
                        robot::animate_robots,
                        sim::sensors,
                        sim::conveyor_sensors,
                        sim::removers,
                        sim::animate_conveyors,
                        shuttle::animate_shuttle,
                        fullcar::animate_fullcar,
                        paper::animate_paper,
                        elevator::animate_elevators,
                        io::publish_outputs,
                    )
                        .chain(),
                )
                    .chain(),
                editor::draw_gizmos,
            )
                .chain(),
        )
        .add_systems(EguiPrimaryContextPass, (ui::ui_system, ui::selection_band).chain())
        .add_systems(Update, auto_screenshot)
        .run();
}

/// `PDT_SCREENSHOT=arquivo.png` saves a screenshot after `PDT_SCREENSHOT_SECS`
/// (default 12 s) and exits; `PDT_SELECT=nome` selects that equipment first,
/// `PDT_FOCUS` points the camera at it, `PDT_TIME_SCALE` speeds up the simulation
/// and `PDT_LAYOUT` opens a layout.
/// Used for automated visual checks.
#[allow(clippy::too_many_arguments)]
fn auto_screenshot(
    time: Res<Time>,
    mut commands: Commands,
    mut done: Local<u8>,
    mut exit: MessageWriter<AppExit>,
    mut selection: ResMut<Selection>,
    mut control: ResMut<SimControl>,
    mut requests: ResMut<EditRequests>,
    equipment: Query<(Entity, &model::Equipment)>,
) {
    use bevy::render::view::screenshot::{Screenshot, save_to_disk};
    let Ok(path) = std::env::var("PDT_SCREENSHOT") else { return };
    if *done == 0
        && let Some(scale) = std::env::var("PDT_TIME_SCALE").ok().and_then(|s| s.parse().ok())
    {
        control.time_scale = scale;
    }
    if selection.0.is_none()
        && let Ok(name) = std::env::var("PDT_SELECT") {
            selection.0 = equipment.iter().find(|(_, e)| e.name == name).map(|(e, _)| e);
            if std::env::var("PDT_FOCUS").is_ok() {
                requests.focus = selection.0;
            }
        }
    let at: f32 = std::env::var("PDT_SCREENSHOT_SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(12.0);
    let elapsed = time.elapsed_secs();
    if *done == 0 && elapsed > at {
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
        *done = 1;
    } else if *done == 1 && elapsed > at + 2.0 {
        exit.write(AppExit::Success);
        *done = 2;
    }
}

fn setup_scene(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, palette: Res<Palette>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(8.0, 7.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        OrbitCamera {
            target: Vec3::new(-0.8, 0.5, -1.2),
            radius: 21.0,
            // From the bottom of the drawing: lines on the left, TDP rows on the right.
            yaw: std::f32::consts::FRAC_PI_2,
            pitch: 1.0,
        },
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 9_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(6.0, 12.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(40.0, 40.0))),
        MeshMaterial3d(palette.floor.clone()),
        Transform::default(),
    ));
}
