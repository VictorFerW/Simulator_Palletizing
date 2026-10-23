//! `PDT_THUMBS=<folder>`: builds each catalog item alone, frames it with the camera
//! and saves a screenshot `<folder>/<id>.png` (the editor panels are hidden), then
//! exits. The images are cropped into `assets/thumbnails/` for the catalog.

use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};

use crate::catalog::ITEMS;
use crate::editor::{EditorSettings, OrbitCamera, spawn_equipment};
use crate::model::{Equipment, TagBindings};
use crate::sim::Load;
use crate::visuals::Palette;

/// Seconds each item stays before its picture (the CAD meshes load meanwhile).
const SETTLE: f32 = 2.5;

#[derive(Default)]
pub struct Progress {
    index: usize,
    since: f32,
    spawned: bool,
    shot: bool,
}

pub fn thumbs_enabled() -> bool {
    std::env::var("PDT_THUMBS").is_ok()
}

#[allow(clippy::too_many_arguments)]
pub fn make_thumbnails(
    time: Res<Time>,
    mut progress: Local<Progress>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    palette: Res<Palette>,
    mut settings: ResMut<EditorSettings>,
    equipment: Query<(Entity, &Equipment)>,
    loads: Query<Entity, With<Load>>,
    mut cameras: Query<&mut OrbitCamera>,
    mut exit: MessageWriter<AppExit>,
) {
    let Ok(dir) = std::env::var("PDT_THUMBS") else { return };
    settings.show_grid = false;
    let Some(item) = ITEMS.get(progress.index) else {
        exit.write(AppExit::Success);
        return;
    };
    if !progress.spawned {
        for (entity, _) in &equipment {
            commands.entity(entity).despawn();
        }
        for entity in &loads {
            commands.entity(entity).despawn();
        }
        let eq = Equipment { name: item.id.to_owned(), kind: item.kind, params: item.params() };
        let (lo, hi) = eq.local_bounds();
        spawn_equipment(&mut commands, &mut meshes, &palette, eq, Transform::IDENTITY, TagBindings::default());
        if let Ok(mut camera) = cameras.single_mut() {
            let size = (hi - lo).max_element().max(0.6);
            camera.target = (lo + hi) / 2.0;
            camera.radius = size * 2.0;
            camera.yaw = 0.75;
            camera.pitch = 0.45;
        }
        progress.spawned = true;
        progress.shot = false;
        progress.since = 0.0;
        return;
    }
    progress.since += time.delta_secs();
    if !progress.shot && progress.since > SETTLE {
        let _ = std::fs::create_dir_all(&dir);
        let path = std::path::Path::new(&dir).join(format!("{}.png", item.id));
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
        progress.shot = true;
    }
    if progress.shot && progress.since > SETTLE + 0.8 {
        progress.index += 1;
        progress.spawned = false;
    }
}
