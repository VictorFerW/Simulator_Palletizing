//! Cell layout persistence (JSON) and the default palletizing cell.

use std::collections::HashMap;
use std::path::PathBuf;

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

use crate::communication::S7Settings;
use crate::editor::spawn_equipment;
use crate::io::{PlcConnection, PlcLink};
use crate::model::{Equipment, EquipmentKind, EquipmentTagBinding, Io, Params, Selection, TagBindings};
use crate::robot::yaw_of;
use crate::sim::{Load, LoadState};
use crate::tia_importer::TiaProject;
use crate::visuals::Palette;
use crate::ProjectState;

#[derive(Serialize, Deserialize)]
struct LayoutFile {
    version: u32,
    equipment: Vec<LayoutItem>,
    #[serde(default)]
    project: TiaProject,
    /// Version 1 had a single PLC.
    #[serde(default, skip_serializing)]
    plc: Option<PlcSettingsFile>,
    #[serde(default)]
    plcs: Vec<PlcSettingsFile>,
}

#[derive(Serialize, Deserialize)]
struct LayoutItem {
    name: String,
    kind: EquipmentKind,
    position: [f32; 3],
    yaw_deg: f32,
    params: Params,
    #[serde(default)]
    bindings: Vec<EquipmentTagBinding>,
    #[serde(default)]
    manual: HashMap<String, f64>,
}

#[derive(Serialize, Deserialize, Default)]
#[serde(default)]
struct PlcSettingsFile {
    name: String,
    plc: String,
    host: String,
    rack: u8,
    slot: u8,
    poll_ms: u64,
}

#[derive(Resource, Default)]
pub struct LayoutRequests {
    pub save: Option<PathBuf>,
    pub load: Option<PathBuf>,
    pub new_default: bool,
    pub clear: bool,
    /// Saved as soon as the new scene is built (a new project gets its file).
    pub save_after_rebuild: Option<PathBuf>,
    /// File of the open project ("Salvar" writes here).
    pub current: Option<PathBuf>,
    pub message: String,
}

/// Top view of a layout file for the start screen: each equipment's footprint.
pub struct Preview {
    pub items: Vec<(EquipmentKind, [Vec2; 4])>,
    pub min: Vec2,
    pub max: Vec2,
}

/// Reads a layout file and returns its footprints (x, z on the floor).
pub fn preview(path: &std::path::Path) -> Result<Preview, String> {
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let file: LayoutFile = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let mut items = Vec::new();
    let (mut min, mut max) = (Vec2::splat(f32::MAX), Vec2::splat(f32::MIN));
    for item in file.equipment {
        let eq = Equipment { name: item.name, kind: item.kind, params: item.params };
        let (lo, hi) = eq.local_bounds();
        let rot = Quat::from_rotation_y(item.yaw_deg.to_radians());
        let at = Vec3::from_array(item.position);
        let corners = [(lo.x, lo.z), (hi.x, lo.z), (hi.x, hi.z), (lo.x, hi.z)].map(|(x, z)| {
            let p = at + rot * Vec3::new(x, 0.0, z);
            Vec2::new(p.x, p.z)
        });
        for c in corners {
            min = min.min(c);
            max = max.max(c);
        }
        items.push((eq.kind, corners));
    }
    if items.is_empty() {
        (min, max) = (Vec2::splat(-1.0), Vec2::splat(1.0));
    }
    Ok(Preview { items, min, max })
}

#[allow(clippy::too_many_arguments)]
pub fn process_layout_requests(
    mut requests: ResMut<LayoutRequests>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    palette: Res<Palette>,
    mut project: ResMut<ProjectState>,
    mut link: ResMut<PlcLink>,
    mut selection: ResMut<Selection>,
    mut history: ResMut<crate::editor::UndoHistory>,
    equipment: Query<(Entity, &Equipment, &Transform, &TagBindings, &Io)>,
    loads: Query<(Entity, &Load)>,
) {
    if let Some(path) = requests.save.take() {
        let file = LayoutFile {
            version: 2,
            equipment: equipment
                .iter()
                .map(|(_, eq, tf, bindings, io)| LayoutItem {
                    name: eq.name.clone(),
                    kind: eq.kind,
                    position: tf.translation.to_array(),
                    yaw_deg: yaw_of(tf.rotation).to_degrees(),
                    params: eq.params.clone(),
                    bindings: bindings.0.clone(),
                    manual: io.manual.clone(),
                })
                .collect(),
            project: project.0.clone(),
            plc: None,
            plcs: link
                .connections
                .iter()
                .map(|c| PlcSettingsFile {
                    name: c.name.clone(),
                    plc: c.plc.clone(),
                    host: c.settings.host.clone(),
                    rack: c.settings.rack,
                    slot: c.settings.slot,
                    poll_ms: c.settings.poll_ms,
                })
                .collect(),
        };
        requests.message = match serde_json::to_string_pretty(&file)
            .map_err(|e| e.to_string())
            .and_then(|json| std::fs::write(&path, json).map_err(|e| e.to_string()))
        {
            Ok(()) => {
                crate::projects::remember(&path);
                requests.current = Some(path.clone());
                format!("Layout salvo em {}", path.display())
            }
            Err(e) => format!("Falha ao salvar: {e}"),
        };
    }

    let load = requests.load.take();
    let rebuild = load.is_some() || requests.new_default || requests.clear;
    if !rebuild {
        return;
    }
    let parsed = match load {
        Some(path) => match std::fs::read_to_string(&path)
            .map_err(|e| e.to_string())
            .and_then(|text| serde_json::from_str::<LayoutFile>(&text).map_err(|e| e.to_string()))
        {
            Ok(file) => {
                requests.message = format!("Layout carregado de {}", path.display());
                crate::projects::remember(&path);
                requests.current = Some(path.clone());
                Some(file)
            }
            Err(e) => {
                requests.message = format!("Falha ao abrir layout: {e}");
                return;
            }
        },
        None => None,
    };
    for (entity, ..) in &equipment {
        commands.entity(entity).despawn();
    }
    for (entity, load) in &loads {
        if !matches!(load.state, LoadState::Placed(_)) {
            commands.entity(entity).despawn();
        }
    }
    selection.0 = None;
    // A new scene starts a new undo history.
    history.reset = true;
    if let Some(mut file) = parsed {
        project.0 = file.project;
        if let Some(old) = file.plc.take().filter(|p| !p.host.is_empty()) {
            file.plcs.push(PlcSettingsFile { name: "CLP_1".into(), ..old });
        }
        if !file.plcs.is_empty() {
            for conn in &mut link.connections {
                conn.disconnect();
            }
            link.connections = file
                .plcs
                .into_iter()
                .map(|p| {
                    PlcConnection::new(
                        p.name,
                        p.plc,
                        S7Settings {
                            host: p.host,
                            rack: p.rack,
                            slot: p.slot,
                            poll_ms: p.poll_ms.max(10),
                        },
                    )
                })
                .collect();
            link.revision += 1;
        }
        for item in file.equipment {
            let entity = spawn_equipment(
                &mut commands,
                &mut meshes,
                &palette,
                Equipment { name: item.name, kind: item.kind, params: item.params },
                Transform::from_translation(Vec3::from_array(item.position))
                    .with_rotation(Quat::from_rotation_y(item.yaw_deg.to_radians())),
                TagBindings(item.bindings),
            );
            commands.entity(entity).insert(Io { manual: item.manual, ..default() });
        }
    } else {
        // New scene (default cell or empty): no file until it is saved.
        requests.current = None;
        if requests.new_default {
            spawn_default_cell(&mut commands, &mut meshes, &palette);
        }
    }
    requests.new_default = false;
    requests.clear = false;
    if let Some(path) = requests.save_after_rebuild.take() {
        requests.save = Some(path);
    }
}

/// Default cell (Selgron layout): Linha 1A/1 (with Linha 1B joining from the side)
/// and Linha 2 feed two Mesas 2 estágios; the FANUC M-410iC/185 palletizes on three
/// rows of TDPs (palletizing module first, then accumulation) that discharge into the
/// carro cheio giratório, which takes the full pallets up to the outfeed.
/// Robot at the origin, X to the right of the drawing, Z down the drawing.
pub fn spawn_default_cell(commands: &mut Commands, meshes: &mut Assets<Mesh>, palette: &Palette) {
    let mut add = |kind: EquipmentKind, name: &str, (x, z): (f32, f32), yaw_deg: f32, tweak: &dyn Fn(&mut Params)| {
        let mut params = Params::for_kind(kind);
        tweak(&mut params);
        spawn_equipment(
            commands,
            meshes,
            palette,
            Equipment { name: name.into(), kind, params },
            Transform::from_xyz(x, 0.0, z).with_rotation(Quat::from_rotation_y(yaw_deg.to_radians())),
            TagBindings::default(),
        );
    };
    let none = |_: &mut Params| {};
    // The belts feed the mesa entry lane (posição 1), offset from the mesa axis.
    let lane = Params::for_kind(EquipmentKind::Table2Stage).box_size[2] / 2.0 + 0.01;

    // Linha 1A + Linha 1 -> Mesa 1; Linha 1B joins Linha 1 from the side.
    let z1 = -0.45;
    add(EquipmentKind::BoxEmitter, "Emissor_Linha1A", (-7.8, z1), 0.0, &none);
    // Guides close to the fardo width keep every fardo on the entry lane (a fardo
    // coming from Linha 1B would otherwise reach the mesa on the pushed lane).
    let guided = Params::for_kind(EquipmentKind::Table2Stage).box_size[2] + 0.02;
    add(EquipmentKind::BeltConveyor, "Linha_1", (-5.5, z1), 0.0, &|p| {
        p.length = 4.6;
        p.width = guided;
    });
    // Linha 1B runs across Linha 1: its fardos are emitted turned so they reach Linha 1
    // in the same orientation as the others.
    add(EquipmentKind::BoxEmitter, "Emissor_Linha1B", (-4.6, -2.6), -90.0, &|p| {
        p.interval = 12.0;
        p.box_size = [p.box_size[2], p.box_size[1], p.box_size[0]];
    });
    add(EquipmentKind::BeltConveyor, "Linha_1B", (-4.6, -1.63), -90.0, &|p| p.length = 2.0);
    add(EquipmentKind::Table2Stage, "Mesa_1", (-2.0, z1 - lane), 0.0, &none);
    // Linha 2 -> Mesa 2.
    let z2 = 1.35;
    add(EquipmentKind::BoxEmitter, "Emissor_Linha2", (-6.2, z2), 0.0, &none);
    add(EquipmentKind::BeltConveyor, "Linha_2", (-4.7, z2), 0.0, &|p| {
        p.length = 3.0;
        p.width = guided;
    });
    add(EquipmentKind::Table2Stage, "Mesa_2", (-2.0, z2 - lane), 0.0, &none);

    // J1 has ±180°: 0° points away from the widest free sector, so no transfer
    // between the mesas and the three pallets crosses the ±180° limit.
    add(EquipmentKind::Robot, "FANUC_M410iC_1", (0.0, 0.0), 141.0, &none);

    // Three rows of TDP 1560 mm toward the carro cheio (receive edge at x = 4.085).
    let palletizing = |p: &mut Params| p.layers = 3;
    for (row, z, first) in [("1", -1.62, 0.185), ("2", -0.15, 1.745), ("3", 1.47, 0.185)] {
        add(EquipmentKind::PalletizingConveyor, &format!("TDP_Palet_{row}"), (first, z), 0.0, &palletizing);
        add(EquipmentKind::PalletDispenser, &format!("Magazine_{row}"), (first, z), 0.0, &none);
        let mut x = first + 1.56;
        let mut n = 1;
        while x < 3.4 {
            add(EquipmentKind::PalletConveyor, &format!("TDP_Linha{row}_{n}"), (x, z), 0.0, &none);
            x += 1.56;
            n += 1;
        }
    }

    // Carro cheio giratório: rail along Z (start at the bottom, z = 3.3); stops from
    // the rail start: espera, entrega (top), linha A (row 3), B (row 2), C (row 1).
    add(EquipmentKind::FullCar, "CCH_1", (5.0, -0.9), 90.0, &|p| {
        p.length = 8.4;
        p.stops = vec![4.2, 7.4, 3.3 - 1.47, 3.3 + 0.15, 3.3 + 1.62];
    });
    // Outfeed at the top (the car delivers along the rail after turning 90°).
    add(EquipmentKind::PalletConveyor, "TDP_Saida", (5.0, -5.8), 90.0, &none);
    add(EquipmentKind::BoxRemover, "Removedor_Saida", (5.0, -6.4), 90.0, &|p| p.length = 0.8);
}

/// At start: the layout of `PDT_LAYOUT`, the default cell for automated screenshots
/// (`PDT_SCREENSHOT`), otherwise the start screen (projects).
pub fn setup_default_cell(mut requests: ResMut<LayoutRequests>, mut start: ResMut<crate::projects::StartScreen>) {
    match std::env::var("PDT_LAYOUT") {
        Ok(path) => requests.load = Some(path.into()),
        Err(_) if crate::thumbs::thumbs_enabled() => {}
        Err(_) if std::env::var("PDT_SCREENSHOT").is_ok() && std::env::var("PDT_START").is_err() => {
            requests.new_default = true
        }
        Err(_) => start.show(),
    }
}
