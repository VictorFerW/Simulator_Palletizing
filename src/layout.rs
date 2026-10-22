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
    pub message: String,
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
            Ok(()) => format!("Layout salvo em {}", path.display()),
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
    } else if requests.new_default {
        spawn_default_cell(&mut commands, &mut meshes, &palette);
    }
    requests.new_default = false;
    requests.clear = false;
}

/// Emitter -> belt -> Mesa 2 estágios -> FANUC M-410iC/185 -> pallet on the
/// TDP de paletização, fed by a pallet magazine and discharged to an outfeed TDP.
pub fn spawn_default_cell(commands: &mut Commands, meshes: &mut Assets<Mesh>, palette: &Palette) {
    let mut add = |kind: EquipmentKind, name: &str, pos: Vec3, yaw_deg: f32| {
        spawn_equipment(
            commands,
            meshes,
            palette,
            Equipment { name: name.into(), kind, params: Params::for_kind(kind) },
            Transform::from_translation(pos).with_rotation(Quat::from_rotation_y(yaw_deg.to_radians())),
            TagBindings::default(),
        );
    };
    add(EquipmentKind::BoxEmitter, "Emissor_1", Vec3::new(-6.0, 0.0, 0.0), 0.0);
    add(EquipmentKind::BeltConveyor, "Esteira_1", Vec3::new(-4.4, 0.0, 0.0), 0.0);
    // The belt feeds the mesa entry lane (posição 1), offset from the mesa axis.
    let entry_lane = Params::for_kind(EquipmentKind::Table2Stage).box_size[2] / 2.0 + 0.01;
    add(EquipmentKind::Table2Stage, "Mesa2E_1", Vec3::new(-1.2, 0.0, -entry_lane), 0.0);
    // J1 has ±180°: 0° points between the mesa (behind) and the pallet (side), so
    // the transfers never cross the ±180° limit.
    add(EquipmentKind::Robot, "FANUC_M410iC_1", Vec3::new(1.6, 0.0, 0.0), -135.0);
    // TDP 1560 mm modules, end to end.
    add(EquipmentKind::PalletConveyor, "TDP_1", Vec3::new(0.04, 0.0, 2.1), 0.0);
    add(EquipmentKind::PalletDispenser, "Magazine_1", Vec3::new(0.04, 0.0, 2.1), 0.0);
    add(EquipmentKind::PalletizingConveyor, "TDP_Palet_1", Vec3::new(1.6, 0.0, 2.1), 0.0);
    add(EquipmentKind::PalletConveyor, "TDP_2", Vec3::new(3.16, 0.0, 2.1), 0.0);
    // Removes the full pallet once it stops at the end of the outfeed TDP.
    spawn_equipment(
        commands,
        meshes,
        palette,
        Equipment {
            name: "Removedor_1".into(),
            kind: EquipmentKind::BoxRemover,
            params: Params { length: 0.8, ..Params::for_kind(EquipmentKind::BoxRemover) },
        },
        Transform::from_xyz(3.7, 0.0, 2.1),
        TagBindings::default(),
    );
}

pub fn setup_default_cell(mut requests: ResMut<LayoutRequests>) {
    match std::env::var("PDT_LAYOUT") {
        Ok(path) => requests.load = Some(path.into()),
        Err(_) => requests.new_default = true,
    }
}
