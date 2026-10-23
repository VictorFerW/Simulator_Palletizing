use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use crossbeam_channel::{Receiver, unbounded};

use crate::communication::S7Settings;
use crate::editor::{EditRequests, EditorSettings, UiFocus};
use crate::io::{PlcConnection, PlcLink, default_manual, resolve_tag};
use crate::layout::LayoutRequests;
use crate::model::{Equipment, EquipmentKind, Io, NeedsRebuild, Selection, SignalDir, SignalType, SimControl, TagBindings};
use crate::robot::{RobotState, yaw_of};
use crate::mesa_logic::MesaLogic;
use crate::elevator::{ElevatorLogic, ElevatorState, MAX_STACK};
use crate::fullcar::FullCarLogic;
use crate::paper::{PaperPlacerLogic, PaperPlacerState};
use crate::shuttle::{ShuttleLogic, ShuttleState};
use crate::sim::{Load, PalletizerState};
use crate::tia_importer::{
    TiaProject, data_block_from_members, import_project_via_openness, parse_db_source, parse_simatic_ml,
    parse_tag_table_csv, parse_tag_table_xlsx, parse_tag_table_xml,
};
use crate::ProjectState;

#[derive(Default, PartialEq, Eq, Clone, Copy)]
pub enum SideTab {
    #[default]
    Properties,
    Tia,
    Signals,
    Plc,
}

/// (key of active tables/DBs, bindable (tag id, label) list).
type TagCache = (Vec<(String, usize, Option<u32>)>, Vec<(String, String)>);

#[derive(Resource)]
pub struct UiState {
    tab: SideTab,
    helper_path: String,
    import_status: String,
    import_rx: Option<Receiver<Result<TiaProject, String>>>,
    tag_filter: String,
    tia_filter: String,
    /// Bindable tag list, rebuilt only when the set of active tables/DBs changes.
    tag_cache: TagCache,
    /// "Padrões de paletização" window.
    pattern: crate::pattern::PatternUi,
    /// Left toolbar: catalog and scene list.
    catalog: crate::ui_catalog::CatalogUi,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            tab: SideTab::Properties,
            helper_path: default_helper_path(),
            import_status: "Selecione um projeto .ap18 do TIA Portal V18.".into(),
            import_rx: None,
            catalog: crate::ui_catalog::CatalogUi::default(),
            tag_filter: String::new(),
            tia_filter: String::new(),
            tag_cache: Default::default(),
            pattern: Default::default(),
        }
    }
}

fn default_helper_path() -> String {
    let candidates = [
        "tools/TiaOpennessBridge/bin/Release/net48/TiaOpennessBridge.exe",
        "TiaOpennessBridge/TiaOpennessBridge.exe",
        "TiaOpennessBridge.exe",
    ];
    let exe_dir = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.to_path_buf()));
    for candidate in candidates {
        if std::path::Path::new(candidate).exists() {
            return candidate.into();
        }
        if let Some(dir) = &exe_dir {
            let full = dir.join(candidate);
            if full.exists() {
                return full.to_string_lossy().into_owned();
            }
        }
    }
    candidates[0].into()
}

type CarQuery<'w, 's> = Query<'w, 's, (&'static ShuttleState, Option<&'static ShuttleLogic>, Option<&'static FullCarLogic>)>;
type PlacerQuery<'w, 's> = Query<'w, 's, (&'static mut PaperPlacerState, &'static PaperPlacerLogic)>;
type ElevatorQuery<'w, 's> = Query<'w, 's, (&'static mut ElevatorState, &'static mut ElevatorLogic)>;

type EquipmentQuery<'w, 's> = Query<
    'w,
    's,
    (Entity, &'static mut Equipment, &'static mut Transform, &'static mut TagBindings, &'static mut Io),
    Without<Load>,
>;

#[allow(clippy::too_many_arguments)]
pub fn ui_system(
    mut contexts: EguiContexts,
    mut state: ResMut<UiState>,
    mut project: ResMut<ProjectState>,
    mut selection: ResMut<Selection>,
    mut control: ResMut<SimControl>,
    mut focus: ResMut<UiFocus>,
    mut settings: ResMut<EditorSettings>,
    mut requests: ResMut<EditRequests>,
    (mut layout, mut start, thumbs): (ResMut<LayoutRequests>, ResMut<crate::projects::StartScreen>, Res<crate::ui_catalog::Thumbs>),
    mut link: ResMut<PlcLink>,
    mut equipment: EquipmentQuery,
    (robots, palletizers): (Query<&RobotState>, Query<&PalletizerState>),
    mesas: Query<&MesaLogic>,
    cars: CarQuery,
    (mut placers, mut elevators): (PlacerQuery, ElevatorQuery),
    mut commands: Commands,
) {
    // Catalog thumbnails as egui textures.
    let tex: Vec<Option<egui::TextureId>> = thumbs
        .0
        .iter()
        .map(|h| h.clone().map(|h| contexts.add_image(bevy_egui::EguiTextureHandle::Strong(h))))
        .collect();
    let Ok(ctx) = contexts.ctx_mut() else { return };
    if crate::thumbs::thumbs_enabled() {
        return;
    }
    // Start screen (projects): the editor is hidden while it is open.
    if crate::projects::start_screen(ctx, &mut start, &mut layout) {
        return;
    }
    if ctx.input(|i| i.modifiers.command && i.key_pressed(egui::Key::S)) {
        save_project(&mut layout);
    }
    // The project is edited through `bypass_change_detection` so that drawing the
    // TIA tab does not mark it changed every frame (tag routes are rebuilt on change).
    let fingerprint_before = project_fingerprint(&project);
    poll_import(&mut state, project.bypass_change_detection());

    egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
        ui.horizontal_wrapped(|ui| {
            ui.menu_button("Arquivo", |ui| {
                if ui.button("Projetos (tela inicial)").clicked() {
                    start.show_from_editor();
                    ui.close();
                }
                ui.separator();
                if ui.add(egui::Button::new("Salvar").shortcut_text("Ctrl+S")).clicked() {
                    save_project(&mut layout);
                    ui.close();
                }
                if ui.button("Salvar como...").clicked() {
                    save_as(&mut layout);
                    ui.close();
                }
                ui.separator();
                if ui.button("Nova célula padrão").clicked() {
                    layout.new_default = true;
                    ui.close();
                }
                if ui.button("Cena vazia").clicked() {
                    layout.clear = true;
                    ui.close();
                }
                ui.separator();
                if ui.button("Abrir arquivo...").clicked() {
                    layout.load = rfd::FileDialog::new().add_filter("Layout de célula", &["json"]).pick_file();
                    ui.close();
                }
            });
            if let Some(path) = &layout.current {
                ui.strong(path.file_stem().map_or_else(String::new, |s| s.to_string_lossy().into_owned()))
                    .on_hover_text(path.display().to_string());
            } else {
                ui.weak("(projeto não salvo)");
            }
            ui.separator();
            let label = if control.running { "⏸ Pausar" } else { "▶ Executar" };
            if ui.button(label).clicked() {
                control.running = !control.running;
            }
            if ui.button("⟲ Reiniciar").clicked() {
                control.reset_requested = true;
            }
            ui.separator();
            if ui.button("▦ Padrões de paletização").clicked() {
                state.pattern.open = true;
            }
            ui.add(egui::Slider::new(&mut control.time_scale, 0.1..=4.0).text("velocidade").logarithmic(true));
            ui.separator();
            ui.checkbox(&mut settings.snap, "Grade");
            ui.add(egui::DragValue::new(&mut settings.grid).speed(0.01).range(0.01..=1.0).suffix(" m"));
            ui.checkbox(&mut settings.show_grid, "Mostrar grade");
            ui.separator();
            let (connected, enabled, total) = (link.connected_count(), link.enabled_count(), link.connections.len());
            let (color, text) = if enabled == 0 {
                (egui::Color32::GRAY, "Simulação interna (sem CLP)".to_owned())
            } else if connected == enabled {
                (egui::Color32::from_rgb(60, 200, 90), format!("CLPs: {connected}/{total} conectados"))
            } else {
                (egui::Color32::from_rgb(230, 170, 40), format!("CLPs: {connected}/{enabled} conectados"))
            };
            ui.colored_label(color, text);
            if !layout.message.is_empty() {
                ui.separator();
                ui.weak(&layout.message);
            }
        });
    });

    egui::SidePanel::right("details").default_width(400.0).show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut state.tab, SideTab::Properties, "Propriedades");
            ui.selectable_value(&mut state.tab, SideTab::Tia, "TIA Portal V18");
            ui.selectable_value(&mut state.tab, SideTab::Signals, "Sinais / I/O");
            ui.selectable_value(&mut state.tab, SideTab::Plc, "CLP");
        });
        ui.separator();
        egui::ScrollArea::vertical().id_salt("details_scroll").show(ui, |ui| match state.tab {
            SideTab::Properties => properties_tab(
                ui,
                &mut state,
                &project,
                &link,
                &mut selection,
                &mut requests,
                &mut equipment,
                &robots,
                &mut placers,
                &mut elevators,
                &palletizers,
                &mesas,
                &cars,
                &mut commands,
            ),
            SideTab::Tia => tia_tab(ui, &mut state, project.bypass_change_detection()),
            SideTab::Signals => signals_tab(ui, &link, &mut selection, &mut equipment),
            SideTab::Plc => plc_tab(ui, &mut link, &project),
        });
    });
    // Padrões de paletização: apply the chosen pattern to the TDP.
    if !state.pattern.env_checked {
        state.pattern.env_checked = true;
        state.pattern.open = std::env::var("PDT_PATTERN").is_ok();
        state.pattern.auto_apply = std::env::var("PDT_PATTERN").is_ok_and(|v| v.starts_with("apply"));
    }
    if state.pattern.open {
        // Each TDP with the top of the operating space (flange center) of the robots
        // that reach it (the highest one), for the reach warning.
        let robot_kins: Vec<(&'static crate::robot::Kin, Transform)> = equipment
            .iter()
            .filter(|(_, e, ..)| e.kind == EquipmentKind::Robot)
            .map(|(_, e, tf, ..)| (crate::robot::kin(e.params.robot_model), *tf))
            .collect();
        let tdps: Vec<(String, f32, f32)> = equipment
            .iter()
            .filter(|(_, e, ..)| e.kind == EquipmentKind::PalletizingConveyor)
            .map(|(_, e, tf, ..)| {
                let top = robot_kins
                    .iter()
                    .filter(|(kin, rtf)| crate::robot::reaches_tdp(kin, rtf, tf))
                    .map(|(kin, rtf)| rtf.translation.y + kin.max_height)
                    .fold(f32::NAN, f32::max);
                (e.name.clone(), tf.translation.y + e.params.height, if top.is_nan() { crate::robot::FANUC.max_height } else { top })
            })
            .collect();
        let mesa_box = equipment.iter().find(|(_, e, ..)| e.kind.is_mesa()).map(|(_, e, ..)| e.params.box_size);
        let sized: Vec<(String, &str, [f32; 3])> = equipment
            .iter()
            .filter(|(_, e, ..)| e.kind == EquipmentKind::BoxEmitter || e.kind.is_mesa())
            .map(|(_, e, ..)| (e.name.clone(), if e.kind.is_mesa() { "mesa" } else { "emissor" }, e.params.box_size))
            .collect();
        if let Some((target, pattern)) = crate::pattern::pattern_window(ctx, &mut state.pattern, &tdps, mesa_box, &sized) {
            for (_, mut eq, ..) in &mut equipment {
                if eq.name == target {
                    eq.params.layers = pattern.layers;
                    eq.params.pattern = Some(pattern.clone());
                }
            }
        }
        // Product size on the chosen emitters and mesas (mesas rebuild lanes and empurrador).
        if let Some((names, size)) = state.pattern.size_request.take() {
            for (entity, mut eq, ..) in &mut equipment {
                if names.contains(&eq.name) {
                    eq.params.box_size = size;
                    if eq.kind.is_mesa() {
                        commands.entity(entity).insert(NeedsRebuild);
                    }
                }
            }
        }
    }
    if project_fingerprint(&project) != fingerprint_before {
        project.set_changed();
    }

    // Toolbar and catalog / scene list over the 3D view (after the other panels).
    let mut rows: Vec<crate::ui_catalog::SceneRow> = equipment
        .iter()
        .map(|(entity, eq, ..)| crate::ui_catalog::SceneRow {
            entity,
            name: eq.name.clone(),
            kind: eq.kind,
            item: crate::ui_catalog::item_of(eq.kind, eq.params.robot_model),
        })
        .collect();
    rows.sort_by(|a, b| a.name.cmp(&b.name));
    crate::ui_catalog::left_side(ctx, &mut state.catalog, &tex, &rows, &mut selection, &mut requests);

    focus.pointer_over_ui = ctx.is_pointer_over_area() || ctx.is_using_pointer();
    focus.keyboard_ui = ctx.wants_keyboard_input();
}

/// Cheap summary of what affects tag resolution (activation, DB numbers, PLC assignment).
fn project_fingerprint(project: &ProjectState) -> Vec<(String, bool, Option<u32>, bool, usize)> {
    let p = &project.0;
    p.tag_tables
        .iter()
        .map(|t| (format!("{}|{}", t.id, t.plc), t.active, None, false, t.tags.len()))
        .chain(p.data_blocks.iter().map(|d| (format!("{}|{}", d.id, d.plc), d.active, d.number, d.optimized, d.tags.len())))
        .chain(std::iter::once((p.name.clone(), false, None, false, 0)))
        .collect()
}

fn poll_import(state: &mut UiState, project: &mut ProjectState) {
    let Some(rx) = &state.import_rx else { return };
    match rx.try_recv() {
        Ok(Ok(imported)) => {
            state.import_status = format!(
                "Projeto '{}' importado: {} tabela(s) de tags, {} DB(s), {} aviso(s).",
                imported.name,
                imported.tag_tables.len(),
                imported.data_blocks.len(),
                imported.diagnostics.len()
            );
            project.0 = imported;
            state.import_rx = None;
        }
        Ok(Err(error)) => {
            state.import_status = error;
            state.import_rx = None;
        }
        Err(crossbeam_channel::TryRecvError::Disconnected) => {
            state.import_status = "O auxiliar Openness terminou sem resposta.".into();
            state.import_rx = None;
        }
        Err(crossbeam_channel::TryRecvError::Empty) => {}
    }
}

#[allow(clippy::too_many_arguments)]
fn properties_tab(
    ui: &mut egui::Ui,
    state: &mut UiState,
    project: &ProjectState,
    link: &PlcLink,
    selection: &mut Selection,
    requests: &mut EditRequests,
    equipment: &mut EquipmentQuery,
    robots: &Query<&RobotState>,
    placers: &mut PlacerQuery,
    elevators: &mut ElevatorQuery,
    palletizers: &Query<&PalletizerState>,
    mesas: &Query<&MesaLogic>,
    cars: &CarQuery,
    commands: &mut Commands,
) {
    let conveyor_names: Vec<String> = equipment
        .iter()
        .filter(|(_, e, ..)| e.kind.is_product_conveyor())
        .map(|(_, e, ..)| e.name.clone())
        .collect();
    let station_names: Vec<String> = equipment
        .iter()
        .filter(|(_, e, ..)| e.kind == EquipmentKind::PalletizingConveyor)
        .map(|(_, e, ..)| e.name.clone())
        .collect();
    // Emitters (name, position, box length) for the belt sensor check.
    let emitters: Vec<(String, Vec3, f32)> = equipment
        .iter()
        .filter(|(_, e, ..)| e.kind == EquipmentKind::BoxEmitter)
        .map(|(_, e, tf, ..)| (e.name.clone(), tf.translation, e.params.box_size[0].max(e.params.box_size[2])))
        .collect();
    // Mesas/belts and palletizing TDPs within reach of the selected robot (for the routes).
    let robot_tf = selection
        .0
        .and_then(|e| equipment.get(e).ok())
        .filter(|(_, e, ..)| e.kind == EquipmentKind::Robot)
        .map(|(_, e, tf, ..)| (crate::robot::kin(e.params.robot_model), *tf));
    let (reach_picks, reach_tdps): (Vec<String>, Vec<String>) = match robot_tf {
        Some((kin, rtf)) => (
            equipment
                .iter()
                .filter(|(_, e, tf, ..)| e.kind.is_product_conveyor() && crate::robot::reaches_pick(kin, &rtf, e, tf))
                .map(|(_, e, ..)| e.name.clone())
                .collect(),
            equipment
                .iter()
                .filter(|(_, e, tf, ..)| e.kind == EquipmentKind::PalletizingConveyor && crate::robot::reaches_tdp(kin, &rtf, tf))
                .map(|(_, e, ..)| e.name.clone())
                .collect(),
        ),
        None => (Vec::new(), Vec::new()),
    };
    if selection.1.len() > 1 {
        ui.heading(format!("{} itens selecionados", selection.1.len()));
        ui.horizontal(|ui| {
            if ui.button("Duplicar todos (Ctrl+D)").clicked() {
                requests.duplicate.extend(selection.1.iter().copied());
            }
            if ui.button("Excluir todos (Del)").clicked() {
                requests.delete.extend(selection.1.iter().copied());
            }
        });
        ui.weak("Arraste qualquer item selecionado para mover o grupo; R gira o grupo. Abaixo, o item principal:");
        ui.separator();
    }
    let Some(entity) = selection.0 else {
        ui.label("Selecione um equipamento na cena ou na lista à esquerda.");
        return;
    };
    let Ok((entity, mut eq, mut tf, mut bindings, mut io)) = equipment.get_mut(entity) else {
        selection.0 = None;
        return;
    };
    if eq.kind == EquipmentKind::Robot {
        ui.heading(format!("Robô {}", eq.params.robot_model.label()));
    } else {
        ui.heading(eq.kind.label());
    }
    ui.horizontal(|ui| {
        ui.label("Nome");
        ui.text_edit_singleline(&mut eq.name);
    });
    ui.horizontal(|ui| {
        if ui.button("Duplicar (Ctrl+D)").clicked() {
            requests.duplicate.push(entity);
        }
        if ui.button("Excluir (Del)").clicked() {
            requests.delete.push(entity);
        }
        if ui.button("Focar (F)").clicked() {
            requests.focus = Some(entity);
        }
    });
    ui.separator();
    egui::Grid::new("transform").num_columns(2).show(ui, |ui| {
        ui.label("X");
        ui.add(egui::DragValue::new(&mut tf.translation.x).speed(0.01).suffix(" m"));
        ui.end_row();
        ui.label("Z");
        ui.add(egui::DragValue::new(&mut tf.translation.z).speed(0.01).suffix(" m"));
        ui.end_row();
        ui.label("Altura (Y)");
        ui.add(egui::DragValue::new(&mut tf.translation.y).speed(0.01).range(0.0..=5.0).suffix(" m"));
        ui.end_row();
        ui.label("Rotação");
        let mut yaw = yaw_of(tf.rotation).to_degrees();
        ui.horizontal(|ui| {
            let changed = ui.add(egui::DragValue::new(&mut yaw).speed(1.0).suffix("°")).changed();
            let minus = ui.small_button("-90°").clicked();
            let plus = ui.small_button("+90°").clicked();
            if minus {
                yaw -= 90.0;
            }
            if plus {
                yaw += 90.0;
            }
            if changed || minus || plus {
                tf.rotation = Quat::from_rotation_y(yaw.to_radians());
            }
        });
        ui.end_row();
    });
    ui.separator();

    // Esteira with its FB: an emitter dropping boxes past the entry sensor never
    // starts the belt (the boxes did not go through the sensor).
    let belt_warning = (eq.kind == EquipmentKind::BeltConveyor && eq.params.belt_logic && eq.params.belt_entry_sensor)
        .then(|| {
            let inverse = tf.compute_affine().inverse();
            let p = &eq.params;
            let sensor = -p.length / 2.0 + p.belt_entry_pos;
            emitters.iter().find_map(|(name, at, len)| {
                let local = inverse.transform_point3(*at);
                (local.z.abs() < p.width / 2.0 && local.x.abs() < p.length / 2.0 && local.x - len / 2.0 > sensor).then(|| {
                    format!(
                        "O {name} solta as caixas depois do sensor de entrada, e elas não ligam a esteira. Coloque o sensor a {:.2} m do início ou o emissor no começo da esteira.",
                        (local.x + p.length / 2.0).max(0.1)
                    )
                })
            })
        })
        .flatten();
    let before = eq.params.clone();
    let kind = eq.kind;
    let tdp_name = eq.name.clone();
    let p = &mut eq.params;
    egui::Grid::new("params").num_columns(2).show(ui, |ui| {
        let row = |ui: &mut egui::Ui, label: &str, value: &mut f32, range: std::ops::RangeInclusive<f32>, suffix: &str| {
            ui.label(label);
            ui.add(egui::DragValue::new(value).speed(0.01).range(range).suffix(suffix));
            ui.end_row();
        };
        match kind {
            EquipmentKind::BeltConveyor | EquipmentKind::PalletConveyor => {
                row(ui, "Comprimento total", &mut p.length, 0.3..=30.0, " m");
                row(ui, "Largura (rolete)", &mut p.width, 0.2..=2.0, " m");
                row(ui, "Altura do topo", &mut p.height, 0.1..=3.0, " m");
                row(ui, "Velocidade", &mut p.speed, 0.0..=3.0, " m/s");
                if kind == EquipmentKind::BeltConveyor {
                    ui.label("Sensor de entrada");
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut p.belt_entry_sensor, "");
                        if p.belt_entry_sensor {
                            ui.add(egui::DragValue::new(&mut p.belt_entry_pos).speed(0.01).range(0.02..=p.length).suffix(" m do início"));
                        }
                    });
                    ui.end_row();
                    ui.label("Sensor de saída");
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut p.belt_exit_sensor, "");
                        if p.belt_exit_sensor {
                            ui.add(egui::DragValue::new(&mut p.belt_exit_pos).speed(0.01).range(0.02..=p.length).suffix(" m do fim"));
                        }
                    });
                    ui.end_row();
                    if let Some(warning) = &belt_warning {
                        ui.label("");
                        ui.colored_label(egui::Color32::from_rgb(230, 150, 40), warning);
                        ui.end_row();
                    }
                    ui.label("Lógica interna");
                    ui.checkbox(&mut p.belt_logic, "FB da esteira sem CLP")
                        .on_hover_text("Liga o motor com produto nos sensores; desliga por ociosidade. Sem a lógica, o motor fica sempre ligado (ou pelo CLP).");
                    ui.end_row();
                    if p.belt_logic {
                        ui.label("Acumulação");
                        ui.checkbox(&mut p.belt_accumulate, "AtivaAcumulo: para com produto na saída se a próxima não pode receber");
                        ui.end_row();
                        row(ui, "TempoOciosidade", &mut p.belt_idle_time, 1.0..=600.0, " s");
                        row(ui, "TempoVaiPara", &mut p.belt_run_on, 0.5..=120.0, " s");
                    }
                }
            }
            EquipmentKind::Table2Stage | EquipmentKind::Table3Stage => {
                row(ui, "Comprimento total", &mut p.length, 0.6..=10.0, " m");
                row(ui, "Largura (rolete)", &mut p.width, 0.4..=2.0, " m");
                row(ui, "Altura do topo", &mut p.height, 0.1..=3.0, " m");
                row(ui, "Velocidade dos roletes", &mut p.speed, 0.0..=3.0, " m/s");
                row(ui, "Fardo: comprimento", &mut p.box_size[0], 0.1..=1.2, " m");
                row(ui, "Fardo: largura", &mut p.box_size[2], 0.1..=1.0, " m");
                if kind == EquipmentKind::Table3Stage {
                    ui.label("Receita 10x1");
                    ui.checkbox(&mut p.fardo_10kg, "Fardo10Kg: grupos de 3 ou 4 fardos lado a lado").on_hover_text(
                        "A garra empurra a fileira a cada fardo, o empurrador leva o primeiro ao batente e a coleta recebe 3 ou 4 fardos, conforme a pega do robô",
                    );
                    ui.end_row();
                }
                ui.label("Lógica interna");
                ui.checkbox(&mut p.auto_logic, "FB da mesa e da coleta sem CLP")
                    .on_hover_text("Executa o port dos FBs Selgron. Atuadores lidos do CLP têm prioridade.");
                ui.end_row();
            }
            EquipmentKind::Robot => {
                ui.label("Modelo");
                egui::ComboBox::from_id_salt("robot_model").selected_text(p.robot_model.label()).show_ui(ui, |ui| {
                    for model in crate::model::RobotModel::ALL {
                        ui.selectable_value(&mut p.robot_model, model, model.label());
                    }
                });
                ui.end_row();
                ui.label("Cavalete (base)");
                ui.checkbox(&mut p.robot_base, "Robô sobre o cavalete Selgron 01F11M000520")
                    .on_hover_text("O topo do cavalete é a altura da base do robô");
                ui.end_row();
                if p.robot_base {
                    row(ui, "Cavalete: altura", &mut p.base_height, 0.2..=4.0, " m");
                    row(ui, "Cavalete: largura", &mut p.base_width, 0.5..=4.0, " m");
                    row(ui, "Cavalete: comprimento", &mut p.base_depth, 0.5..=6.0, " m");
                    ui.label("Cavalete: giro");
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut p.base_yaw).speed(1.0).range(-180.0..=180.0).suffix("°"));
                        if ui.small_button("+90°").clicked() {
                            p.base_yaw = if p.base_yaw >= 90.0 { p.base_yaw - 270.0 } else { p.base_yaw + 90.0 };
                        }
                    });
                    ui.end_row();
                }
                ui.label("Coletar de");
                names_select(ui, "pick_from", &mut p.pick_from, &conveyor_names, "(todas ao alcance)");
                ui.end_row();
                ui.label("Paletizar em");
                names_select(ui, "place_on", &mut p.place_on, &station_names, "(todos ao alcance)");
                ui.end_row();
                ui.label("Rotas no alcance");
                ui.vertical(|ui| {
                    if reach_tdps.is_empty() {
                        ui.weak("Nenhum TDP de paletização no alcance do robô");
                    }
                    for tdp in &reach_tdps {
                        ui.horizontal(|ui| {
                            ui.label(format!("{tdp} recebe de"));
                            let mut from = p.routes.iter().find(|r| &r.tdp == tdp).map(|r| r.from.clone()).unwrap_or_default();
                            let before = from.clone();
                            names_select(ui, &format!("route_{tdp}"), &mut from, &reach_picks, "(todas as mesas)");
                            if from != before {
                                p.routes.retain(|r| &r.tdp != tdp);
                                if !from.is_empty() {
                                    p.routes.push(crate::model::Route { tdp: tdp.clone(), from });
                                }
                            }
                        });
                    }
                    if !reach_tdps.is_empty() {
                        ui.weak(format!("{} mesa(s)/esteira(s) e {} TDP(s) no alcance", reach_picks.len(), reach_tdps.len()));
                    }
                });
                ui.end_row();
                ui.label("Limites de giro");
                ui.vertical(|ui| {
                    // Software limits inside the data sheet (manufacturer's axes).
                    let sheet = crate::robot::kin(p.robot_model);
                    if p.axis_limits.len() < 4 {
                        let current = p.axis_limits.clone();
                        p.axis_limits = (0..4)
                            .map(|i| current.get(i).copied().unwrap_or([sheet.min[i].to_degrees(), sheet.max[i].to_degrees()]))
                            .collect();
                    }
                    egui::Grid::new("axis_limits").num_columns(4).show(ui, |ui| {
                        for h in ["Eixo", "Mín.", "Máx.", "Folha de dados"] {
                            ui.weak(h);
                        }
                        ui.end_row();
                        for i in 0..4 {
                            let (lo, hi) = (sheet.min[i].to_degrees(), sheet.max[i].to_degrees());
                            let [min, max] = &mut p.axis_limits[i];
                            ui.label(sheet.axis_names[i]);
                            ui.add(egui::DragValue::new(min).speed(1.0).range(lo..=*max - 1.0).suffix("°"));
                            ui.add(egui::DragValue::new(max).speed(1.0).range(*min + 1.0..=hi).suffix("°"));
                            ui.weak(format!("{lo:.0}° … {hi:.0}°"));
                            ui.end_row();
                        }
                    });
                    let full = (0..4).all(|i| {
                        (p.axis_limits[i][0] - sheet.min[i].to_degrees()).abs() < 0.01
                            && (p.axis_limits[i][1] - sheet.max[i].to_degrees()).abs() < 0.01
                    });
                    if full {
                        ui.weak("Faixas da folha de dados");
                        // Nothing to store: the file keeps following the data sheet.
                        p.axis_limits.clear();
                    } else if ui.button("Voltar à folha de dados").clicked() {
                        p.axis_limits.clear();
                    }
                });
                ui.end_row();
                ui.label("Override de velocidade");
                ui.add(egui::Slider::new(&mut p.speed_override, 0.05..=1.0));
                ui.end_row();
            }
            EquipmentKind::PalletizingConveyor => {
                row(ui, "Comprimento", &mut p.length, 0.6..=5.0, " m");
                row(ui, "Largura (rolete)", &mut p.width, 0.6..=2.0, " m");
                row(ui, "Altura do topo", &mut p.height, 0.1..=3.0, " m");
                row(ui, "Velocidade", &mut p.speed, 0.0..=3.0, " m/s");
                ui.label("Camadas");
                ui.add(egui::DragValue::new(&mut p.layers).range(1..=20));
                ui.end_row();
                ui.label("Padrão de paletização");
                ui.vertical(|ui| {
                    match &p.pattern {
                        Some(chosen) => {
                            ui.label(format!(
                                "{} · {} por camada × {}",
                                chosen.name,
                                chosen.odd.len(),
                                p.layers
                            ));
                        }
                        None => {
                            ui.weak("automático (grade, camadas giradas)");
                        }
                    }
                    ui.horizontal(|ui| {
                        if ui.button("Escolher padrão…").clicked() {
                            state.pattern.open = true;
                            state.pattern.target = tdp_name.clone();
                        }
                        if p.pattern.is_some() && ui.button("Usar automático").clicked() {
                            p.pattern = None;
                        }
                    });
                });
                ui.end_row();
                if p.pattern.is_none() {
                    ui.label("Camadas intertravadas");
                    ui.checkbox(&mut p.interlock, "girar camadas ímpares 90°");
                    ui.end_row();
                }
                ui.label("Lógica interna");
                ui.checkbox(&mut p.auto_logic, "motor/batente automáticos sem CLP")
                    .on_hover_text("Traz o pallet vazio, segura no batente enquanto o robô paletiza e libera quando completa. Entradas lidas do CLP têm prioridade.");
                ui.end_row();
            }
            EquipmentKind::PalletDispenser => {
                row(ui, "Intervalo", &mut p.interval, 0.2..=60.0, " s");
                row(ui, "Altura de saída (topo do TDP)", &mut p.height, 0.0..=3.0, " m");
                row(ui, "Pallet: comprimento (sentido do TDP)", &mut p.pallet_size[0], 0.6..=1.6, " m");
                row(ui, "Pallet: largura", &mut p.pallet_size[2], 0.6..=1.6, " m");
            }
            EquipmentKind::BoxEmitter => {
                row(ui, "Intervalo", &mut p.interval, 0.2..=60.0, " s");
                row(ui, "Altura de saída", &mut p.height, 0.0..=3.0, " m");
                row(ui, "Caixa: comprimento", &mut p.box_size[0], 0.1..=1.2, " m");
                row(ui, "Caixa: altura", &mut p.box_size[1], 0.05..=1.0, " m");
                row(ui, "Caixa: largura", &mut p.box_size[2], 0.1..=1.2, " m");
            }
            EquipmentKind::BoxRemover => {
                row(ui, "Comprimento", &mut p.length, 0.2..=3.0, " m");
                row(ui, "Largura", &mut p.width, 0.2..=3.0, " m");
                row(ui, "Altura base", &mut p.height, 0.0..=3.0, " m");
            }
            EquipmentKind::FullCar => {
                row(ui, "Comprimento do trilho", &mut p.length, 2.0..=60.0, " m");
                row(ui, "TDP do carro: comprimento", &mut p.car_length, 0.6..=3.0, " m");
                row(ui, "TDP do carro: largura", &mut p.car_width, 0.6..=3.0, " m");
                row(ui, "Altura do topo", &mut p.height, 0.1..=3.0, " m");
                row(ui, "Velocidade dos rolos", &mut p.speed, 0.0..=3.0, " m/s");
                row(ui, "Velocidade com pallet", &mut p.car_speed, 0.1..=4.0, " m/s");
                row(ui, "Velocidade vazio", &mut p.car_speed_empty, 0.1..=4.0, " m/s");
                row(ui, "Giro de receber para entregar", &mut p.giro_deg, -180.0..=180.0, "°");
                row(ui, "Velocidade do giro", &mut p.giro_speed, 5.0..=120.0, "°/s");
                ui.label("Paradas (sensor de distância)");
                ui.vertical(|ui| {
                    let mut remove = None;
                    for (i, stop) in p.stops.iter_mut().enumerate() {
                        ui.horizontal(|ui| {
                            let name = match i {
                                0 => "Espera (ID 10)".to_owned(),
                                1 => "Entrega (ID 20)".to_owned(),
                                _ => format!("Linha {} (ID {})", (b'A' + (i as u8 - 2)) as char, (i - 1) * 100),
                            };
                            ui.label(name);
                            ui.add(egui::DragValue::new(stop).speed(0.01).range(0.0..=60.0).suffix(" m"));
                            if i > 2 && ui.small_button("🗑").clicked() {
                                remove = Some(i);
                            }
                        });
                    }
                    if let Some(i) = remove {
                        p.stops.remove(i);
                    }
                    if p.stops.len() < 8 && ui.small_button("+ linha").clicked() {
                        let next = p.stops.last().copied().unwrap_or(1.0) + 2.0;
                        p.stops.push(next);
                    }
                });
                ui.end_row();
                ui.label("Lógica interna");
                ui.checkbox(&mut p.auto_logic, "FBs do carro cheio sem CLP").on_hover_text(
                    "Busca o pallet completo na linha que pede (ID_Busca), gira e entrega; sem pedido, vai para a espera. Entradas lidas do CLP têm prioridade.",
                );
                ui.end_row();
            }
            EquipmentKind::ShuttleCar => {
                row(ui, "Comprimento do trilho", &mut p.length, 2.0..=60.0, " m");
                row(ui, "TDP do carro: comprimento", &mut p.car_length, 0.6..=3.0, " m");
                row(ui, "TDP do carro: largura", &mut p.car_width, 0.6..=3.0, " m");
                row(ui, "Altura do topo", &mut p.height, 0.1..=3.0, " m");
                row(ui, "Velocidade dos rolos", &mut p.speed, 0.0..=3.0, " m/s");
                row(ui, "Velocidade nominal do carro", &mut p.car_speed, 0.1..=4.0, " m/s");
                ui.label("Paradas (sensor de distância)");
                ui.vertical(|ui| {
                    let mut remove = None;
                    let paper_stop = p.paper_stop;
                    let mut new_paper_stop = paper_stop;
                    let mut pickups: Vec<usize> = (0..p.stops.len()).filter(|&i| p.is_pickup(i)).collect();
                    let mut new_pickups = pickups.clone();
                    let (mut letter, mut coleta) = (b'A', 0);
                    for (i, stop) in p.stops.iter_mut().enumerate() {
                        ui.horizontal(|ui| {
                            let is_pickup = pickups.contains(&i);
                            let name = if is_pickup {
                                coleta += 1;
                                format!("Coleta {coleta}")
                            } else if i as i32 == paper_stop {
                                "Papelão".to_owned()
                            } else {
                                letter += 1;
                                format!("Posição {}", (letter - 1) as char)
                            };
                            ui.label(name);
                            ui.add(egui::DragValue::new(stop).speed(0.01).range(0.0..=60.0).suffix(" m"));
                            let mut pick = is_pickup;
                            if ui
                                .checkbox(&mut pick, "Coleta")
                                .on_hover_text("O carro recebe pallets vazios aqui, do lado em que estiver o TDP que entrega")
                                .changed()
                            {
                                if pick {
                                    new_pickups.push(i);
                                } else if new_pickups.len() > 1 {
                                    new_pickups.retain(|&k| k != i);
                                }
                            }
                            if !is_pickup {
                                let mut is_paper = i as i32 == paper_stop;
                                if ui
                                    .checkbox(&mut is_paper, "CP")
                                    .on_hover_text("Parada sob um colocador de papelão: o carro espera o papelão antes do destino")
                                    .changed()
                                {
                                    new_paper_stop = if is_paper { i as i32 } else { -1 };
                                }
                            }
                            if i > 0 && ui.small_button("🗑").clicked() {
                                remove = Some(i);
                            }
                        });
                    }
                    p.paper_stop = new_paper_stop;
                    new_pickups.sort_unstable();
                    new_pickups.dedup();
                    if p.paper_stop >= 0 && new_pickups.contains(&(p.paper_stop as usize)) {
                        p.paper_stop = -1;
                    }
                    pickups = new_pickups;
                    if let Some(i) = remove {
                        p.stops.remove(i);
                        if p.paper_stop == i as i32 {
                            p.paper_stop = -1;
                        } else if p.paper_stop > i as i32 {
                            p.paper_stop -= 1;
                        }
                        pickups = pickups.into_iter().filter(|&k| k != i).map(|k| if k > i { k - 1 } else { k }).collect();
                        if pickups.is_empty() {
                            pickups.push(0);
                        }
                    }
                    p.pickup_stops = pickups;
                    if p.stops.len() < 8 && ui.small_button("+ posição").clicked() {
                        let next = p.stops.last().copied().unwrap_or(1.0) + 1.5;
                        p.stops.push(next);
                    }
                });
                ui.end_row();
                ui.label("Lógica interna");
                ui.checkbox(&mut p.auto_logic, "posicionamento do FB CVZ sem CLP")
                    .on_hover_text("Recebe pallet na coleta que tem pallet esperando, leva às posições A, B... em sequência e volta. Recebe e descarrega do lado em que está o TDP. Entradas lidas do CLP têm prioridade.");
                ui.end_row();
            }
            EquipmentKind::CardboardPlacer => {
                row(ui, "Curso da pega até a entrega", &mut p.length, 0.5..=6.0, " m");
                row(ui, "Curso do cilindro", &mut p.cp_stroke, 0.2..=1.3, " m");
                row(ui, "Papelão: comprimento", &mut p.pallet_size[0], 0.6..=1.6, " m");
                row(ui, "Papelão: largura", &mut p.pallet_size[2], 0.6..=1.6, " m");
                ui.label("Aplicar papelão");
                ui.checkbox(&mut p.apply_paper, "nos pallets que passam");
                ui.end_row();
                ui.label("Aplicar 2x papelão");
                ui.checkbox(&mut p.cp_two_sheets, "dois papelões por pallet");
                ui.end_row();
                ui.label("Lógica interna");
                ui.checkbox(&mut p.auto_logic, "FB do colocador sem CLP")
                    .on_hover_text("Segura o pallet sob a entrega e aplica o papelão. Entradas lidas do CLP têm prioridade.");
                ui.end_row();
            }
            EquipmentKind::PalletElevator => {
                row(ui, "Altura do TDP (topo)", &mut p.height, 0.1..=2.0, " m");
                row(ui, "Quadro: comprimento", &mut p.length, 0.8..=3.0, " m");
                row(ui, "Quadro: largura", &mut p.width, 0.8..=3.0, " m");
                row(ui, "Pallet: comprimento", &mut p.pallet_size[0], 0.6..=1.6, " m");
                row(ui, "Pallet: altura", &mut p.pallet_size[1], 0.08..=0.3, " m");
                row(ui, "Pallet: largura", &mut p.pallet_size[2], 0.6..=1.6, " m");
                ui.label("Pallets na pilha cheia");
                ui.add(egui::DragValue::new(&mut p.stack_size).range(1..=MAX_STACK));
                ui.end_row();
                ui.label("Abastecer automaticamente");
                ui.checkbox(&mut p.auto_refill, "nova pilha quando acabar")
                    .on_hover_text("Quando o último pallet sai e o elevador espera vazio embaixo, uma pilha cheia é colocada no TDP após 3 s.");
                ui.end_row();
                ui.label("Lógica interna");
                ui.checkbox(&mut p.auto_logic, "FB do elevador sem CLP").on_hover_text(
                    "Desce a pilha, solta o pallet de baixo no TDP e levanta o resto. TDPProntoReceber vem do TDP sob o elevador. Entradas lidas do CLP têm prioridade.",
                );
                ui.end_row();
            }
            EquipmentKind::Sensor => {
                row(ui, "Comprimento do feixe", &mut p.width, 0.1..=5.0, " m");
                row(ui, "Altura do feixe", &mut p.height, 0.05..=3.0, " m");
            }
        }
    });
    let geometry_changed = before.length != eq.params.length
        || before.width != eq.params.width
        || before.height != eq.params.height
        || (kind.is_mesa() && (before.box_size != eq.params.box_size || before.fardo_10kg != eq.params.fardo_10kg))
        || (kind == EquipmentKind::CardboardPlacer && before.pallet_size != eq.params.pallet_size)
        || before.belt_entry_sensor != eq.params.belt_entry_sensor
        || before.belt_entry_pos != eq.params.belt_entry_pos
        || before.belt_exit_pos != eq.params.belt_exit_pos
        || before.belt_exit_sensor != eq.params.belt_exit_sensor
        || before.robot_model != eq.params.robot_model
        || before.robot_base != eq.params.robot_base
        || before.base_height != eq.params.base_height
        || before.base_width != eq.params.base_width
        || before.base_depth != eq.params.base_depth
        || before.base_yaw != eq.params.base_yaw
        || before.car_length != eq.params.car_length
        || before.car_width != eq.params.car_width
        || (kind == EquipmentKind::PalletElevator
            && (before.pallet_size != eq.params.pallet_size || before.stack_size != eq.params.stack_size));
    if before.robot_model != eq.params.robot_model {
        // The limits of the other model do not apply.
        eq.params.axis_limits.clear();
    }
    if geometry_changed {
        commands.entity(entity).insert(NeedsRebuild);
    }

    match kind {
        EquipmentKind::Robot => {
            if let Ok(robot) = robots.get(entity) {
                ui.separator();
                ui.label(format!("Estado: {}", robot.status));
                ui.label(format!("Ciclos executados: {}", robot.cycles));
                let red = egui::Color32::from_rgb(230, 90, 70);
                if robot.status.starts_with("Falha") {
                    ui.colored_label(red, &robot.status);
                }
                egui::CollapsingHeader::new(format!("Log de falhas ({})", robot.faults.len()))
                    .default_open(!robot.faults.is_empty())
                    .show(ui, |ui| {
                        if robot.faults.is_empty() {
                            ui.weak("Nenhuma falha.");
                        }
                        for (t, details) in robot.faults.iter().rev().take(20) {
                            ui.horizontal_wrapped(|ui| {
                                ui.colored_label(red, format!("{t:.1} s"));
                                ui.label(details);
                            });
                        }
                    });
                ui.weak(format!(
                    "TCP: θ {:.0}° · r {:.2} m · z {:.2} m",
                    robot.pose.theta.to_degrees(),
                    robot.pose.r,
                    robot.pose.y
                ));
                // Axes against the data sheet limits and speeds.
                let kin = &robot.kin;
                let axes = kin.to_motor(kin.axes(&robot.pose));
                egui::Grid::new("robot_axes").striped(true).show(ui, |ui| {
                    for h in ["Eixo", "Posição", "Faixa", "Velocidade", "Máx."] {
                        ui.strong(h);
                    }
                    ui.end_row();
                    for (i, axis) in axes.iter().enumerate() {
                        let (lo, hi) = (kin.min[i].to_degrees(), kin.max[i].to_degrees());
                        let pos = axis.to_degrees();
                        let speed = robot.axis_speed[i].abs().to_degrees();
                        let max = kin.speed[i].to_degrees();
                        ui.label(kin.axis_names[i]);
                        let text = format!("{pos:.1}°");
                        if pos - lo < 3.0 || hi - pos < 3.0 {
                            ui.colored_label(egui::Color32::from_rgb(230, 150, 40), text).on_hover_text("Perto do limite");
                        } else {
                            ui.label(text);
                        }
                        ui.weak(format!("{lo:.0}° … {hi:.0}°"));
                        ui.label(format!("{speed:.0}°/s ({:.0}%)", 100.0 * speed / max));
                        ui.weak(format!("{max:.0}°/s"));
                        ui.end_row();
                    }
                });
                if kin.relative_a3 {
                    ui.weak(format!(
                        "A5 (faixa -55° … 150°, 225°/s) mantém o flange na horizontal. A3 é o ângulo entre os braços; o braço sobe no máximo {:.0}° acima da horizontal.",
                        kin.max_elevation.to_degrees()
                    ));
                }
                ui.collapsing(format!("Folha de dados {}", kin.model.label()), |ui| {
                    ui.label(format!(
                        "Carga {:.0} kg · alcance {:.0} mm · repetibilidade ±{} mm · massa {:.0} kg",
                        kin.payload_kg,
                        kin.reach * 1000.0,
                        kin.repeatability_mm,
                        kin.mass_kg
                    ));
                    ui.label(format!(
                        "Centro do flange: de {:.0} a {:.0} mm de altura · raio mínimo {:.0} mm no piso · interferência traseira R{:.0}",
                        kin.min_height * 1000.0,
                        kin.max_height * 1000.0,
                        kin.min_radius * 1000.0,
                        kin.rear_radius * 1000.0
                    ));
                    ui.weak("Movimentos como nas rotinas TP: transferências em J 100 % e subidas/descidas em L (PICK 3000/1800 mm/s, PLACE 2000/4000 mm/s), rampa de aceleração de 0,35 s, tudo multiplicado pelo override.");
                });
            }
        }
        EquipmentKind::CardboardPlacer => {
            if let Ok((mut cp, logic)) = placers.get_mut(entity) {
                ui.separator();
                ui.label(format!(
                    "Passo {} · papelões no depósito: {}{}",
                    logic.passo,
                    cp.sheets_left,
                    if logic.sem_papelao { " · SEM PAPELÃO" } else { "" }
                ));
                ui.label(format!(
                    "Pallet sob a entrega: {} · FimDoCiclo {}",
                    if cp.pallet.is_some() { "sim" } else { "não" },
                    u8::from(logic.fim_do_ciclo)
                ));
                if ui.button("Reabastecer depósito (100)").clicked() {
                    cp.sheets_left = 100;
                }
            }
        }
        EquipmentKind::PalletElevator => {
            if let Ok((mut elevator, mut logic)) = elevators.get_mut(entity) {
                ui.separator();
                let garra = if elevator.clamp >= 0.98 {
                    "fechada"
                } else if elevator.clamp <= 0.02 {
                    "aberta"
                } else {
                    "em movimento"
                };
                ui.label(format!(
                    "Passo {} · pallets na pilha: {} · garra {garra} a {:.0} mm",
                    logic.passo,
                    elevator.count,
                    elevator.z * 1000.0
                ));
                ui.label(format!("FimDoCiclo {}", u8::from(logic.fim_do_ciclo)));
                let red = egui::Color32::from_rgb(230, 70, 60);
                for (on, text) in [
                    (logic.luz_sem_pallet, "SEM PALLET NA PILHA"),
                    (logic.falha_sensor_central, "Falha: sensor intermediário não detectado"),
                    (logic.falha_abre_fecha, "Falha: tempo de abrir/fechar a garra"),
                    (logic.falha_sobe_desce, "Falha: tempo de subir/descer"),
                ] {
                    if on {
                        ui.colored_label(red, text);
                    }
                }
                ui.horizontal(|ui| {
                    let full = eq.params.stack_size.min(MAX_STACK);
                    if ui.button(format!("Reabastecer pilha ({full})")).clicked() {
                        // A new stack is set on the TDP, or hangs from closed clamps.
                        elevator.resting = elevator.clamp < 0.98;
                        elevator.count = full;
                    }
                    if ui.button("Reiniciar (CmdReiniciar)").clicked() {
                        logic.reiniciar = true;
                    }
                });
            }
        }
        EquipmentKind::FullCar => {
            if let Ok((car, ..)) = cars.get(entity) {
                ui.separator();
                ui.label(format!(
                    "Posição atual {:.0} mm · velocidade {:.0} mm/s · giro {:.0}°{}",
                    car.x * 1000.0,
                    car.v * 1000.0,
                    car.giro.to_degrees(),
                    if car.loaded { " · com pallet" } else { "" }
                ));
            }
            if let Ok((_, _, Some(logic))) = cars.get(entity)
                && eq.params.auto_logic
            {
                ui.label(format!(
                    "Movimento passo {} · TDP passo {} · ID_Busca {} · ID_Destino {} · INP {}",
                    logic.passo,
                    logic.tdp_passo,
                    logic.id_busca,
                    logic.id_destino,
                    u8::from(logic.inp)
                ));
            }
        }
        EquipmentKind::ShuttleCar => {
            if let Ok((car, Some(logic), _)) = cars.get(entity) {
                ui.separator();
                ui.label(format!(
                    "Posição atual {:.0} mm · velocidade {:.0} mm/s{}",
                    car.x * 1000.0,
                    car.v * 1000.0,
                    if car.loaded { " · com pallet" } else { "" }
                ));
                if eq.params.auto_logic {
                    ui.label(format!(
                        "Lógica interna: {:?} · parada {} · lado {:+} · INP {}",
                        logic.step,
                        logic.target,
                        logic.side,
                        u8::from(logic.inp)
                    ));
                }
            }
        }
        k if k.is_mesa() => {
            if let Ok(logic) = mesas.get(entity) {
                ui.separator();
                if eq.params.auto_logic {
                    ui.label(logic.status());
                    ui.label(format!(
                        "Coleta: pronta receber {} · recebido {} · Autoriza_Pick {}",
                        u8::from(logic.coleta.pronto_receber),
                        u8::from(logic.coleta.recebido),
                        u8::from(logic.coleta.autoriza_pick)
                    ));
                    if eq.params.fardo_10kg {
                        ui.label(format!("Receita 10x1 · próximo grupo: {} fardos (FardosPonto)", logic.fardos_ponto));
                    }
                } else {
                    ui.weak("Lógica interna desligada: os atuadores vêm do CLP ou do \"forçar\".");
                }
            }
        }
        EquipmentKind::PalletizingConveyor => {
            if let Ok(tdp) = palletizers.get(entity) {
                ui.separator();
                if tdp.pallet.is_some() {
                    ui.label(format!(
                        "Pallet em posição: {} / {} coletas ({} fardos){}",
                        tdp.units,
                        tdp.capacity,
                        tdp.placed,
                        if tdp.full { " · COMPLETO" } else { "" }
                    ));
                } else {
                    ui.label("Sem pallet no batente.");
                }
            }
        }
        _ => {}
    }

    ui.separator();
    ui.heading("Sinais e tags");
    let key = tag_cache_key(project);
    if key != state.tag_cache.0 {
        state.tag_cache = (key, tag_options(project));
    }
    let tags = state.tag_cache.1.clone();
    egui::Grid::new("signal_bindings").num_columns(3).striped(true).show(ui, |ui| {
        for signal in kind.signals() {
            ui.label(format!("[{}] {}", if signal.dir == SignalDir::In { "E" } else { "S" }, signal.label))
                    .on_hover_text(direction_hint(signal.dir));
            let current = bindings.tag_for(signal.key).map(str::to_owned);
            let mut chosen = current.clone();
            tag_combo(ui, signal.key, &mut chosen, &tags, &mut state.tag_filter);
            if chosen != current {
                bindings.set(signal.key, chosen.clone());
            }
            let auto = auto_driven(&eq, signal.key);
            value_cell(ui, signal, kind, &mut io, chosen.as_deref().is_some_and(|t| link.tag_online(t)), auto);
            ui.end_row();
            if let Some(tag) = &chosen
                && let Err(problem) = resolve_tag(project, tag) {
                    ui.label("");
                    ui.colored_label(egui::Color32::from_rgb(230, 160, 40), format!("⚠ {problem}"));
                    ui.end_row();
                }
        }
    });
    if tags.is_empty() {
        ui.weak("Importe o projeto TIA e marque tabelas/DBs na aba \"TIA Portal V18\" para vincular tags.");
    }
}

fn direction_hint(dir: SignalDir) -> &'static str {
    match dir {
        SignalDir::In => "Entrada do simulador: escrita pelo CLP",
        SignalDir::Out => "Saída do simulador: escrita no CLP",
    }
}

/// Several equipment names; none selected means every one.
fn names_select(ui: &mut egui::Ui, id: &str, value: &mut Vec<String>, options: &[String], empty_label: &str) {
    let text = if value.is_empty() { empty_label.to_owned() } else { value.join(", ") };
    egui::ComboBox::from_id_salt(id).selected_text(text).show_ui(ui, |ui| {
        if ui.selectable_label(value.is_empty(), empty_label).clicked() {
            value.clear();
        }
        for name in options {
            let mut on = value.contains(name);
            if ui.checkbox(&mut on, name).changed() {
                if on {
                    value.push(name.clone());
                } else {
                    value.retain(|n| n != name);
                }
            }
        }
    });
}

fn tag_cache_key(project: &ProjectState) -> Vec<(String, usize, Option<u32>)> {
    let tables = project.0.tag_tables.iter().filter(|t| t.active).map(|t| (t.id.clone(), t.tags.len(), None));
    let dbs = project
        .0
        .data_blocks
        .iter()
        .filter(|d| d.active)
        .map(|d| (d.id.clone(), d.tags.len() * 2 + usize::from(d.optimized), d.number));
    tables.chain(dbs).collect()
}

/// (tag id, label) of every tag in active tables/DBs.
fn tag_options(project: &ProjectState) -> Vec<(String, String)> {
    project
        .0
        .active_tags()
        .map(|(tag, address, _)| {
            (
                tag.id.clone(),
                format!("{} · {} · {}", tag.id, tag.data_type.short(), address.as_deref().unwrap_or("sem endereço")),
            )
        })
        .collect()
}

fn tag_combo(ui: &mut egui::Ui, id: &str, chosen: &mut Option<String>, tags: &[(String, String)], filter: &mut String) {
    let text = chosen.clone().unwrap_or_else(|| "(sem tag)".into());
    egui::ComboBox::from_id_salt(("tag", id))
        .selected_text(text)
        .width(200.0)
        .height(400.0)
        .show_ui(ui, |ui| {
            ui.add(egui::TextEdit::singleline(filter).hint_text("filtrar..."));
            if ui.selectable_label(chosen.is_none(), "(sem tag)").clicked() {
                *chosen = None;
            }
            let needle = filter.to_lowercase();
            for (tag_id, label) in tags.iter().filter(|(_, l)| needle.is_empty() || l.to_lowercase().contains(&needle)).take(300) {
                if ui.selectable_label(chosen.as_deref() == Some(tag_id), label).clicked() {
                    *chosen = Some(tag_id.clone());
                }
            }
        });
}

/// Shows the live value; unbound/offline inputs get a manual toggle.
/// Inputs driven by the internal logic of a TDP de paletização (not editable).
fn auto_driven(eq: &Equipment, key: &str) -> bool {
    let actuator = match eq.kind {
        EquipmentKind::PalletizingConveyor => matches!(key, "motor" | "stop"),
        k if k.is_mesa()
            || matches!(
                k,
                EquipmentKind::ShuttleCar | EquipmentKind::FullCar | EquipmentKind::CardboardPlacer | EquipmentKind::PalletElevator
            ) => eq.kind.signals().iter().any(|s| s.key == key && s.dir == SignalDir::In),
        _ => false,
    };
    actuator && eq.params.auto_logic
}

fn value_cell(ui: &mut egui::Ui, signal: &crate::model::SignalDef, kind: EquipmentKind, io: &mut Io, from_plc: bool, auto: bool) {
    let value = io.get(signal.key);
    if signal.dir == SignalDir::In && !from_plc && auto {
        let text = match signal.ty {
            SignalType::Int => format!("{value:.0} (auto)"),
            SignalType::Bool => format!("{} (auto)", u8::from(value >= 0.5)),
        };
        let color = if value != 0.0 { egui::Color32::from_rgb(60, 200, 90) } else { egui::Color32::GRAY };
        ui.colored_label(color, text).on_hover_text("Controlado pela lógica interna do equipamento");
    } else if signal.dir == SignalDir::In && !from_plc && signal.ty == SignalType::Int {
        let mut manual = io.manual.get(signal.key).copied().unwrap_or_else(|| default_manual(kind, signal.key));
        if ui.add(egui::DragValue::new(&mut manual).speed(10.0)).on_hover_text("forçar valor").changed() {
            io.manual.insert(signal.key.to_owned(), manual.round());
        }
    } else if signal.dir == SignalDir::In && !from_plc {
        let mut manual = io
            .manual
            .get(signal.key)
            .copied()
            .unwrap_or_else(|| default_manual(kind, signal.key))
            >= 0.5;
        if ui.checkbox(&mut manual, "forçar").changed() {
            io.manual.insert(signal.key.to_owned(), if manual { 1.0 } else { 0.0 });
        }
    } else {
        match signal.ty {
            SignalType::Bool => {
                let (color, text) = if value >= 0.5 {
                    (egui::Color32::from_rgb(60, 200, 90), "1")
                } else {
                    (egui::Color32::GRAY, "0")
                };
                ui.colored_label(color, text);
            }
            SignalType::Int => {
                ui.label(format!("{value:.0}"));
            }
        }
    }
}

fn tia_tab(ui: &mut egui::Ui, state: &mut UiState, project: &mut ProjectState) {
    ui.heading("Projeto TIA Portal V18");
    ui.label("Auxiliar Openness (TiaOpennessBridge.exe):");
    ui.horizontal(|ui| {
        ui.add(egui::TextEdit::singleline(&mut state.helper_path).desired_width(280.0));
        if ui.button("...").clicked()
            && let Some(path) = rfd::FileDialog::new().add_filter("Executável", &["exe"]).pick_file() {
                state.helper_path = path.to_string_lossy().into_owned();
            }
    });
    let busy = state.import_rx.is_some();
    if ui
        .add_enabled(!busy, egui::Button::new("📂 Importar projeto .ap18 (Openness)..."))
        .clicked()
        && let Some(project_path) = rfd::FileDialog::new()
            .add_filter("Projeto TIA Portal V18", &["ap18"])
            .add_filter("Projeto TIA Portal", &["ap15_1", "ap16", "ap17", "ap18", "ap19"])
            .pick_file()
        {
            let helper = std::path::PathBuf::from(&state.helper_path);
            let (tx, rx) = unbounded();
            state.import_status = "TIA Portal Openness está abrindo o projeto em segundo plano (pode levar alguns minutos; confirme a permissão de acesso Openness se o TIA perguntar)...".into();
            state.import_rx = Some(rx);
            std::thread::spawn(move || {
                let _ = tx.send(import_project_via_openness(&helper, &project_path));
            });
        }
    if busy {
        ui.horizontal(|ui| {
            ui.spinner();
            ui.label("Importando...");
        });
    }
    ui.weak(&state.import_status);
    ui.collapsing("Importar arquivos exportados manualmente", |ui| {
        if ui.button("Tabela de tags (XML / CSV / XLSX)...").clicked()
            && let Some(path) = rfd::FileDialog::new().add_filter("Tabela de tags", &["xml", "xlsx", "csv"]).pick_file() {
                let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("Tabela").to_owned();
                let ext = path.extension().and_then(|x| x.to_str()).unwrap_or("").to_ascii_lowercase();
                let parsed = match ext.as_str() {
                    "xml" => std::fs::File::open(&path)
                        .map_err(|e| e.to_string())
                        .and_then(|f| parse_tag_table_xml(std::io::BufReader::new(f), Some(&stem))),
                    "csv" => std::fs::File::open(&path)
                        .map_err(|e| e.to_string())
                        .and_then(|f| parse_tag_table_csv(f, &stem)),
                    _ => parse_tag_table_xlsx(&path, &stem),
                };
                match parsed {
                    Ok(mut table) => {
                        table.active = true;
                        state.import_status = format!("Tabela '{}' importada com {} tags.", table.name, table.tags.len());
                        project.0.tag_tables.retain(|t| t.id != table.id);
                        project.0.tag_tables.push(table);
                    }
                    Err(err) => state.import_status = format!("Falha ao importar tabela: {err}"),
                }
            }
        if ui.button("Fonte de DB (.db / .scl / .udt)...").clicked()
            && let Some(path) = rfd::FileDialog::new().add_filter("Fonte TIA", &["db", "scl", "udt"]).pick_file() {
                match std::fs::read_to_string(&path) {
                    Ok(source) => {
                        let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("DB");
                        let (mut db, warnings) = parse_db_source(&source, stem);
                        db.active = true;
                        state.import_status = format!(
                            "DB '{}' importado com {} tags. Informe o número do DB abaixo.{}",
                            db.name,
                            db.tags.len(),
                            if warnings.is_empty() { String::new() } else { format!(" Avisos: {}", warnings.join("; ")) }
                        );
                        project.0.data_blocks.retain(|d| d.id != db.id);
                        project.0.data_blocks.push(db);
                    }
                    Err(e) => state.import_status = format!("Falha ao ler fonte: {e}"),
                }
            }
        if ui.button("Bloco SimaticML (XML exportado via Openness)...").clicked()
            && let Some(path) = rfd::FileDialog::new().add_filter("SimaticML", &["xml"]).pick_file() {
                match std::fs::read_to_string(&path).map_err(|e| e.to_string()).and_then(|x| parse_simatic_ml(&x)) {
                    Ok(block) => {
                        let mut diags = Vec::new();
                        let mut db = data_block_from_members(
                            &block.name,
                            block.number,
                            block.optimized,
                            &block.members,
                            &Default::default(),
                            &mut diags,
                        );
                        db.active = true;
                        state.import_status = format!("DB '{}' importado com {} tags.", db.name, db.tags.len());
                        project.0.diagnostics.extend(diags);
                        project.0.data_blocks.retain(|d| d.id != db.id);
                        project.0.data_blocks.push(db);
                    }
                    Err(e) => state.import_status = format!("Falha: {e}"),
                }
            }
    });
    ui.separator();
    if project.0.tag_tables.is_empty() && project.0.data_blocks.is_empty() {
        ui.label("Nenhum projeto importado.");
        return;
    }
    if !project.0.name.is_empty() {
        ui.label(format!("Projeto: {}", project.0.name));
    }
    ui.horizontal(|ui| {
        ui.label("Filtro:");
        ui.text_edit_singleline(&mut state.tia_filter);
        if ui.small_button("Limpar projeto").clicked() {
            project.0 = TiaProject::default();
        }
    });
    let filter = state.tia_filter.to_lowercase();
    ui.label("Marque as tabelas e DBs cujas tags devem ficar disponíveis para vínculo:");
    ui.strong("Tabelas de tags");
    for table in &mut project.0.tag_tables {
        ui.horizontal(|ui| {
            ui.checkbox(&mut table.active, "");
            let plc = if table.plc.is_empty() { String::new() } else { format!(" · {}", table.plc) };
            egui::CollapsingHeader::new(format!("{} ({}){plc}", table.name, table.tags.len()))
                .id_salt(("table", table.id.clone()))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label("CLP:");
                        ui.add(egui::TextEdit::singleline(&mut table.plc).desired_width(160.0));
                    });
                    tag_grid(ui, &table.id, table.tags.iter().map(|t| (t, t.address.clone())), &filter)
                });
        });
    }
    ui.strong("Blocos de dados (DB)");
    for db in &mut project.0.data_blocks {
        ui.horizontal(|ui| {
            ui.checkbox(&mut db.active, "");
            let header = format!(
                "{}{} ({}){}{}",
                db.name,
                db.number.map(|n| format!(" [DB{n}]")).unwrap_or_default(),
                db.tags.len(),
                if db.optimized { " · otimizado" } else { "" },
                if db.plc.is_empty() { String::new() } else { format!(" · {}", db.plc) }
            );
            let tags: Vec<_> = db.tags.iter().map(|t| (t.clone(), db.absolute_address(t))).collect();
            egui::CollapsingHeader::new(header).id_salt(("db", db.id.clone())).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("CLP:");
                    ui.add(egui::TextEdit::singleline(&mut db.plc).desired_width(160.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Número do DB:");
                    let mut number = db.number.unwrap_or(0);
                    if ui.add(egui::DragValue::new(&mut number).range(0..=65535)).changed() {
                        db.number = (number > 0).then_some(number);
                    }
                });
                ui.checkbox(&mut db.optimized, "Acesso otimizado (sem endereço absoluto)");
                if db.optimized {
                    ui.colored_label(
                        egui::Color32::from_rgb(230, 160, 40),
                        "Para comunicação S7, desative \"Optimized block access\" nas propriedades do DB, compile e importe de novo.",
                    );
                }
                tag_grid(ui, &db.id, tags.iter().map(|(t, a)| (t, a.clone())), &filter);
            });
        });
    }
    if !project.0.diagnostics.is_empty() {
        egui::CollapsingHeader::new(format!("Avisos da importação ({})", project.0.diagnostics.len()))
            .id_salt("diagnostics")
            .show(ui, |ui| {
                for d in &project.0.diagnostics {
                    ui.weak(d);
                }
            });
    }
}

fn tag_grid<'a>(
    ui: &mut egui::Ui,
    id: &str,
    tags: impl Iterator<Item = (&'a crate::tia_importer::PlcTag, Option<String>)>,
    filter: &str,
) {
    egui::Grid::new(("tags", id)).striped(true).num_columns(3).show(ui, |ui| {
        for (tag, address) in tags.filter(|(t, _)| filter.is_empty() || t.name.to_lowercase().contains(filter)).take(500) {
            ui.label(&tag.name).on_hover_text(tag.comment.clone().unwrap_or_default());
            ui.weak(tag.data_type.short());
            ui.monospace(address.unwrap_or_else(|| "—".into()));
            ui.end_row();
        }
    });
}

fn signals_tab(ui: &mut egui::Ui, link: &PlcLink, selection: &mut Selection, equipment: &mut EquipmentQuery) {
    ui.heading("Mapa de I/O da célula");
    ui.weak("[E] = escrita pelo CLP, [S] = escrita pelo simulador. Entradas sem tag (ou com o CLP desconectado) usam o valor forçado.");
    let mut rows: Vec<_> = equipment.iter_mut().collect();
    rows.sort_by(|a, b| a.1.name.cmp(&b.1.name));
    egui::Grid::new("io_map").striped(true).num_columns(4).show(ui, |ui| {
        ui.strong("Equipamento");
        ui.strong("Sinal ([E] entrada / [S] saída)");
        ui.strong("Tag");
        ui.strong("Valor");
        ui.end_row();
        for (entity, eq, _, bindings, io) in rows.iter_mut() {
            for signal in eq.kind.signals() {
                if ui.selectable_label(selection.0 == Some(*entity), &eq.name).clicked() {
                    selection.0 = Some(*entity);
                }
                ui.label(format!("[{}] {}", if signal.dir == SignalDir::In { "E" } else { "S" }, signal.label))
                    .on_hover_text(direction_hint(signal.dir));
                let tag = bindings.tag_for(signal.key);
                ui.monospace(tag.unwrap_or("—"));
                let auto = auto_driven(eq, signal.key);
                value_cell(ui, signal, eq.kind, io, tag.is_some_and(|t| link.tag_online(t)), auto);
                ui.end_row();
            }
        }
    });
}

fn plc_tab(ui: &mut egui::Ui, link: &mut PlcLink, project: &ProjectState) {
    ui.heading("Comunicação S7 (PUT/GET)");
    ui.weak("Uma conexão por CLP. Cada tag é lida/escrita na conexão do CLP a que pertence no projeto TIA.");
    let plc_names = project.0.plc_names();
    let mut remove = None;
    let mut changed = false;
    for (index, conn) in link.connections.iter_mut().enumerate() {
        let (color, state) = if conn.connected {
            (egui::Color32::from_rgb(60, 200, 90), "conectado")
        } else if conn.enabled {
            (egui::Color32::from_rgb(230, 170, 40), "conectando")
        } else {
            (egui::Color32::GRAY, "desconectado")
        };
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.add(egui::TextEdit::singleline(&mut conn.name).desired_width(120.0));
                ui.colored_label(color, state);
                if ui.small_button("🗑").on_hover_text("Remover conexão").clicked() {
                    remove = Some(index);
                }
            });
            egui::Grid::new(("plc_settings", index)).num_columns(2).show(ui, |ui| {
                ui.label("CLP do projeto");
                let before = conn.plc.clone();
                egui::ComboBox::from_id_salt(("plc_name", index))
                    .selected_text(if conn.plc.is_empty() { "(qualquer / sem CLP)".to_owned() } else { conn.plc.clone() })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut conn.plc, String::new(), "(qualquer / sem CLP)");
                        for name in &plc_names {
                            ui.selectable_value(&mut conn.plc, name.clone(), name);
                        }
                    });
                changed |= before != conn.plc;
                ui.end_row();
                ui.label("IP do CLP / PLCSIM Advanced");
                ui.text_edit_singleline(&mut conn.settings.host);
                ui.end_row();
                ui.label("Rack / Slot");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut conn.settings.rack).range(0..=7));
                    ui.add(egui::DragValue::new(&mut conn.settings.slot).range(0..=31));
                });
                ui.end_row();
                ui.label("Ciclo de leitura");
                ui.add(egui::DragValue::new(&mut conn.settings.poll_ms).range(10..=2000).suffix(" ms"));
                ui.end_row();
            });
            ui.horizontal(|ui| {
                if ui.button("Conectar").clicked() {
                    conn.connect();
                }
                if ui.button("Desconectar").clicked() {
                    conn.disconnect();
                }
            });
            ui.weak(&conn.status);
        });
    }
    if let Some(index) = remove {
        let mut conn = link.connections.remove(index);
        conn.disconnect();
        changed = true;
    }
    if ui.button("+ Adicionar CLP").clicked() {
        let taken: Vec<&str> = link.connections.iter().map(|c| c.plc.as_str()).collect();
        let plc = plc_names.iter().find(|n| !taken.contains(&n.as_str())).cloned().unwrap_or_default();
        let name = crate::model::unique_name("CLP", link.connections.iter().map(|c| c.name.as_str()));
        link.connections.push(PlcConnection::new(name, plc, S7Settings::default()));
        changed = true;
    }
    if changed {
        link.revision += 1;
    }
    ui.separator();
    let mut ids: Vec<String> = link.values.keys().cloned().collect();
    if !ids.is_empty() {
        ui.strong("Tags em uso");
        ids.sort();
        egui::Grid::new("plc_values").striped(true).num_columns(3).show(ui, |ui| {
            for id in ids {
                ui.monospace(&id);
                ui.weak(link.route_label(&id).unwrap_or_default());
                ui.label(format!("{:.3}", link.values.get(&id).copied().unwrap_or_default()));
                ui.end_row();
            }
        });
        ui.separator();
    }
    ui.collapsing("Configuração necessária no TIA Portal", |ui| {
        ui.label(
            "• CPU > Proteção e segurança > Mecanismos de conexão: marque \"Permitir acesso com comunicação PUT/GET\".\n\
             • DBs usados pelo simulador: desmarque \"Acesso otimizado ao bloco\" e compile.\n\
             • S7-1200/1500: rack 0, slot 1. PLCSIM Advanced: modo de comunicação TCP/IP (adaptador virtual) e use o IP da instância.\n\
             • Sinais simulador > CLP: prefira %M ou DB. %I só funciona em endereços sem módulo de entrada atribuído.",
        );
    });
}

/// Rubber-band rectangle while selecting by dragging on empty space.
pub fn selection_band(mut contexts: EguiContexts, drag: Res<crate::editor::DragState>) {
    let Some((a, b)) = drag.band else { return };
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let rect = egui::Rect::from_two_pos(egui::pos2(a.x, a.y), egui::pos2(b.x, b.y));
    let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("selection_band")));
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgba_unmultiplied(80, 170, 255, 40));
    painter.rect_stroke(rect, 0.0, egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(80, 170, 255)), egui::StrokeKind::Inside);
}

/// "Salvar": the open project's file, or ask for one.
fn save_project(layout: &mut LayoutRequests) {
    match layout.current.clone() {
        Some(path) => layout.save = Some(path),
        None => save_as(layout),
    }
}

/// "Salvar como": a file in the projects folder by default.
fn save_as(layout: &mut LayoutRequests) {
    let name = layout
        .current
        .as_ref()
        .and_then(|p| p.file_name())
        .map_or_else(|| "Novo projeto.json".to_owned(), |n| n.to_string_lossy().into_owned());
    if let Some(path) = rfd::FileDialog::new()
        .add_filter("Layout de célula", &["json"])
        .set_directory(crate::projects::projects_dir())
        .set_file_name(name)
        .save_file()
    {
        layout.save = Some(path);
    }
}
