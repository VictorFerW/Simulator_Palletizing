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
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            tab: SideTab::Properties,
            helper_path: default_helper_path(),
            import_status: "Selecione um projeto .ap18 do TIA Portal V18.".into(),
            import_rx: None,
            tag_filter: String::new(),
            tia_filter: String::new(),
            tag_cache: Default::default(),
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
    mut layout: ResMut<LayoutRequests>,
    mut link: ResMut<PlcLink>,
    mut equipment: EquipmentQuery,
    robots: Query<&RobotState>,
    palletizers: Query<&PalletizerState>,
    mesas: Query<&MesaLogic>,
    cars: Query<(&ShuttleState, &ShuttleLogic)>,
    mut commands: Commands,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    // The project is edited through `bypass_change_detection` so that drawing the
    // TIA tab does not mark it changed every frame (tag routes are rebuilt on change).
    let fingerprint_before = project_fingerprint(&project);
    poll_import(&mut state, project.bypass_change_detection());

    egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
        ui.horizontal_wrapped(|ui| {
            ui.menu_button("Arquivo", |ui| {
                if ui.button("Nova célula padrão").clicked() {
                    layout.new_default = true;
                    ui.close();
                }
                if ui.button("Cena vazia").clicked() {
                    layout.clear = true;
                    ui.close();
                }
                ui.separator();
                if ui.button("Abrir layout...").clicked() {
                    layout.load = rfd::FileDialog::new().add_filter("Layout de célula", &["json"]).pick_file();
                    ui.close();
                }
                if ui.button("Salvar layout...").clicked() {
                    layout.save = rfd::FileDialog::new()
                        .add_filter("Layout de célula", &["json"])
                        .set_file_name("celula_paletizacao.json")
                        .save_file();
                    ui.close();
                }
            });
            ui.separator();
            let label = if control.running { "⏸ Pausar" } else { "▶ Executar" };
            if ui.button(label).clicked() {
                control.running = !control.running;
            }
            if ui.button("⟲ Reiniciar").clicked() {
                control.reset_requested = true;
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

    egui::SidePanel::left("catalog").default_width(230.0).show(ctx, |ui| {
        ui.heading("Catálogo");
        for kind in EquipmentKind::ALL {
            if ui.button(format!("+ {}", kind.label())).clicked() {
                requests.add.push(kind);
            }
        }
        ui.separator();
        ui.heading("Cena");
        let mut items: Vec<(Entity, String, EquipmentKind)> =
            equipment.iter().map(|(e, eq, ..)| (e, eq.name.clone(), eq.kind)).collect();
        items.sort_by(|a, b| a.1.cmp(&b.1));
        egui::ScrollArea::vertical().id_salt("scene_list").max_height(320.0).show(ui, |ui| {
            for (entity, name, kind) in items {
                ui.horizontal(|ui| {
                    let marked = selection.1.contains(&entity);
                    if ui
                        .selectable_label(marked, format!("{name}  ·  {}", kind.prefix()))
                        .on_hover_text("Ctrl+clique: adicionar/remover da seleção")
                        .clicked()
                    {
                        if ui.input(|i| i.modifiers.command) {
                            if marked {
                                selection.1.retain(|e| *e != entity);
                                selection.0 = selection.1.first().copied();
                            } else {
                                selection.1.push(entity);
                                selection.0 = Some(entity);
                            }
                        } else {
                            selection.1 = vec![entity];
                            selection.0 = Some(entity);
                        }
                    }
                    if ui.small_button("🗑").on_hover_text("Excluir").clicked() {
                        requests.delete.push(entity);
                    }
                });
            }
        });
        ui.separator();
        ui.collapsing("Controles", |ui| {
            ui.label(
                "Clique: selecionar · Ctrl+clique: somar à seleção\n\
                 Arrastar no vazio: seleção por retângulo\n\
                 Arrastar item: mover a seleção (Alt inverte a grade)\n\
                 R / Shift+R: girar ±90° · Q/E: girar ±15°\n\
                 Setas: mover 0,1 m (Shift 0,5 m) · PgUp/PgDn: altura\n\
                 Del: excluir · Ctrl+D: duplicar · F: focar · Esc: desmarcar\n\
                 Botão direito: orbitar · Meio ou Shift+direito: deslocar · Roda: zoom",
            );
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
    if project_fingerprint(&project) != fingerprint_before {
        project.set_changed();
    }

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
    palletizers: &Query<&PalletizerState>,
    mesas: &Query<&MesaLogic>,
    cars: &Query<(&ShuttleState, &ShuttleLogic)>,
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
    ui.heading(eq.kind.label());
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

    let before = eq.params.clone();
    let kind = eq.kind;
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
            }
            EquipmentKind::Table2Stage | EquipmentKind::Table3Stage => {
                row(ui, "Comprimento total", &mut p.length, 0.6..=10.0, " m");
                row(ui, "Largura (rolete)", &mut p.width, 0.4..=2.0, " m");
                row(ui, "Altura do topo", &mut p.height, 0.1..=3.0, " m");
                row(ui, "Velocidade dos roletes", &mut p.speed, 0.0..=3.0, " m/s");
                row(ui, "Fardo: comprimento", &mut p.box_size[0], 0.1..=1.2, " m");
                row(ui, "Fardo: largura", &mut p.box_size[2], 0.1..=1.0, " m");
                ui.label("Lógica interna");
                ui.checkbox(&mut p.auto_logic, "FB da mesa e da coleta sem CLP")
                    .on_hover_text("Executa o port dos FBs Selgron. Atuadores lidos do CLP têm prioridade.");
                ui.end_row();
            }
            EquipmentKind::Robot => {
                ui.label("Coletar de");
                name_combo(ui, "pick_from", &mut p.pick_from, &conveyor_names, "(mais próximo)");
                ui.end_row();
                ui.label("Paletizar em");
                name_combo(ui, "place_on", &mut p.place_on, &station_names, "(mais próximo)");
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
                ui.label("Camadas intertravadas");
                ui.checkbox(&mut p.interlock, "girar camadas ímpares 90°");
                ui.end_row();
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
                    for (i, stop) in p.stops.iter_mut().enumerate() {
                        ui.horizontal(|ui| {
                            ui.label(if i == 0 { "Coleta".to_owned() } else { format!("Posição {}", (b'A' + (i as u8 - 1)) as char) });
                            ui.add(egui::DragValue::new(stop).speed(0.01).range(0.0..=60.0).suffix(" m"));
                            if i > 0 && ui.small_button("🗑").clicked() {
                                remove = Some(i);
                            }
                        });
                    }
                    if let Some(i) = remove {
                        p.stops.remove(i);
                    }
                    if p.stops.len() < 8 && ui.small_button("+ posição").clicked() {
                        let next = p.stops.last().copied().unwrap_or(1.0) + 1.5;
                        p.stops.push(next);
                    }
                });
                ui.end_row();
                ui.label("Lógica interna");
                ui.checkbox(&mut p.auto_logic, "posicionamento do FB CVZ sem CLP")
                    .on_hover_text("Recebe pallet na coleta, leva às posições A, B... em sequência e volta. Entradas lidas do CLP têm prioridade.");
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
        || (kind.is_mesa() && before.box_size != eq.params.box_size);
    if geometry_changed {
        commands.entity(entity).insert(NeedsRebuild);
    }

    match kind {
        EquipmentKind::Robot => {
            if let Ok(robot) = robots.get(entity) {
                ui.separator();
                ui.label(format!("Estado: {}", robot.status));
                ui.label(format!("Ciclos executados: {}", robot.cycles));
                ui.weak(format!(
                    "TCP: θ {:.0}° · r {:.2} m · z {:.2} m",
                    robot.pose.theta.to_degrees(),
                    robot.pose.r,
                    robot.pose.y
                ));
            }
        }
        EquipmentKind::ShuttleCar => {
            if let Ok((car, logic)) = cars.get(entity) {
                ui.separator();
                ui.label(format!(
                    "Posição atual {:.0} mm · velocidade {:.0} mm/s{}",
                    car.x * 1000.0,
                    car.v * 1000.0,
                    if car.loaded { " · com pallet" } else { "" }
                ));
                if eq.params.auto_logic {
                    ui.label(format!(
                        "Lógica interna: {:?} → parada {} · INP {}",
                        logic.step,
                        logic.target,
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

fn name_combo(ui: &mut egui::Ui, id: &str, value: &mut String, options: &[String], empty_label: &str) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(if value.is_empty() { empty_label.to_owned() } else { value.clone() })
        .show_ui(ui, |ui| {
            ui.selectable_value(value, String::new(), empty_label);
            for name in options {
                ui.selectable_value(value, name.clone(), name);
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
        k if k.is_mesa() || k == EquipmentKind::ShuttleCar => eq.kind.signals().iter().any(|s| s.key == key && s.dir == SignalDir::In),
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
