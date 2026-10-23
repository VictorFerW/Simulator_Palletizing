//! Start screen: new project (empty or the default cell), open one of the saved
//! projects (with a top view preview) or another file, and delete projects.
//! Projects live in `Documentos\Simulador de Paletização\Projetos` (or
//! `PDT_PROJECTS`); layouts saved or opened elsewhere are kept in a recent list.

use std::path::{Path, PathBuf};
use std::time::SystemTime;

use bevy::prelude::*;
use bevy_egui::egui;

use crate::layout::{LayoutRequests, Preview};
use crate::model::EquipmentKind;

const RECENT_MAX: usize = 30;

/// Folder of the projects (created on first use).
pub fn projects_dir() -> PathBuf {
    let dir = std::env::var_os("PDT_PROJECTS").map(PathBuf::from).unwrap_or_else(|| {
        std::env::var_os("USERPROFILE")
            .or_else(|| std::env::var_os("HOME"))
            .map(|home| PathBuf::from(home).join("Documents").join("Simulador de Paletização").join("Projetos"))
            .unwrap_or_else(|| PathBuf::from("projetos"))
    });
    let _ = std::fs::create_dir_all(&dir);
    dir
}

/// Absolute path without the `\\?\` prefix Windows adds when canonicalizing.
fn canon(path: &Path) -> PathBuf {
    let full = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    let text = full.to_string_lossy();
    match text.strip_prefix(r"\\?\") {
        Some(rest) => PathBuf::from(rest),
        None => full,
    }
}

fn recent_file() -> PathBuf {
    projects_dir().join(".recentes.json")
}

fn recent_list() -> Vec<PathBuf> {
    std::fs::read_to_string(recent_file())
        .ok()
        .and_then(|text| serde_json::from_str::<Vec<PathBuf>>(&text).ok())
        .unwrap_or_default()
}

fn store_recent(list: &[PathBuf]) {
    if let Ok(json) = serde_json::to_string_pretty(list) {
        let _ = std::fs::write(recent_file(), json);
    }
}

/// A layout was opened or saved: first in the recent list.
pub fn remember(path: &Path) {
    let path = canon(path);
    let mut list = recent_list();
    list.retain(|p| *p != path);
    list.insert(0, path);
    list.truncate(RECENT_MAX);
    store_recent(&list);
}

fn forget(path: &Path) {
    let mut list = recent_list();
    list.retain(|p| p != path);
    store_recent(&list);
}

/// Project file name from a typed name (no folders, `.json`).
fn file_name(name: &str) -> String {
    let clean: String = name
        .trim()
        .chars()
        .map(|c| if "\\/:*?\"<>|".contains(c) { '_' } else { c })
        .collect();
    let clean = if clean.is_empty() { "Novo projeto".to_owned() } else { clean };
    if clean.to_lowercase().ends_with(".json") { clean } else { format!("{clean}.json") }
}

pub struct Entry {
    pub path: PathBuf,
    pub name: String,
    pub modified: Option<SystemTime>,
    /// In the projects folder (otherwise a recent file from elsewhere).
    pub in_folder: bool,
    pub preview: Result<Preview, String>,
}

#[derive(Resource, Default)]
pub struct StartScreen {
    pub open: bool,
    pub entries: Vec<Entry>,
    new_name: String,
    /// Waiting for the delete confirmation.
    confirm_delete: Option<usize>,
    message: String,
    /// Opened from the editor: a scene is open behind it.
    from_editor: bool,
}

impl StartScreen {
    /// Opens the screen with the list read again from disk.
    pub fn show(&mut self) {
        self.open = true;
        self.from_editor = false;
        self.confirm_delete = None;
        self.refresh();
    }

    /// Opened from the Arquivo menu (the open scene can be resumed).
    pub fn show_from_editor(&mut self) {
        self.show();
        self.from_editor = true;
    }

    fn refresh(&mut self) {
        let dir = projects_dir();
        let canonical_dir = canon(&dir);
        let mut paths: Vec<PathBuf> = std::fs::read_dir(&dir)
            .map(|rd| {
                rd.filter_map(|e| e.ok().map(|e| e.path()))
                    .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("json")))
                    .filter(|p| !p.file_name().is_some_and(|n| n.to_string_lossy().starts_with('.')))
                    .map(|p| canon(&p))
                    .collect()
            })
            .unwrap_or_default();
        for p in recent_list() {
            if p.exists() && !paths.contains(&p) {
                paths.push(p);
            }
        }
        self.entries = paths
            .into_iter()
            .map(|path| Entry {
                name: path.file_stem().map_or_else(String::new, |s| s.to_string_lossy().into_owned()),
                modified: std::fs::metadata(&path).and_then(|m| m.modified()).ok(),
                in_folder: path.parent().is_some_and(|p| p == canonical_dir),
                preview: crate::layout::preview(&path),
                path,
            })
            .collect();
        self.entries.sort_by_key(|e| std::cmp::Reverse(e.modified));
    }
}

/// "há 5 min", "há 2 dias"...
fn age(modified: Option<SystemTime>) -> String {
    let Some(secs) = modified.and_then(|m| SystemTime::now().duration_since(m).ok()).map(|d| d.as_secs()) else {
        return String::new();
    };
    match secs {
        0..60 => "agora".into(),
        60..3600 => format!("há {} min", secs / 60),
        3600..86_400 => format!("há {} h", secs / 3600),
        86_400..172_800 => "ontem".into(),
        _ => format!("há {} dias", secs / 86_400),
    }
}

fn kind_color(kind: EquipmentKind) -> egui::Color32 {
    use EquipmentKind as K;
    match kind {
        K::Robot => egui::Color32::from_rgb(240, 120, 30),
        K::Table2Stage | K::Table3Stage => egui::Color32::from_rgb(90, 150, 220),
        K::BeltConveyor => egui::Color32::from_rgb(110, 110, 120),
        K::PalletizingConveyor => egui::Color32::from_rgb(200, 160, 90),
        K::PalletConveyor => egui::Color32::from_rgb(150, 150, 160),
        K::ShuttleCar | K::FullCar => egui::Color32::from_rgb(230, 200, 60),
        K::CardboardPlacer | K::PalletElevator => egui::Color32::from_rgb(90, 180, 120),
        K::BoxEmitter | K::PalletDispenser => egui::Color32::from_rgb(60, 190, 90),
        K::BoxRemover => egui::Color32::from_rgb(210, 70, 60),
        _ => egui::Color32::from_rgb(170, 170, 170),
    }
}

/// Top view of a layout in `rect`, x to the right and z down, keeping proportions.
fn draw_preview(painter: &egui::Painter, rect: egui::Rect, preview: &Preview) {
    painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(38, 40, 44));
    let size = (preview.max - preview.min).max(Vec2::splat(0.5));
    let scale = ((rect.width() - 12.0) / size.x).min((rect.height() - 12.0) / size.y);
    let center = (preview.min + preview.max) / 2.0;
    let to_screen = |p: Vec2| rect.center() + egui::vec2((p.x - center.x) * scale, (p.y - center.y) * scale);
    // Robots last, on top.
    let mut items: Vec<&(EquipmentKind, [Vec2; 4])> = preview.items.iter().collect();
    items.sort_by_key(|(k, _)| *k == EquipmentKind::Robot);
    for (kind, corners) in items {
        let color = kind_color(*kind);
        if *kind == EquipmentKind::Robot {
            let c = (corners[0] + corners[2]) / 2.0;
            painter.circle_filled(to_screen(c), (0.7 * scale).max(3.0), color);
        } else {
            let points: Vec<egui::Pos2> = corners.iter().map(|&p| to_screen(p)).collect();
            painter.add(egui::Shape::convex_polygon(
                points,
                color.gamma_multiply(0.55),
                egui::Stroke::new(1.0_f32, color),
            ));
        }
    }
}

/// Draws the start screen; returns true while it is open (the editor is hidden).
pub fn start_screen(ctx: &egui::Context, start: &mut StartScreen, layout: &mut LayoutRequests) -> bool {
    if !start.open {
        return false;
    }
    let mut close = false;
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.add_space(12.0);
        ui.vertical_centered(|ui| {
            ui.heading("Simulador de paletização");
            ui.weak(format!("Projetos em {}", projects_dir().display()));
        });
        ui.add_space(12.0);
        ui.horizontal(|ui| {
            ui.label("Nome do projeto");
            ui.add(egui::TextEdit::singleline(&mut start.new_name).hint_text("Novo projeto").desired_width(220.0));
            let target = projects_dir().join(file_name(&start.new_name));
            let exists = target.exists();
            if ui.add_enabled(!exists, egui::Button::new("+ Novo projeto (vazio)")).clicked() {
                layout.clear = true;
                layout.save_after_rebuild = Some(target.clone());
                close = true;
            }
            if ui.add_enabled(!exists, egui::Button::new("+ Novo com a célula padrão")).clicked() {
                layout.new_default = true;
                layout.save_after_rebuild = Some(target.clone());
                close = true;
            }
            if exists {
                ui.colored_label(egui::Color32::from_rgb(230, 150, 40), "Já existe um projeto com esse nome");
            }
            ui.separator();
            if ui.button("Abrir outro arquivo...").clicked()
                && let Some(path) = rfd::FileDialog::new().add_filter("Layout de célula", &["json"]).pick_file()
            {
                layout.load = Some(path);
                close = true;
            }
            if start.from_editor && ui.button("Voltar ao projeto aberto").clicked() {
                close = true;
            }
        });
        if !start.message.is_empty() {
            ui.colored_label(egui::Color32::from_rgb(230, 150, 40), &start.message);
        }
        ui.separator();
        if start.entries.is_empty() {
            ui.weak("Nenhum projeto salvo ainda. Crie um novo acima.");
        }
        let mut open = None;
        let mut delete = None;
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                for (i, entry) in start.entries.iter().enumerate() {
                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.set_width(250.0);
                        ui.vertical(|ui| {
                            let (rect, response) = ui.allocate_exact_size(egui::vec2(250.0, 160.0), egui::Sense::click());
                            match &entry.preview {
                                Ok(p) => draw_preview(ui.painter(), rect, p),
                                Err(e) => {
                                    ui.painter().rect_filled(rect, 4.0, egui::Color32::from_rgb(60, 30, 30));
                                    ui.painter().text(
                                        rect.center(),
                                        egui::Align2::CENTER_CENTER,
                                        "Arquivo inválido",
                                        egui::FontId::proportional(14.0),
                                        egui::Color32::LIGHT_RED,
                                    );
                                    let _ = e;
                                }
                            }
                            let response = response.on_hover_text(entry.path.display().to_string());
                            if response.double_clicked() || response.clicked() {
                                open = Some(i);
                            }
                            ui.strong(&entry.name);
                            let count = entry.preview.as_ref().map_or(0, |p| p.items.len());
                            ui.weak(format!(
                                "{count} equipamentos · {}{}",
                                age(entry.modified),
                                if entry.in_folder { "" } else { " · fora da pasta" }
                            ));
                            ui.horizontal(|ui| {
                                if ui.button("Abrir").clicked() {
                                    open = Some(i);
                                }
                                if start.confirm_delete == Some(i) {
                                    let red = egui::Color32::from_rgb(220, 70, 60);
                                    if ui.button(egui::RichText::new("Excluir o arquivo").color(red)).clicked() {
                                        delete = Some((i, true));
                                    }
                                    if !entry.in_folder && ui.button("Só tirar da lista").clicked() {
                                        delete = Some((i, false));
                                    }
                                    if ui.button("Cancelar").clicked() {
                                        start.confirm_delete = None;
                                    }
                                } else if ui.button("Excluir").clicked() {
                                    start.confirm_delete = Some(i);
                                }
                            });
                        });
                    });
                }
            });
        });
        if let Some(i) = open {
            layout.load = Some(start.entries[i].path.clone());
            close = true;
        }
        if let Some((i, from_disk)) = delete {
            let path = start.entries[i].path.clone();
            start.message = if from_disk {
                match std::fs::remove_file(&path) {
                    Ok(()) => {
                        if layout.current.as_ref().is_some_and(|c| canon(c) == path) {
                            layout.current = None;
                        }
                        format!("\"{}\" excluído.", start.entries[i].name)
                    }
                    Err(e) => format!("Não foi possível excluir: {e}"),
                }
            } else {
                format!("\"{}\" tirado da lista (o arquivo continua no disco).", start.entries[i].name)
            };
            forget(&path);
            start.confirm_delete = None;
            start.refresh();
        }
    });
    if close {
        start.open = false;
        start.message.clear();
        start.new_name.clear();
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn project_file_names() {
        assert_eq!(file_name("Célula 1"), "Célula 1.json");
        assert_eq!(file_name("a/b:c"), "a_b_c.json");
        assert_eq!(file_name("  "), "Novo projeto.json");
        assert_eq!(file_name("x.JSON"), "x.JSON");
    }
}
