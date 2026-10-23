//! Left side of the editor: a floating toolbar with "Adicionar" (catalog of
//! equipment with categories, cards and 3D thumbnails; click to add at the center of
//! the view or drag a card into the 3D area) and "Editar" (the objects in the scene).

use bevy::prelude::*;
use bevy_egui::egui;

use crate::catalog::{Category, ITEMS, Item, THUMB_DIR};
use crate::editor::EditRequests;
use crate::model::{EquipmentKind, Selection};

/// Thumbnails of the catalog items (same order as `ITEMS`), when the files exist.
#[derive(Resource, Default)]
pub struct Thumbs(pub Vec<Option<Handle<Image>>>);

pub fn load_thumbs(mut commands: Commands, assets: Res<AssetServer>) {
    let root = crate::visuals::asset_root().join(THUMB_DIR);
    let handles = ITEMS
        .iter()
        .map(|item| {
            let file = format!("{}.png", item.id);
            root.join(&file).exists().then(|| assets.load(format!("{THUMB_DIR}/{file}")))
        })
        .collect();
    commands.insert_resource(Thumbs(handles));
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LeftPanel {
    None,
    Add,
    #[default]
    Edit,
}

pub struct CatalogUi {
    pub panel: LeftPanel,
    category: Category,
    /// Card being dragged toward the 3D area.
    dragging: Option<usize>,
    /// Card dropped this frame and where (screen).
    dropped: Option<(usize, egui::Pos2)>,
    filter: String,
    scene_filter: String,
}

impl Default for CatalogUi {
    fn default() -> Self {
        Self {
            // PDT_PANEL=add opens the catalog (automated screenshots).
            panel: if std::env::var("PDT_PANEL").is_ok_and(|p| p == "add") { LeftPanel::Add } else { LeftPanel::Edit },
            category: Category::All,
            dragging: None,
            dropped: None,
            filter: String::new(),
            scene_filter: String::new(),
        }
    }
}

/// One object of the scene for the "Editar" list.
pub struct SceneRow {
    pub entity: Entity,
    pub name: String,
    pub kind: EquipmentKind,
    pub item: Option<usize>,
}

const PANEL_FILL: egui::Color32 = egui::Color32::from_rgba_premultiplied(30, 32, 36, 235);
const CARD_FILL: egui::Color32 = egui::Color32::from_rgb(58, 61, 68);
const CARD_HOVER: egui::Color32 = egui::Color32::from_rgb(74, 78, 88);
const ACCENT: egui::Color32 = egui::Color32::from_rgb(255, 140, 40);
const SELECT: egui::Color32 = egui::Color32::from_rgb(40, 150, 255);

/// Catalog item of a scene object (thumbnail and label).
pub fn item_of(kind: EquipmentKind, model: crate::model::RobotModel) -> Option<usize> {
    ITEMS.iter().position(|i| i.kind == kind && i.model.is_none_or(|m| m == model))
}

/// Square toolbar button with a painted icon.
fn tool_button(ui: &mut egui::Ui, active: bool, tip: &str, icon: impl Fn(&egui::Painter, egui::Rect, egui::Color32)) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(46.0, 46.0), egui::Sense::click());
    let painter = ui.painter();
    let fill = if response.hovered() { CARD_HOVER } else { egui::Color32::from_rgba_premultiplied(40, 42, 47, 230) };
    painter.rect_filled(rect, 8.0, fill);
    if active {
        painter.rect_stroke(rect, 8.0, egui::Stroke::new(2.0_f32, SELECT), egui::StrokeKind::Inside);
    }
    let color = if active { ACCENT } else { egui::Color32::from_gray(220) };
    icon(painter, rect.shrink(11.0), color);
    response.on_hover_text(tip)
}

/// "Adicionar": two stacked sheets with a plus.
fn icon_add(p: &egui::Painter, r: egui::Rect, c: egui::Color32) {
    let s = egui::Stroke::new(2.0_f32, c);
    let back = egui::Rect::from_min_size(r.min, r.size() * 0.78);
    let front = egui::Rect::from_min_size(r.min + r.size() * 0.22, r.size() * 0.78);
    p.rect_stroke(back, 3.0, s, egui::StrokeKind::Middle);
    p.rect_filled(front, 3.0, egui::Color32::from_rgba_premultiplied(40, 42, 47, 255));
    p.rect_stroke(front, 3.0, s, egui::StrokeKind::Middle);
    let m = front.center();
    let d = front.width() * 0.25;
    p.line_segment([m - egui::vec2(d, 0.0), m + egui::vec2(d, 0.0)], s);
    p.line_segment([m - egui::vec2(0.0, d), m + egui::vec2(0.0, d)], s);
}

/// "Editar": a sheet with a pencil.
fn icon_edit(p: &egui::Painter, r: egui::Rect, c: egui::Color32) {
    let s = egui::Stroke::new(2.0_f32, c);
    let sheet = egui::Rect::from_min_max(r.min + egui::vec2(0.0, r.height() * 0.12), r.max - egui::vec2(r.width() * 0.12, 0.0));
    p.rect_stroke(sheet, 3.0, s, egui::StrokeKind::Middle);
    let a = egui::pos2(r.left() + r.width() * 0.35, r.bottom() - r.height() * 0.35);
    let b = egui::pos2(r.right(), r.top());
    p.line_segment([a, b], egui::Stroke::new(3.0_f32, c));
    p.circle_filled(a, 1.8, c);
}

/// "Tudo": three columns.
fn icon_all(p: &egui::Painter, r: egui::Rect, c: egui::Color32) {
    let w = r.width() / 3.0;
    for i in 0..3 {
        let col = egui::Rect::from_min_size(egui::pos2(r.left() + i as f32 * w + 2.0, r.top()), egui::vec2(w - 4.0, r.height()));
        p.rect_stroke(col, 2.0, egui::Stroke::new(2.0_f32, c), egui::StrokeKind::Middle);
    }
}

fn thumb(ui: &mut egui::Ui, tex: Option<egui::TextureId>, size: f32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(size, size), egui::Sense::hover());
    match tex {
        Some(id) => {
            ui.painter().image(id, rect, egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)), egui::Color32::WHITE);
        }
        None => {
            ui.painter().rect_filled(rect, 6.0, egui::Color32::from_gray(45));
        }
    }
}

/// Category button: thumbnail (or the "Tudo" icon) with its name below.
fn category_button(ui: &mut egui::Ui, active: bool, label: &str, tex: Option<egui::TextureId>) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(54.0, 64.0), egui::Sense::click());
    let painter = ui.painter();
    let icon = egui::Rect::from_min_size(rect.min + egui::vec2(5.0, 0.0), egui::vec2(44.0, 44.0));
    painter.rect_filled(icon, 8.0, if response.hovered() { CARD_HOVER } else { CARD_FILL });
    match tex {
        Some(id) => {
            painter.image(id, icon.shrink(3.0), egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)), egui::Color32::WHITE);
        }
        None => icon_all(painter, icon.shrink(12.0), egui::Color32::from_gray(220)),
    }
    if active {
        painter.rect_stroke(icon, 8.0, egui::Stroke::new(2.0_f32, ACCENT), egui::StrokeKind::Inside);
    }
    painter.text(
        egui::pos2(rect.center().x, rect.bottom() - 9.0),
        egui::Align2::CENTER_CENTER,
        label,
        egui::FontId::proportional(12.5),
        if active { egui::Color32::WHITE } else { egui::Color32::from_gray(190) },
    );
    response
}

/// Thumbnail standing for a category.
fn category_thumb(category: Category, tex: &[Option<egui::TextureId>]) -> Option<egui::TextureId> {
    let id = match category {
        Category::All => return None,
        Category::Robots => "robot_kuka_kr180",
        Category::Conveyors => "belt",
        Category::Tables => "table2",
        Category::Pallets => "pallet_dispenser",
        Category::Others => "sensor",
    };
    ITEMS.iter().position(|i| i.id == id).and_then(|i| tex.get(i).copied().flatten())
}

/// Card of a catalog item; returns its response (click to add, drag to place).
fn card(ui: &mut egui::Ui, item: &Item, tex: Option<egui::TextureId>, width: f32) -> egui::Response {
    let height = 104.0;
    let (rect, response) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::click_and_drag());
    let painter = ui.painter();
    painter.rect_filled(rect, 8.0, if response.hovered() || response.dragged() { CARD_HOVER } else { CARD_FILL });
    let pic = egui::Rect::from_min_size(egui::pos2(rect.right() - height + 6.0, rect.top() + 6.0), egui::vec2(height - 12.0, height - 12.0));
    match tex {
        Some(id) => {
            painter.image(id, pic, egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)), egui::Color32::WHITE);
        }
        None => {
            painter.rect_filled(pic, 6.0, egui::Color32::from_gray(45));
        }
    }
    let text_width = width - height - 6.0;
    let name = painter.layout(item.name.to_owned(), egui::FontId::proportional(15.0), egui::Color32::WHITE, text_width);
    let name_height = name.size().y;
    painter.galley(rect.min + egui::vec2(10.0, 9.0), name, egui::Color32::WHITE);
    let caption = painter.layout_no_wrap("Características:".to_owned(), egui::FontId::proportional(11.5), egui::Color32::from_gray(170));
    painter.galley(rect.min + egui::vec2(10.0, 15.0 + name_height), caption, egui::Color32::WHITE);
    let specs = painter.layout(item.specs.to_owned(), egui::FontId::proportional(11.5), egui::Color32::from_gray(205), text_width);
    painter.galley(rect.min + egui::vec2(10.0, 31.0 + name_height), specs, egui::Color32::WHITE);
    response.on_hover_text("Clique para adicionar no centro da vista ou arraste para a área 3D")
}

/// Draws the toolbar and the open panel over the 3D view (call it after the other
/// panels, so the free area is the 3D view).
pub fn left_side(
    ctx: &egui::Context,
    ui_state: &mut CatalogUi,
    tex: &[Option<egui::TextureId>],
    rows: &[SceneRow],
    selection: &mut Selection,
    requests: &mut EditRequests,
) {
    let top = ctx.available_rect().top() + 10.0;
    let left = ctx.available_rect().left() + 10.0;
    egui::Area::new(egui::Id::new("left_toolbar")).fixed_pos(egui::pos2(left, top)).show(ctx, |ui| {
        ui.spacing_mut().item_spacing = egui::vec2(0.0, 8.0);
        let add = ui_state.panel == LeftPanel::Add;
        if tool_button(ui, add, "Adicionar equipamentos", icon_add).clicked() {
            ui_state.panel = if add { LeftPanel::None } else { LeftPanel::Add };
        }
        let edit = ui_state.panel == LeftPanel::Edit;
        if tool_button(ui, edit, "Objetos da cena", icon_edit).clicked() {
            ui_state.panel = if edit { LeftPanel::None } else { LeftPanel::Edit };
        }
    });
    if ui_state.panel == LeftPanel::None {
        return;
    }
    let view = ctx.available_rect();
    let panel_top = top + 2.0 * 46.0 + 18.0;
    let height = (view.bottom() - panel_top - 10.0).max(200.0);
    let panel = egui::Area::new(egui::Id::new("left_panel")).fixed_pos(egui::pos2(left, panel_top)).show(ctx, |ui| {
        egui::Frame::new().fill(PANEL_FILL).corner_radius(12.0).inner_margin(10.0).show(ui, |ui| {
            ui.set_width(350.0);
            ui.set_height(height - 20.0);
            match ui_state.panel {
                LeftPanel::Add => add_panel(ui, ui_state, tex, requests),
                LeftPanel::Edit => edit_panel(ui, ui_state, tex, rows, selection, requests),
                LeftPanel::None => {}
            }
        });
    });
    // Dropped on the 3D view (not on a panel): place it there.
    if let Some((i, pos)) = ui_state.dropped.take()
        && view.contains(pos)
        && !panel.response.rect.contains(pos)
    {
        requests.place.push((i, Some(Vec2::new(pos.x, pos.y))));
    }
    // A card dragged toward the 3D area: its picture follows the pointer.
    if let Some(i) = ui_state.dragging
        && let Some(pos) = ctx.pointer_hover_pos()
    {
        egui::Area::new(egui::Id::new("catalog_drag"))
            .order(egui::Order::Tooltip)
            .fixed_pos(pos + egui::vec2(-40.0, -40.0))
            .interactable(false)
            .show(ctx, |ui| {
                egui::Frame::new().fill(egui::Color32::from_rgba_premultiplied(30, 32, 36, 200)).corner_radius(8.0).inner_margin(4.0).show(ui, |ui| {
                    thumb(ui, tex.get(i).copied().flatten(), 80.0);
                    ui.label(egui::RichText::new(ITEMS[i].name).small().color(egui::Color32::WHITE));
                });
            });
    }
}

fn add_panel(ui: &mut egui::Ui, state: &mut CatalogUi, tex: &[Option<egui::TextureId>], requests: &mut EditRequests) {
    ui.horizontal_wrapped(|ui| {
        ui.spacing_mut().item_spacing = egui::vec2(4.0, 6.0);
        for category in Category::ALL {
            if category_button(ui, state.category == category, category.label(), category_thumb(category, tex)).clicked() {
                state.category = category;
            }
        }
    });
    ui.add_space(4.0);
    ui.add(egui::TextEdit::singleline(&mut state.filter).hint_text("Buscar equipamento").desired_width(f32::INFINITY));
    ui.add_space(6.0);
    let filter = state.filter.to_lowercase();
    egui::ScrollArea::vertical().id_salt("catalog_cards").show(ui, |ui| {
        ui.spacing_mut().item_spacing = egui::vec2(0.0, 8.0);
        for (i, item) in ITEMS.iter().enumerate() {
            if state.category != Category::All && item.category != state.category {
                continue;
            }
            if !filter.is_empty() && !item.name.to_lowercase().contains(&filter) {
                continue;
            }
            let response = card(ui, item, tex.get(i).copied().flatten(), ui.available_width() - 4.0);
            if response.clicked() {
                requests.place.push((i, None));
            }
            if response.drag_started() {
                state.dragging = Some(i);
            }
            if response.drag_stopped() {
                state.dropped = ui.ctx().pointer_interact_pos().map(|pos| (i, pos));
                state.dragging = None;
            }
        }
    });
}

fn edit_panel(
    ui: &mut egui::Ui,
    state: &mut CatalogUi,
    tex: &[Option<egui::TextureId>],
    rows: &[SceneRow],
    selection: &mut Selection,
    requests: &mut EditRequests,
) {
    ui.horizontal(|ui| {
        ui.heading(egui::RichText::new("Cena").color(egui::Color32::WHITE));
        ui.weak(format!("{} objetos", rows.len()));
    });
    ui.add(egui::TextEdit::singleline(&mut state.scene_filter).hint_text("Buscar na cena").desired_width(f32::INFINITY));
    ui.add_space(6.0);
    let filter = state.scene_filter.to_lowercase();
    egui::ScrollArea::vertical().id_salt("scene_rows").max_height(ui.available_height() - 40.0).show(ui, |ui| {
        ui.spacing_mut().item_spacing = egui::vec2(0.0, 4.0);
        for row in rows.iter().filter(|r| filter.is_empty() || r.name.to_lowercase().contains(&filter)) {
            let marked = selection.1.contains(&row.entity);
            let (rect, response) = ui.allocate_exact_size(egui::vec2(ui.available_width() - 4.0, 40.0), egui::Sense::click());
            let painter = ui.painter();
            let fill = if marked {
                egui::Color32::from_rgb(34, 70, 112)
            } else if response.hovered() {
                CARD_HOVER
            } else {
                CARD_FILL
            };
            painter.rect_filled(rect, 6.0, fill);
            if marked {
                painter.rect_stroke(rect, 6.0, egui::Stroke::new(1.5_f32, SELECT), egui::StrokeKind::Inside);
            }
            let pic = egui::Rect::from_min_size(rect.min + egui::vec2(4.0, 4.0), egui::vec2(32.0, 32.0));
            if let Some(id) = row.item.and_then(|i| tex.get(i).copied().flatten()) {
                painter.image(id, pic, egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)), egui::Color32::WHITE);
            }
            painter.text(rect.min + egui::vec2(44.0, 12.0), egui::Align2::LEFT_CENTER, &row.name, egui::FontId::proportional(14.0), egui::Color32::WHITE);
            let kind = row.item.map_or(row.kind.label(), |i| ITEMS[i].name);
            painter.text(rect.min + egui::vec2(44.0, 29.0), egui::Align2::LEFT_CENTER, kind, egui::FontId::proportional(11.0), egui::Color32::from_gray(170));
            // Delete button at the right.
            let bin = egui::Rect::from_center_size(egui::pos2(rect.right() - 18.0, rect.center().y), egui::vec2(24.0, 24.0));
            let bin_hover = ui.ctx().pointer_hover_pos().is_some_and(|p| bin.contains(p));
            if response.hovered() || marked {
                let c = if bin_hover { egui::Color32::from_rgb(240, 90, 80) } else { egui::Color32::from_gray(170) };
                let s = egui::Stroke::new(1.6_f32, c);
                let body = egui::Rect::from_min_max(bin.min + egui::vec2(7.0, 8.0), bin.max - egui::vec2(7.0, 4.0));
                painter.rect_stroke(body, 2.0, s, egui::StrokeKind::Middle);
                painter.line_segment([egui::pos2(bin.left() + 5.0, bin.top() + 7.0), egui::pos2(bin.right() - 5.0, bin.top() + 7.0)], s);
                painter.line_segment([egui::pos2(bin.center().x - 2.0, bin.top() + 4.5), egui::pos2(bin.center().x + 2.0, bin.top() + 4.5)], s);
            }
            if response.clicked() {
                if bin_hover {
                    requests.delete.push(row.entity);
                } else if ui.input(|i| i.modifiers.command) {
                    if marked {
                        selection.1.retain(|e| *e != row.entity);
                        selection.0 = selection.1.first().copied();
                    } else {
                        selection.1.push(row.entity);
                        selection.0 = Some(row.entity);
                    }
                } else {
                    selection.1 = vec![row.entity];
                    selection.0 = Some(row.entity);
                }
            }
            if response.double_clicked() {
                requests.focus = Some(row.entity);
            }
            response.on_hover_text("Clique: selecionar · Ctrl+clique: somar · duplo clique: focar");
        }
    });
    ui.collapsing("Controles", |ui| {
        ui.label(
            "Clique: selecionar · Ctrl+clique: somar à seleção\n\
             Arrastar no vazio: seleção por retângulo\n\
             Arrastar item: mover a seleção (Alt inverte a grade)\n\
             R / Shift+R: girar ±90° · Q/E: girar ±15°\n\
             Setas: mover 0,1 m (Shift 0,5 m) · PgUp/PgDn: altura · V + mouse: altura\n\
             Del: excluir · Ctrl+D: duplicar · Ctrl+Z / Ctrl+Y: desfazer / refazer\n\
             F: focar · Esc: desmarcar\n\
             Botão direito: orbitar · Meio ou Shift+direito: deslocar · Roda: zoom",
        );
    });
}
