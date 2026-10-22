//! Colocador de Papelão (CP), Selgron "MOD COLOCADOR PAPELAO C2370": a carriage
//! on an overhead beam moves between the pick position (over the cardboard
//! deposit) and the delivery position (over a pallet on a TDP or on the CVZ). A
//! vertical cylinder lowers a floating vacuum head:
//! - `SensorPapelaoInferior` is on while the head hangs free with no sheet;
//! - `SensorPapelaoSuperior` turns on when the head is pressed against the stack
//!   or the pallet.
//!
//! The internal logic ports the automatic steps of the CP FB: pick a sheet
//! (descend until the upper sensor, vacuum, stabilise, rise, check the lower
//! sensor), move to delivery, descend until the upper sensor, release, rise,
//! return; optionally twice (`Aplicar2xPapelao`). A pallet without cardboard is
//! held under the head (like the TDP stopping on its sensor) until `FimDoCiclo`.

use bevy::prelude::*;

use crate::model::{Equipment, EquipmentKind, Io, SimControl};
use crate::sim::{Load, LoadKind, LoadState, PalletLoad};
use crate::visuals::Palette;

/// Cardboard sheet thickness.
pub const SHEET_THICKNESS: f32 = 0.005;
/// Head bottom height with the cylinder retracted.
pub const HEAD_RETRACTED_Y: f32 = 1.30;
/// Floating travel of the head on its springs.
const HEAD_FLOAT: f32 = 0.03;
/// Deposit floor (bottom of the cardboard stack).
pub const DEPOSIT_FLOOR: f32 = 0.20;
const CYLINDER_SPEED: f32 = 0.45; // m/s
const CARRIAGE_SPEED: f32 = 0.5; // m/s
const TEMPO_ESTABILIZA_ENTREGA: f32 = 1.5;
const TEMPO_ESTABILIZAR_VACUO: f32 = 0.5;

/// Moving parts and contents of the CP.
#[derive(Component, Debug)]
pub struct PaperPlacerState {
    /// Carriage position along the beam, 0 = pick (deposit) .. 1 = delivery.
    pub carriage: f32,
    /// Cylinder extension (m).
    pub extension: f32,
    /// Compression of the floating head (m).
    pub compression: f32,
    /// Sheets left in the deposit.
    pub sheets_left: u32,
    /// Sheet held by the vacuum head.
    pub held: Option<Entity>,
    /// Pallet currently under the delivery position.
    pub pallet: Option<Entity>,
}

impl Default for PaperPlacerState {
    fn default() -> Self {
        Self {
            carriage: 0.0,
            extension: 0.0,
            compression: 0.0,
            sheets_left: 100,
            held: None,
            pallet: None,
        }
    }
}

/// Port of the CP FB (automatic mode).
#[derive(Component, Debug, Default)]
pub struct PaperPlacerLogic {
    pub passo: u32,
    segundo_pap: bool,
    pub fim_do_ciclo: bool,
    trig_estabiliza: bool,
    estabiliza: f32,
    trig_vacuo: bool,
    vacuo_t: f32,
    motor_coleta: bool,
    motor_entrega: bool,
    cil_avancar: bool,
    cil_recuar: bool,
    vacuo: bool,
    pub sem_papelao: bool,
    last_passo: u32,
}

/// Marker of the moving carriage (with column and head) of a CP.
#[derive(Component)]
pub struct PaperCarriage;

/// Marker of the vacuum head, moved by the cylinder.
#[derive(Component)]
pub struct PaperHead;

/// Marker of the cardboard stack in the deposit (scaled with the sheets left).
#[derive(Component)]
pub struct PaperStack;

/// A cardboard sheet (held by the head, or on a pallet).
#[derive(Component)]
pub struct Sheet;

/// Local X of the pick and delivery positions along the beam.
pub fn pick_delivery_x(eq: &Equipment) -> (f32, f32) {
    (-eq.params.length / 2.0, eq.params.length / 2.0)
}

/// Delivery point (head center) on the floor, world space.
pub fn delivery_point(eq: &Equipment, tf: &Transform) -> Vec3 {
    tf.transform_point(Vec3::new(pick_delivery_x(eq).1, 0.0, 0.0))
}

fn head_local_x(eq: &Equipment, state: &PaperPlacerState) -> f32 {
    let (pick, delivery) = pick_delivery_x(eq);
    pick + (delivery - pick) * state.carriage
}

/// Mechanics: carriage, cylinder, floating head, vacuum pick and release.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub fn paper_mechanics(
    time: Res<Time>,
    control: Res<SimControl>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    palette: Res<Palette>,
    mut placers: Query<(&Equipment, &Transform, &mut Io, &mut PaperPlacerState), Without<Load>>,
    mut pallets: Query<(Entity, &mut Load, &Transform, &mut PalletLoad), Without<Equipment>>,
    mut sheets: Query<&mut Transform, (With<Sheet>, Without<Load>, Without<Equipment>)>,
) {
    let dt = if control.running { time.delta_secs() * control.time_scale } else { 0.0 };
    for (eq, tf, mut io, mut state) in &mut placers {
        if eq.kind != EquipmentKind::CardboardPlacer {
            continue;
        }
        let p = &eq.params;
        let length = p.length.max(0.1);
        // Carriage (motor with chain): never moves with the cylinder down in the FB, but
        // the physics just follows the motor outputs.
        let toward_pick = io.get_bool("MotorPosicaoColeta") && !io.get_bool("MotorPosicaoEntrega");
        let toward_delivery = io.get_bool("MotorPosicaoEntrega") && !io.get_bool("MotorPosicaoColeta");
        let step = CARRIAGE_SPEED * dt / length;
        if toward_pick {
            state.carriage = (state.carriage - step).max(0.0);
        }
        if toward_delivery {
            state.carriage = (state.carriage + step).min(1.0);
        }

        // Pallet under the delivery position (SensorTDP).
        let delivery = delivery_point(eq, tf);
        let under = pallets
            .iter()
            .filter(|(_, l, ..)| l.kind == LoadKind::Pallet && l.state == LoadState::Free)
            .find(|(_, l, ptf, _)| {
                let local = ptf.compute_affine().inverse().transform_point3(delivery);
                local.x.abs() <= l.size.x / 2.0 && local.z.abs() <= l.size.z / 2.0
            })
            .map(|(e, l, ptf, pl)| (e, ptf.translation.y + l.size.y / 2.0 + pl.sheets as f32 * SHEET_THICKNESS));
        state.pallet = under.map(|(e, _)| e);

        // Surface under the head: the stack at the pick position, the pallet at delivery.
        let at_pick = state.carriage <= 0.01;
        let at_delivery = state.carriage >= 0.99;
        let surface = if at_pick {
            Some(DEPOSIT_FLOOR + state.sheets_left as f32 * SHEET_THICKNESS)
        } else if at_delivery {
            under.map(|(_, top)| top)
        } else {
            None
        };

        // Cylinder: stalls when the floating head is fully compressed.
        let held_t = if state.held.is_some() { SHEET_THICKNESS } else { 0.0 };
        let stroke = p.cp_stroke;
        let target = match (io.get_bool("CilindroAvancar"), io.get_bool("CilindroRecuar")) {
            (true, false) => Some(stroke),
            (false, true) => Some(0.0),
            _ => None,
        };
        if let Some(t) = target {
            let s = CYLINDER_SPEED * dt;
            state.extension += (t - state.extension).clamp(-s, s);
        }
        let bottom = |ext: f32| HEAD_RETRACTED_Y - ext - held_t;
        if let Some(top) = surface
            && bottom(state.extension) < top - HEAD_FLOAT
        {
            state.extension = HEAD_RETRACTED_Y - held_t - (top - HEAD_FLOAT);
        }
        state.compression = surface.map_or(0.0, |top| (top - bottom(state.extension)).clamp(0.0, HEAD_FLOAT));

        // Vacuum: take the top sheet of the stack; release onto the pallet.
        let vacuum = io.get_bool("Vacuo");
        if vacuum && state.held.is_none() && at_pick && state.compression > 0.003 && state.sheets_left > 0 {
            state.sheets_left -= 1;
            let size = Vec3::new(p.pallet_size[0] - 0.02, SHEET_THICKNESS, p.pallet_size[2] - 0.02);
            let sheet = commands
                .spawn((
                    Mesh3d(meshes.add(Cuboid::new(size.x, size.y, size.z))),
                    MeshMaterial3d(palette.cardboard.clone()),
                    Transform::default(),
                    Visibility::default(),
                    Sheet,
                ))
                .id();
            state.held = Some(sheet);
        }
        if !vacuum && let Some(sheet) = state.held.take() {
            match under.filter(|_| at_delivery && state.compression > 0.0) {
                Some((pallet_entity, _)) => {
                    let Ok((_, load, ptf, mut content)) = pallets.get_mut(pallet_entity) else { continue };
                    // Lay the sheet on the pallet (frame of the pallet, origin at its center).
                    let y = load.size.y / 2.0 + content.sheets as f32 * SHEET_THICKNESS + SHEET_THICKNESS / 2.0;
                    let _ = ptf;
                    content.sheets += 1;
                    if std::env::var("PDT_TRACE").is_ok() {
                        eprintln!("[{:.2}] {}: papelão aplicado no pallet {pallet_entity} ({} folha(s))", time.elapsed_secs(), eq.name, content.sheets);
                    }
                    // The sheet is cut to the pallet size: lay it square on the pallet.
                    commands.entity(sheet).insert((ChildOf(pallet_entity), Transform::from_xyz(0.0, y, 0.0)));
                }
                None => {
                    commands.entity(sheet).despawn();
                }
            }
        }
        // The held sheet hangs under the head.
        if let Some(sheet) = state.held
            && let Ok(mut stf) = sheets.get_mut(sheet)
        {
            let head = Vec3::new(head_local_x(eq, &state), bottom(state.extension) + held_t - SHEET_THICKNESS / 2.0, 0.0);
            *stf = Transform::from_translation(tf.transform_point(head)).with_rotation(tf.rotation);
        }

        io.set_bool("LimitePega", state.carriage <= 0.002);
        io.set_bool("LimiteEntrega", state.carriage >= 0.998);
        io.set_bool("SensorCilindroAv", state.extension >= stroke - 0.005);
        io.set_bool("SensorCilindroRc", state.extension <= 0.005);
        io.set_bool("SensorPapelaoInferior", state.held.is_none() && state.compression < 0.003);
        io.set_bool("SensorPapelaoSuperior", state.compression >= HEAD_FLOAT - 0.005);
        io.set_bool("SensorTDP", state.pallet.is_some());
    }
}

/// Holds a pallet without cardboard under the delivery position while the CP works
/// (the TDP stopping on its sensor), and marks it done at the end of the cycle.
#[allow(clippy::type_complexity)]
pub fn paper_hold(
    placers: Query<(&Equipment, &Transform, &PaperPlacerLogic, &PaperPlacerState)>,
    mut pallets: Query<(Entity, &mut Load, &Transform, &mut PalletLoad), Without<Equipment>>,
) {
    for (entity, mut load, ptf, mut content) in &mut pallets {
        if load.kind != LoadKind::Pallet || load.state != LoadState::Free {
            continue;
        }
        let mut hold = false;
        for (eq, tf, logic, state) in &placers {
            if eq.kind != EquipmentKind::CardboardPlacer || !eq.params.auto_logic {
                continue;
            }
            let point = delivery_point(eq, tf);
            let d = Vec2::new(ptf.translation.x - point.x, ptf.translation.z - point.z).length();
            if d < 0.04 && !content.papered {
                hold = true;
            }
            // FimDoCiclo: the pallet is done (cardboard applied, or none requested).
            if logic.fim_do_ciclo && state.pallet == Some(entity) {
                content.papered = true;
            }
        }
        load.hold = hold;
    }
}

fn drive(io: &mut Io, key: &'static str, value: bool) {
    if !io.from_plc.contains(key) {
        io.set_bool(key, value);
    }
}

pub fn paper_logic(
    time: Res<Time>,
    control: Res<SimControl>,
    mut placers: Query<(&Equipment, &mut Io, &mut PaperPlacerLogic, &PaperPlacerState)>,
    pallets: Query<&PalletLoad>,
) {
    if !control.running {
        return;
    }
    let dt = time.delta_secs() * control.time_scale;
    for (eq, mut io, mut l, state) in &mut placers {
        if eq.kind != EquipmentKind::CardboardPlacer || !eq.params.auto_logic {
            continue;
        }
        let s = |k: &str| io.get_bool(k);
        let (limite_pega, limite_entrega) = (s("LimitePega"), s("LimiteEntrega"));
        let (cil_av, cil_rc) = (s("SensorCilindroAv"), s("SensorCilindroRc"));
        let (pap_inf, pap_sup) = (s("SensorPapelaoInferior"), s("SensorPapelaoSuperior"));
        let sensor_tdp = s("SensorTDP");
        let pallet = state.pallet.and_then(|e| pallets.get(e).ok());

        l.estabiliza = if l.trig_estabiliza { l.estabiliza + dt } else { 0.0 };
        let q_estabiliza = l.trig_estabiliza && l.estabiliza >= TEMPO_ESTABILIZA_ENTREGA;
        l.vacuo_t = if l.trig_vacuo { l.vacuo_t + dt } else { 0.0 };
        let q_vacuo = l.trig_vacuo && l.vacuo_t >= TEMPO_ESTABILIZAR_VACUO;

        match l.passo {
            0 => {
                // Reset
                l.motor_entrega = false;
                l.motor_coleta = false;
                l.vacuo = false;
                l.cil_recuar = false;
                l.cil_avancar = false;
                l.fim_do_ciclo = false;
                l.trig_vacuo = false;
                l.sem_papelao = false;
                l.segundo_pap = false;
                l.passo = 1;
            }
            1 => {
                // Inicialização: cilindro recuado
                if !cil_rc {
                    l.cil_recuar = true;
                } else {
                    l.cil_recuar = false;
                    l.passo = 2;
                }
            }
            2 => {
                // Inicialização: posição de coleta
                if !limite_pega {
                    l.motor_coleta = true;
                } else {
                    l.motor_coleta = false;
                    l.passo = 100;
                }
            }
            100 => {
                // Aguarda pallet "em branco" parado sob a entrega.
                l.segundo_pap = false;
                if sensor_tdp && let Some(content) = pallet {
                    l.passo = if content.papered {
                        200
                    } else if eq.params.apply_paper {
                        101
                    } else {
                        200
                    };
                }
            }
            101 => {
                if cil_rc {
                    l.passo = 103;
                } else {
                    l.cil_recuar = true;
                    l.cil_avancar = false;
                }
            }
            103 => {
                if limite_pega {
                    l.passo = 105;
                    l.motor_coleta = false;
                } else {
                    l.motor_entrega = false;
                    l.motor_coleta = true;
                }
            }
            105 => {
                // Avança (desce) cilindro
                l.trig_estabiliza = true;
                if q_estabiliza {
                    l.trig_estabiliza = false;
                    l.cil_avancar = true;
                    l.cil_recuar = false;
                    l.passo = 106;
                }
            }
            106 => {
                // Ambos sensores de papelão desativados: liga vácuo
                if !pap_inf && !pap_sup {
                    l.vacuo = true;
                    l.passo = 107;
                }
                if cil_av {
                    l.passo = 403;
                }
            }
            107 => {
                if pap_sup {
                    l.cil_avancar = false;
                    l.passo = 108;
                    l.trig_vacuo = true;
                }
            }
            108 => {
                if q_vacuo {
                    l.trig_vacuo = false;
                    l.passo = 109;
                }
            }
            109 => {
                l.cil_recuar = true;
                l.passo = 110;
            }
            110 => {
                if cil_rc && pap_inf {
                    l.passo = 400; // não tem mais papelão
                }
                if cil_rc && !pap_inf {
                    l.passo = 111;
                }
            }
            111 => {
                l.cil_recuar = false;
                l.motor_entrega = true;
                l.passo = 112;
            }
            112 => {
                if cil_rc && pap_inf {
                    l.motor_entrega = false;
                    l.passo = 400; // perdeu o papelão no trajeto
                }
                if limite_entrega {
                    l.motor_entrega = false;
                    l.trig_estabiliza = true;
                    l.passo = 113;
                }
            }
            113 => {
                l.trig_estabiliza = true;
                if q_estabiliza {
                    l.trig_estabiliza = false;
                    l.cil_avancar = true;
                    l.cil_recuar = false;
                    l.passo = 114;
                }
            }
            114 => {
                if pap_sup {
                    l.cil_avancar = false;
                    l.vacuo = false;
                    l.cil_recuar = true;
                    l.passo = 115;
                }
            }
            115 => {
                if cil_rc {
                    l.cil_recuar = false;
                    l.passo = 116;
                }
            }
            116 => {
                if eq.params.cp_two_sheets && !l.segundo_pap {
                    l.segundo_pap = true;
                    l.passo = 101;
                } else {
                    l.fim_do_ciclo = true;
                    l.motor_coleta = true;
                    l.passo = 117;
                }
            }
            117 => {
                if limite_pega {
                    l.motor_coleta = false;
                }
                // FimDoCiclo is reset once the pallet is released (the TDP restarts in the FB).
                if pallet.is_none_or(|p| p.papered) {
                    l.fim_do_ciclo = false;
                    l.segundo_pap = false;
                }
                if !l.motor_coleta && !l.fim_do_ciclo {
                    l.passo = 100;
                }
            }
            200 => {
                // Pallet sem papelão a aplicar (ou já aplicado): libera.
                l.fim_do_ciclo = true;
                l.passo = 201;
            }
            201 => {
                if !sensor_tdp {
                    l.fim_do_ciclo = false;
                    l.passo = 100;
                }
            }
            400 | 403 => {
                // Sem papelão / nível baixo: recua e volta à coleta, aguarda reabastecer.
                l.sem_papelao = true;
                l.vacuo = false;
                if !cil_rc {
                    l.cil_avancar = false;
                    l.cil_recuar = true;
                }
                if !limite_pega && cil_rc {
                    l.motor_entrega = false;
                    l.motor_coleta = true;
                }
                if limite_pega {
                    l.motor_coleta = false;
                }
                if state.sheets_left > 0 && limite_pega && cil_rc {
                    l.sem_papelao = false;
                    l.passo = 100;
                }
            }
            _ => l.passo = 0,
        }
        if std::env::var("PDT_TRACE").is_ok() && l.passo != l.last_passo {
            eprintln!(
                "[{:.2}] {}: passo {} -> {} (tdp {} inf {} sup {} rc {} av {})",
                time.elapsed_secs(), eq.name, l.last_passo, l.passo, u8::from(sensor_tdp), u8::from(pap_inf), u8::from(pap_sup), u8::from(cil_rc), u8::from(cil_av)
            );
            l.last_passo = l.passo;
        }
        drive(&mut io, "MotorPosicaoColeta", l.motor_coleta);
        drive(&mut io, "MotorPosicaoEntrega", l.motor_entrega);
        drive(&mut io, "CilindroAvancar", l.cil_avancar);
        drive(&mut io, "CilindroRecuar", l.cil_recuar);
        drive(&mut io, "Vacuo", l.vacuo);
    }
}

#[allow(clippy::type_complexity)]
pub fn animate_paper(
    placers: Query<(&Equipment, &PaperPlacerState, &Children)>,
    mut carriages: Query<(&mut Transform, &Children), (With<PaperCarriage>, Without<PaperHead>, Without<PaperStack>)>,
    mut heads: Query<&mut Transform, (With<PaperHead>, Without<PaperCarriage>, Without<PaperStack>)>,
    mut stacks: Query<&mut Transform, (With<PaperStack>, Without<PaperCarriage>, Without<PaperHead>)>,
) {
    for (eq, state, children) in &placers {
        for child in children.iter() {
            if let Ok((mut tf, carriage_children)) = carriages.get_mut(child) {
                tf.translation.x = head_local_x(eq, state);
                for c in carriage_children.iter() {
                    if let Ok(mut head) = heads.get_mut(c) {
                        head.translation.y = HEAD_RETRACTED_Y - state.extension + state.compression;
                    }
                }
            } else if let Ok(mut stack) = stacks.get_mut(child) {
                let h = (state.sheets_left as f32 * SHEET_THICKNESS).max(0.001);
                stack.scale.y = h;
                stack.translation.y = DEPOSIT_FLOOR + h / 2.0;
            }
        }
    }
}
