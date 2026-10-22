use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EquipmentKind {
    BeltConveyor,
    /// Mesa pré antecipada de 2 estágios (entrada + coleta), onde o produto chega.
    #[serde(alias = "RollerConveyor")]
    Table2Stage,
    /// Mesa pré-forma de 3 estágios (entrada, mesa pré, coleta).
    Table3Stage,
    /// TDP: transportador de roletes para pallets.
    PalletConveyor,
    /// TDP de paletização: segura o pallet no batente enquanto o robô empilha.
    #[serde(alias = "PalletStation")]
    PalletizingConveyor,
    Robot,
    BoxEmitter,
    /// Magazine que entrega pallets vazios sobre um TDP.
    PalletDispenser,
    BoxRemover,
    Sensor,
    /// Carro de pallet vazio (CVZ): car with a TDP on a rail.
    ShuttleCar,
    /// Colocador de papelão (CP): lays cardboard sheets on pallets.
    CardboardPlacer,
}

impl EquipmentKind {
    pub const ALL: [Self; 12] = [
        Self::BeltConveyor,
        Self::Table2Stage,
        Self::Table3Stage,
        Self::PalletConveyor,
        Self::PalletizingConveyor,
        Self::ShuttleCar,
        Self::CardboardPlacer,
        Self::Robot,
        Self::BoxEmitter,
        Self::PalletDispenser,
        Self::BoxRemover,
        Self::Sensor,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::BeltConveyor => "Esteira de correia",
            Self::Table2Stage => "Mesa antecipada 2 estágios",
            Self::Table3Stage => "Mesa pré-forma 3 estágios",
            Self::PalletConveyor => "TDP (transportador de pallets)",
            Self::PalletizingConveyor => "TDP de paletização",
            Self::Robot => "Robô FANUC M-410iC/185",
            Self::BoxEmitter => "Emissor de caixas",
            Self::PalletDispenser => "Magazine de pallets",
            Self::BoxRemover => "Removedor (caixas/pallets)",
            Self::Sensor => "Sensor fotoelétrico",
            Self::ShuttleCar => "Carro de pallet vazio (CVZ)",
            Self::CardboardPlacer => "Colocador de papelão (CP)",
        }
    }

    pub const fn prefix(self) -> &'static str {
        match self {
            Self::BeltConveyor => "Esteira",
            Self::Table2Stage => "Mesa2E",
            Self::Table3Stage => "Mesa3E",
            Self::PalletConveyor => "TDP",
            Self::PalletizingConveyor => "TDP_Palet",
            Self::Robot => "FANUC_M410iC",
            Self::BoxEmitter => "Emissor",
            Self::PalletDispenser => "Magazine",
            Self::BoxRemover => "Removedor",
            Self::Sensor => "Sensor",
            Self::ShuttleCar => "CVZ",
            Self::CardboardPlacer => "CP",
        }
    }

    /// Carries loads (boxes or pallets) on its top surface.
    pub const fn is_conveyor(self) -> bool {
        matches!(
            self,
            Self::BeltConveyor
                | Self::Table2Stage
                | Self::Table3Stage
                | Self::PalletConveyor
                | Self::PalletizingConveyor
                | Self::ShuttleCar
        )
    }

    /// Conveyors where products arrive for the robot to pick.
    pub const fn is_product_conveyor(self) -> bool {
        matches!(self, Self::BeltConveyor | Self::Table2Stage | Self::Table3Stage)
    }

    pub const fn is_mesa(self) -> bool {
        matches!(self, Self::Table2Stage | Self::Table3Stage)
    }

    /// Signals exposed to the PLC. `In` is written by the PLC (or forced in the
    /// I/O panel), `Out` is produced by the simulation.
    pub fn signals(self) -> &'static [SignalDef] {
        match self {
            Self::BeltConveyor => BELT_SIGNALS,
            Self::Table2Stage => TABLE2_SIGNALS,
            Self::Table3Stage => TABLE3_SIGNALS,
            Self::PalletConveyor => TDP_SIGNALS,
            Self::PalletizingConveyor => PALLETIZING_TDP_SIGNALS,
            Self::Robot => ROBOT_SIGNALS,
            Self::BoxEmitter => EMITTER_SIGNALS,
            Self::PalletDispenser => DISPENSER_SIGNALS,
            Self::BoxRemover => REMOVER_SIGNALS,
            Self::Sensor => SENSOR_SIGNALS,
            Self::ShuttleCar => SHUTTLE_SIGNALS,
            Self::CardboardPlacer => PAPER_SIGNALS,
        }
    }
}

use SignalDir::{In, Out};
use SignalType::{Bool, Int};

const BELT_SIGNALS: &[SignalDef] = &[
    SignalDef::new("motor", "Motor ligado", In, Bool),
    SignalDef::new("reverse", "Sentido reverso", In, Bool),
];
// Mesa Antecipada Roletada (FB "Mesa_2estagios") + Mesa Área de Coleta.
const TABLE2_SIGNALS: &[SignalDef] = &[
    SignalDef::new("Motor", "Motor roletada", In, Bool),
    SignalDef::new("Bloqueio_Entrada", "Bloqueio de entrada", In, Bool),
    SignalDef::new("Bloqueio_Coleta", "Bloqueio da coleta", In, Bool),
    SignalDef::new("EmpurradorAv", "Empurrador avançar", In, Bool),
    SignalDef::new("EmpurradorRc", "Empurrador recuar", In, Bool),
    SignalDef::new("Motor_Coleta", "Motor coleta", In, Bool),
    SignalDef::new("Batente_Coleta", "Batente coleta", In, Bool),
    SignalDef::new("Sensor_EsteiraFim", "Sensor fim da esteira", Out, Bool),
    SignalDef::new("Sensor_AntesBloqueio", "Sensor antes do bloqueio", Out, Bool),
    SignalDef::new("Sensor_AposBloqueio", "Sensor após o bloqueio", Out, Bool),
    SignalDef::new("Sensor_RoletadaPos1", "Sensor roletada pos. 1", Out, Bool),
    SignalDef::new("Sensor_RoletadaPos2", "Sensor roletada pos. 2", Out, Bool),
    SignalDef::new("Sensor_RoletadaPos3", "Sensor roletada pos. 3", Out, Bool),
    SignalDef::new("Sensor_EmpurradorAv", "Sensor empurrador avançado", Out, Bool),
    SignalDef::new("Sensor_EmpurradorRc", "Sensor empurrador recuado", Out, Bool),
    SignalDef::new("Sensor_Coleta1", "Sensor coleta pos. 1", Out, Bool),
    SignalDef::new("Sensor_Coleta2", "Sensor coleta pos. 2", Out, Bool),
];
// Mesa Pré-forma 3 estágios (FB "Mesa_3_Estagios") + Mesa Área de Coleta.
const TABLE3_SIGNALS: &[SignalDef] = &[
    SignalDef::new("Motor", "Motor roletada + pré-formadora", In, Bool),
    SignalDef::new("BloqueioEntrada", "Bloqueio de entrada", In, Bool),
    SignalDef::new("BloqueioMeio", "Bloqueio do meio", In, Bool),
    SignalDef::new("BloqueioColeta", "Bloqueio da coleta", In, Bool),
    SignalDef::new("GarraSobe", "Garra subir", In, Bool),
    SignalDef::new("GarraDesce", "Garra descer", In, Bool),
    SignalDef::new("GarraAvancar", "Garra avançar", In, Bool),
    SignalDef::new("GarraRecuar", "Garra recuar", In, Bool),
    SignalDef::new("EmpurradorAvancar", "Empurrador pré avançar", In, Bool),
    SignalDef::new("EmpurradorRecuar", "Empurrador pré recuar", In, Bool),
    SignalDef::new("Motor_Coleta", "Motor coleta", In, Bool),
    SignalDef::new("Batente_Coleta", "Batente coleta", In, Bool),
    SignalDef::new("SensorEsteira", "Sensor esteira", Out, Bool),
    SignalDef::new("SensorEtapa1Pac1ProxBloq", "Etapa 1 pac. 1 próx. bloqueio", Out, Bool),
    SignalDef::new("SensorEtapa1Pac1", "Etapa 1 pac. 1", Out, Bool),
    SignalDef::new("SensorEtapa1Pac2", "Etapa 1 pac. 2", Out, Bool),
    SignalDef::new("SensorEtapa1Pac3", "Etapa 1 pac. 3", Out, Bool),
    SignalDef::new("SensorGarraLevantada", "Garra levantada", Out, Bool),
    SignalDef::new("SensorGarraRecolhida", "Garra recolhida", Out, Bool),
    SignalDef::new("SensorGarraAv", "Garra avançada", Out, Bool),
    SignalDef::new("SensorGarraRec", "Garra recuada", Out, Bool),
    SignalDef::new("SensorEtapa2Pac1", "Etapa 2 pac. 1", Out, Bool),
    SignalDef::new("SensorEtapa2Pac2", "Etapa 2 pac. 2", Out, Bool),
    SignalDef::new("SensorEtapa2Pac3", "Etapa 2 pac. 3", Out, Bool),
    SignalDef::new("SensorEmpurradorAv", "Empurrador pré avançado", Out, Bool),
    SignalDef::new("SensorEmpurradorRec", "Empurrador pré recuado", Out, Bool),
    SignalDef::new("Sensor_Coleta1", "Sensor coleta pos. 1", Out, Bool),
    SignalDef::new("Sensor_Coleta2", "Sensor coleta pos. 2", Out, Bool),
];
const TDP_SIGNALS: &[SignalDef] = &[
    SignalDef::new("motor", "Motor ligado", In, Bool),
    SignalDef::new("reverse", "Sentido reverso", In, Bool),
    SignalDef::new("presence", "Pallet no fim do TDP", Out, Bool),
];
const PALLETIZING_TDP_SIGNALS: &[SignalDef] = &[
    SignalDef::new("motor", "Motor ligado", In, Bool),
    SignalDef::new("stop", "Batente de posicionamento", In, Bool),
    SignalDef::new("present", "Pallet em posição", Out, Bool),
    SignalDef::new("full", "Pallet completo", Out, Bool),
    SignalDef::new("count", "Caixas no pallet", Out, Int),
];
const ROBOT_SIGNALS: &[SignalDef] = &[
    SignalDef::new("start", "Liberar ciclo", In, Bool),
    SignalDef::new("Autoriza_Pick", "Autoriza pick na coleta", In, Bool),
    SignalDef::new("busy", "Em ciclo", Out, Bool),
    SignalDef::new("Robo_Fardos_Coletados", "Fardos coletados", Out, Bool),
    SignalDef::new("pallet_full", "Pallet completo", Out, Bool),
    SignalDef::new("count", "Caixas no pallet", Out, Int),
];
const EMITTER_SIGNALS: &[SignalDef] = &[SignalDef::new("enable", "Emitir caixas", In, Bool)];
const DISPENSER_SIGNALS: &[SignalDef] = &[SignalDef::new("enable", "Liberar pallet", In, Bool)];
const REMOVER_SIGNALS: &[SignalDef] = &[SignalDef::new("enable", "Remover cargas", In, Bool)];
const SENSOR_SIGNALS: &[SignalDef] = &[SignalDef::new("detected", "Objeto detectado", Out, Bool)];
// FB "CP" (colocador de papelão).
const PAPER_SIGNALS: &[SignalDef] = &[
    SignalDef::new("MotorPosicaoColeta", "Motor carro p/ coleta", In, Bool),
    SignalDef::new("MotorPosicaoEntrega", "Motor carro p/ entrega", In, Bool),
    SignalDef::new("CilindroAvancar", "Cilindro avançar (desce)", In, Bool),
    SignalDef::new("CilindroRecuar", "Cilindro recuar (sobe)", In, Bool),
    SignalDef::new("Vacuo", "Vácuo", In, Bool),
    SignalDef::new("LimitePega", "Fim de curso pega", Out, Bool),
    SignalDef::new("LimiteEntrega", "Fim de curso entrega", Out, Bool),
    SignalDef::new("SensorCilindroAv", "Cilindro avançado", Out, Bool),
    SignalDef::new("SensorCilindroRc", "Cilindro recuado", Out, Bool),
    SignalDef::new("SensorPapelaoInferior", "Sensor papelão inferior", Out, Bool),
    SignalDef::new("SensorPapelaoSuperior", "Sensor papelão superior", Out, Bool),
    SignalDef::new("SensorTDP", "Pallet sob a entrega", Out, Bool),
];
// FB "CVZ" (carro vazio): inverter, brake, distance sensor, TDP on board.
const SHUTTLE_SIGNALS: &[SignalDef] = &[
    SignalDef::new("LigarDrive", "Ligar inversor", In, Bool),
    SignalDef::new("LiberarFreio", "Liberar freio", In, Bool),
    SignalDef::new("Velocidade", "Velocidade (mm/s, com sinal)", In, Int),
    SignalDef::new("Motor_TDP", "Motor TDP do carro", In, Bool),
    SignalDef::new("Reverso_TDP", "TDP do carro reverso", In, Bool),
    SignalDef::new("PosicaoAtual", "Sensor de distância (mm)", Out, Int),
    SignalDef::new("DriverFDBVelocidade", "Velocidade real (mm/s)", Out, Int),
    SignalDef::new("DriverEmFalha", "Inversor em falha", Out, Bool),
    SignalDef::new("SensorExtremoAtivoTDP", "Sensor de extremo", Out, Bool),
    SignalDef::new("Sensor_Pallet_Carro", "Pallet no carro", Out, Bool),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SignalDir {
    In,
    Out,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SignalType {
    Bool,
    Int,
}

#[derive(Clone, Copy, Debug)]
pub struct SignalDef {
    pub key: &'static str,
    pub label: &'static str,
    pub dir: SignalDir,
    pub ty: SignalType,
}

impl SignalDef {
    const fn new(key: &'static str, label: &'static str, dir: SignalDir, ty: SignalType) -> Self {
        Self { key, label, dir, ty }
    }
}

/// Geometry and behaviour parameters. Not every field applies to every kind;
/// the property panel only shows the relevant ones.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Params {
    /// Length along local X (transport direction for conveyors).
    pub length: f32,
    /// Width along local Z.
    pub width: f32,
    /// Height of the working surface (conveyor top, emitter output).
    pub height: f32,
    /// Conveyor speed in m/s.
    pub speed: f32,
    /// Emitter/dispenser interval in seconds.
    pub interval: f32,
    /// Box size emitted [length X, height Y, width Z]; on mesas, the fardo size (lanes, pushers).
    pub box_size: [f32; 3],
    /// Pallet size [along X, height, along Z] (PBR 1000 x 1200 travelling on the TDP).
    pub pallet_size: [f32; 3],
    /// Number of layers stacked by the robot on a pallet.
    pub layers: u32,
    /// Odd layers are rotated 90° (interlocked pattern).
    pub interlock: bool,
    /// Mesas and TDP de paletização: internal logic (port of the PLC FBs) drives the
    /// actuators that are not driven by the PLC.
    #[serde(alias = "auto_change")]
    pub auto_logic: bool,
    /// Robot: name of the conveyor to pick from (empty = nearest in reach).
    pub pick_from: String,
    /// Robot: name of the palletizing TDP to place on (empty = nearest in reach).
    pub place_on: String,
    /// Robot speed override 0.1..=1.0.
    pub speed_override: f32,
    /// CVZ: TDP on the car, length along its transport (across the rail).
    pub car_length: f32,
    /// CVZ: TDP on the car, width (along the rail).
    pub car_width: f32,
    /// CVZ: nominal travel speed (m/s).
    pub car_speed: f32,
    /// CVZ: stop positions read by the distance sensor (m): coleta, then posição A, B...
    pub stops: Vec<f32>,
    /// CVZ: index of the stop under a colocador de papelão (-1: none).
    pub paper_stop: i32,
    /// CP: vertical cylinder stroke (m).
    pub cp_stroke: f32,
    /// CP: lay cardboard on the pallets (else they are just released).
    pub apply_paper: bool,
    /// CP: two sheets per pallet (Aplicar2xPapelao).
    pub cp_two_sheets: bool,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            length: 3.0,
            width: 0.8,
            height: 0.9,
            speed: 0.5,
            interval: 5.0,
            box_size: [0.4, 0.25, 0.3],
            pallet_size: [1.0, 0.144, 1.2],
            layers: 5,
            interlock: true,
            auto_logic: true,
            pick_from: String::new(),
            place_on: String::new(),
            speed_override: 1.0,
            car_length: 1.56,
            car_width: 1.28,
            car_speed: 1.0,
            stops: vec![1.0, 4.0, 7.0],
            paper_stop: -1,
            cp_stroke: 1.05,
            apply_paper: true,
            cp_two_sheets: false,
        }
    }
}

impl Params {
    pub fn for_kind(kind: EquipmentKind) -> Self {
        let base = Self::default();
        match kind {
            EquipmentKind::BeltConveyor => Self { length: 4.0, ..base },
            // Selgron: roletes L=1200, cada estágio ~1,2 m; roletes mais rápidos que a esteira
            // de entrada para abrir espaço entre os fardos.
            EquipmentKind::Table2Stage => Self { length: 2.4, width: 1.2, speed: 0.8, ..base },
            EquipmentKind::Table3Stage => Self { length: 3.6, width: 1.2, speed: 0.8, ..base },
            // TDP 1560 x 1280 x 120: rolos Ø76,2, topo a 500 mm, 15 m/min.
            EquipmentKind::PalletConveyor | EquipmentKind::PalletizingConveyor => Self {
                length: 1.56,
                width: 1.28,
                height: 0.5,
                speed: 0.25,
                ..base
            },
            EquipmentKind::Robot => Self { length: 1.1, width: 1.1, height: 0.0, ..base },
            EquipmentKind::BoxEmitter => Self { length: 0.7, width: 0.7, ..base },
            EquipmentKind::PalletDispenser => Self {
                length: 1.15,
                width: 1.4,
                height: 0.5,
                interval: 2.0,
                ..base
            },
            EquipmentKind::BoxRemover => Self { length: 1.2, width: 1.4, height: 0.0, ..base },
            // Rail 8 m; car TDP 1560 x 1280 at 500 mm, like the fixed TDPs.
            EquipmentKind::ShuttleCar => Self { length: 8.0, width: 1.56, height: 0.5, speed: 0.25, ..base },
            // CP C2370: 1887 mm between the deposit and the delivery position.
            EquipmentKind::CardboardPlacer => Self { length: 1.887, width: 1.6, height: 0.0, ..base },
            // Sensor: `width` = beam length, `height` = beam height above floor.
            EquipmentKind::Sensor => Self { length: 0.1, width: 1.0, height: 1.02, ..base },
        }
    }

    pub fn box_vec(&self) -> Vec3 {
        Vec3::from_array(self.box_size)
    }

    pub fn pallet_vec(&self) -> Vec3 {
        Vec3::from_array(self.pallet_size)
    }
}

#[derive(Component, Clone, Debug)]
pub struct Equipment {
    pub name: String,
    pub kind: EquipmentKind,
    pub params: Params,
}

impl Equipment {
    /// Local-space bounding box used for picking and selection outline.
    pub fn local_bounds(&self) -> (Vec3, Vec3) {
        let p = &self.params;
        match self.kind {
            EquipmentKind::CardboardPlacer => (
                Vec3::new(-p.length / 2.0 - 0.75, 0.0, -0.8),
                Vec3::new(p.length / 2.0 + 1.0, 2.35, 0.8),
            ),
            EquipmentKind::ShuttleCar => (
                Vec3::new(-p.length / 2.0, 0.0, -p.car_length / 2.0 - 0.1),
                Vec3::new(p.length / 2.0, p.height + 0.1, p.car_length / 2.0 + 0.3),
            ),
            k if k.is_conveyor() => (
                Vec3::new(-p.length / 2.0, 0.0, -p.width / 2.0 - 0.08),
                Vec3::new(p.length / 2.0, p.height + 0.1, p.width / 2.0 + 0.08),
            ),
            EquipmentKind::Robot => (Vec3::new(-0.76, 0.0, -0.76), Vec3::new(0.76, 2.8, 0.76)),
            EquipmentKind::Sensor => (
                Vec3::new(-0.08, 0.0, -0.08),
                Vec3::new(0.08, p.height + 0.1, p.width + 0.08),
            ),
            _ => (
                Vec3::new(-p.length / 2.0, p.height, -p.width / 2.0),
                Vec3::new(p.length / 2.0, p.height + 1.2, p.width / 2.0),
            ),
        }
    }
}

/// Current I/O values for an equipment, keyed by `SignalDef::key`.
/// Booleans are stored as 0.0 / 1.0.
#[derive(Component, Default, Clone, Debug)]
pub struct Io {
    pub values: HashMap<&'static str, f64>,
    /// Inputs with no PLC tag use this manual value (I/O panel "forçar").
    pub manual: HashMap<String, f64>,
    /// Inputs whose value came from a connected PLC this frame.
    pub from_plc: HashSet<&'static str>,
}

impl Io {
    pub fn get(&self, key: &str) -> f64 {
        self.values.get(key).copied().unwrap_or(0.0)
    }

    pub fn get_bool(&self, key: &str) -> bool {
        self.get(key) >= 0.5
    }

    pub fn set(&mut self, key: &'static str, value: f64) {
        self.values.insert(key, value);
    }

    pub fn set_bool(&mut self, key: &'static str, value: bool) {
        self.set(key, if value { 1.0 } else { 0.0 });
    }
}

#[derive(Component, Default, Clone, Debug, Serialize, Deserialize)]
pub struct TagBindings(pub Vec<EquipmentTagBinding>);

impl TagBindings {
    pub fn tag_for(&self, signal: &str) -> Option<&str> {
        self.0
            .iter()
            .find(|b| b.signal == signal)
            .map(|b| b.tag_id.as_str())
    }

    pub fn set(&mut self, signal: &str, tag_id: Option<String>) {
        self.0.retain(|b| b.signal != signal);
        if let Some(tag_id) = tag_id {
            self.0.push(EquipmentTagBinding {
                signal: signal.to_owned(),
                tag_id,
            });
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EquipmentTagBinding {
    pub signal: String,
    pub tag_id: String,
}

/// Marks an equipment whose child meshes must be rebuilt after a parameter change.
#[derive(Component)]
pub struct NeedsRebuild;

/// Primary selected item (property panel) and every selected item.
#[derive(Resource, Default)]
pub struct Selection(pub Option<Entity>, pub Vec<Entity>);

#[derive(Resource)]
pub struct SimControl {
    pub running: bool,
    pub time_scale: f32,
    pub reset_requested: bool,
}

impl Default for SimControl {
    fn default() -> Self {
        Self {
            running: true,
            time_scale: 1.0,
            reset_requested: false,
        }
    }
}

/// Creates a unique name like `Esteira_3`.
pub fn unique_name<'a>(prefix: &str, existing: impl Iterator<Item = &'a str>) -> String {
    let taken: Vec<&str> = existing.collect();
    (1..)
        .map(|n| format!("{prefix}_{n}"))
        .find(|candidate| !taken.contains(&candidate.as_str()))
        .expect("infinite iterator")
}
