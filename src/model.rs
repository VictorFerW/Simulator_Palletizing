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
    /// Carro cheio giratório (CCH): car with a rotating TDP that takes full pallets
    /// from the palletizing lines to the delivery.
    FullCar,
    /// Colocador de papelão (CP): lays cardboard sheets on pallets.
    CardboardPlacer,
    /// Elevador de pallets (alimentador): keeps a stack of empty pallets lifted
    /// over a TDP and releases the bottom one at a time.
    PalletElevator,
}

impl EquipmentKind {
    #[cfg(test)]
    pub const ALL: [Self; 14] = [
        Self::BeltConveyor,
        Self::Table2Stage,
        Self::Table3Stage,
        Self::PalletConveyor,
        Self::PalletizingConveyor,
        Self::ShuttleCar,
        Self::FullCar,
        Self::CardboardPlacer,
        Self::PalletElevator,
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
            Self::FullCar => "Carro cheio giratório (CCH)",
            Self::CardboardPlacer => "Colocador de papelão (CP)",
            Self::PalletElevator => "Elevador de pallets (alimentador)",
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
            Self::FullCar => "CCH",
            Self::CardboardPlacer => "CP",
            Self::PalletElevator => "Elevador",
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
                | Self::FullCar
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
            Self::FullCar => FULLCAR_SIGNALS,
            Self::CardboardPlacer => PAPER_SIGNALS,
            Self::PalletElevator => ELEVATOR_SIGNALS,
        }
    }
}

use SignalDir::{In, Out};
use SignalType::{Bool, Int};

// Esteira de produtos (FB da esteira): sensores de entrada e saída opcionais.
const BELT_SIGNALS: &[SignalDef] = &[
    SignalDef::new("motor", "Motor ligado", In, Bool),
    SignalDef::new("reverse", "Sentido reverso", In, Bool),
    SignalDef::new("EsteiraPosteriorOK", "Esteira posterior OK", In, Bool),
    SignalDef::new("SensorEntrada", "Sensor de entrada", Out, Bool),
    SignalDef::new("SensorSaida", "Sensor de saída", Out, Bool),
    SignalDef::new("EsteiraCheia", "Esteira cheia", Out, Bool),
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
// FB do elevador / alimentador de pallets.
const ELEVATOR_SIGNALS: &[SignalDef] = &[
    SignalDef::new("MotorSobe", "Motor subir", In, Bool),
    SignalDef::new("MotorDesce", "Motor descer", In, Bool),
    SignalDef::new("Garra_Abrir", "Garra abrir", In, Bool),
    SignalDef::new("Garra_Fechar", "Garra fechar", In, Bool),
    SignalDef::new("TDPProntoReceber", "TDP pronto p/ receber", In, Bool),
    SignalDef::new("TDPosteriorProntoReceber", "TDP posterior pronto p/ receber", In, Bool),
    SignalDef::new("SensorPosInferior", "Sensor posição inferior", Out, Bool),
    SignalDef::new("SensorPosInferiorMec", "Fim de curso inferior", Out, Bool),
    SignalDef::new("SensorPosIntermediario", "Sensor posição intermediária", Out, Bool),
    SignalDef::new("SensorPosSuperior", "Sensor posição superior", Out, Bool),
    SignalDef::new("SensorPosSuperiorMec", "Fim de curso superior", Out, Bool),
    SignalDef::new("SensorGarraAberta", "Garra aberta", Out, Bool),
    SignalDef::new("SensorGarraFechada", "Garra fechada", Out, Bool),
    SignalDef::new("SensorPilhaPallet", "Pallet na pilha", Out, Bool),
    SignalDef::new("SensorTDP", "Pallet no TDP", Out, Bool),
];
// FBs "Carro Cheio Movimento" + "Carro Cheio TDP" (CCH).
const FULLCAR_SIGNALS: &[SignalDef] = &[
    SignalDef::new("Motor", "Motor do carro", In, Bool),
    SignalDef::new("LiberarFreio", "Liberar freio do carro", In, Bool),
    SignalDef::new("Velocidade", "Velocidade do carro (mm/s, com sinal)", In, Int),
    SignalDef::new("MotorTDP", "Motor TDP", In, Bool),
    SignalDef::new("LiberarFreioTDP", "Liberar freio TDP", In, Bool),
    SignalDef::new("VelocidadeTDP", "Velocidade TDP (mm/s, com sinal)", In, Int),
    SignalDef::new("MotorGiro", "Motor giro", In, Bool),
    SignalDef::new("LiberarFreioGiro", "Liberar freio giro", In, Bool),
    SignalDef::new("VelocidadeGiro", "Velocidade giro (°/s, + entregar)", In, Int),
    SignalDef::new("PosicaoAtual", "Sensor de distância (mm)", Out, Int),
    SignalDef::new("DriverVelocidade", "Velocidade real do carro (mm/s)", Out, Int),
    SignalDef::new("DriverFalha", "Inversor em falha", Out, Bool),
    SignalDef::new("FimDeCurso", "Fim de curso do trilho", Out, Bool),
    SignalDef::new("Sensor1", "Sensor 1 (entrada)", Out, Bool),
    SignalDef::new("Sensor2", "Sensor 2", Out, Bool),
    SignalDef::new("Sensor3", "Sensor 3 (centro)", Out, Bool),
    SignalDef::new("Sensor4", "Sensor 4 (fora do centro)", Out, Bool),
    SignalDef::new("Giro_Sensor_PosReceber", "Giro na posição receber", Out, Bool),
    SignalDef::new("Giro_Sensor_PosEntregar", "Giro na posição entregar", Out, Bool),
    SignalDef::new("VelocidadeAtualTDP", "Velocidade real TDP (mm/s)", Out, Int),
    SignalDef::new("VelocidadeAtualGiro", "Velocidade real giro (°/s)", Out, Int),
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
    /// Robot: mesas/belts to pick from (empty = every one in reach).
    #[serde(deserialize_with = "names")]
    pub pick_from: Vec<String>,
    /// Robot: palletizing TDPs to place on (empty = every one in reach).
    #[serde(deserialize_with = "names")]
    pub place_on: Vec<String>,
    /// Robot speed override 0.1..=1.0.
    pub speed_override: f32,
    /// Robot: manufacturer model (kinematics, limits, speeds and 3D model).
    pub robot_model: RobotModel,
    /// Robot: mounted on the cavalete (Selgron 01F11M000520), whose top is the robot
    /// base height; size and turn relative to the robot.
    pub robot_base: bool,
    pub base_height: f32,
    pub base_width: f32,
    pub base_depth: f32,
    pub base_yaw: f32,
    /// Robot: software limits of the axes (degrees, [min, max] per axis in the
    /// manufacturer's axes: J1..J4 / A1, A2, A3, A6), inside the data sheet; empty =
    /// the data sheet ranges.
    pub axis_limits: Vec<[f32; 2]>,
    /// Esteira de produtos: sensors at the entry and at the exit (each optional),
    /// accumulation (AtivaAcumulo), TempoOciosidade and TempoVaiPara (s).
    /// Esteira de produtos: run by its FB (otherwise the motor stays on unless the
    /// PLC drives it). Off in layouts saved before it existed.
    pub belt_logic: bool,
    pub belt_entry_sensor: bool,
    pub belt_exit_sensor: bool,
    /// Entry sensor from the belt start and exit sensor from its end (m).
    pub belt_entry_pos: f32,
    pub belt_exit_pos: f32,
    pub belt_accumulate: bool,
    pub belt_idle_time: f32,
    pub belt_run_on: f32,
    /// Robot: which mesas feed each palletizing TDP (none listed = every one).
    pub routes: Vec<Route>,
    /// CVZ: TDP on the car, length along its transport (across the rail).
    pub car_length: f32,
    /// CVZ: TDP on the car, width (along the rail).
    pub car_width: f32,
    /// CVZ: nominal travel speed (m/s); CCH: speed with a pallet (VelocidadeRapida).
    pub car_speed: f32,
    /// CCH: speed without a pallet (VelocidadeRapidoVazio).
    pub car_speed_empty: f32,
    /// CCH: rotation of the TDP from receiving to delivering (degrees, + counter-clockwise).
    pub giro_deg: f32,
    /// CCH: giro speed (°/s).
    pub giro_speed: f32,
    /// CVZ: stop positions read by the distance sensor (m): coleta, then posição A, B...
    pub stops: Vec<f32>,
    /// CVZ: index of the stop under a colocador de papelão (-1: none).
    pub paper_stop: i32,
    /// CVZ: stops where the car takes empty pallets (coleta); empty = the first stop.
    pub pickup_stops: Vec<usize>,
    /// CP: vertical cylinder stroke (m).
    pub cp_stroke: f32,
    /// CP: lay cardboard on the pallets (else they are just released).
    pub apply_paper: bool,
    /// CP: two sheets per pallet (Aplicar2xPapelao).
    pub cp_two_sheets: bool,
    /// Elevador: pallets in a full stack.
    pub stack_size: u32,
    /// Mesa 3 estágios: receita 10x1 (Fardo10Kg): groups of 3 or 4 fardos side by side.
    pub fardo_10kg: bool,
    /// TDP de paletização: layer pattern chosen in "Padrões de paletização" (else
    /// the automatic column / interlocked grid).
    pub pattern: Option<crate::pattern::Pattern>,
    /// Elevador: a new stack is set on the TDP when the last pallet leaves.
    pub auto_refill: bool,
}

/// Palletizing robot models.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
pub enum RobotModel {
    #[default]
    FanucM410iC185,
    KukaKr180R3200_2Pa,
}

impl RobotModel {
    pub const ALL: [Self; 2] = [Self::FanucM410iC185, Self::KukaKr180R3200_2Pa];

    pub fn label(self) -> &'static str {
        match self {
            Self::FanucM410iC185 => "FANUC M-410iC/185",
            Self::KukaKr180R3200_2Pa => "KUKA KR 180 R3200-2 PA",
        }
    }

    /// Name prefix of new robots of this model.
    pub fn prefix(self) -> &'static str {
        match self {
            Self::FanucM410iC185 => "FANUC_M410iC",
            Self::KukaKr180R3200_2Pa => "KUKA_KR180",
        }
    }
}

/// Robot route: the mesas/belts whose products go to the TDP `tdp`.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Route {
    pub tdp: String,
    pub from: Vec<String>,
}

impl Params {
    /// Robot: the pick conveyor `pick` may feed the TDP `tdp`.
    pub fn routed(&self, tdp: &str, pick: &str) -> bool {
        self.routes.iter().find(|r| r.tdp == tdp).is_none_or(|r| r.from.is_empty() || r.from.iter().any(|n| n == pick))
    }
}

/// A list of equipment names; older layouts stored a single name (empty = none).
fn names<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Vec<String>, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum OneOrMany {
        One(String),
        Many(Vec<String>),
    }
    Ok(match OneOrMany::deserialize(d)? {
        OneOrMany::One(s) if s.is_empty() => Vec::new(),
        OneOrMany::One(s) => vec![s],
        OneOrMany::Many(v) => v,
    })
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
            pick_from: Vec::new(),
            place_on: Vec::new(),
            speed_override: 1.0,
            robot_model: RobotModel::default(),
            robot_base: false,
            base_height: 1.5945,
            base_width: 1.15,
            base_depth: 1.60,
            base_yaw: 0.0,
            axis_limits: Vec::new(),
            belt_logic: false,
            belt_entry_sensor: true,
            belt_exit_sensor: true,
            belt_entry_pos: 0.1,
            belt_exit_pos: 0.1,
            belt_accumulate: true,
            belt_idle_time: 30.0,
            belt_run_on: 5.0,
            routes: Vec::new(),
            car_length: 1.56,
            car_width: 1.28,
            car_speed: 1.0,
            car_speed_empty: 1.0,
            giro_deg: 90.0,
            giro_speed: 30.0,
            stops: vec![1.0, 4.0, 7.0],
            paper_stop: -1,
            pickup_stops: Vec::new(),
            cp_stroke: 1.05,
            apply_paper: true,
            cp_two_sheets: false,
            stack_size: 10,
            fardo_10kg: false,
            pattern: None,
            auto_refill: true,
        }
    }
}

impl Params {
    /// CVZ: the stop `i` is a coleta.
    pub fn is_pickup(&self, i: usize) -> bool {
        if self.pickup_stops.is_empty() { i == 0 } else { self.pickup_stops.contains(&i) }
    }

    pub fn for_kind(kind: EquipmentKind) -> Self {
        let base = Self::default();
        match kind {
            EquipmentKind::BeltConveyor => Self { length: 4.0, belt_logic: true, ..base },
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
            // CCH (MOD CARRO ALIMENT ROTATIVO): TDP 1829 x 1380 at 514 mm on a slewing ring.
            // Stops: espera, entrega, linha A, linha B.
            EquipmentKind::FullCar => Self {
                length: 10.0,
                width: 1.38,
                height: 0.514,
                speed: 0.25,
                car_length: 1.829,
                car_width: 1.38,
                car_speed: 0.6,
                car_speed_empty: 1.0,
                stops: vec![1.0, 9.0, 3.0, 5.5],
                ..base
            },
            // CP C2370: 1887 mm between the deposit and the delivery position.
            EquipmentKind::CardboardPlacer => Self { length: 1.887, width: 1.6, height: 0.0, ..base },
            // Over a TDP: frame around the pallet, `height` = TDP top.
            EquipmentKind::PalletElevator => Self { length: 1.3, width: 1.75, height: 0.5, ..base },
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
            EquipmentKind::PalletElevator => (
                Vec3::new(-p.length / 2.0 - 0.06, 0.0, -p.width / 2.0 - 0.06),
                Vec3::new(p.length / 2.0 + 0.06, p.height + 2.1, p.width / 2.0 + 0.06),
            ),
            EquipmentKind::FullCar => {
                // Round TDP turning on the car.
                let half = crate::fullcar::turn_radius(p);
                (Vec3::new(-p.length / 2.0, 0.0, -half), Vec3::new(p.length / 2.0, p.height + 0.1, half + 0.3))
            }
            EquipmentKind::ShuttleCar => (
                Vec3::new(-p.length / 2.0, 0.0, -p.car_length / 2.0 - 0.1),
                Vec3::new(p.length / 2.0, p.height + 0.1, p.car_length / 2.0 + 0.3),
            ),
            k if k.is_conveyor() => (
                Vec3::new(-p.length / 2.0, 0.0, -p.width / 2.0 - 0.08),
                Vec3::new(p.length / 2.0, p.height + 0.1, p.width / 2.0 + 0.08),
            ),
            EquipmentKind::Robot if p.robot_base => {
                // The cavalete under the robot (turned by base_yaw: take the larger side).
                let half = (p.base_width.max(p.base_depth) / 2.0).max(0.76);
                (Vec3::new(-half, -p.base_height, -half), Vec3::new(half, 2.8, half))
            }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn routes_pick_the_mesa_of_each_tdp() {
        let mut p = Params::default();
        assert!(p.routed("TDP_A", "Mesa_A"));
        p.routes = vec![
            Route { tdp: "TDP_A".into(), from: vec!["Mesa_B".into()] },
            Route { tdp: "TDP_B".into(), from: vec!["Mesa_A".into()] },
        ];
        assert!(p.routed("TDP_A", "Mesa_B") && !p.routed("TDP_A", "Mesa_A"));
        assert!(p.routed("TDP_B", "Mesa_A") && !p.routed("TDP_B", "Mesa_B"));
        // A TDP with no route takes from every mesa.
        assert!(p.routed("TDP_C", "Mesa_A"));
    }

    #[test]
    fn old_single_name_routes_still_load() {
        let p: Params = serde_json::from_str(r#"{"pick_from": "Mesa_1", "place_on": ""}"#).unwrap();
        assert_eq!(p.pick_from, vec!["Mesa_1".to_owned()]);
        assert!(p.place_on.is_empty() && p.routes.is_empty());
    }
}
