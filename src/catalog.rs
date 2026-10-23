//! Catalog of equipment the editor can add: category, name, short characteristics
//! and the thumbnail (`assets/thumbnails/<id>.png`, made with `PDT_THUMBS`).

use crate::model::{EquipmentKind, Params, RobotModel};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Category {
    All,
    Robots,
    Conveyors,
    Tables,
    Pallets,
    Others,
}

impl Category {
    pub const ALL: [Self; 6] = [Self::All, Self::Robots, Self::Conveyors, Self::Tables, Self::Pallets, Self::Others];

    pub fn label(self) -> &'static str {
        match self {
            Self::All => "Tudo",
            Self::Robots => "Robôs",
            Self::Conveyors => "Transporte",
            Self::Tables => "Mesas",
            Self::Pallets => "Pallets",
            Self::Others => "Outros",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Item {
    /// File name of the thumbnail.
    pub id: &'static str,
    pub kind: EquipmentKind,
    pub model: Option<RobotModel>,
    pub name: &'static str,
    pub category: Category,
    pub specs: &'static str,
}

impl Item {
    /// Parameters of a new one.
    pub fn params(&self) -> Params {
        let mut params = Params::for_kind(self.kind);
        if let Some(model) = self.model {
            params.robot_model = model;
        }
        params
    }

    /// Name prefix of a new one.
    pub fn prefix(&self) -> &'static str {
        self.model.map_or(self.kind.prefix(), |m| m.prefix())
    }
}

const fn item(id: &'static str, kind: EquipmentKind, name: &'static str, category: Category, specs: &'static str) -> Item {
    Item { id, kind, model: None, name, category, specs }
}

pub const ITEMS: [Item; 15] = [
    Item {
        id: "robot_kuka_kr180",
        kind: EquipmentKind::Robot,
        model: Some(RobotModel::KukaKr180R3200_2Pa),
        name: "KUKA KR 180 R3200-2 PA",
        category: Category::Robots,
        specs: "180 kg · alcance 3195 mm · 5 eixos · A1 ±185°",
    },
    Item {
        id: "robot_fanuc_m410ic185",
        kind: EquipmentKind::Robot,
        model: Some(RobotModel::FanucM410iC185),
        name: "FANUC M-410iC/185",
        category: Category::Robots,
        specs: "185 kg · alcance 3143 mm · 4 eixos · J1 ±180°",
    },
    item("belt", EquipmentKind::BeltConveyor, "Esteira de correia", Category::Conveyors, "FB da esteira · sensores de entrada e saída"),
    item("pallet_conveyor", EquipmentKind::PalletConveyor, "TDP (transportador de pallets)", Category::Conveyors, "Roletes · sensor de presença no fim"),
    item("palletizing_conveyor", EquipmentKind::PalletizingConveyor, "TDP de paletização", Category::Conveyors, "Posição de paletização · padrão e camadas"),
    item("shuttle_car", EquipmentKind::ShuttleCar, "Carro de pallet vazio (CVZ)", Category::Conveyors, "Trilho com paradas · coleta dos dois lados"),
    item("full_car", EquipmentKind::FullCar, "Carro cheio giratório (CCH)", Category::Conveyors, "TDP redondo que gira · fila de pedidos"),
    item("table2", EquipmentKind::Table2Stage, "Mesa antecipada 2 estágios", Category::Tables, "Roletada com empurrador + coleta"),
    item("table3", EquipmentKind::Table3Stage, "Mesa pré-forma 3 estágios", Category::Tables, "Garra + pré-formadora · receita 10x1"),
    item("pallet_dispenser", EquipmentKind::PalletDispenser, "Magazine de pallets", Category::Pallets, "Solta um pallet vazio por vez"),
    item("pallet_elevator", EquipmentKind::PalletElevator, "Elevador de pallets (alimentador)", Category::Pallets, "Pilha com garra · reabastece sozinho"),
    item("cardboard_placer", EquipmentKind::CardboardPlacer, "Colocador de papelão (CP)", Category::Pallets, "Carro com cabeça de vácuo · 1 ou 2 papelões"),
    item("box_emitter", EquipmentKind::BoxEmitter, "Emissor de caixas", Category::Others, "Gera fardos no intervalo definido"),
    item("box_remover", EquipmentKind::BoxRemover, "Removedor (caixas/pallets)", Category::Others, "Retira o que chega no fim da linha"),
    item("sensor", EquipmentKind::Sensor, "Sensor fotoelétrico", Category::Others, "Feixe com saída digital"),
];

/// Thumbnail folder under `assets/`.
pub const THUMB_DIR: &str = "thumbnails";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_kind_and_robot_model_is_in_the_catalog() {
        for kind in EquipmentKind::ALL {
            assert!(ITEMS.iter().any(|i| i.kind == kind), "{kind:?}");
        }
        for model in RobotModel::ALL {
            assert!(ITEMS.iter().any(|i| i.model == Some(model)), "{model:?}");
        }
        let mut ids: Vec<&str> = ITEMS.iter().map(|i| i.id).collect();
        ids.sort();
        ids.dedup();
        assert_eq!(ids.len(), ITEMS.len());
    }
}
