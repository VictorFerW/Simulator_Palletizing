//! Bridges equipment signals and PLC tags, with one S7 connection per PLC.
//!
//! Inputs (PLC -> simulation) come from the PLC owning the tag while its
//! connection is up, otherwise from the manual value set in the I/O panel.
//! Outputs (simulation -> PLC) are written when they change.

use std::collections::HashMap;

use bevy::prelude::*;

use crate::communication::{self, PlcCommand, PlcCommandSender, PlcEvent, PlcEventReceiver, S7Settings, TagSpec};
use crate::model::{Equipment, EquipmentKind, Io, SignalDir, TagBindings};
use crate::s7::S7Address;
use crate::ProjectState;

/// One S7 connection, with its own communication thread.
pub struct PlcConnection {
    /// Display name in the UI.
    pub name: String,
    /// PLC (CPU) of the imported project served by this connection; empty = tags without PLC.
    pub plc: String,
    pub settings: S7Settings,
    pub enabled: bool,
    pub connected: bool,
    pub status: String,
    sender: PlcCommandSender,
    receiver: PlcEventReceiver,
    last_reads: Vec<String>,
}

impl PlcConnection {
    pub fn new(name: String, plc: String, settings: S7Settings) -> Self {
        let (sender, receiver) = communication::start();
        Self {
            name,
            plc,
            settings,
            enabled: false,
            connected: false,
            status: "Desconectado.".into(),
            sender,
            receiver,
            last_reads: Vec::new(),
        }
    }

    pub fn connect(&mut self) {
        self.enabled = true;
        self.status = "Conectando...".into();
        let _ = self.sender.0.send(PlcCommand::Connect(self.settings.clone()));
    }

    pub fn disconnect(&mut self) {
        self.enabled = false;
        self.connected = false;
        let _ = self.sender.0.send(PlcCommand::Disconnect);
    }
}

#[derive(Resource)]
pub struct PlcLink {
    pub connections: Vec<PlcConnection>,
    /// Last known value of every bound tag (read from a PLC or written by the simulation).
    pub values: HashMap<String, f64>,
    /// Bumped by the UI whenever connections are added/removed or re-assigned.
    pub revision: u32,
    last_sent: HashMap<String, f64>,
    /// tag id -> (connection index, resolved address or the reason it cannot be used).
    routes: HashMap<String, (Option<usize>, Result<TagSpec, String>)>,
    routes_revision: u32,
}

impl Default for PlcLink {
    fn default() -> Self {
        Self {
            connections: vec![PlcConnection::new("CLP_1".into(), String::new(), S7Settings::default())],
            values: HashMap::new(),
            revision: 1,
            last_sent: HashMap::new(),
            routes: HashMap::new(),
            routes_revision: 0,
        }
    }
}

impl PlcLink {
    pub fn connected_count(&self) -> usize {
        self.connections.iter().filter(|c| c.connected).count()
    }

    pub fn enabled_count(&self) -> usize {
        self.connections.iter().filter(|c| c.enabled).count()
    }

    /// Connection serving a PLC: the one assigned to it, otherwise one marked
    /// "any PLC" (empty), otherwise the only connection.
    pub fn connection_for(&self, plc: &str) -> Option<usize> {
        self.connections
            .iter()
            .position(|c| !plc.is_empty() && c.plc == plc)
            .or_else(|| self.connections.iter().position(|c| c.plc.is_empty()))
            .or_else(|| (self.connections.len() == 1).then_some(0))
    }

    /// Whether the PLC serving this tag is online.
    pub fn tag_online(&self, tag_id: &str) -> bool {
        self.routes
            .get(tag_id)
            .and_then(|(conn, _)| *conn)
            .and_then(|i| self.connections.get(i))
            .is_some_and(|c| c.connected)
    }

    pub fn route_label(&self, tag_id: &str) -> Option<String> {
        let (conn, spec) = self.routes.get(tag_id)?;
        let conn = conn.and_then(|i| self.connections.get(i)).map_or("sem conexão".to_owned(), |c| c.name.clone());
        Some(match spec {
            Ok(spec) => format!("{conn}: {:?} byte {} bit {}", spec.address.area, spec.address.byte, spec.address.bit),
            Err(e) => format!("{conn}: {e}"),
        })
    }
}

/// Value of an input with no PLC and no internal logic driving it (I/O panel "forçar").
pub fn default_manual(kind: EquipmentKind, key: &str) -> f64 {
    let _ = kind;
    if key == "Velocidade" {
        return 0.0;
    }
    let on = matches!(
        key,
        "motor" | "Motor" | "Motor_Coleta" | "Batente_Coleta" | "stop" | "start" | "enable" | "Autoriza_Pick"
            | "EmpurradorRc" | "EmpurradorRecuar" | "GarraDesce" | "GarraRecuar"
    );
    if on { 1.0 } else { 0.0 }
}

/// Resolves a tag to its S7 address and the PLC that owns it.
pub fn resolve_tag(project: &ProjectState, tag_id: &str) -> Result<(TagSpec, String), String> {
    let (tag, address, plc) = project
        .0
        .find_tag(tag_id)
        .ok_or_else(|| format!("tag '{tag_id}' não existe no projeto importado"))?;
    let address = address.ok_or_else(|| {
        format!("'{tag_id}' sem endereço absoluto (DB otimizado ou número do DB desconhecido)")
    })?;
    let parsed = S7Address::parse(&address, &tag.data_type)
        .ok_or_else(|| format!("endereço '{address}' não suportado"))?;
    Ok((
        TagSpec {
            id: tag_id.to_owned(),
            address: parsed,
            data_type: tag.data_type.clone(),
        },
        plc.to_owned(),
    ))
}

pub fn plc_events(mut link: ResMut<PlcLink>) {
    let link = &mut *link;
    let mut reconnected = false;
    for conn in &mut link.connections {
        for event in conn.receiver.0.try_iter() {
            match event {
                PlcEvent::Status { connected, message } => {
                    reconnected |= connected && !conn.connected;
                    conn.connected = connected && conn.enabled;
                    conn.status = message;
                }
                PlcEvent::Values(values) => {
                    for (id, value) in values {
                        link.values.insert(id, value);
                    }
                }
            }
        }
    }
    if reconnected {
        // Fresh connection: push every output again.
        link.last_sent.clear();
    }
}

pub fn apply_inputs(link: Res<PlcLink>, mut equipment: Query<(&Equipment, &TagBindings, &mut Io)>) {
    for (eq, bindings, mut io) in &mut equipment {
        io.from_plc.clear();
        for signal in eq.kind.signals().iter().filter(|s| s.dir == SignalDir::In) {
            let from_plc = bindings
                .tag_for(signal.key)
                .filter(|tag| link.tag_online(tag))
                .and_then(|tag| link.values.get(tag).copied());
            let value = match from_plc {
                Some(v) => {
                    io.from_plc.insert(signal.key);
                    v
                }
                None => io
                    .manual
                    .get(signal.key)
                    .copied()
                    .unwrap_or_else(|| default_manual(eq.kind, signal.key)),
            };
            io.set(signal.key, value);
        }
    }
}

pub fn publish_outputs(project: Res<ProjectState>, mut link: ResMut<PlcLink>, equipment: Query<(&Equipment, &TagBindings, &Io)>) {
    let link = &mut *link;
    let mut bound: Vec<(String, SignalDir, f64)> = Vec::new();
    for (eq, bindings, io) in &equipment {
        for signal in eq.kind.signals() {
            if let Some(tag_id) = bindings.tag_for(signal.key) {
                bound.push((tag_id.to_owned(), signal.dir, io.get(signal.key)));
            }
        }
    }

    // Re-resolve addresses/connections when the project, the connections or the bindings change.
    let stale = project.is_changed()
        || link.routes_revision != link.revision
        || bound.iter().any(|(id, ..)| !link.routes.contains_key(id));
    if stale {
        let routes = bound
            .iter()
            .map(|(id, ..)| {
                let route = match resolve_tag(&project, id) {
                    Ok((spec, plc)) => (link.connection_for(&plc), Ok(spec)),
                    Err(e) => (link.connection_for(""), Err(e)),
                };
                (id.clone(), route)
            })
            .collect();
        link.routes = routes;
        link.routes_revision = link.revision;
    }

    for (tag_id, dir, value) in &bound {
        if *dir != SignalDir::Out {
            continue;
        }
        link.values.insert(tag_id.clone(), *value);
        let Some((Some(conn), Ok(spec))) = link.routes.get(tag_id) else { continue };
        let Some(connection) = link.connections.get(*conn) else { continue };
        if !connection.connected || link.last_sent.get(tag_id) == Some(value) {
            continue;
        }
        link.last_sent.insert(tag_id.clone(), *value);
        let _ = connection.sender.0.send(PlcCommand::Write { tag: spec.clone(), value: *value });
    }

    for (index, connection) in link.connections.iter_mut().enumerate() {
        let mut reads: Vec<String> = bound
            .iter()
            .filter(|(id, dir, _)| {
                *dir == SignalDir::In && matches!(link.routes.get(id), Some((Some(c), Ok(_))) if *c == index)
            })
            .map(|(id, ..)| id.clone())
            .collect();
        reads.sort();
        reads.dedup();
        if reads != connection.last_reads || stale {
            let specs = reads
                .iter()
                .filter_map(|id| match link.routes.get(id) {
                    Some((_, Ok(spec))) => Some(spec.clone()),
                    _ => None,
                })
                .collect();
            let _ = connection.sender.0.send(PlcCommand::Configure { reads: specs });
            connection.last_reads = reads;
        }
    }
}
