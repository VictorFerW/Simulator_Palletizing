//! PLC communication thread. The Bevy schedule never blocks on the network:
//! it sends `PlcCommand`s and drains `PlcEvent`s once per frame.

use std::collections::{BTreeMap, HashMap};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, unbounded};

use crate::s7::{Area, S7Address, S7Client};
use crate::tia_importer::DataType;

#[derive(Debug, Clone, PartialEq)]
pub struct S7Settings {
    pub host: String,
    pub rack: u8,
    pub slot: u8,
    pub poll_ms: u64,
}

impl Default for S7Settings {
    fn default() -> Self {
        Self {
            host: "192.168.0.1".into(),
            rack: 0,
            slot: 1,
            poll_ms: 50,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TagSpec {
    pub id: String,
    pub address: S7Address,
    pub data_type: DataType,
}

#[derive(Debug, Clone)]
pub enum PlcCommand {
    /// Sets the tags read cyclically (PLC -> simulation).
    Configure { reads: Vec<TagSpec> },
    Connect(S7Settings),
    Disconnect,
    Write { tag: TagSpec, value: f64 },
}

#[derive(Debug, Clone)]
pub enum PlcEvent {
    Status { connected: bool, message: String },
    Values(Vec<(String, f64)>),
}

#[derive(Clone, bevy::prelude::Resource)]
pub struct PlcCommandSender(pub Sender<PlcCommand>);

#[derive(bevy::prelude::Resource)]
pub struct PlcEventReceiver(pub Receiver<PlcEvent>);

pub fn start() -> (PlcCommandSender, PlcEventReceiver) {
    let (cmd_tx, cmd_rx) = unbounded();
    let (event_tx, event_rx) = unbounded();
    std::thread::Builder::new()
        .name("plc-s7".into())
        .spawn(move || run(cmd_rx, event_tx))
        .expect("falha ao iniciar thread de comunicação");
    (PlcCommandSender(cmd_tx), PlcEventReceiver(event_rx))
}

/// Contiguous byte range read in one request.
struct ReadBlock {
    area: Area,
    start: u32,
    len: u32,
    tags: Vec<TagSpec>,
}

fn plan_reads(tags: &[TagSpec]) -> Vec<ReadBlock> {
    let mut by_area: BTreeMap<Area, Vec<&TagSpec>> = BTreeMap::new();
    for tag in tags {
        by_area.entry(tag.address.area).or_default().push(tag);
    }
    let mut blocks = Vec::new();
    for (area, mut list) in by_area {
        list.sort_by_key(|t| t.address.byte);
        let mut current: Option<ReadBlock> = None;
        for tag in list {
            let start = tag.address.byte;
            let end = start + tag.address.byte_len();
            match current.as_mut() {
                // Merge when the gap is small: one bigger read beats two round trips.
                Some(block) if start <= block.start + block.len + 32 => {
                    block.len = block.len.max(end - block.start);
                    block.tags.push(tag.clone());
                }
                _ => {
                    if let Some(done) = current.take() {
                        blocks.push(done);
                    }
                    current = Some(ReadBlock {
                        area,
                        start,
                        len: end - start,
                        tags: vec![tag.clone()],
                    });
                }
            }
        }
        blocks.extend(current);
    }
    blocks
}

fn run(commands: Receiver<PlcCommand>, events: Sender<PlcEvent>) {
    let mut settings: Option<S7Settings> = None;
    let mut client: Option<S7Client> = None;
    let mut reads: Vec<ReadBlock> = Vec::new();
    let mut pending_writes: HashMap<String, (TagSpec, f64)> = HashMap::new();
    let mut last_attempt = Instant::now() - Duration::from_secs(10);
    let mut last_warning: Option<String> = None;
    let status = |connected: bool, message: String| {
        let _ = events.send(PlcEvent::Status { connected, message });
    };

    loop {
        let wait = settings
            .as_ref()
            .map(|s| Duration::from_millis(s.poll_ms.max(10)))
            .unwrap_or(Duration::from_millis(200));
        match commands.recv_timeout(wait) {
            Ok(command) => {
                let mut batch = vec![command];
                batch.extend(commands.try_iter());
                for command in batch {
                    match command {
                        PlcCommand::Configure { reads: tags } => reads = plan_reads(&tags),
                        PlcCommand::Connect(new_settings) => {
                            settings = Some(new_settings);
                            client = None;
                            last_attempt = Instant::now() - Duration::from_secs(10);
                        }
                        PlcCommand::Disconnect => {
                            settings = None;
                            client = None;
                            status(false, "Desconectado (simulação interna).".into());
                        }
                        PlcCommand::Write { tag, value } => {
                            pending_writes.insert(tag.id.clone(), (tag, value));
                        }
                    }
                }
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return,
        }

        let Some(active) = settings.as_ref() else {
            pending_writes.clear();
            continue;
        };
        if client.is_none() {
            if last_attempt.elapsed() < Duration::from_secs(2) {
                continue;
            }
            last_attempt = Instant::now();
            match S7Client::connect(&active.host, active.rack, active.slot) {
                Ok(c) => {
                    status(true, format!("Conectado a {} (rack {}, slot {}).", active.host, active.rack, active.slot));
                    client = Some(c);
                }
                Err(e) => {
                    status(false, format!("Falha ao conectar em {}: {e}. Nova tentativa em 2 s.", active.host));
                    continue;
                }
            }
        }
        let c = client.as_mut().expect("connected above");
        // Protocol refusals (InvalidData) keep the link up; socket errors drop it.
        let mut failure = None;
        let mut warning = None;
        for (_, (tag, value)) in pending_writes.drain() {
            let bytes = tag.address.encode(value, &tag.data_type);
            match c.write(&tag.address, &bytes) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::InvalidData => {
                    warning = Some(format!("Escrita em {} falhou: {e}", tag.id));
                }
                Err(e) => {
                    failure = Some(format!("Escrita em {} falhou: {e}", tag.id));
                    break;
                }
            }
        }
        if failure.is_none() {
            let mut values = Vec::new();
            for block in &reads {
                match c.read(block.area, block.start, block.len as usize) {
                    Ok(bytes) => {
                        for tag in &block.tags {
                            let offset = (tag.address.byte - block.start) as usize;
                            if let Some(v) = bytes.get(offset..).and_then(|b| tag.address.decode(b, &tag.data_type)) {
                                values.push((tag.id.clone(), v));
                            }
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::InvalidData => {
                        warning = Some(format!("Leitura falhou: {e}"));
                    }
                    Err(e) => {
                        failure = Some(format!("Leitura falhou: {e}"));
                        break;
                    }
                }
            }
            if !values.is_empty() {
                let _ = events.send(PlcEvent::Values(values));
            }
        }
        if let Some(message) = failure {
            status(false, message);
            client = None;
        } else if let Some(message) = warning
            && last_warning.as_ref() != Some(&message) {
                status(true, message.clone());
                last_warning = Some(message);
            }
    }
}
