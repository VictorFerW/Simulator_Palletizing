//! TIA Portal project model and importers.
//!
//! `.ap18` files are only a small header: the real project lives in TIA's
//! proprietary database. It is read through TIA Portal Openness by the .NET
//! helper in `tools/TiaOpennessBridge`, which exports tag tables, data blocks
//! and PLC data types (SimaticML XML). This module turns that output, or files
//! exported manually (tag table XML/CSV/XLSX, DB/SCL sources), into tags with
//! absolute addresses usable by the S7 driver.

mod layout;
mod simatic_ml;

use std::collections::HashMap;
use std::io::BufRead;

use quick_xml::{Reader, events::Event};
use serde::{Deserialize, Serialize};

pub use layout::layout_members;
pub use simatic_ml::parse_simatic_ml;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TiaProject {
    pub name: String,
    pub tag_tables: Vec<TiaTagTable>,
    pub data_blocks: Vec<TiaDataBlock>,
    #[serde(default)]
    pub diagnostics: Vec<String>,
}

impl TiaProject {
    /// Iterates every tag of active tables/DBs with its resolved absolute address.
    pub fn active_tags(&self) -> impl Iterator<Item = (&PlcTag, Option<String>, String)> {
        let tables = self.tag_tables.iter().filter(|t| t.active).flat_map(|t| {
            t.tags
                .iter()
                .map(move |tag| (tag, tag.address.clone(), t.name.clone()))
        });
        let dbs = self.data_blocks.iter().filter(|d| d.active).flat_map(|d| {
            d.tags
                .iter()
                .map(move |tag| (tag, d.absolute_address(tag), d.name.clone()))
        });
        tables.chain(dbs)
    }

    /// Tag, its absolute address and the name of the PLC it belongs to.
    pub fn find_tag(&self, id: &str) -> Option<(&PlcTag, Option<String>, &str)> {
        for table in &self.tag_tables {
            if let Some(tag) = table.tags.iter().find(|t| t.id == id) {
                return Some((tag, tag.address.clone(), table.plc.as_str()));
            }
        }
        for db in &self.data_blocks {
            if let Some(tag) = db.tags.iter().find(|t| t.id == id) {
                return Some((tag, db.absolute_address(tag), db.plc.as_str()));
            }
        }
        None
    }

    /// Distinct PLC names found in the project (empty name = imported manually).
    pub fn plc_names(&self) -> Vec<String> {
        let mut names: Vec<String> = self
            .tag_tables
            .iter()
            .map(|t| t.plc.clone())
            .chain(self.data_blocks.iter().map(|d| d.plc.clone()))
            .filter(|n| !n.is_empty())
            .collect();
        names.sort();
        names.dedup();
        names
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TiaTagTable {
    pub id: String,
    pub name: String,
    /// PLC (CPU) this table belongs to.
    #[serde(default)]
    pub plc: String,
    pub tags: Vec<PlcTag>,
    #[serde(default)]
    pub active: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TiaDataBlock {
    pub id: String,
    pub name: String,
    /// PLC (CPU) this DB belongs to.
    #[serde(default)]
    pub plc: String,
    pub tags: Vec<PlcTag>,
    #[serde(default)]
    pub active: bool,
    /// DB number; required for S7 absolute access.
    #[serde(default)]
    pub number: Option<u32>,
    /// Optimized block access: no absolute offsets exist, S7 PUT/GET cannot reach it.
    #[serde(default)]
    pub optimized: bool,
}

impl TiaDataBlock {
    /// Tag addresses inside DBs are stored relative (`DBX2.1`); this prefixes the DB number.
    pub fn absolute_address(&self, tag: &PlcTag) -> Option<String> {
        if self.optimized {
            return None;
        }
        let relative = tag.address.as_deref()?;
        if relative.starts_with('%') {
            return Some(relative.to_owned());
        }
        Some(format!("%DB{}.{relative}", self.number?))
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PlcTag {
    pub id: String,
    pub name: String,
    pub data_type: DataType,
    pub address: Option<String>,
    pub comment: Option<String>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "element", rename_all = "snake_case")]
pub enum DataType {
    Bool,
    Byte,
    Char,
    SInt,
    USInt,
    Word,
    Int,
    UInt,
    DWord,
    DInt,
    UDInt,
    Real,
    Time,
    LReal,
    LInt,
    ULInt,
    LWord,
    String,
    Array(String),
    Udt(String),
    #[default]
    Unknown,
}

impl DataType {
    pub fn from_tia(text: &str) -> Self {
        let normalized = text.trim();
        let upper = normalized.to_ascii_uppercase();
        match upper.as_str() {
            "BOOL" => Self::Bool,
            "BYTE" => Self::Byte,
            "CHAR" => Self::Char,
            "SINT" => Self::SInt,
            "USINT" => Self::USInt,
            "WORD" => Self::Word,
            "INT" => Self::Int,
            "UINT" => Self::UInt,
            "DWORD" => Self::DWord,
            "DINT" => Self::DInt,
            "UDINT" => Self::UDInt,
            "REAL" => Self::Real,
            "TIME" => Self::Time,
            "LREAL" => Self::LReal,
            "LINT" => Self::LInt,
            "ULINT" => Self::ULInt,
            "LWORD" => Self::LWord,
            _ if upper.starts_with("STRING") || upper.starts_with("WSTRING") => Self::String,
            _ if upper.starts_with("ARRAY") => Self::Array(normalized.to_owned()),
            _ if upper.starts_with("UDT") || upper.starts_with("STRUCT") || normalized.starts_with('"') => {
                Self::Udt(normalized.trim_matches('"').to_owned())
            }
            _ => Self::Unknown,
        }
    }

    /// Size in bits of scalar types (Bool = 1). `None` for aggregates/unknown.
    pub fn bits(&self) -> Option<u32> {
        Some(match self {
            Self::Bool => 1,
            Self::Byte | Self::Char | Self::SInt | Self::USInt => 8,
            Self::Word | Self::Int | Self::UInt => 16,
            Self::DWord | Self::DInt | Self::UDInt | Self::Real | Self::Time => 32,
            Self::LReal | Self::LInt | Self::ULInt | Self::LWord => 64,
            _ => return None,
        })
    }

    pub fn is_scalar(&self) -> bool {
        self.bits().is_some()
    }

    pub fn short(&self) -> String {
        match self {
            Self::Array(s) | Self::Udt(s) => s.clone(),
            other => format!("{other:?}"),
        }
    }
}

/// One declaration of a DB/UDT interface, as found in SimaticML or SCL sources.
#[derive(Debug, Clone, Default)]
pub struct Member {
    pub name: String,
    pub datatype: String,
    pub comment: Option<String>,
    pub children: Vec<Member>,
}

// --------------------------------------------------------------------------
// Openness helper

/// JSON request sent to the Windows-only .NET helper built against
/// `Siemens.Engineering.dll` V18. The helper owns all use of Openness; this
/// process never loads Siemens' managed DLLs.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpennessRequest {
    pub schema_version: String,
    pub operation: String,
    pub project_path: String,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct BridgeResponse {
    pub project: BridgeProject,
    #[serde(default)]
    pub diagnostics: Vec<String>,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct BridgeProject {
    pub name: String,
    #[serde(default)]
    pub plcs: Vec<BridgePlc>,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct BridgePlc {
    pub name: String,
    #[serde(default)]
    pub tag_tables: Vec<BridgeTagTable>,
    #[serde(default)]
    pub data_blocks: Vec<BridgeBlock>,
    #[serde(default)]
    pub types: Vec<BridgeBlock>,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct BridgeTagTable {
    pub name: String,
    #[serde(default)]
    pub tags: Vec<BridgeTag>,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct BridgeTag {
    pub name: String,
    pub data_type: String,
    #[serde(default)]
    pub address: Option<String>,
    #[serde(default)]
    pub comment: Option<String>,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct BridgeBlock {
    pub name: String,
    #[serde(default)]
    pub number: Option<u32>,
    #[serde(default)]
    pub optimized: Option<bool>,
    #[serde(default)]
    pub xml: Option<String>,
    #[serde(default)]
    pub error: Option<String>,
}

/// Runs the helper: one JSON request on stdin, one `BridgeResponse` on stdout.
/// Blocking; call it from a background thread (opening a project takes minutes).
pub fn import_project_via_openness(
    helper_path: &std::path::Path,
    project_path: &std::path::Path,
) -> Result<TiaProject, String> {
    use std::io::Write;
    use std::process::{Command, Stdio};

    let request = OpennessRequest {
        schema_version: "2.0".into(),
        operation: "export_project_structure".into(),
        project_path: project_path.to_string_lossy().into_owned(),
    };
    let request_json = serde_json::to_vec(&request).map_err(|e| e.to_string())?;
    let mut command = Command::new(helper_path);
    command
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        const CREATE_NO_WINDOW: u32 = 0x0800_0000;
        command.creation_flags(CREATE_NO_WINDOW);
    }
    let mut child = command.spawn().map_err(|e| {
        format!(
            "Não foi possível iniciar o auxiliar Openness '{}': {e}. Compile-o com tools/TiaOpennessBridge/build.ps1.",
            helper_path.display()
        )
    })?;
    {
        let mut stdin = child.stdin.take().ok_or("stdin do auxiliar indisponível")?;
        stdin.write_all(&request_json).map_err(|e| e.to_string())?;
        stdin.write_all(b"\n").map_err(|e| e.to_string())?;
    }
    let output = child.wait_with_output().map_err(|e| e.to_string())?;
    if !output.status.success() {
        return Err(format!(
            "Auxiliar Openness terminou com {}: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }
    let response: BridgeResponse = serde_json::from_slice(&output.stdout)
        .map_err(|e| format!("Resposta JSON inválida do auxiliar Openness: {e}"))?;
    Ok(project_from_bridge(response))
}

pub fn project_from_bridge(response: BridgeResponse) -> TiaProject {
    let mut project = TiaProject {
        name: response.project.name,
        diagnostics: response.diagnostics,
        ..Default::default()
    };
    let multi_plc = response.project.plcs.len() > 1;
    for plc in response.project.plcs {
        let prefix = if multi_plc { format!("{}/", plc.name) } else { String::new() };
        for table in plc.tag_tables {
            let table_name = format!("{prefix}{}", table.name);
            project.tag_tables.push(TiaTagTable {
                id: table_name.clone(),
                tags: table
                    .tags
                    .into_iter()
                    .map(|t| PlcTag {
                        id: format!("{table_name}.{}", t.name),
                        name: t.name,
                        data_type: DataType::from_tia(&t.data_type),
                        address: t.address.filter(|a| !a.is_empty()),
                        comment: t.comment.filter(|c| !c.is_empty()),
                    })
                    .collect(),
                name: table_name,
                plc: plc.name.clone(),
                active: false,
            });
        }
        let mut udts = HashMap::new();
        for ty in &plc.types {
            if let Some(xml) = &ty.xml {
                match parse_simatic_ml(xml) {
                    Ok(parsed) => {
                        udts.insert(parsed.name.clone(), parsed.members);
                    }
                    Err(e) => project.diagnostics.push(format!("UDT {}: {e}", ty.name)),
                }
            }
        }
        for block in plc.data_blocks {
            let block_name = format!("{prefix}{}", block.name);
            if let Some(error) = &block.error {
                project.diagnostics.push(format!("DB {block_name}: {error}"));
            }
            let Some(xml) = &block.xml else { continue };
            match parse_simatic_ml(xml) {
                Ok(parsed) => {
                    let optimized = block.optimized.unwrap_or(parsed.optimized);
                    let mut db = data_block_from_members(
                        &block_name,
                        block.number.or(parsed.number),
                        optimized,
                        &parsed.members,
                        &udts,
                        &mut project.diagnostics,
                    );
                    db.active = false;
                    db.plc = plc.name.clone();
                    project.data_blocks.push(db);
                }
                Err(e) => project.diagnostics.push(format!("DB {block_name}: {e}")),
            }
        }
    }
    project
}

pub fn data_block_from_members(
    block_name: &str,
    number: Option<u32>,
    optimized: bool,
    members: &[Member],
    udts: &HashMap<String, Vec<Member>>,
    diagnostics: &mut Vec<String>,
) -> TiaDataBlock {
    let (tags, warnings) = layout_members(block_name, members, udts);
    diagnostics.extend(warnings);
    TiaDataBlock {
        plc: String::new(),
        id: block_name.to_owned(),
        name: block_name.to_owned(),
        tags,
        active: false,
        number,
        optimized,
    }
}

// --------------------------------------------------------------------------
// Manually exported files

/// Reads tag tables exported as XML (typical `TagTable/Tag/Name/DataType/LogicalAddress`
/// pattern and `AttributeList` elements with the same fields).
pub fn parse_tag_table_xml<R: BufRead>(
    reader: R,
    table_hint: Option<&str>,
) -> Result<TiaTagTable, String> {
    let mut xml = Reader::from_reader(reader);
    xml.config_mut().trim_text(true);
    let mut buf = Vec::new();
    let mut table_name = table_hint.unwrap_or("Tabela importada").to_owned();
    let mut tags = Vec::new();
    let mut current_tag: Option<PlcTag> = None;
    let mut current_field = String::new();
    let mut in_tag = false;

    loop {
        match xml
            .read_event_into(&mut buf)
            .map_err(|e| format!("XML inválido: {e}"))?
        {
            Event::Start(e) | Event::Empty(e) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).to_ascii_lowercase();
                if !in_tag && (name.contains("tagtable") || name == "plctagtable") {
                    for attr in e.attributes().flatten() {
                        if attr.key.local_name().as_ref().eq_ignore_ascii_case(b"name") {
                            table_name = String::from_utf8_lossy(&attr.value).into_owned();
                        }
                    }
                }
                if name.contains("plctag") && !name.contains("table") || name == "tag" {
                    current_tag = Some(PlcTag::default());
                    in_tag = true;
                }
                if in_tag {
                    current_field = name.clone();
                }
                if let Some(tag) = current_tag.as_mut() {
                    for attr in e.attributes().flatten() {
                        let key = String::from_utf8_lossy(attr.key.local_name().as_ref())
                            .to_ascii_lowercase();
                        let value = String::from_utf8_lossy(&attr.value).into_owned();
                        match key.as_str() {
                            "name" => tag.name = value,
                            "datatype" | "data_type" => tag.data_type = DataType::from_tia(&value),
                            "logicaladdress" | "address" => tag.address = Some(value),
                            _ => {}
                        }
                    }
                }
                if !in_tag && name == "name" {
                    current_field = "table_name".into();
                }
            }
            Event::Text(e) => {
                let text = e.decode().map_err(|e| e.to_string())?.into_owned();
                if in_tag {
                    if let Some(tag) = current_tag.as_mut() {
                        match current_field.as_str() {
                            "name" => {
                                if tag.name.is_empty() {
                                    tag.name = text
                                }
                            }
                            "datatype" | "datatypename" => tag.data_type = DataType::from_tia(&text),
                            "logicaladdress" | "address" => tag.address = Some(text),
                            "comment" | "commentary" | "text" => tag.comment = Some(text),
                            _ => {}
                        }
                    }
                } else if current_field == "table_name" && !text.trim().is_empty() {
                    table_name = text;
                }
            }
            Event::End(e) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).to_ascii_lowercase();
                if in_tag && (name.contains("plctag") && !name.contains("table") || name == "tag") {
                    if let Some(mut tag) = current_tag.take() {
                        if tag.id.is_empty() {
                            tag.id = format!("{}.{}", table_name, tag.name);
                        }
                        tags.push(tag);
                    }
                    in_tag = false;
                    current_field.clear();
                }
            }
            Event::Eof => break,
            _ => {}
        }
        buf.clear();
    }
    for tag in &mut tags {
        tag.id = format!("{}.{}", table_name, tag.name);
    }

    Ok(TiaTagTable {
        plc: String::new(),
        id: table_name.clone(),
        name: table_name,
        tags,
        active: false,
    })
}

pub fn parse_tag_table_csv<R: std::io::Read>(
    reader: R,
    table_name: &str,
) -> Result<TiaTagTable, String> {
    let mut csv = csv::ReaderBuilder::new().flexible(true).from_reader(reader);
    let headers = csv.headers().map_err(|e| e.to_string())?.clone();
    let find = |names: &[&str]| {
        headers
            .iter()
            .position(|h| names.iter().any(|n| h.eq_ignore_ascii_case(n)))
    };
    let name_i =
        find(&["name", "tag", "tagname", "nome"]).ok_or("CSV precisa de uma coluna Name/Tag")?;
    let type_i = find(&["datatype", "data type", "tipo"]);
    let addr_i = find(&["address", "logicaladdress", "logical address", "endereço", "endereco"]);
    let comment_i = find(&["comment", "comentário", "comentario"]);
    let mut tags = Vec::new();
    for row in csv.records() {
        let row = row.map_err(|e| e.to_string())?;
        let name = row.get(name_i).unwrap_or_default().to_owned();
        if name.trim().is_empty() {
            continue;
        }
        tags.push(PlcTag {
            id: format!("{table_name}.{name}"),
            name,
            data_type: type_i
                .and_then(|i| row.get(i))
                .map(DataType::from_tia)
                .unwrap_or_default(),
            address: addr_i.and_then(|i| row.get(i)).map(str::to_owned).filter(|s| !s.is_empty()),
            comment: comment_i.and_then(|i| row.get(i)).map(str::to_owned).filter(|s| !s.is_empty()),
        });
    }
    Ok(TiaTagTable {
        plc: String::new(),
        id: table_name.into(),
        name: table_name.into(),
        tags,
        active: false,
    })
}

/// Reads the XLSX produced by TIA's tag table "Export" (columns Name, Data Type, Logical Address...).
pub fn parse_tag_table_xlsx(
    path: &std::path::Path,
    table_name: &str,
) -> Result<TiaTagTable, String> {
    use calamine::{Reader, open_workbook_auto};
    let mut workbook =
        open_workbook_auto(path).map_err(|e| format!("Não foi possível abrir XLSX: {e}"))?;
    let range = workbook
        .worksheet_range_at(0)
        .ok_or("A planilha não contém abas")?
        .map_err(|e| format!("Erro lendo planilha: {e}"))?;
    let mut rows = range.rows();
    let header: Vec<String> = rows
        .next()
        .ok_or("Planilha vazia")?
        .iter()
        .map(ToString::to_string)
        .collect();
    let find = |names: &[&str]| {
        header
            .iter()
            .position(|h| names.iter().any(|n| h.eq_ignore_ascii_case(n)))
    };
    let name_i =
        find(&["name", "tag", "tagname", "nome"]).ok_or("XLSX precisa de uma coluna Name/Tag")?;
    let type_i = find(&["datatype", "data type", "tipo"]);
    let addr_i = find(&["address", "logicaladdress", "logical address", "endereço", "endereco"]);
    let comment_i = find(&["comment", "comentário", "comentario"]);
    let tags = rows
        .filter_map(|row| {
            let name = row.get(name_i).map(ToString::to_string).unwrap_or_default();
            if name.trim().is_empty() {
                return None;
            }
            let cell = |i: Option<usize>| {
                i.and_then(|i| row.get(i))
                    .map(ToString::to_string)
                    .filter(|s| !s.is_empty())
            };
            Some(PlcTag {
                id: format!("{table_name}.{name}"),
                data_type: cell(type_i).map(|v| DataType::from_tia(&v)).unwrap_or_default(),
                address: cell(addr_i),
                comment: cell(comment_i),
                name,
            })
        })
        .collect();
    Ok(TiaTagTable {
        plc: String::new(),
        id: table_name.into(),
        name: table_name.into(),
        tags,
        active: false,
    })
}

/// Reads a DB source generated by TIA ("Gerar fonte a partir de blocos", `.db`/`.scl`),
/// including nested `Struct ... END_STRUCT` and the `S7_Optimized_Access` attribute.
/// UDT-typed members are resolved against `TYPE ... END_TYPE` definitions in the same file.
pub fn parse_db_source(source: &str, fallback_name: &str) -> (TiaDataBlock, Vec<String>) {
    let header = regex::Regex::new(r#"(?im)^\s*DATA_BLOCK\s+"?([^"\r\n]+)"?"#).expect("regex");
    let optimized_re =
        regex::Regex::new(r"(?i)S7_Optimized_Access\s*:=\s*'(TRUE|FALSE)'").expect("regex");
    let block_name = header
        .captures(source)
        .and_then(|c| c.get(1))
        .map(|m| m.as_str().trim().to_owned())
        .unwrap_or_else(|| fallback_name.to_owned());
    // DB sources default to optimized access in S7-1200/1500 projects.
    let db_part = source
        .find("DATA_BLOCK")
        .map(|i| &source[i..])
        .unwrap_or(source);
    let optimized = optimized_re
        .captures(db_part)
        .map(|c| c[1].eq_ignore_ascii_case("TRUE"))
        .unwrap_or(true);

    let mut udts = HashMap::new();
    let type_re = regex::Regex::new(r#"(?is)TYPE\s+"?([^"\r\n]+)"?(.*?)END_TYPE"#).expect("regex");
    for c in type_re.captures_iter(source) {
        udts.insert(c[1].trim().to_owned(), parse_scl_declarations(&c[2]));
    }
    let members = parse_scl_declarations(db_part);
    let mut diagnostics = Vec::new();
    let db = data_block_from_members(&block_name, None, optimized, &members, &udts, &mut diagnostics);
    (db, diagnostics)
}

/// Parses `VAR ... END_VAR` style declarations, stopping at `BEGIN`.
fn parse_scl_declarations(text: &str) -> Vec<Member> {
    let decl = regex::Regex::new(
        r#"^\s*"?([A-Za-z_][\w]*)"?\s*(?:\{[^}]*\})?\s*:\s*([^;:=]+?)\s*(?::=\s*[^;]*)?;?\s*(?://\s*(.*))?$"#,
    )
    .expect("regex");
    let mut stack: Vec<Member> = vec![Member::default()];
    let mut in_var = false;
    for raw in text.lines() {
        let line = raw.trim();
        let upper = line.to_ascii_uppercase();
        if upper.starts_with("BEGIN") {
            break;
        }
        if upper.starts_with("VAR") || upper.starts_with("STRUCT") && stack.len() == 1 && !line.contains(':') {
            in_var = true;
            continue;
        }
        if upper.starts_with("END_VAR") {
            in_var = false;
            continue;
        }
        if !in_var {
            continue;
        }
        if upper.starts_with("END_STRUCT") {
            if stack.len() > 1 {
                let done = stack.pop().expect("len checked");
                stack.last_mut().expect("root").children.push(done);
            }
            continue;
        }
        let Some(c) = decl.captures(line) else { continue };
        let datatype = c[2].trim().to_owned();
        let member = Member {
            name: c[1].to_owned(),
            comment: c.get(3).map(|m| m.as_str().trim().to_owned()),
            datatype: datatype.clone(),
            children: Vec::new(),
        };
        let upper_type = datatype.to_ascii_uppercase();
        if upper_type == "STRUCT" || upper_type.ends_with(" OF STRUCT") {
            stack.push(member);
        } else {
            stack.last_mut().expect("root").children.push(member);
        }
    }
    while stack.len() > 1 {
        let done = stack.pop().expect("len checked");
        stack.last_mut().expect("root").children.push(done);
    }
    stack.pop().map(|root| root.children).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_typical_tia_tag_table_xml() {
        let xml = br#"<TagTable name="Paletizacao"><Tag><Name>Motor</Name><DataType>Bool</DataType><LogicalAddress>%Q0.0</LogicalAddress></Tag></TagTable>"#;
        let table = parse_tag_table_xml(std::io::Cursor::new(xml), None).unwrap();
        assert_eq!(table.name, "Paletizacao");
        assert_eq!(table.tags[0].id, "Paletizacao.Motor");
        assert!(matches!(table.tags[0].data_type, DataType::Bool));
        assert_eq!(table.tags[0].address.as_deref(), Some("%Q0.0"));
    }

    /// `PDT_BRIDGE_JSON=out.json cargo test -- --ignored --nocapture real_bridge_output`
    #[test]
    #[ignore]
    fn real_bridge_output() {
        let path = std::env::var("PDT_BRIDGE_JSON").expect("defina PDT_BRIDGE_JSON");
        let text = std::fs::read_to_string(path).unwrap();
        let project = project_from_bridge(serde_json::from_str(&text).unwrap());
        let table_tags: usize = project.tag_tables.iter().map(|t| t.tags.len()).sum();
        println!("tabelas={} tags={table_tags} dbs={}", project.tag_tables.len(), project.data_blocks.len());
        for db in &project.data_blocks {
            let with_address = db.tags.iter().filter(|t| t.address.is_some()).count();
            println!("  {} DB{:?} opt={} tags={} com_endereço={}", db.name, db.number, db.optimized, db.tags.len(), with_address);
        }
        for d in project.diagnostics.iter().take(25) {
            println!("  aviso: {d}");
        }
        if let Some(db) = project.data_blocks.iter().find(|d| !d.optimized && !d.tags.is_empty()) {
            for tag in db.tags.iter().take(12) {
                println!("    {} {:?} {:?}", tag.name, tag.data_type, db.absolute_address(tag));
            }
        }
        assert!(!project.data_blocks.is_empty());
    }

    /// `PDT_AP18=C:\curto\proj.ap18 cargo test -- --ignored --nocapture openness_end_to_end`
    #[test]
    #[ignore]
    fn openness_end_to_end() {
        let project_path = std::env::var("PDT_AP18").expect("defina PDT_AP18");
        let helper = std::path::Path::new("tools/TiaOpennessBridge/bin/Release/net48/TiaOpennessBridge.exe");
        let project = import_project_via_openness(helper, std::path::Path::new(&project_path)).unwrap();
        println!(
            "{}: {} tabelas, {} DBs, {} avisos",
            project.name,
            project.tag_tables.len(),
            project.data_blocks.len(),
            project.diagnostics.len()
        );
        assert!(!project.data_blocks.is_empty());
    }

    #[test]
    fn reads_db_source_with_struct_and_offsets() {
        let src = r#"DATA_BLOCK "DB_Celula"
{ S7_Optimized_Access := 'FALSE' }
VERSION : 0.1
NON_RETAIN
   VAR
      Esteira1_Motor : Bool;   // comando
      Esteira2_Motor : Bool;
      Robo : Struct
         Iniciar : Bool;
         Contador : Int;
      END_STRUCT;
      Velocidade : Real := 0.5;
   END_VAR
BEGIN
END_DATA_BLOCK
"#;
        let (mut db, diags) = parse_db_source(src, "x");
        assert!(diags.is_empty(), "{diags:?}");
        assert_eq!(db.name, "DB_Celula");
        assert!(!db.optimized);
        db.number = Some(10);
        let addr: Vec<_> = db.tags.iter().map(|t| (t.name.clone(), db.absolute_address(t).unwrap())).collect();
        assert_eq!(
            addr,
            vec![
                ("Esteira1_Motor".into(), "%DB10.DBX0.0".into()),
                ("Esteira2_Motor".into(), "%DB10.DBX0.1".into()),
                ("Robo.Iniciar".into(), "%DB10.DBX2.0".into()),
                ("Robo.Contador".into(), "%DB10.DBW4".into()),
                ("Velocidade".into(), "%DB10.DBD6".into()),
            ]
        );
        assert_eq!(db.tags[0].comment.as_deref(), Some("comando"));
    }
}
