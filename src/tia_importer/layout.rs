//! Offset calculation for *standard* (non-optimized) S7 data blocks.
//!
//! Rules of the S7 standard memory layout:
//! - Bool occupies one bit; consecutive Bools share a byte.
//! - Byte-sized types start on the next byte.
//! - Every type of 16 bits or more, Strings, Structs/UDTs and Arrays start on
//!   an even byte; Structs/UDTs and Arrays also end on an even byte.

use std::collections::HashMap;

use super::{DataType, Member, PlcTag};

/// Arrays larger than this are laid out but not expanded into individual tags.
const MAX_EXPANDED_ELEMENTS: usize = 256;

pub fn layout_members(
    block_name: &str,
    members: &[Member],
    udts: &HashMap<String, Vec<Member>>,
) -> (Vec<PlcTag>, Vec<String>) {
    let mut layouter = Layouter {
        block: block_name,
        udts,
        bits: 0,
        valid: true,
        emit: true,
        tags: Vec::new(),
        warnings: Vec::new(),
        depth: 0,
    };
    for member in members {
        layouter.place(&member.name, &member.datatype, &member.children, member.comment.as_deref());
    }
    (layouter.tags, layouter.warnings)
}

struct Layouter<'a> {
    block: &'a str,
    udts: &'a HashMap<String, Vec<Member>>,
    bits: u32,
    /// Becomes false after an element of unknown size: later offsets are unknown.
    valid: bool,
    emit: bool,
    tags: Vec<PlcTag>,
    warnings: Vec<String>,
    depth: u32,
}

impl Layouter<'_> {
    fn align(&mut self, bits: u32) {
        self.bits = self.bits.div_ceil(bits) * bits;
    }

    fn place(&mut self, path: &str, datatype: &str, children: &[Member], comment: Option<&str>) {
        self.depth += 1;
        if self.depth > 32 {
            self.invalidate(path, "aninhamento excessivo");
            self.depth -= 1;
            return;
        }
        let trimmed = datatype.trim();
        if let Some((dims, element)) = parse_array(trimmed) {
            self.align(16);
            let count: usize = dims.iter().map(|(lo, hi)| (hi - lo + 1).max(0) as usize).product();
            let saved_emit = self.emit;
            if count > MAX_EXPANDED_ELEMENTS {
                self.emit = false;
            }
            let mut index: Vec<i64> = dims.iter().map(|(lo, _)| *lo).collect();
            for _ in 0..count {
                let suffix = index.iter().map(i64::to_string).collect::<Vec<_>>().join(",");
                self.place(&format!("{path}[{suffix}]"), &element, children, comment);
                // Advance the multi-dimensional index, last dimension fastest.
                for d in (0..index.len()).rev() {
                    index[d] += 1;
                    if index[d] <= dims[d].1 {
                        break;
                    }
                    index[d] = dims[d].0;
                }
            }
            self.emit = saved_emit;
            self.align(16);
        } else if is_struct(trimmed, children) {
            let resolved: Vec<Member> = if !children.is_empty() {
                children.to_vec()
            } else {
                match self.udts.get(trimmed.trim_matches('"')) {
                    Some(m) => m.clone(),
                    None => {
                        self.invalidate(path, &format!("tipo '{trimmed}' não encontrado"));
                        self.depth -= 1;
                        return;
                    }
                }
            };
            self.align(16);
            for m in &resolved {
                self.place(&format!("{path}.{}", m.name), &m.datatype, &m.children, m.comment.as_deref());
            }
            self.align(16);
        } else if let Some(string_bytes) = string_size(trimmed) {
            self.align(16);
            self.push(path, DataType::String, Some(format!("DBB{}", self.bits / 8)), comment);
            self.bits += string_bytes * 8;
        } else {
            let data_type = DataType::from_tia(trimmed);
            let Some(size) = data_type.bits().or_else(|| other_scalar_bits(trimmed)) else {
                self.invalidate(path, &format!("tipo '{trimmed}' sem tamanho conhecido"));
                self.depth -= 1;
                return;
            };
            match size {
                1 => {}
                8 => self.align(8),
                _ => self.align(16),
            }
            let byte = self.bits / 8;
            let address = match size {
                1 => format!("DBX{}.{}", byte, self.bits % 8),
                8 => format!("DBB{byte}"),
                16 => format!("DBW{byte}"),
                32 => format!("DBD{byte}"),
                _ => format!("DBB{byte}"),
            };
            self.push(path, data_type, Some(address), comment);
            self.bits += size;
        }
        self.depth -= 1;
    }

    fn push(&mut self, path: &str, data_type: DataType, address: Option<String>, comment: Option<&str>) {
        if !self.emit {
            return;
        }
        self.tags.push(PlcTag {
            id: format!("{}.{path}", self.block),
            name: path.to_owned(),
            data_type,
            address: if self.valid { address } else { None },
            comment: comment.map(str::to_owned),
        });
    }

    fn invalidate(&mut self, path: &str, reason: &str) {
        if self.valid {
            self.warnings.push(format!(
                "{}: {path}: {reason}; endereços a partir daqui não puderam ser calculados",
                self.block
            ));
        }
        self.valid = false;
    }
}

fn is_struct(datatype: &str, children: &[Member]) -> bool {
    !children.is_empty() || datatype.eq_ignore_ascii_case("struct") || datatype.starts_with('"')
}

/// `Array[0..9, 1..2] of Int` -> ([(0,9),(1,2)], "Int")
fn parse_array(datatype: &str) -> Option<(Vec<(i64, i64)>, String)> {
    let upper = datatype.to_ascii_uppercase();
    if !upper.starts_with("ARRAY") {
        return None;
    }
    let open = datatype.find('[')?;
    let close = datatype.find(']')?;
    let of = upper[close..].find(" OF ")? + close;
    let dims = datatype[open + 1..close]
        .split(',')
        .map(|d| {
            let (lo, hi) = d.split_once("..")?;
            Some((lo.trim().parse().ok()?, hi.trim().parse().ok()?))
        })
        .collect::<Option<Vec<_>>>()?;
    Some((dims, datatype[of + 4..].trim().to_owned()))
}

/// Total bytes of `String[n]` / `WString[n]` including the header.
fn string_size(datatype: &str) -> Option<u32> {
    let upper = datatype.to_ascii_uppercase();
    let (wide, rest) = if let Some(rest) = upper.strip_prefix("WSTRING") {
        (true, rest)
    } else {
        let rest = upper.strip_prefix("STRING")?;
        (false, rest)
    };
    let len: u32 = rest
        .trim()
        .strip_prefix('[')
        .and_then(|r| r.strip_suffix(']'))
        .and_then(|n| n.trim().parse().ok())
        .unwrap_or(254);
    Some(if wide { len * 2 + 4 } else { len + 2 })
}

fn other_scalar_bits(datatype: &str) -> Option<u32> {
    let upper = datatype.to_ascii_uppercase();
    // System data types for hardware identifiers, DB/OB numbers and connections are Words.
    if upper.starts_with("HW_") || upper.starts_with("OB_") || upper.starts_with("DB_") {
        return Some(16);
    }
    Some(match upper.as_str() {
        "WCHAR" | "DATE" | "S5TIME" | "PORT" | "RTM" | "PIP" | "CONN_ANY" | "CONN_OUC" | "CONN_PRG" => 16,
        "TIME_OF_DAY" | "TOD" | "CONN_R_ID" | "EVENT_ANY" | "EVENT_ATT" | "EVENT_HWINT" => 32,
        "LTIME" | "LTOD" | "LTIME_OF_DAY" | "LDT" | "DATE_AND_TIME" | "DT" => 64,
        "DTL" => 96,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn m(name: &str, ty: &str) -> Member {
        Member {
            name: name.into(),
            datatype: ty.into(),
            ..Default::default()
        }
    }

    #[test]
    fn bool_array_then_int() {
        let members = vec![m("Bits", "Array[0..9] of Bool"), m("Valor", "Int"), m("B", "Byte"), m("S", "String[10]"), m("R", "Real")];
        let (tags, warnings) = layout_members("DB", &members, &HashMap::new());
        assert!(warnings.is_empty());
        let addr = |n: &str| tags.iter().find(|t| t.name == n).unwrap().address.clone().unwrap();
        assert_eq!(addr("Bits[0]"), "DBX0.0");
        assert_eq!(addr("Bits[9]"), "DBX1.1");
        assert_eq!(addr("Valor"), "DBW2");
        assert_eq!(addr("B"), "DBB4");
        assert_eq!(addr("S"), "DBB6");
        assert_eq!(addr("R"), "DBD18");
    }

    #[test]
    fn udt_resolution() {
        let mut udts = HashMap::new();
        udts.insert("UDT_Motor".to_string(), vec![m("Liga", "Bool"), m("Vel", "Real")]);
        let members = vec![m("Flag", "Bool"), m("M1", "\"UDT_Motor\""), m("M2", "\"UDT_Motor\"")];
        let (tags, _) = layout_members("DB", &members, &udts);
        let addr = |n: &str| tags.iter().find(|t| t.name == n).unwrap().address.clone().unwrap();
        assert_eq!(addr("M1.Liga"), "DBX2.0");
        assert_eq!(addr("M1.Vel"), "DBD4");
        assert_eq!(addr("M2.Liga"), "DBX8.0");
    }
}
