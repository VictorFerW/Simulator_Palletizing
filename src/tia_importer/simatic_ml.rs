//! Reader for SimaticML block/type exports (`PlcBlock.Export` / `PlcType.Export`
//! in Openness). Namespaces change between TIA versions, so elements are
//! matched by local name only.

use quick_xml::{Reader, events::Event};

use super::Member;

#[derive(Debug, Clone, Default)]
pub struct ParsedBlock {
    pub name: String,
    pub number: Option<u32>,
    pub optimized: bool,
    pub members: Vec<Member>,
}

/// Interface sections that occupy DB memory (instance DBs include the FB's parameters).
const DATA_SECTIONS: [&str; 5] = ["Input", "Output", "InOut", "Static", "None"];

pub fn parse_simatic_ml(xml: &str) -> Result<ParsedBlock, String> {
    let mut reader = Reader::from_str(xml);
    reader.config_mut().trim_text(true);
    let mut parsed = ParsedBlock {
        optimized: true,
        ..Default::default()
    };
    let mut path: Vec<String> = Vec::new();
    // Members currently open; index 0 is a virtual root collecting top-level members.
    let mut members: Vec<Member> = vec![Member::default()];
    let mut section_included = false;
    let mut found_block = false;

    loop {
        let event = reader.read_event().map_err(|e| format!("XML inválido: {e}"))?;
        match event {
            Event::Start(e) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).into_owned();
                if name.starts_with("SW.Blocks.") || name.starts_with("SW.Types.") {
                    found_block = true;
                }
                if name == "Section" {
                    let section = attr(&e, "Name").unwrap_or_default();
                    section_included = DATA_SECTIONS.contains(&section.as_str());
                }
                if name == "Member" && section_included {
                    members.push(Member {
                        name: attr(&e, "Name").unwrap_or_default(),
                        datatype: attr(&e, "Datatype").unwrap_or_default(),
                        ..Default::default()
                    });
                }
                path.push(name);
            }
            Event::Empty(e) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).into_owned();
                if name == "Member" && section_included {
                    let member = Member {
                        name: attr(&e, "Name").unwrap_or_default(),
                        datatype: attr(&e, "Datatype").unwrap_or_default(),
                        ..Default::default()
                    };
                    members.last_mut().expect("root").children.push(member);
                }
            }
            Event::Text(t) => {
                let text = t.decode().map_err(|e| e.to_string())?.trim().to_owned();
                if text.is_empty() {
                    continue;
                }
                let n = path.len();
                let parent = |i: usize| path.get(n.wrapping_sub(i)).map(String::as_str);
                // Block attributes: <SW.Blocks.GlobalDB><AttributeList><Name>..</Name>
                if parent(2) == Some("AttributeList")
                    && parent(3).is_some_and(|p| p.starts_with("SW."))
                {
                    match parent(1) {
                        Some("Name") => parsed.name = text,
                        Some("Number") => parsed.number = text.parse().ok(),
                        Some("MemoryLayout") => parsed.optimized = text.eq_ignore_ascii_case("Optimized"),
                        _ => {}
                    }
                } else if parent(1) == Some("MultiLanguageText")
                    && parent(2) == Some("Comment")
                    && parent(3) == Some("Member")
                    && members.len() > 1
                {
                    let current = members.last_mut().expect("len checked");
                    if current.comment.is_none() {
                        current.comment = Some(text);
                    }
                }
            }
            Event::End(e) => {
                let name = String::from_utf8_lossy(e.local_name().as_ref()).into_owned();
                if name == "Member" && section_included && members.len() > 1 {
                    let done = members.pop().expect("len checked");
                    members.last_mut().expect("root").children.push(done);
                }
                if name == "Section" && members.len() == 1 {
                    section_included = false;
                }
                path.pop();
            }
            Event::Eof => break,
            _ => {}
        }
    }
    if !found_block {
        return Err("arquivo não é um export SimaticML de bloco/tipo".into());
    }
    parsed.members = members.swap_remove(0).children;
    Ok(parsed)
}

fn attr(e: &quick_xml::events::BytesStart, key: &str) -> Option<String> {
    e.attributes()
        .flatten()
        .find(|a| a.key.local_name().as_ref() == key.as_bytes())
        .and_then(|a| a.unescape_value().ok().map(|v| v.into_owned()))
}

#[cfg(test)]
mod tests {
    use super::*;

    const DB_XML: &str = r#"<?xml version="1.0" encoding="utf-8"?>
<Document>
  <SW.Blocks.GlobalDB ID="0">
    <AttributeList>
      <Interface><Sections xmlns="http://www.siemens.com/automation/Openness/SW/Interface/v5">
  <Section Name="Static">
    <Member Name="Motor" Datatype="Bool">
      <Comment><MultiLanguageText Lang="pt-BR">Liga esteira</MultiLanguageText></Comment>
    </Member>
    <Member Name="Robo" Datatype="Struct">
      <Member Name="Start" Datatype="Bool" />
      <Member Name="Count" Datatype="Int" />
    </Member>
    <Member Name="Udt" Datatype="&quot;UDT_X&quot;">
      <Sections><Section Name="None"><Member Name="A" Datatype="Real" /></Section></Sections>
    </Member>
  </Section>
</Sections></Interface>
      <MemoryLayout>Standard</MemoryLayout>
      <Name>DB_Celula</Name>
      <Number>7</Number>
    </AttributeList>
  </SW.Blocks.GlobalDB>
</Document>"#;

    #[test]
    fn reads_global_db() {
        let block = parse_simatic_ml(DB_XML).unwrap();
        assert_eq!(block.name, "DB_Celula");
        assert_eq!(block.number, Some(7));
        assert!(!block.optimized);
        assert_eq!(block.members.len(), 3);
        assert_eq!(block.members[0].comment.as_deref(), Some("Liga esteira"));
        assert_eq!(block.members[1].children.len(), 2);
        assert_eq!(block.members[2].datatype, "\"UDT_X\"");
        assert_eq!(block.members[2].children[0].name, "A");
    }
}
