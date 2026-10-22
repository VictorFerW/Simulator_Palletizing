//! Minimal S7comm client (ISO-on-TCP, port 102): connect, read and write
//! process image (I/Q), merker (M) and data blocks (DB) by absolute address.
//!
//! Works with S7-1200/1500 (and PLCSIM Advanced in TCP/IP mode) when
//! "Permit access with PUT/GET communication" is enabled in the CPU and the
//! DBs used have "Optimized block access" disabled.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::tia_importer::DataType;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Area {
    Input,
    Output,
    Merker,
    Db(u16),
}

impl Area {
    fn code(self) -> u8 {
        match self {
            Self::Input => 0x81,
            Self::Output => 0x82,
            Self::Merker => 0x83,
            Self::Db(_) => 0x84,
        }
    }

    fn db_number(self) -> u16 {
        match self {
            Self::Db(n) => n,
            _ => 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct S7Address {
    pub area: Area,
    pub byte: u32,
    pub bit: u8,
    /// Width in bits: 1, 8, 16, 32 or 64.
    pub width: u32,
}

impl S7Address {
    pub fn byte_len(&self) -> u32 {
        self.width.div_ceil(8)
    }

    /// Parses TIA notation: `%I0.0`, `%QW2`, `%MD20`, `%DB5.DBX2.1`, `%DB5.DBW4`,
    /// German mnemonics `%E0.0`/`%A0.0`, and `P`-suffixed peripheral forms (`%I0.0:P`).
    /// `data_type` widens byte addresses for 64-bit types.
    pub fn parse(text: &str, data_type: &DataType) -> Option<Self> {
        let upper = text.trim().trim_start_matches('%').to_ascii_uppercase();
        let upper = upper.split(':').next()?.to_owned();
        let (area, rest) = if let Some(rest) = upper.strip_prefix("DB") {
            let (number, rest) = rest.split_once('.')?;
            let rest = rest.strip_prefix("DB")?;
            (Area::Db(number.parse().ok()?), rest.to_owned())
        } else {
            let mut chars = upper.chars();
            let area = match chars.next()? {
                'I' | 'E' => Area::Input,
                'Q' | 'A' => Area::Output,
                'M' => Area::Merker,
                _ => return None,
            };
            (area, chars.as_str().to_owned())
        };
        let (width, offset) = match rest.chars().next()? {
            'X' => (1, &rest[1..]),
            'B' => (8, &rest[1..]),
            'W' => (16, &rest[1..]),
            'D' => (32, &rest[1..]),
            c if c.is_ascii_digit() => (1, rest.as_str()),
            _ => return None,
        };
        let (byte, bit) = if width == 1 {
            let (b, bit) = offset.split_once('.')?;
            (b.parse().ok()?, bit.parse().ok()?)
        } else {
            (offset.parse().ok()?, 0)
        };
        if bit > 7 {
            return None;
        }
        let width = match data_type.bits() {
            Some(64) if width != 1 => 64,
            _ => width,
        };
        Some(Self { area, byte, bit, width })
    }

    /// Converts raw big-endian bytes (starting at `self.byte`) to a number.
    pub fn decode(&self, bytes: &[u8], data_type: &DataType) -> Option<f64> {
        let n = self.byte_len() as usize;
        let raw = bytes.get(..n)?;
        Some(match self.width {
            1 => ((raw[0] >> self.bit) & 1) as f64,
            8 => match data_type {
                DataType::SInt => raw[0] as i8 as f64,
                _ => raw[0] as f64,
            },
            16 => {
                let v = u16::from_be_bytes([raw[0], raw[1]]);
                match data_type {
                    DataType::Int => v as i16 as f64,
                    _ => v as f64,
                }
            }
            32 => {
                let v = u32::from_be_bytes([raw[0], raw[1], raw[2], raw[3]]);
                match data_type {
                    DataType::Real => f32::from_bits(v) as f64,
                    DataType::DInt | DataType::Time => v as i32 as f64,
                    _ => v as f64,
                }
            }
            _ => {
                let v = u64::from_be_bytes(raw.try_into().ok()?);
                match data_type {
                    DataType::LReal => f64::from_bits(v),
                    DataType::LInt => v as i64 as f64,
                    _ => v as f64,
                }
            }
        })
    }

    pub fn encode(&self, value: f64, data_type: &DataType) -> Vec<u8> {
        match self.width {
            1 => vec![u8::from(value >= 0.5)],
            8 => vec![value.round().clamp(-128.0, 255.0) as i64 as u8],
            16 => (value.round() as i64 as u16).to_be_bytes().to_vec(),
            32 => match data_type {
                DataType::Real => (value as f32).to_bits().to_be_bytes().to_vec(),
                _ => (value.round() as i64 as u32).to_be_bytes().to_vec(),
            },
            _ => match data_type {
                DataType::LReal => value.to_bits().to_be_bytes().to_vec(),
                _ => (value.round() as i64).to_be_bytes().to_vec(),
            },
        }
    }
}

pub struct S7Client {
    stream: TcpStream,
    pdu_size: usize,
    pdu_ref: u16,
}

const TIMEOUT: Duration = Duration::from_secs(3);

impl S7Client {
    pub fn connect(host: &str, rack: u8, slot: u8) -> io::Result<Self> {
        let addr: SocketAddr = (host, 102)
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "endereço inválido"))?;
        let stream = TcpStream::connect_timeout(&addr, TIMEOUT)?;
        stream.set_read_timeout(Some(TIMEOUT))?;
        stream.set_write_timeout(Some(TIMEOUT))?;
        stream.set_nodelay(true)?;
        let mut client = Self {
            stream,
            pdu_size: 240,
            pdu_ref: 0,
        };
        // COTP connection request; remote TSAP = PG connection (0x01) + rack/slot.
        let remote_tsap = [0x01, rack * 0x20 + slot];
        let cr = [
            0x11, 0xE0, 0x00, 0x00, 0x00, 0x01, 0x00, // CR header, class 0
            0xC0, 0x01, 0x0A, // TPDU size 1024
            0xC1, 0x02, 0x01, 0x00, // calling TSAP
            0xC2, 0x02, remote_tsap[0], remote_tsap[1], // called TSAP
        ];
        client.send_tpkt(&cr)?;
        let reply = client.recv_tpkt()?;
        if reply.get(1) != Some(&0xD0) {
            return Err(proto("CPU recusou a conexão COTP (verifique rack/slot)"));
        }
        // S7 "setup communication".
        let params = [0xF0, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0xE0];
        let reply = client.job(&params, &[])?;
        let pdu = reply
            .params
            .get(6..8)
            .map(|b| u16::from_be_bytes([b[0], b[1]]) as usize)
            .ok_or_else(|| proto("resposta de setup inválida"))?;
        client.pdu_size = pdu.clamp(240, 960);
        Ok(client)
    }

    pub fn read(&mut self, area: Area, start: u32, len: usize) -> io::Result<Vec<u8>> {
        let mut out = Vec::with_capacity(len);
        let chunk_max = self.pdu_size - 18;
        while out.len() < len {
            let chunk = (len - out.len()).min(chunk_max);
            let offset = start + out.len() as u32;
            let mut params = vec![0x04, 0x01];
            params.extend(item(area, 0x02, chunk as u16, offset * 8));
            let reply = self.job(&params, &[])?;
            let data = reply.data;
            if data.first() != Some(&0xFF) {
                return Err(proto(&format!(
                    "leitura recusada (código 0x{:02X}) em {area:?} byte {offset}: verifique PUT/GET, se o DB existe e se está com acesso otimizado desativado",
                    data.first().copied().unwrap_or(0)
                )));
            }
            let payload = data.get(4..4 + chunk).ok_or_else(|| proto("resposta de leitura curta"))?;
            out.extend_from_slice(payload);
        }
        Ok(out)
    }

    pub fn write_bytes(&mut self, area: Area, start: u32, bytes: &[u8]) -> io::Result<()> {
        let mut params = vec![0x05, 0x01];
        params.extend(item(area, 0x02, bytes.len() as u16, start * 8));
        let bits = (bytes.len() * 8) as u16;
        let mut data = vec![0x00, 0x04, (bits >> 8) as u8, bits as u8];
        data.extend_from_slice(bytes);
        self.write_job(&params, &data)
    }

    pub fn write_bit(&mut self, area: Area, byte: u32, bit: u8, value: bool) -> io::Result<()> {
        let mut params = vec![0x05, 0x01];
        params.extend(item(area, 0x01, 1, byte * 8 + bit as u32));
        let data = [0x00, 0x03, 0x00, 0x01, u8::from(value)];
        self.write_job(&params, &data)
    }

    pub fn write(&mut self, address: &S7Address, bytes: &[u8]) -> io::Result<()> {
        if address.width == 1 {
            self.write_bit(address.area, address.byte, address.bit, bytes.first().is_some_and(|b| *b != 0))
        } else {
            self.write_bytes(address.area, address.byte, bytes)
        }
    }

    fn write_job(&mut self, params: &[u8], data: &[u8]) -> io::Result<()> {
        let reply = self.job(params, data)?;
        match reply.data.first() {
            Some(0xFF) => Ok(()),
            code => Err(proto(&format!(
                "escrita recusada (código 0x{:02X})",
                code.copied().unwrap_or(0)
            ))),
        }
    }

    fn job(&mut self, params: &[u8], data: &[u8]) -> io::Result<S7Reply> {
        self.pdu_ref = self.pdu_ref.wrapping_add(1);
        let mut frame = vec![0x02, 0xF0, 0x80, 0x32, 0x01, 0x00, 0x00];
        frame.extend(self.pdu_ref.to_be_bytes());
        frame.extend((params.len() as u16).to_be_bytes());
        frame.extend((data.len() as u16).to_be_bytes());
        frame.extend_from_slice(params);
        frame.extend_from_slice(data);
        self.send_tpkt(&frame)?;
        let reply = self.recv_tpkt()?;
        // reply = COTP DT (3) + S7 header (12 for ack-data) + params + data
        if reply.len() < 15 || reply[3] != 0x32 {
            return Err(proto("resposta S7 inválida"));
        }
        let error = u16::from_be_bytes([reply[13], reply[14]]);
        if error != 0 {
            return Err(proto(&format!("erro S7 0x{error:04X}")));
        }
        let param_len = u16::from_be_bytes([reply[9], reply[10]]) as usize;
        let data_len = u16::from_be_bytes([reply[11], reply[12]]) as usize;
        let params = reply.get(15..15 + param_len).unwrap_or_default().to_vec();
        let data = reply
            .get(15 + param_len..15 + param_len + data_len)
            .unwrap_or_default()
            .to_vec();
        Ok(S7Reply { params, data })
    }

    fn send_tpkt(&mut self, payload: &[u8]) -> io::Result<()> {
        let len = (payload.len() + 4) as u16;
        let mut frame = vec![0x03, 0x00, (len >> 8) as u8, len as u8];
        frame.extend_from_slice(payload);
        self.stream.write_all(&frame)
    }

    fn recv_tpkt(&mut self) -> io::Result<Vec<u8>> {
        let mut header = [0u8; 4];
        self.stream.read_exact(&mut header)?;
        let len = u16::from_be_bytes([header[2], header[3]]) as usize;
        if len < 7 {
            return Err(proto("TPKT curto"));
        }
        let mut payload = vec![0u8; len - 4];
        self.stream.read_exact(&mut payload)?;
        Ok(payload)
    }
}

struct S7Reply {
    params: Vec<u8>,
    data: Vec<u8>,
}

fn item(area: Area, transport: u8, count: u16, bit_address: u32) -> Vec<u8> {
    let db = area.db_number();
    vec![
        0x12,
        0x0A,
        0x10,
        transport,
        (count >> 8) as u8,
        count as u8,
        (db >> 8) as u8,
        db as u8,
        area.code(),
        (bit_address >> 16) as u8,
        (bit_address >> 8) as u8,
        bit_address as u8,
    ]
}

fn proto(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_addresses() {
        let b = DataType::Bool;
        assert_eq!(
            S7Address::parse("%I0.3", &b),
            Some(S7Address { area: Area::Input, byte: 0, bit: 3, width: 1 })
        );
        assert_eq!(
            S7Address::parse("%DB12.DBX4.7", &b),
            Some(S7Address { area: Area::Db(12), byte: 4, bit: 7, width: 1 })
        );
        assert_eq!(
            S7Address::parse("%MD20", &DataType::Real),
            Some(S7Address { area: Area::Merker, byte: 20, bit: 0, width: 32 })
        );
        assert_eq!(S7Address::parse("%A1.0", &b).unwrap().area, Area::Output);
        assert_eq!(S7Address::parse("%QW8", &DataType::Int).unwrap().width, 16);
        assert!(S7Address::parse("%I0.8", &b).is_none());
    }

    #[test]
    fn codec_roundtrip() {
        let a = S7Address::parse("%DB1.DBD0", &DataType::Real).unwrap();
        let bytes = a.encode(1.5, &DataType::Real);
        assert_eq!(a.decode(&bytes, &DataType::Real), Some(1.5));
        let i = S7Address::parse("%MW2", &DataType::Int).unwrap();
        assert_eq!(i.decode(&i.encode(-3.0, &DataType::Int), &DataType::Int), Some(-3.0));
    }
}
