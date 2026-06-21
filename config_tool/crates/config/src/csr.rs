//! Dynamic CSR map loaded from the LiteX-generated `csr.csv` or `csr.json`.
//!
//! Addresses are **never** hard-coded; the tool reads whichever map file the
//! gateware build emitted (`litex_soc/build/<target>/csr.{csv,json}`) so the
//! register layout always tracks the gateware it talks to.

use std::collections::BTreeMap;
use std::path::Path;

use serde::Deserialize;

use crate::ConfigError;

/// Access mode of a register, as reported by LiteX.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    ReadOnly,
    ReadWrite,
    Unknown,
}

impl Access {
    pub fn writable(self) -> bool {
        matches!(self, Access::ReadWrite)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Access::ReadOnly => "ro",
            Access::ReadWrite => "rw",
            Access::Unknown => "??",
        }
    }
}

impl From<&str> for Access {
    fn from(s: &str) -> Self {
        match s.trim() {
            "ro" => Access::ReadOnly,
            "rw" => Access::ReadWrite,
            _ => Access::Unknown,
        }
    }
}

/// A single CSR register entry.
#[derive(Debug, Clone)]
pub struct Register {
    pub name: String,
    /// Absolute Wishbone **byte** address of the (first) sub-register.
    pub addr: u32,
    /// Number of 32-bit sub-registers (CSRs wider than the bus span several).
    pub size: u32,
    pub access: Access,
}

/// The full register map: names → [`Register`], plus memory regions.
#[derive(Debug, Default, Clone)]
pub struct CsrMap {
    registers: BTreeMap<String, Register>,
    regions: BTreeMap<String, (u32, u32)>, // name → (base, size)
}

impl CsrMap {
    /// Load a map, picking the parser from the file extension (`.json` → JSON,
    /// anything else → CSV).
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Self, ConfigError> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)
            .map_err(|e| ConfigError::Io(format!("{}: {e}", path.display())))?;
        match path.extension().and_then(|e| e.to_str()) {
            Some("json") => Self::from_json(&text),
            _ => Self::from_csv(&text),
        }
    }

    /// Parse a LiteX `csr.csv`.
    ///
    /// Relevant row kinds:
    ///   `csr_register,<name>,<addr>,<size>,<rw|ro>`
    ///   `memory_region,<name>,<base>,<size>,<type>`
    pub fn from_csv(text: &str) -> Result<Self, ConfigError> {
        let mut map = CsrMap::default();
        for (lineno, line) in text.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let f: Vec<&str> = line.split(',').collect();
            match f.first().copied() {
                Some("csr_register") if f.len() >= 5 => {
                    let name = f[1].to_string();
                    let addr = parse_u32(f[2]).map_err(|e| {
                        ConfigError::Parse(format!("csr.csv line {}: {e}", lineno + 1))
                    })?;
                    let size = parse_u32(f[3]).unwrap_or(1);
                    map.registers.insert(
                        name.clone(),
                        Register { name, addr, size, access: Access::from(f[4]) },
                    );
                }
                Some("memory_region") if f.len() >= 4 => {
                    let base = parse_u32(f[2]).unwrap_or(0);
                    let size = parse_u32(f[3]).unwrap_or(0);
                    map.regions.insert(f[1].to_string(), (base, size));
                }
                _ => {}
            }
        }
        map.validate()
    }

    /// Parse a LiteX `csr.json`.
    pub fn from_json(text: &str) -> Result<Self, ConfigError> {
        #[derive(Deserialize)]
        struct RawReg {
            addr: u32,
            size: u32,
            #[serde(rename = "type")]
            ty: String,
        }
        #[derive(Deserialize)]
        struct RawRegion {
            base: u32,
            size: u32,
        }
        #[derive(Deserialize)]
        struct Raw {
            #[serde(default)]
            csr_registers: BTreeMap<String, RawReg>,
            #[serde(default)]
            memories: BTreeMap<String, RawRegion>,
        }

        let raw: Raw = serde_json::from_str(text)
            .map_err(|e| ConfigError::Parse(format!("csr.json: {e}")))?;
        let mut map = CsrMap::default();
        for (name, r) in raw.csr_registers {
            map.registers.insert(
                name.clone(),
                Register { name, addr: r.addr, size: r.size, access: Access::from(r.ty.as_str()) },
            );
        }
        for (name, r) in raw.memories {
            map.regions.insert(name, (r.base, r.size));
        }
        map.validate()
    }

    fn validate(self) -> Result<Self, ConfigError> {
        if self.registers.is_empty() {
            return Err(ConfigError::Parse(
                "CSR map contains no registers — wrong file? (expected an aes67_bridge csr.csv/json)"
                    .into(),
            ));
        }
        Ok(self)
    }

    /// Look up a register by exact name.
    pub fn get(&self, name: &str) -> Option<&Register> {
        self.registers.get(name)
    }

    /// All registers, ordered by name.
    pub fn registers(&self) -> impl Iterator<Item = &Register> {
        self.registers.values()
    }

    /// Memory regions, ordered by name (name → (base, size)).
    pub fn regions(&self) -> impl Iterator<Item = (&String, &(u32, u32))> {
        self.regions.iter()
    }

    /// Look up a memory region by name, returning `(base, size)` in bytes.
    pub fn region(&self, name: &str) -> Option<(u32, u32)> {
        self.regions.get(name).copied()
    }

    /// Number of registers in the map.
    pub fn len(&self) -> usize {
        self.registers.len()
    }

    pub fn is_empty(&self) -> bool {
        self.registers.is_empty()
    }
}

/// Parse an unsigned 32-bit integer accepting `0x`/`0b`/`0o` prefixes and plain
/// decimal. LiteX `csr.csv` uses `0x…`; `csr.json` uses decimal.
pub fn parse_u32(s: &str) -> Result<u32, String> {
    let s = s.trim();
    let parsed = if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(h, 16)
    } else if let Some(b) = s.strip_prefix("0b").or_else(|| s.strip_prefix("0B")) {
        u32::from_str_radix(b, 2)
    } else if let Some(o) = s.strip_prefix("0o").or_else(|| s.strip_prefix("0O")) {
        u32::from_str_radix(o, 8)
    } else {
        s.parse::<u32>()
    };
    parsed.map_err(|e| format!("invalid number '{s}': {e}"))
}
