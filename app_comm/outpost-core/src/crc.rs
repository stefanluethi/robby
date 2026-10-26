//! CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, not reflected, no final XOR.

const fn make_table() -> [u16; 256] {
    let mut table = [0_u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

static TABLE: [u16; 256] = make_table();

/// The CRC of `data`, continued from `crc` (start with [`INIT`]).
pub fn update(mut crc: u16, data: &[u8]) -> u16 {
    for &byte in data {
        crc = (crc << 8) ^ TABLE[usize::from((crc >> 8) as u8 ^ byte)];
    }
    crc
}

pub const INIT: u16 = 0xFFFF;

/// The CRC of `data`.
pub fn crc16(data: &[u8]) -> u16 {
    update(INIT, data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_value() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn crc_over_payload_and_trailer_is_zero() {
        let crc = crc16(b"robby");
        let mut frame = *b"robby\0\0";
        frame[5..].copy_from_slice(&crc.to_be_bytes());
        assert_eq!(crc16(&frame), 0);
    }
}
