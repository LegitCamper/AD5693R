//! Functions for encoding messages to send to the AD569x

use crate::{CommandType, Mode};

/// Encode command type, mode and data into a three byte command
pub fn encode_command(command: CommandType, mode: Mode, data: u16) -> [u8; 3] {
    [
        command as u8 + ((mode as u8) << 1),
        (data >> 8) as u8,
        (data & 0x000f) as u8,
    ]
}
