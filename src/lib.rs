//! *Microchip AD569x DAC Driver for Rust Embedded HAL*
//! This is a driver crate for embedded Rust. It's built on top of the Rust
//! [embedded HAL](https://github.com/rust-embedded/embedded-hal)
//! It supports sending commands to a AD569x DAC over I2C.
//! To get started you can look at the
//! [examples](https://github.com/mendelt/mcp4725/tree/master/bluepill-examples/examples)
//! on how to use this driver on an inexpensive blue pill STM32F103 board.
//!
//! The driver can be initialized by calling create and passing it an I2C interface. The three least
//! significant bits of the device address (A2, A1 and A0) also need to be specified. A2 and A1 are
//! set in the device. A0 can be set by pulling the corresponding connection on the device high or
//! low.
//! ```
//! # use embedded_hal_mock::i2c::Mock;
//! # use mcp4725::*;
//! # let mut i2c = Mock::new(&[]);
//! let mut dac = AD569x::new(i2c, 0b010);
//! ```
//!
//! To set the dac output and powermode the dac register can be set;
//! ```
//! # use embedded_hal_mock::i2c::{Mock, Transaction};
//! # use mcp4725::*;
//! # let mut i2c = Mock::new(&[Transaction::write(98, vec![0x40, 0xff, 0xf0]),]);
//! # let mut dac = AD569x::new(i2c, 0b010);
//! dac.set_dac(PowerDown::Normal, 0x0fff);
//! ```
//!
//! The AD569x has a built in eeprom that is used to initialize the dac register on power up.
//! The values in the eeprom can be set with the `set_dac_and_eeprom` method;
//! ```
//! # use embedded_hal_mock::i2c::{Mock, Transaction};
//! # use mcp4725::*;
//! # let mut i2c = Mock::new(&[Transaction::write(98, vec![0x64, 0xff, 0xf0])]);
//! # let mut dac = AD569x::new(i2c, 0b010);
//! dac.set_dac_and_eeprom(PowerDown::Resistor100kOhm, 0x0fff);
//! ```
//!
//! ## More information
//! - [AD569x datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/22039d.pdf)
//! - [API documentation](https://docs.rs/mcp4725/)
//! - [Github repository](https://github.com/mendelt/mcp4725)
//! - [Crates.io](https://crates.io/crates/mcp4725)
//!
#![no_std]
#![warn(missing_debug_implementations, missing_docs)]

mod encode;
mod status;

use core::fmt::Debug;
use embedded_hal::blocking::i2c::{Read, Write};
use encode::encode_command;

/// AD569x DAC driver. Wraps an I2C port to send commands to an AD569x
#[derive(Debug)]
pub struct AD569x<I2C>
where
    I2C: Read + Write,
{
    i2c: I2C,
    mode: Option<Mode>,
    address: u8,
}

impl<I2C, E> AD569x<I2C>
where
    I2C: Read<Error = E> + Write<Error = E>,
{
    /// Construct a new AD569x driver instance.
    /// i2c is the ixitialized i2c driver port to use,
    /// user_address is the three bit user-part of the i2c address where the AD569x can be reached
    ///   - The least significant bit of this address can be set externally by pulling the A0 leg of
    ///     the chip low (0) or high (1)
    ///   The two most significant bits are set in the factory. There are four variants of the chip
    ///     with different addresses.
    pub fn new(i2c: I2C, user_address: u8) -> Result<Self, E> {
        let mut dac = AD569x {
            i2c,
            mode: None,
            address: user_address,
        };

        dac.set_mode(Mode::Normal, true, false);
        Ok(dac)
    }

    pub fn set_mode(&mut self, mode: Mode, enable_ref: bool, gain2x: bool) -> Result<(), E> {
        self.mode = Some(mode);

        let mut data = 0x0000;
        data |= (mode as u16) << 13; // Set bits 14-13 for operating mode
        data |= (!enable_ref as u16) << 12; // Set bit 12 for enable_ref
        data |= (gain2x as u16) << 11; // Set bit 11 for gain

        self.write(CommandType::WriteControl, data)
    }

    pub fn write_update(&mut self, data: u16) -> Result<(), E> {
        self.write(CommandType::WriteDacAndInput, data)
    }

    pub fn write_dac(&mut self, data: u16) -> Result<(), E> {
        self.write(CommandType::WriteInput, data)
    }

    pub fn update_dac(&mut self, data: u16) -> Result<(), E> {
        self.write(CommandType::UpdateDac, data)
    }

    fn write(&mut self, command: CommandType, data: u16) -> Result<(), E> {
        let bytes = encode_command(command, self.mode.unwrap(), data);
        self.i2c.write(self.address, &bytes)
    }

    /// Send a reset command on the I2C bus.
    /// WARNING: This is a general call command and can reset other devices on the bus as well.
    pub fn reset(&mut self) -> Result<(), E> {
        self.i2c.write(0x00, &[0x09u8])?;
        Ok(())
    }

    /// Destroy the AD569x driver, return the wrapped I2C
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

/// Sets the output mode
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum Mode {
    /// Normal mode
    Normal = 0x00,
    /// Power down mode, set outputs to 1k ohm to ground
    Resistor1kOhm = 0x01,
    /// Power down mode, set outputs to 100k ohm to ground
    Resistor100kOhm = 0x10,
    /// Tristate output mode
    OutputTristate = 0x03,
}

/// The type of the command to send for a Command
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
enum CommandType {
    /// Write to the input register.
    WriteInput = 0x10,
    /// Write to the input register.
    UpdateDac = 0x20,
    /// Write to the input register and update the DAC register.
    WriteDacAndInput = 0x30,
    /// Write to the control register.
    WriteControl = 0x40,
}

/// A Command to send to the AD569x.
/// Using the address(), command_type(), power_mode() and data() builder methods the
/// parameters for this command can be set. Commands can be sent using the send method on the
/// AD569x driver.
/// A command can (and should) be re-used. data() can be used to re-set the data while keeping other
/// parameters the same.
#[derive(Debug, Eq, PartialEq)]
pub struct Command {
    command_byte: u8,
    data_byte_0: u8,
    data_byte_1: u8,
}

impl Default for Command {
    /// Instantiate a command with sane defaults.
    fn default() -> Self {
        Self {
            command_byte: CommandType::WriteDacAndInput as u8,
            data_byte_0: 0,
            data_byte_1: 0,
        }
    }
}
