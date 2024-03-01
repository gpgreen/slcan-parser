#![no_std]
//! # slcan_parser
//!
//! ## Features
//!
//! This crate parses can frames that are in Serial format as used by the Linux kernel. The
//! frames begin with one character that defines what type of CAN frame it is:
//! 't' - a standard id CAN frame
//! 'r' - a standard id RTR CAN frame
//! 'T' - an extended id CAN frame
//! 'R' - an extended id RTR CAN frame
//! Following this identifier, there are either 3 hex bytes for a standard id, or 8 hex bytes
//! for an extended id.
//! Then there is one hex byte specifying the data length in bytes. For CAN this can be 0 - 8 bytes
//! per frame.
//! Following the data length, the data section is a stream of hex bytes, each byte is a nibble of
//! the data, so that a byte is 2 hex characters.
//!
//! An example for a standard id CAN frame:
//! id: 0x23
//! data length: 2
//! data bytes: 0xa0 0xb0
//!
//! the bytes to represent this are:
//! t0232A0B0
//!
//! This crate defines a `FrameParser`` which is a state machine that parses
//! SlcanFrame's a byte at a time. This is intended for use with Serial Port
//! hardware. The parser is implemented with the `machine` crate.

#[macro_use]
extern crate machine;

// pull in library
mod parser;

use crate::parser::FrameParser;
use core::convert::TryFrom;
use embedded_hal::can::{Error, ErrorKind, Frame, Id, StandardId};

/// Errors that can be encountered in this crate
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SlcanError {
    /// Char received for binary field not a hexadecimal character
    NotAHexChar,
    /// Data length not within bounds
    DataLen(usize),
    /// Standard id value too large
    StandardIdOverflow,
}

impl Error for SlcanError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// Constrain a usize to values allowed for frame data length [0..8]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct FrameDataLen(usize);

impl FrameDataLen {
    /// Create a new `FrameDataLen``
    pub const fn new(value: usize) -> Result<Self, SlcanError> {
        if value <= 8 {
            Ok(Self(value))
        } else {
            Err(SlcanError::DataLen(value))
        }
    }
    /// Get the raw value of the FrameDataLen
    pub const fn raw(&self) -> usize {
        self.0
    }
}

impl TryFrom<usize> for FrameDataLen {
    type Error = SlcanError;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        if value <= 8 {
            Ok(Self(value))
        } else {
            Err(SlcanError::DataLen(value))
        }
    }
}

/// CanserialFrame
#[derive(Debug, Copy, Clone)]
pub struct CanserialFrame {
    rtr: bool,
    dlc: FrameDataLen,
    id: Id,
    data: [u8; 8],
}

impl CanserialFrame {
    /// an empty frame
    pub fn empty() -> Self {
        CanserialFrame {
            rtr: false,
            dlc: FrameDataLen::new(0).unwrap(),
            id: Id::Standard(StandardId::ZERO),
            data: [0; 8],
        }
    }

    /// a frame
    pub fn new_frame(can_id: impl Into<Id>, data: &[u8]) -> Result<Self, SlcanError> {
        let mut frame = CanserialFrame {
            rtr: false,
            dlc: FrameDataLen::try_from(data.len())?,
            id: can_id.into(),
            data: [0; 8],
        };
        frame.data[..frame.dlc.raw()].clone_from_slice(data);
        Ok(frame)
    }

    pub fn new_remote_frame(can_id: impl Into<Id>, dlc: usize) -> Result<Self, SlcanError> {
        Ok(CanserialFrame {
            rtr: true,
            dlc: FrameDataLen::try_from(dlc)?,
            id: can_id.into(),
            data: [0; 8],
        })
    }
}

impl core::fmt::Display for CanserialFrame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let starting_char = ['t', 'r', 'T', 'R'];
        let (mut offset, id) = match self.id {
            Id::Standard(std) => (0, std.as_raw() as u32),
            Id::Extended(ext) => (2, ext.as_raw()),
        };
        if self.rtr {
            offset += 1;
        }
        match self.id {
            Id::Standard(_) => write!(f, "{}{:03x}", starting_char[offset], id)?,
            Id::Extended(_) => write!(f, "{}{:08x}", starting_char[offset], id)?,
        }
        write!(f, "{:x}", self.dlc.0)?;
        if !self.rtr && self.dlc.0 != 0 {
            for byte in self.data.iter() {
                write!(f, "{:x}", *byte)?;
            }
        }
        Ok(())
    }
}

impl Frame for CanserialFrame {
    /// Creates a new frame.
    ///
    /// This will return `None` if the data slice is too long.
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        match CanserialFrame::new_frame(id, data) {
            Ok(frame) => Some(frame),
            Err(_) => None,
        }
    }

    /// Creates a new remote frame (RTR bit set).
    ///
    /// This will return `None` if the data length code (DLC) is not valid.
    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        match CanserialFrame::new_remote_frame(id, dlc) {
            Ok(frame) => Some(frame),
            Err(_) => None,
        }
    }

    /// Returns true if this frame is a extended frame.
    fn is_extended(&self) -> bool {
        match self.id {
            Id::Standard(_) => false,
            Id::Extended(_) => true,
        }
    }

    /// Returns true if this frame is a remote frame.
    fn is_remote_frame(&self) -> bool {
        !self.rtr
    }

    /// Returns the frame identifier.
    fn id(&self) -> Id {
        self.id
    }

    /// Returns the data length code (DLC) which is in the range 0..8.
    ///
    /// For data frames the DLC value always matches the length of the data.
    /// Remote frames do not carry any data, yet the DLC can be greater than 0.
    fn dlc(&self) -> usize {
        self.dlc.raw()
    }

    /// Returns the frame data (0..8 bytes in length).
    fn data(&self) -> &[u8] {
        &self.data
    }
}

/// Implements a parser on a stream of bytes
#[derive(Debug)]
pub struct FrameByteStreamHandler {
    parser: FrameParser,
    frame_buffer: CanserialFrame,
}

impl FrameByteStreamHandler {
    /// Create a new handler
    pub fn new() -> Self {
        FrameByteStreamHandler {
            parser: FrameParser::new(),
            frame_buffer: CanserialFrame::empty(),
        }
    }

    /// parser that works incrementally
    ///
    /// If the byte completes an incoming frame, that frame is returned, otherwise
    /// The ok result is None. If there is a parsing error, that is returned.
    /// If a frame was returned, the internal state is reset and the parser continues
    /// to work.
    pub fn feed(&mut self, byte: u8) -> Result<Option<CanserialFrame>, SlcanError> {
        self.parser = self
            .parser
            .clone()
            .parse_received_byte(byte, &mut self.frame_buffer)?;
        if self.parser.have_complete_frame().is_some() {
            // parser found complete frame
            let frame = self.frame_buffer;
            // reset input frame
            self.frame_buffer = CanserialFrame::empty();
            Ok(Some(frame))
        } else {
            Ok(None)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::can::ExtendedId;
    use log::*;
    use test_log::test;

    #[test]
    fn frame_data_len_range() {
        assert!(FrameDataLen::new(0).is_ok());
        assert!(FrameDataLen::new(8).is_ok());
        assert!(matches!(
            FrameDataLen::new(9),
            Err(crate::SlcanError::DataLen(9))
        ));
        assert!(FrameDataLen::try_from(5_usize).is_ok());
        assert!(matches!(
            FrameDataLen::try_from(10_usize),
            Err(crate::SlcanError::DataLen(10))
        ));
    }

    #[test]
    fn read_std_frame() {
        // a standard frame with 8 data bytes
        let framedata = b"t0018DEADBEEFDEADBEEF";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        if parser.have_complete_frame().is_none() {
            panic!();
        }
        assert_eq!(frame.id, Id::Standard(StandardId::new(0x1).unwrap()));
        assert_eq!(frame.dlc, FrameDataLen::new(8).unwrap());
        assert_eq!(frame.rtr, false);
        assert_eq!(frame.data, [0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef]);
        info!("frame: {}", frame);
    }

    #[test]
    fn read_std_rtr_frame() {
        // a standard rtr frame with 4 data bytes
        let framedata = b"r0014";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        if parser.have_complete_frame().is_none() {
            panic!();
        }
        assert_eq!(frame.id, Id::Standard(StandardId::new(0x1).unwrap()));
        assert_eq!(frame.dlc, FrameDataLen::new(4).unwrap());
        assert_eq!(frame.rtr, true);
        info!("frame: {}", frame);
    }

    #[test]
    fn read_ext_frame() {
        // a extended frame with 4 data bytes
        let framedata = b"T000000014DEADBEEF";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        if parser.have_complete_frame().is_none() {
            panic!();
        }
        assert_eq!(frame.id, Id::Extended(ExtendedId::new(0x1).unwrap()));
        assert_eq!(frame.dlc, FrameDataLen::new(4).unwrap());
        assert_eq!(frame.rtr, false);
        assert_eq!(frame.data[..4], [0xde, 0xad, 0xbe, 0xef]);
        info!("frame: {}", frame);
    }

    #[test]
    fn read_ext_rtr_frame() {
        // a extended rtr frame with 4 data bytes
        let framedata = b"R000000014";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        if parser.have_complete_frame().is_none() {
            panic!();
        }
        assert_eq!(frame.id, Id::Extended(ExtendedId::new(0x1).unwrap()));
        assert_eq!(frame.dlc, FrameDataLen::new(4).unwrap());
        assert_eq!(frame.rtr, true);
        info!("frame: {}", frame);
    }

    #[test]
    fn read_std_empty_frame() {
        // a standard frame with 0 data bytes
        let framedata = b"t0270";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        if parser.have_complete_frame().is_none() {
            panic!();
        }
        assert_eq!(frame.id, Id::Standard(StandardId::new(0x27).unwrap()));
        assert_eq!(frame.dlc, FrameDataLen::new(0).unwrap());
        assert_eq!(frame.rtr, false);
        assert_eq!(frame.data, [0, 0, 0, 0, 0, 0, 0, 0]);
        info!("frame: {}", frame);
    }

    #[test]
    fn data_len_too_long() {
        // a corrupted data test packet, data length of 9 bytes
        let framedata = b"t0019DEADBEEFDEADBEEFDE";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for (cnt, byte) in framedata.iter().enumerate() {
            let n = parser.parse_received_byte(*byte, &mut frame);
            if cnt < 5 {
                parser = n.unwrap();
                if parser.have_complete_frame().is_some() {
                    panic!();
                }
            } else {
                assert!(n.is_err());
                assert!(matches!(n, Err(crate::SlcanError::DataLen(9))));
                break;
            }
        }
    }

    #[test]
    fn midstream() {
        // parse the frame assuming start in middle of a frame
        let framedata = b"BEEFDEADBEEFDEt0014DEADBEEF";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        if parser.have_complete_frame().is_none() {
            panic!();
        }
        assert_eq!(frame.id, Id::Standard(StandardId::new(0x1).unwrap()));
        assert_eq!(frame.dlc, FrameDataLen::new(4).unwrap());
        assert_eq!(frame.rtr, false);
        assert_eq!(frame.data[..4], [0xde, 0xad, 0xbe, 0xef]);
        info!("frame: {}", frame);
    }
}
