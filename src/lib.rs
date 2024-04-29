#![no_std]
//! # slcan_parser
//!
//! ## Features
//!
//! This crate parses can frames that are in Serial format as used by the can-utils package used in the Linux kernel.
//! A serial port can be used as a CAN interface driver that will setup the CAN hardware, transmit and receive
//! CAN frames over the serial port in the format defined in that utility. For more information see that package.
//!
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
//! The alternative to a frame, are commands prefixed by a single character
//! 'O' - open CAN
//! 'C' - close CAN
//! 'L' - listen only on CAN
//! 'F' - read status flags to reset error states
//! 'Sc' - set CAN speed where c is 0..8
//! 'shhhhhhhh' - set bit time register value, maximum of 8 hex characters to follow
//! The speed values are as follows:
//!   <speed>           Bitrate
//!         0             10 Kbit/s
//!         1             20 Kbit/s
//!         2             50 Kbit/s
//!         3            100 Kbit/s
//!         4            125 Kbit/s
//!         5            250 Kbit/s
//!         6            500 Kbit/s
//!         7            800 Kbit/s
//!         8           1000 Kbit/s
//!
//! The commands are followed by a CR-LF sequence
//!
//! This crate defines a `FrameParser`` which is a state machine that parses
//! SlcanFrame's a byte at a time. This is intended for use with Serial Port
//! hardware. The parser is implemented with the `machine` crate.

#[macro_use]
extern crate machine;

mod parser;

#[cfg(feature = "defmt")]
use defmt::Format;

use crate::parser::FrameParser;
use core::convert::TryFrom;
use embedded_hal::can::{Error, ErrorKind, Frame, Id, StandardId};

/// Errors that can be encountered in this crate
#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum SlcanError {
    /// Char received for binary field not a hexadecimal character
    NotAHexChar,
    /// Data length not within bounds
    DataLen(usize),
    /// Standard id value too large
    StandardIdOverflow,
    /// CAN Bus speed not known
    UnknownCanBusSpeed,
}

impl Error for SlcanError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// Enum for CAN bus speed
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum SlCanBusSpeed {
    C10,
    C20,
    C50,
    C100,
    C125,
    C250,
    C500,
    C800,
    C1000,
}

impl TryFrom<u8> for SlCanBusSpeed {
    type Error = SlcanError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(SlCanBusSpeed::C10),
            1 => Ok(SlCanBusSpeed::C20),
            2 => Ok(SlCanBusSpeed::C50),
            3 => Ok(SlCanBusSpeed::C100),
            4 => Ok(SlCanBusSpeed::C125),
            5 => Ok(SlCanBusSpeed::C250),
            6 => Ok(SlCanBusSpeed::C500),
            7 => Ok(SlCanBusSpeed::C800),
            8 => Ok(SlCanBusSpeed::C1000),
            _ => Err(SlcanError::UnknownCanBusSpeed),
        }
    }
}

/// Enum for possible parser completions
///
/// This enum is updated after each byte is received
/// by parser
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub enum SlcanIncoming {
    Frame(CanserialFrame),
    Open,
    Close,
    ReadStatus,
    Listen,
    Speed(SlCanBusSpeed),
    BitTime(u32),
    Wait,
}

/// Constrain a usize to values allowed for frame data length [0..=8]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
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

/// wrap Id, so we can implement defmt::Format on embedded_hal::can::Id
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct SlcanId(Id);

#[cfg(feature = "defmt")]
impl Format for SlcanId {
    fn format(&self, fmt: defmt::Formatter) {
        // Format as hexadecimal.
        let id = match self.0 {
            Id::Standard(id) => ("Standard", id.as_raw() as u32),
            Id::Extended(id) => ("Extended", id.as_raw()),
        };
        defmt::write!(fmt, "Id({}:{:x})", id.0, id.1);
    }
}

/// CanserialFrame
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(Format))]
pub struct CanserialFrame {
    rtr: bool,
    dlc: FrameDataLen,
    id: SlcanId,
    data: [u8; 8],
}

impl CanserialFrame {
    /// an empty frame
    pub fn empty() -> Self {
        CanserialFrame {
            rtr: false,
            dlc: FrameDataLen::new(0).unwrap(),
            id: SlcanId(Id::Standard(StandardId::ZERO)),
            data: [0; 8],
        }
    }

    /// a frame
    pub fn new_frame(can_id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        match FrameDataLen::try_from(data.len()) {
            Ok(fdl) => {
                let mut frame = CanserialFrame {
                    rtr: false,
                    dlc: fdl,
                    id: SlcanId(can_id.into()),
                    data: [0_u8; 8],
                };
                frame.data[..frame.dlc.raw()].clone_from_slice(data);
                Some(frame)
            }
            Err(_) => None,
        }
    }

    pub fn new_remote_frame(can_id: impl Into<Id>, dlc: usize) -> Option<Self> {
        match FrameDataLen::try_from(dlc) {
            Ok(fdl) => {
                let frame = CanserialFrame {
                    rtr: true,
                    dlc: fdl,
                    id: SlcanId(can_id.into()),
                    data: [0_u8; 8],
                };
                Some(frame)
            }
            Err(_) => None,
        }
    }
}

impl core::fmt::Display for CanserialFrame {
    /// Implement Display to get Serial CAN frame strings
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let starting_char = ['t', 'r', 'T', 'R'];
        let (mut offset, id) = match self.id.0 {
            Id::Standard(std) => (0, std.as_raw() as u32),
            Id::Extended(ext) => (2, ext.as_raw()),
        };
        if self.rtr {
            offset += 1;
        }
        match self.id.0 {
            Id::Standard(_) => write!(f, "{}{:03x}", starting_char[offset], id)?,
            Id::Extended(_) => write!(f, "{}{:08x}", starting_char[offset], id)?,
        }
        write!(f, "{:x}", self.dlc.0)?;
        if !self.rtr && self.dlc.0 != 0 {
            for byte in self.data.iter() {
                write!(f, "{:02x}", *byte)?;
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
        CanserialFrame::new_frame(id, data)
    }

    /// Creates a new remote frame (RTR bit set).
    ///
    /// This will return `None` if the data length code (DLC) is not valid.
    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        CanserialFrame::new_remote_frame(id, dlc)
    }

    /// Returns true if this frame is a extended frame.
    fn is_extended(&self) -> bool {
        match self.id.0 {
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
        self.id.0
    }

    /// Returns the data length code (DLC) which is in the range 0..=8.
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
    /// If the byte completes an incoming frame, that frame is returned.
    /// If the byte completes a command, that command is returned, otherwise
    /// will return `SlcanIncoming::Wait`. If there is a parsing error, that is returned.
    /// If a frame was returned, the internal state is reset and the parser continues
    /// to work.
    pub fn feed(&mut self, byte: u8) -> Result<SlcanIncoming, SlcanError> {
        self.parser = self
            .parser
            .clone()
            .parse_received_byte(byte, &mut self.frame_buffer)?;
        if self.parser.have_complete_frame().is_some() {
            // parser found complete frame
            let frame = self.frame_buffer;
            // reset input frame
            self.frame_buffer = CanserialFrame::empty();
            Ok(SlcanIncoming::Frame(frame))
        } else if self.parser.call_open().is_some() {
            Ok(SlcanIncoming::Open)
        } else if self.parser.call_close().is_some() {
            Ok(SlcanIncoming::Close)
        } else if self.parser.call_listen().is_some() {
            Ok(SlcanIncoming::Listen)
        } else if self.parser.call_read_status().is_some() {
            Ok(SlcanIncoming::ReadStatus)
        } else if self.parser.call_speed().is_some() {
            let speed = match self.parser.speed() {
                Some(s) => SlCanBusSpeed::try_from(*s)?,
                None => panic!("speed command has no value"),
            };
            Ok(SlcanIncoming::Speed(speed))
        } else if self.parser.call_bit_time().is_some() {
            let bt = match self.parser.bt() {
                Some(bt) => *bt,
                None => panic!("bit time command has no value"),
            };
            Ok(SlcanIncoming::BitTime(bt))
        } else {
            Ok(SlcanIncoming::Wait)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write;
    use embedded_hal::can::ExtendedId;
    use heapless::String;
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
    fn new_frame() {
        let id = Id::Standard(StandardId::new(0x1).unwrap());
        let frame =
            CanserialFrame::new_frame(id, &[0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8]).unwrap();
        assert_eq!(frame.id.0, Id::Standard(StandardId::new(0x1).unwrap()));
        assert_eq!(frame.dlc, FrameDataLen::new(8).unwrap());
        assert_eq!(frame.rtr, false);
        assert_eq!(frame.data, [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8]);
        let mut buffer: String<32> = String::new();
        write!(&mut buffer, "{}", frame).unwrap();
        assert_eq!(buffer, "t00180102030405060708");

        let id = Id::Standard(StandardId::new(0x1).unwrap());
        let frame = CanserialFrame::new_remote_frame(id, 4).unwrap();
        assert_eq!(frame.id.0, Id::Standard(StandardId::new(0x1).unwrap()));
        assert_eq!(frame.dlc, FrameDataLen::new(4).unwrap());
        assert_eq!(frame.rtr, true);
        let mut buffer: String<32> = String::new();
        write!(&mut buffer, "{}", frame).unwrap();
        assert_eq!(buffer, "r0014");
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
        assert_eq!(frame.id.0, Id::Standard(StandardId::new(0x1).unwrap()));
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
        assert_eq!(frame.id.0, Id::Standard(StandardId::new(0x1).unwrap()));
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
        assert_eq!(frame.id.0, Id::Extended(ExtendedId::new(0x1).unwrap()));
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
        assert_eq!(frame.id.0, Id::Extended(ExtendedId::new(0x1).unwrap()));
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
        assert_eq!(frame.id.0, Id::Standard(StandardId::new(0x27).unwrap()));
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
        assert_eq!(frame.id.0, Id::Standard(StandardId::new(0x1).unwrap()));
        assert_eq!(frame.dlc, FrameDataLen::new(4).unwrap());
        assert_eq!(frame.rtr, false);
        assert_eq!(frame.data[..4], [0xde, 0xad, 0xbe, 0xef]);
        info!("frame: {}", frame);
    }

    #[test]
    fn test_empty() {
        let framedata = b"\r";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        parser = parser
            .parse_received_byte(framedata[0], &mut frame)
            .unwrap();
        assert_eq!(parser, FrameParser::Wait(parser::Wait {}));
    }

    #[test]
    fn test_open() {
        let framedata = b"O\r";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        parser = parser
            .parse_received_byte(framedata[0], &mut frame)
            .unwrap();
        assert!(parser.call_open().unwrap());
    }

    #[test]
    fn test_listen() {
        let framedata = b"L";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        parser = parser
            .parse_received_byte(framedata[0], &mut frame)
            .unwrap();
        assert!(parser.call_listen().unwrap());
    }

    #[test]
    fn test_close() {
        let framedata = b"C\r";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        assert!(parser.call_close().is_some());
    }

    #[test]
    fn test_bad_close() {
        let framedata = b"CD";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        assert!(parser.call_close().is_none());
    }

    #[test]
    fn test_read_status() {
        let framedata = b"F\r";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        assert!(parser.call_read_status().is_some());
    }

    #[test]
    fn test_speed() {
        let framedata = b"S3";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        assert!(parser.call_speed().is_some());
        assert!(parser.speed().is_some());
        let speed = *parser.speed().unwrap();
        assert_eq!(speed, 3);
        assert_eq!(SlCanBusSpeed::try_from(speed).unwrap(), SlCanBusSpeed::C100);
    }

    #[test]
    fn test_bit_time() {
        let framedata = b"sFFFFFFFF\r";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        assert!(parser.call_bit_time().is_some());
        assert!(parser.bt().is_some());
        let bt = *parser.bt().unwrap();
        assert_eq!(bt, 0xFFFFFFFF);
    }

    #[test]
    fn test_bit_time_too_long() {
        // use 9 hex chars
        let framedata = b"sFFFFFFFFF\r";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            if parser.call_bit_time().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        assert!(parser.call_bit_time().is_none());
    }

    #[test]
    fn test_bit_time_empty() {
        let framedata = b"s\r";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        assert!(parser.call_bit_time().is_some());
        assert!(parser.bt().is_some());
        let bt = *parser.bt().unwrap();
        assert_eq!(bt, 0);
    }

    #[test]
    fn command_mid_sequence() {
        // parse commands assuming start in middle of a frame
        let framedata = b"BEEFDEADBEEFDE\r\nO\r\nS5\r\n";
        let mut frame = CanserialFrame::empty();
        let mut parser = FrameParser::new();
        let mut got_open = false;
        let mut got_speed = Ok(SlCanBusSpeed::C10);
        for byte in framedata.iter() {
            if parser.have_complete_frame().is_some() {
                panic!();
            }
            if parser.call_open().is_some() {
                got_open = true;
            }
            if parser.call_speed().is_some() {
                got_speed = SlCanBusSpeed::try_from(*parser.speed().unwrap());
            }
            parser = parser.parse_received_byte(*byte, &mut frame).unwrap();
        }
        if parser.have_complete_frame().is_some() {
            panic!();
        }
        assert!(got_open);
        assert_eq!(got_speed, Ok(SlCanBusSpeed::C250));
    }

    #[test]
    fn feed_empty() {
        let framedata = b"C\rS5\r";
        let mut handler = FrameByteStreamHandler::new();
        for byte in framedata.iter() {
            let res = handler.feed(*byte);
            debug!("{:?}", res);
        }
    }

    #[test]
    fn feed_short_tx() {
        let framedata = b"t0270\r";
        let mut handler = FrameByteStreamHandler::new();
        for byte in framedata.iter() {
            let incoming = handler.feed(*byte).unwrap();
            if let SlcanIncoming::Frame(frame) = incoming {
                assert_eq!(frame.id.0, Id::Standard(StandardId::new(0x27).unwrap()));
                assert_eq!(frame.dlc, FrameDataLen::new(0).unwrap());
                assert_eq!(frame.rtr, false);
                assert_eq!(frame.data, [0, 0, 0, 0, 0, 0, 0, 0]);
            }
            debug!("{:?}", incoming);
        }
    }

    #[test]
    fn feed_speed_short_tx() {
        let framedata = b"S5\r\nt0270\r\n";
        let mut handler = FrameByteStreamHandler::new();
        for byte in framedata.iter() {
            let incoming = handler.feed(*byte).unwrap();
            if let SlcanIncoming::Frame(frame) = incoming {
                assert_eq!(frame.id.0, Id::Standard(StandardId::new(0x27).unwrap()));
                assert_eq!(frame.dlc, FrameDataLen::new(0).unwrap());
                assert_eq!(frame.rtr, false);
                assert_eq!(frame.data, [0, 0, 0, 0, 0, 0, 0, 0]);
            }
            if let SlcanIncoming::Speed(s) = incoming {
                assert_eq!(s, SlCanBusSpeed::C250);
            }
            debug!("{:?}", incoming);
        }
    }
}
