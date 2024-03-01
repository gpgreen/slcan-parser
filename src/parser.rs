use crate::CanserialFrame;
use crate::FrameDataLen;
use crate::SlcanError;
use embedded_hal::can;
use log::*;

/// function to convert a hex char byte to it's binary value
fn from_hex(byte: u8) -> Result<u8, SlcanError> {
    if (b'a'..=b'f').contains(&byte) {
        Ok(byte - b'a' + 10)
    } else if (b'A'..=b'F').contains(&byte) {
        Ok(byte - b'A' + 10)
    } else if byte.is_ascii_digit() {
        Ok(byte - b'0')
    } else {
        Err(SlcanError::NotAHexChar)
    }
}

// Define the machine states
machine!(
    #[derive(Clone, Debug, PartialEq)]
    enum FrameParser {
        Wait,
        StandardId { offset: usize, stdrtr: bool },
        ExtendedId { offset: usize, extrtr: bool },
        DataLen { rtr: bool },
        DataBytes { len: usize, offset: usize },
        HaveFrame { flen: usize },
    }
);

/// The state transition types
#[derive(Clone, Debug, PartialEq)]
pub struct Advance {
    byte: u8,
}

// the allowed state, transition pairs
transitions!(FrameParser,
             [
                 (Wait, Advance) => [StandardId, ExtendedId, Wait],
                 (StandardId, Advance) => [StandardId, DataLen],
                 (ExtendedId, Advance) => [ExtendedId, DataLen],
                 (DataLen, Advance) => [DataBytes, HaveFrame],
                 (DataBytes, Advance) => [DataBytes, HaveFrame],
                 (HaveFrame, Advance) => Wait
             ]
);

// additional methods to add to the parser for retrieving data
methods!(FrameParser,
         [
             StandardId => get stdrtr: bool,
             StandardId => fn can_collect_stdid(&self) -> bool,
             ExtendedId => get extrtr: bool,
             ExtendedId => fn can_collect_extid(&self) -> bool,
             DataLen => get rtr: bool,
             DataBytes => get offset: usize,
             DataBytes => get len: usize,
             DataBytes => fn can_collect_data(&self) -> bool,
             HaveFrame => get flen: usize,
             HaveFrame => fn have_complete_frame(&self) -> bool
         ]
);

/// wait state looking for packet header of '[trTR]'
impl Wait {
    pub fn on_advance(self, input: Advance) -> FrameParser {
        match input.byte {
            b't' => FrameParser::standard_id(0, false),
            b'r' => FrameParser::standard_id(0, true),
            b'T' => FrameParser::extended_id(0, false),
            b'R' => FrameParser::extended_id(0, true),
            _ => FrameParser::wait(),
        }
    }
}

/// StandardId, get Id Bytes
impl StandardId {
    pub fn on_advance(self, _input: Advance) -> FrameParser {
        if self.offset < 3 - 1 {
            FrameParser::standard_id(self.offset + 1, self.stdrtr)
        } else {
            FrameParser::data_len(self.stdrtr)
        }
    }

    pub fn can_collect_stdid(&self) -> bool {
        true
    }
}

/// ExtendedId, get Id Bytes
impl ExtendedId {
    pub fn on_advance(self, _input: Advance) -> FrameParser {
        if self.offset < 8 - 1 {
            FrameParser::extended_id(self.offset + 1, self.extrtr)
        } else {
            FrameParser::data_len(self.extrtr)
        }
    }

    pub fn can_collect_extid(&self) -> bool {
        true
    }
}

/// After the Id, we get the number of data bytes, unless length is 0, or the frame has RTR bit set, where we signal end of frame
impl DataLen {
    pub fn on_advance(self, input: Advance) -> FrameParser {
        let len = from_hex(input.byte).unwrap() as usize;
        if len == 0 {
            FrameParser::have_frame(0)
        } else if self.rtr {
            FrameParser::have_frame(len)
        } else {
            FrameParser::data_bytes(len, 0)
        }
    }
}

/// get the packet data (if any)
impl DataBytes {
    pub fn on_advance(self, _input: Advance) -> FrameParser {
        if self.offset < self.len * 2 - 1 {
            FrameParser::data_bytes(self.len, self.offset + 1)
        } else {
            FrameParser::have_frame(self.len)
        }
    }

    pub fn can_collect_data(&self) -> bool {
        true
    }
}

/// end of state machine
impl HaveFrame {
    pub fn on_advance(self, _: Advance) -> Wait {
        Wait {}
    }

    pub fn have_complete_frame(&self) -> bool {
        true
    }
}

impl FrameParser {
    pub fn new() -> FrameParser {
        FrameParser::Wait(Wait {})
    }

    pub fn parse_received_byte(
        self,
        byte: u8,
        frame: &mut CanserialFrame,
    ) -> Result<FrameParser, SlcanError> {
        debug!("Parser <- advance {:?} byte:0x{:x}", self, byte);
        if self.can_collect_data().is_some() {
            let i = *self.offset().expect("frame data offset is None");
            let converted = from_hex(byte)?;
            if i % 2 == 0 {
                frame.data[i >> 1] = converted << 4;
            } else {
                frame.data[i >> 1] += converted;
            }
            // store the data length on the first nibble received
            if i == 0 {
                match FrameDataLen::new(*self.len().expect("frame dlc is None")) {
                    Ok(dlc) => frame.dlc = dlc,
                    Err(e) => return Err(e),
                }
                debug!("FrameDataLen: {:?}", frame.dlc);
            }
            debug!("frame data {} is {}", i >> 1, frame.data[i >> 1]);
        } else if self.can_collect_stdid().is_some() {
            frame.rtr = *self.stdrtr().expect("frame rtr is None");
            let raw_id = match frame.id {
                can::Id::Standard(id) => id.as_raw(),
                _ => 0,
            };
            let converted = from_hex(byte)? as u16;
            let new_id = match can::StandardId::new((raw_id << 4) + converted) {
                Some(x) => x,
                None => return Err(SlcanError::StandardIdOverflow),
            };
            frame.id = can::Id::Standard(new_id);
            debug!("frame id: {:?}", frame.id);
        } else if self.can_collect_extid().is_some() {
            frame.rtr = *self.extrtr().expect("frame rtr is None");
            let raw_id = match frame.id {
                can::Id::Standard(sid) => sid.as_raw() as u32,
                can::Id::Extended(eid) => eid.as_raw(),
            };
            let converted = from_hex(byte)? as u32;
            let new_id = match embedded_hal::can::ExtendedId::new((raw_id << 4) + converted) {
                Some(x) => x,
                None => return Err(SlcanError::StandardIdOverflow),
            };
            frame.id = can::Id::Extended(new_id);
            debug!("frame id: {:?}", frame.id);
        }
        let p = self.on_advance(Advance { byte });
        debug!("Parser -> advance {:?} byte:0x{:x}", p, byte);
        if frame.rtr && p.have_complete_frame().is_some() {
            // if a remote frame, then we have to store the dlc, as there is no data to do it then
            let len = *p.flen().expect("frame len is None");
            match FrameDataLen::new(len) {
                Ok(dlc) => frame.dlc = dlc,
                Err(e) => return Err(e),
            }
            debug!("FrameDataLen: {:?}", frame.dlc);
        }
        Ok(p)
    }
}
