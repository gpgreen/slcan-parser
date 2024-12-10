use super::*;

impl embedded_hal_02::can::Error for SlcanError {
    fn kind(&self) -> embedded_hal_02::can::ErrorKind {
        embedded_hal_02::can::ErrorKind::Other
    }
}

impl From<embedded_hal_02::can::Id> for SlcanId {
    fn from(val: embedded_hal_02::can::Id) -> Self {
        match val {
            // note(unsafe) the Id has to be ok, unless it was itself created unsafely
            embedded_hal_02::can::Id::Standard(si) => SlcanId(embedded_can::Id::Standard(unsafe {
                embedded_can::StandardId::new_unchecked(si.as_raw())
            })),
            // note(unsafe) the Id has to be ok, unless it was itself created unsafely
            embedded_hal_02::can::Id::Extended(ei) => SlcanId(embedded_can::Id::Extended(unsafe {
                embedded_can::ExtendedId::new_unchecked(ei.as_raw())
            })),
        }
    }
}

impl embedded_hal_02::can::Frame for CanserialFrame {
    /// Creates a new frame.
    ///
    /// This will return `None` if the data slice is too long.
    fn new(id: impl Into<embedded_hal_02::can::Id>, data: &[u8]) -> Option<Self> {
        let nid: embedded_hal_02::can::Id = id.into();
        CanserialFrame::new_frame(SlcanId::from(nid), data)
    }

    /// Creates a new remote frame (RTR bit set).
    ///
    /// This will return `None` if the data length code (DLC) is not valid.
    fn new_remote(id: impl Into<embedded_hal_02::can::Id>, dlc: usize) -> Option<Self> {
        let nid: embedded_hal_02::can::Id = id.into();
        CanserialFrame::new_remote_frame(SlcanId::from(nid), dlc)
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
    fn id(&self) -> embedded_hal_02::can::Id {
        match self.id.0 {
            // note(unsafe) the Id has to be ok as it was checked when creating the frame
            embedded_can::Id::Extended(i) => embedded_hal_02::can::Id::from(unsafe {
                embedded_hal_02::can::ExtendedId::new_unchecked(i.as_raw())
            }),
            // note(unsafe) the Id has to be ok, as it was checked when creating the frame
            embedded_can::Id::Standard(i) => embedded_hal_02::can::Id::from(unsafe {
                embedded_hal_02::can::StandardId::new_unchecked(i.as_raw())
            }),
        }
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
        if embedded_hal_02::can::Frame::is_extended(self) {
            &self.data[0..0]
        } else {
            &self.data[0..self.dlc.raw()]
        }
    }
}
