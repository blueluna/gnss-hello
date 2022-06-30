/// Crate error type
#[derive(Debug, PartialEq)]
pub enum Error {
    /// NMEA parse / generate? error
    NMEAError(mini_nmea::Error),
    /// uBlox parse error
    UBXParserError(ublox::ParserError),
}

impl From<mini_nmea::Error> for Error {
    fn from(error: mini_nmea::Error) -> Self {
        Error::NMEAError(error)
    }
}

impl From<ublox::ParserError> for Error {
    fn from(error: ublox::ParserError) -> Self {
        Error::UBXParserError(error)
    }
}

impl core::fmt::Display for Error {
    /// Format error
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        match self {
            Error::NMEAError(error) => { write!(f, "Error {}", error) }
            Error::UBXParserError(error) => { write!(f, "Error {}", error) }
        }
    }
}
