use std::time::Duration;
use chrono::prelude::*;
use mini_nmea;
use ublox;

use crate::error::Error;

fn nmea_start_position(data: &[u8]) -> Option<usize>
{
    for n in 0..data.len() {
        if data[n] == b'$' {
            return Some(n);
        }
    }
    return None;
}

pub enum UbloxDeviceState
{
    Initial,
    GetUartConfiguration,
    SetUartConfiguration,
    GetPowerSaveConfiguration,
    SetPowerSaveConfiguration,
    StoreSettings,
    Receiving,
}

pub struct UbloxDevice {
    state: UbloxDeviceState,
    date_time: DateTime<Utc>,
    tx_buffer: [u8; 64],
}

impl UbloxDevice {
    pub fn new() -> Self {
        Self {
            state: UbloxDeviceState::Initial,
            date_time: DateTime::<Utc>::from_utc(NaiveDate::from_ymd(1900, 1, 1).and_hms(0, 0, 0), Utc),
            tx_buffer: [0u8; 64],
        }
    }

    pub fn parse(&mut self, data: &[u8]) -> usize
    {

        let nmea_start = nmea_start_position(data);
        let ubx_start = match ublox::find_sync_position(data) {
            Ok(position) => {
                Some(position)
            }
            Err(_) => {
                None
            }
        };

        let parse_result = match (nmea_start, ubx_start) {
            (None, None) => { Ok(0) },
            (Some(nmea_position), None) => {
                self.parse_nmea(data, nmea_position)
            }
            (None, Some(ubx_position)) => {
                self.parse_ubx(data, ubx_position)
            }
            (Some(nmea_position), Some(ubx_position)) => {
                if nmea_position < ubx_position {
                    self.parse_nmea(data, nmea_position)
                }
                else {
                    self.parse_ubx(data, ubx_position)
                }
            }
        };
        if let Ok(consumed) = parse_result {
            consumed
        }
        else { 0 }
    }

    pub fn update(&mut self) -> (Duration, &[u8])
    {
        /*
        match self.state {

        }
        */
        let tx_size = 0;
        (Duration::new(1, 0), &self.tx_buffer[..tx_size])
    }

    fn parse_nmea(&mut self, data: &[u8], position: usize) -> Result<usize, Error>
    {
        let consumed = match mini_nmea::Sentence::parse(&data[position..]) {
            Ok((sentence, consumed)) => {
                // println!("NMEA {}", sentence);
                position + consumed
            }
            Err(error) => {
                match error {
                    mini_nmea::Error::NotEnoughData => (),
                    _ => {
                        println!("NMEA error {}", error);
                    }
                }
                position
            }
        };
        Ok(consumed)
    }

    fn parse_ubx(&mut self, data: &[u8], position: usize) -> Result<usize, Error>
    {
        use ublox::{PacketRef, GpsFix};
        let (result, consumed) = ublox::parse_slice(&data[position..]);
        match result {
            Ok(packet) => {
                match packet {
                    PacketRef::NavPosVelTime(navigation) => {
                        let fix_type = navigation.fix_type();
                        let has_time = fix_type == GpsFix::Fix3D
                            || fix_type == GpsFix::GPSPlusDeadReckoning
                            || fix_type == GpsFix::TimeOnlyFix;
                        if has_time {
                            if let Ok(dt) = DateTime::<Utc>::try_from(&navigation) {
                                self.date_time = dt;
                                println!("{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:09}", dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second(), dt.nanosecond());
                            }
                        }
                    }
                    _ => {
                        println!("UBX {:?}", packet);
                    },
                }
            }
            Err(error) => {
                match error {
                    ublox::ParserError::MoreDataRequired{..} => (),
                    _ => {
                        println!("UBX error {}", error);
                    }
                }
            }
        }
        Ok(position + consumed)
    }
}