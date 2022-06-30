mod gnss;
mod error;

use bbqueue::BBBuffer;
use mio::{Events, Interest, Poll, Token};
use mio_serial::SerialPortBuilderExt;
use mio_timerfd::{ClockId, TimerFd};
use std::env;
use std::io;
use std::io::{Read, Write};
use std::time::Duration;
use std::str;

const SERIAL_TOKEN: Token = Token(0);
const TIMER_TOKEN: Token = Token(1);
const DEFAULT_TTY: &str = "/dev/ttyUSB0";
const DEFAULT_BAUD: u32 = 9600;

pub fn main() -> io::Result<()> {
    let mut args = env::args();
    let path = args.nth(1).unwrap_or(DEFAULT_TTY.into());
    // let baud = DEFAULT_BAUD;

    // Create a poll instance.
    let mut poll = Poll::new()?;
    // Create storage for events. Since we will only register a single serialport, a
    // capacity of 1 will do.
    let mut events = Events::with_capacity(1);

    // Create the serial port
    println!("Opening {} at 9600,8N1", path);
    let mut serial_port = mio_serial::new(path, DEFAULT_BAUD).timeout(Duration::from_secs(1)).open_native_async()?;

    let buffer: BBBuffer<1024> = BBBuffer::new();

    let (mut buffer_producer, mut buffer_consumer) = buffer.try_split().unwrap();


    let mut timer = TimerFd::new(ClockId::Monotonic).unwrap();
    timer.set_timeout_interval(&Duration::from_secs(120)).unwrap();

    poll.registry()
        .register(&mut serial_port, SERIAL_TOKEN, Interest::READABLE)?;
    poll.registry().register(&mut timer, TIMER_TOKEN, Interest::READABLE)?;

    let _ = serial_port.write(&[0xff]);
    let combined = &mut [0u8; 1024];

    let mut ublox_device = crate::gnss::UbloxDevice::new();

    loop {
        poll.poll(&mut events, None)?;
        for event in events.iter() {
            match event.token() {
                SERIAL_TOKEN => {
                    loop {
                        let mut write_grant = buffer_producer.grant_max_remaining(256).unwrap();
                        // In this loop we receive all packets queued for the socket.
                        match serial_port.read(&mut write_grant.buf()) {
                            Ok(count) => {
                                write_grant.commit(count);
                                if let Ok(read_grant) = buffer_consumer.split_read() {
                                    let (a, b) = read_grant.bufs();
                                    let a_end = a.len();
                                    let b_end = a_end + b.len();
                                    combined[..a_end].clone_from_slice(a);
                                    combined[a_end..b_end].clone_from_slice(b);
                                    let consumed = ublox_device.parse(&combined[..b_end]);
                                    read_grant.release(consumed);
                                }
                            }
                            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                                break;
                            }
                            Err(e) => {
                                println!("Quitting due to read error: {}", e);
                                return Err(e);
                            }
                        }
                    }
                },
                TIMER_TOKEN => {
                    let _ = timer.read();
                    let _ = serial_port.write(&[0xff]);
                }
                _ => {
                }
            }
        }
    }
}
