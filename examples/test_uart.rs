use std::io::{Read, Write};

use uart_linux::{BaudRate, Uart};

fn main() {
    let mut uart = Uart::new_default_locked("/dev/ttyUSB2");

    uart.baudrate = BaudRate::Baud4800;

    uart.timeout_us = 1000;

    uart.apply_settings();

    let mut buf = [0u8; 256];
    let mut rbuf2 = [0u8; 256];
    let mut rbuf1 = [0u8; 256];

    for i in 0..=255 {
        buf[i] = i as u8;
    }

    uart.write(&buf).unwrap();
    uart.read_exact(&mut rbuf2).unwrap();

    uart.write(&buf).unwrap();
    let _len = uart.read(&mut rbuf1).unwrap();

    assert_eq!(buf, rbuf2);
    assert_eq!(buf, rbuf1);

    println!("OK");
}
