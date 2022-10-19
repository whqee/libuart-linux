///
/// Lib Uart for linux
///
use std::{
    ffi::OsStr,
    io::{self, Read, Write},
    mem::size_of,
    os::fd::AsRawFd,
    ptr::null,
};

use libc::c_void;

#[allow(unused)]
pub enum BaudRate {
    Baud0, // hang up
    Baud50,
    Baud75,
    Baud110,
    Baud134,
    Baud150,
    Baud200,
    Baud300,
    Baud600,
    Baud1200,
    Baud1800,
    Baud2400,
    Baud4800,
    Baud9600,
    Baud19200,
    Baud38400,
    Baud57600,
    Baud115200,
    Baud230400,
    Baud460800,
    Baud500000,
    Baud576000,
    Baud921600,
    Baud1000000,
    Baud1152000,
    Baud1500000,
    Baud2000000,
    Baud2500000,
    Baud3000000,
    Baud3500000,
    Baud4000000,
}

#[allow(unused)]
impl BaudRate {
    fn speed(&self) -> u32 {
        match self {
            // c_cflag bit meaning
            BaudRate::Baud0 => 0000000, // hang up
            BaudRate::Baud50 => 0000001,
            BaudRate::Baud75 => 0000002,
            BaudRate::Baud110 => 0000003,
            BaudRate::Baud134 => 0000004,
            BaudRate::Baud150 => 0000005,
            BaudRate::Baud200 => 0000006,
            BaudRate::Baud300 => 0000007,
            BaudRate::Baud600 => 0000010,
            BaudRate::Baud1200 => 0000011,
            BaudRate::Baud1800 => 0000012,
            BaudRate::Baud2400 => 0000013,
            BaudRate::Baud4800 => 0000014,
            BaudRate::Baud9600 => 0000015,
            BaudRate::Baud19200 => 0000016,
            BaudRate::Baud38400 => 0000017,
            BaudRate::Baud57600 => 0010001,
            BaudRate::Baud115200 => 0010002,
            BaudRate::Baud230400 => 0010003,
            BaudRate::Baud460800 => 0010004,
            BaudRate::Baud500000 => 0010005,
            BaudRate::Baud576000 => 0010006,
            BaudRate::Baud921600 => 0010007,
            BaudRate::Baud1000000 => 0010010,
            BaudRate::Baud1152000 => 0010011,
            BaudRate::Baud1500000 => 0010012,
            BaudRate::Baud2000000 => 0010013,
            BaudRate::Baud2500000 => 0010014,
            BaudRate::Baud3000000 => 0010015,
            BaudRate::Baud3500000 => 0010016,
            BaudRate::Baud4000000 => 0010017,
        }
    }
}

#[allow(unused)]
pub enum FlowCtrl {
    FlowCtrlNone,
    FlowCtrl1,
    FlowCtrl2,
}

#[allow(unused)]
pub enum DataBits {
    DataBits5 = 5,
    DataBits6,
    DataBits7,
    DataBits8,
}

#[allow(unused)]
pub enum StopBits {
    StopBits1 = 1,
    StopBits2,
}

#[allow(unused)]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
    ParitySpace,
}

///
/// # Example
/// ```
/// use std::io::{Read, Write};
///
/// use uart_linux::{BaudRate, Uart};
///
/// let mut uart = Uart::new_default_locked("/dev/ttyUSB2");
///
/// uart.baudrate = BaudRate::Baud4800;
///
/// uart.timeout_us = 1000;
///
/// uart.apply_settings();
///
/// let mut buf = [0u8; 256];
/// let mut rbuf2 = [0u8; 256];
/// let mut rbuf1 = [0u8; 256];
///
/// for i in 0..=255 {
///     buf[i] = i as u8;
/// }
///
/// uart.write(&buf).unwrap();
/// uart.read_exact(&mut rbuf2).unwrap();
///
/// uart.write(&buf).unwrap();
/// let _len = uart.read(&mut rbuf1).unwrap();
///
/// assert_eq!(buf, rbuf2);
/// assert_eq!(buf, rbuf1);
///
/// println!("OK");
/// ```
////
pub struct Uart {
    file: std::fs::File,

    pub baudrate: BaudRate,
    pub flowctrl: FlowCtrl,
    pub databits: DataBits,
    pub stopbits: StopBits,
    pub parity: Parity,
    pub rs232_485: Option<RS232_485>,
    pub timeout_us: i64,

    /// effects in blocking mode, block to read at least 'vmin' bytes
    pub vmin: u8,
    // /// read a c_char to wait vtime*(1/10)s = 0.1s * vtime
    // pub vtime: u8,
    // pub lock_the_uart: bool,
}

pub struct RS232_485 {
    pub ops_before_read: fn() -> io::Result<()>,
    pub ops_before_write: fn() -> io::Result<()>,
}

impl Uart {
    #[inline]
    ///
    /// default to lock the serial device
    ///
    pub fn new_default_locked(path: &str) -> Uart {
        Self::new_default(path, true)
    }

    #[inline]
    pub fn new_default(path: &str, lock_it: bool) -> Uart {
        let mut file = std::fs::File::options()
            .read(true)
            .write(true)
            .open(&OsStr::new(path))
            .unwrap();

        // check if is a tty
        if unsafe { libc::isatty(file.as_raw_fd()) } == 0 {
            panic!("it's not a terminal device")
        }

        // flush before we use it
        file.flush().unwrap();

        // lock this serial device ?
        if lock_it {
            // try locking it:
            if unsafe { libc::ioctl(file.as_raw_fd(), libc::TIOCEXCL) } != 0 {
                panic!("cannot lock the uart: {}", io::Error::last_os_error())
            }
        }

        Uart {
            file,
            baudrate: BaudRate::Baud115200,
            flowctrl: FlowCtrl::FlowCtrlNone,
            databits: DataBits::DataBits8,
            stopbits: StopBits::StopBits1,
            parity: Parity::ParityNone,
            rs232_485: None,
            timeout_us: 1200,
            vmin: 0,
            // lock_the_uart: true,
            // vtime: 12, // 1200us
        }
    }

    pub fn apply_settings(&self) {
        let mut options = libc::termios {
            c_iflag: 0,
            c_oflag: 0,
            c_cflag: 0,
            c_lflag: 0,
            c_line: 0,
            c_cc: [0; 32],
            c_ispeed: 0,
            c_ospeed: 0,
        };

        // try getting termios attributes and checking if we can setup the uart
        if unsafe { libc::tcgetattr(self.file.as_raw_fd(), &mut options) } != 0 {
            panic!("Failed to setup serial: {}", io::Error::last_os_error())
        }

        // tells linux kernel to set uart speed
        unsafe {
            // ispeed
            libc::cfsetispeed(&mut options, self.baudrate.speed());
            // ospeed
            libc::cfsetospeed(&mut options, self.baudrate.speed());
        }

        // set c_cflag with "serial not occupied"
        options.c_cflag |= libc::CLOCAL;

        // // set c_cflag with "serial occupied"
        // options.c_cflag &= !libc::CLOCAL;

        // set c_cflag to enable input
        options.c_cflag |= libc::CREAD;

        // set flowctrl mode
        match self.flowctrl {
            // None FlowCtrl
            FlowCtrl::FlowCtrlNone => options.c_cflag &= !libc::CRTSCTS,
            // Controlled by Hardware
            FlowCtrl::FlowCtrl1 => options.c_cflag |= libc::CRTSCTS,
            // Controlled by Software
            FlowCtrl::FlowCtrl2 => options.c_cflag |= libc::IXON | libc::IXOFF | libc::IXANY,
        }

        // set parity
        match self.parity {
            Parity::ParityNone => {
                options.c_cflag &= !libc::PARENB;
                options.c_iflag &= !libc::INPCK;
            }
            Parity::ParityEven => {
                options.c_cflag |= libc::PARENB;
                options.c_cflag &= !libc::PARODD;
            }
            Parity::ParityOdd => {
                options.c_cflag |= libc::PARODD | libc::PARENB;
                options.c_iflag |= libc::INPCK;
            }
            Parity::ParitySpace => {
                options.c_cflag &= !libc::PARENB;
                options.c_cflag &= !libc::CSTOPB;
            }
        }

        // set databits
        options.c_cflag &= !libc::CSIZE;
        match self.databits {
            DataBits::DataBits5 => options.c_cflag |= libc::CS5,
            DataBits::DataBits6 => options.c_cflag |= libc::CS6,
            DataBits::DataBits7 => options.c_cflag |= libc::CS7,
            DataBits::DataBits8 => options.c_cflag |= libc::CS8,
        }

        // set stopbits
        match self.stopbits {
            StopBits::StopBits1 => options.c_cflag &= !libc::CSTOPB,
            StopBits::StopBits2 => options.c_cflag |= libc::CSTOPB,
        }

        // set output mode: output raw data
        options.c_oflag &= !libc::OPOST;
        options.c_lflag &= !(libc::ICANON | libc::ECHO | libc::ECHOE | libc::ISIG);

        // set input mode: input raw data
        options.c_iflag &=
            !(libc::IXON | libc::IXOFF | libc::IXANY | libc::BRKINT | libc::ICRNL | libc::ISTRIP);

        // set time to wait
        // and "minimal counts c_char to recieve" = 0
        options.c_cc[libc::VTIME] = 0; // read a c_char to wait n*(1/10)s
        options.c_cc[libc::VMIN] = self.vmin; // min to read

        // tells kernel that if uart's rx buffer is overflow, flush it and continue to recieve
        unsafe { libc::tcflush(self.file.as_raw_fd(), libc::TCIFLUSH) };

        // apply the termios's config
        if unsafe { libc::tcsetattr(self.file.as_raw_fd(), libc::TCSANOW, &mut options) } != 0 {
            panic!(
                "Error occurs when setup serial, tried to get cause: {}",
                io::Error::last_os_error()
            )
        }

        ()
    }

    /// read bytes with timeout, "len to be read" = buf.len()
    /// because timeout (by vtime::u8) is too short for uart, we use 'select() + read()' instead
    fn read_bytes(&self, buf: &mut [u8], timeout_us: i64) -> io::Result<usize> {
        if buf.len() == 0 {
            println!("buf.len == 0, return directly");
            return Ok(0);
        }

        if let Some(rs232_485) = &self.rs232_485 {
            (rs232_485.ops_before_read)()?
        }

        let fs_read =
            [0usize; libc::FD_SETSIZE / size_of::<usize>()].as_mut_ptr() as *mut libc::fd_set;

        // struct time_info for select()
        let mut time_info = libc::timeval {
            tv_sec: 0,
            tv_usec: timeout_us,
        };

        let _fs_sel = unsafe {
            libc::select(
                self.file.as_raw_fd() + 1,
                fs_read,
                null::<libc::fd_set>() as *mut libc::fd_set,
                null::<libc::fd_set>() as *mut libc::fd_set,
                &mut time_info,
            )
        };

        let buf_ptr = buf.as_mut_ptr() as *mut c_void;

        let len = unsafe { libc::read(self.file.as_raw_fd(), buf_ptr, buf.len()) };

        if len < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(len as usize)
        }
    }
}

impl Write for Uart {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if let Some(rs232_485) = &self.rs232_485 {
            (rs232_485.ops_before_write)()?
        }
        self.file.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.file.flush()
    }
}

impl Read for Uart {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if let Some(rs232_485) = &self.rs232_485 {
            (rs232_485.ops_before_read)()?
        }

        // timeout by vtime(u8) is too short for uart, we use 'select + read' instead
        self.read_bytes(buf, self.timeout_us * buf.len() as i64)
    }
}

// No tests here. See examples/test_uart
//
// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn it_works() {

//     }
// }
