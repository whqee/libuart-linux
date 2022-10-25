///
/// Lib Uart for linux
///
use std::{
    ffi::OsStr,
    io::{self, Read, Write},
    mem::size_of,
    os::unix::io::AsRawFd,
    ptr::null,
};

use libc::c_void;

#[allow(unused)]
#[derive(Debug, PartialEq, Eq)]
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
    fn speed_flag(&self) -> u32 {
        match self {
            // c_cflag bit meaning
            BaudRate::Baud0 => 0000000, // hang up
            BaudRate::Baud50 => 1,
            BaudRate::Baud75 => 2,
            BaudRate::Baud110 => 3,
            BaudRate::Baud134 => 4,
            BaudRate::Baud150 => 5,
            BaudRate::Baud200 => 6,
            BaudRate::Baud300 => 7,
            BaudRate::Baud600 => 8,
            BaudRate::Baud1200 => 9,
            BaudRate::Baud1800 => 10,
            BaudRate::Baud2400 => 11,
            BaudRate::Baud4800 => 12,
            BaudRate::Baud9600 => 13,
            BaudRate::Baud19200 => 14,
            BaudRate::Baud38400 => 15,
            BaudRate::Baud57600 => 4097,
            BaudRate::Baud115200 => 4098,
            BaudRate::Baud230400 => 4099,
            BaudRate::Baud460800 => 4100,
            BaudRate::Baud500000 => 4101,
            BaudRate::Baud576000 => 4102,
            BaudRate::Baud921600 => 4103,
            BaudRate::Baud1000000 => 4104,
            BaudRate::Baud1152000 => 4105,
            BaudRate::Baud1500000 => 4106,
            BaudRate::Baud2000000 => 4107,
            BaudRate::Baud2500000 => 4108,
            BaudRate::Baud3000000 => 4109,
            BaudRate::Baud3500000 => 4110,
            BaudRate::Baud4000000 => 4111,
        }
    }

    #[cfg(feature = "scan_sys_baudrate")]
    pub fn test_sys_baudrate_config(uart_path: &str) {
        let mut file = std::fs::File::options()
            .read(true)
            .write(true)
            .open(&OsStr::new(uart_path))
            .unwrap();

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
        if unsafe { libc::tcgetattr(file.as_raw_fd(), &mut options) } != 0 {
            panic!("Failed to setup serial: {}", io::Error::last_os_error())
        }

        // tells linux kernel to set uart speed
        unsafe {
            // ispeed
            libc::cfsetispeed(&mut options, 0);
            // ospeed
            libc::cfsetospeed(&mut options, 0);
        }
        // apply the termios's config
        if unsafe { libc::tcsetattr(file.as_raw_fd(), libc::TCSANOW, &mut options) } != 0 {
            panic!(
                "Error occurs when setup serial, tried to get cause: {}",
                io::Error::last_os_error()
            )
        }
        let mut last_sys_baud = 0;

        for i in 0..0xFFFFFFF {
            // tells linux kernel to set uart speed
            unsafe {
                // ispeed
                libc::cfsetispeed(&mut options, i);
                // ospeed
                libc::cfsetospeed(&mut options, i);
            }
            // apply the termios's config
            if unsafe { libc::tcsetattr(file.as_raw_fd(), libc::TCSANOW, &mut options) } != 0 {
                panic!(
                    "Error occurs when setup serial, tried to get cause: {}",
                    io::Error::last_os_error()
                )
            }
            let baud = Self::get_real_baudrate(uart_path).parse().unwrap();
            match baud {
                0 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud0 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                50 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud50 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                75 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud75 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                110 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud110 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                134 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud134 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                150 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud150 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                200 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud200 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                300 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud300 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                600 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud600 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                1200 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud1200 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                1800 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud1800 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                2400 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud2400 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                4800 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud4800 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                9600 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud9600 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                19200 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud19200 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                38400 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud38400 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                57600 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud57600 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                115200 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud115200 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                230400 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud230400 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                460800 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud460800 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                500000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud500000 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                576000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud576000 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                921600 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud921600 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                1000000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud1000000 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                1152000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud1152000 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                1500000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud1500000 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                2000000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud2000000 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                2500000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud2500000 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                3000000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud3000000 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                3500000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud3500000 => {},", i);
                        last_sys_baud = baud;
                    }
                }
                4000000 => {
                    if baud != last_sys_baud {
                        println!("BaudRate::Baud4000000 => {},", i);
                        break;
                    }
                }
                _ => {}
            }
        }
        println!("done.");
    }

    fn get_real_baudrate(dev: &str) -> String {
        use std::process::Command;
        let mut cmd = String::from("stty -F ");
        cmd.push_str(dev);
        let output = Command::new("bash")
            .arg("-c")
            .arg(cmd)
            .output()
            .expect("命令执行异常错误提示");
        let output = String::from_utf8(output.stdout).unwrap();
        let output = output.split_whitespace();
        let mut iter = output.into_iter();
        let _ = iter.next();
        let speed = iter.next();
        speed.unwrap().into()
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
/// # Example: Self loop test
/// ```
/// use std::io::{Read, Write};
///
/// use uart_linux::{BaudRate, Uart};
///
/// let mut uart = Uart::new_default_locked("/dev/ttyUSB0", uart_linux::Permission::RW);
///
/// uart.baudrate = BaudRate::Baud4800;
///
/// uart.timeout_20us = 5000; // 120ms
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
    pub timeout_s: i64,
    pub timeout_20us: i64,

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

#[derive(Debug, PartialEq, Eq)]
pub enum Permission {
    RO,
    WO,
    RW,
}

impl Uart {
    #[inline]
    ///
    /// default to lock the serial device with RW Permission
    ///
    pub fn new_default_locked(path: &str, rw: Permission) -> Uart {
        Self::__new_default(path, true, rw)
    }

    #[inline]
    /// unlock, read write
    pub fn new_default(path: &str, rw: Permission) -> Uart {
        Self::__new_default(path, false, rw)
    }

    fn __new_default(path: &str, lock_it: bool, rw: Permission) -> Uart {
        let mut r = true;
        let mut w = true;

        match rw {
            Permission::RO => w = false,
            Permission::WO => r = false,
            Permission::RW => {}
        }

        let mut file = std::fs::File::options()
            .read(r)
            .write(w)
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
            timeout_s: 0,
            timeout_20us: 5000, // 100ms
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
            libc::cfsetispeed(&mut options, self.baudrate.speed_flag());
            // ospeed
            libc::cfsetospeed(&mut options, self.baudrate.speed_flag());
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
    fn __read_bytes(&self, buf: &mut [u8], timeout_s: i64, timeout_20us: i64) -> io::Result<usize> {
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
            tv_sec: timeout_s,
            tv_usec: timeout_20us,
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

    // fn read_exact(&self, buf: &mut [u8]) -> io::Result<()> {
    //     for i in 0..buf.len() {
    //         self.__read_bytes(
    //             &mut unsafe { *(&mut buf[i] as *mut u8 as *mut [u8; 1]) },
    //             self.timeout_s,
    //             self.timeout_20us,
    //         )?;
    //     }
    //     Ok(())
    // }
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
        self.__read_bytes(buf, self.timeout_s, self.timeout_20us * buf.len() as i64)
    }
}

// impl Drop for Uart {
//     fn drop(&mut self) {
//         println!("[DebugInfo] Droped Uart")
//     }
// }

// No tests here. See examples/self_loop.rs
// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn it_works() {

//     }
// }
