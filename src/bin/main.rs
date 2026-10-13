//! Firmware for the Waveshare ESP32-C3-LCD-1.47: shows whole 172×320 frames rendered on the PC (the forecast ribbon), taken over Wi-Fi (TCP, after a keyed-BLAKE3 challenge) or the USB serial link, and keeps the last one in flash so it's back straight away after a power cut. The USB link also carries the patch commands used to characterise the panel.
//!
//! Line commands, the same on both links (numbers decimal, colour channels 0..=255):
//! - `d hash` — followed by one frame: 172 × 320 RGB565 pixels, row by row, high byte first (110,080 bytes), checked against its BLAKE3 (64 hex digits), then drawn and kept in flash. Answers `ok d` or `err d …`
//! - `g hash` — followed by the signal cells: twelve 30 × 13 RGB565 pictures, high byte first (9,360 bytes), of the dozenal digits Zil..Stelor each in its colour, checked against their BLAKE3 and kept in flash. Answers `ok g` or `err g …`. The board draws the cell for its own Wi-Fi signal over the frame's top-left corner, redrawn as the signal changes: Zil at −80 dBm or worse (or not connected) up to Stelor at −40 or better.
//! - `b n` — backlight duty 0..=255, kept in flash
//! - `info` (backlight, signal strength in dBm or 0 when not connected, the signal cells' BLAKE3 as 16 hex digits or `-`, address), `ping`
//!
//! Over TCP (port 7766, `forecast.local`), the board first sends `hello <nonce>`, and nothing but `ping` works until the client answers `auth <BLAKE3 keyed with the shared key, of the nonce>`.
//!
//! USB only:
//! - `wifi <ssid> <password>` — store the network to join and restart; `key <64 hex digits>` — store the shared key; `net` — report the board's address
//! - `f r g b` — fill the whole panel; `w s r g b` — an s×s square (s ≤ 172) centred on black; both answered once drawn. The next frame replaces them.

#![no_std]
#![no_main]

extern crate alloc;

mod net;

use core::fmt::Write as _;

use c3_forecast::expander::{self, Expander};
use c3_forecast::st7789::{HEIGHT, St7789, WIDTH};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx};
use esp_hal::{Async, dma_rx_buffer, dma_tx_buffer};
use esp_println::println;
use esp_storage::FlashStorage;
use static_cell::{ConstStaticCell, StaticCell};

esp_bootloader_esp_idf::esp_app_desc!();

const FRAME_BYTES: usize = WIDTH as usize * HEIGHT as usize * 2;

/// One whole frame, word-aligned for the ROM flash routines.
#[repr(C, align(4))]
struct Frame([u8; FRAME_BYTES]);
static FRAME: ConstStaticCell<Frame> = ConstStaticCell::new(Frame([0; FRAME_BYTES]));

/// The `frame` partition (partitions.csv): a header sector (magic and the frame's BLAKE3), then the frame. The header goes last when saving, so a frame cut short by a power cut is never taken for whole.
const STORE: u32 = 0x1D_0000;
const STORE_DATA: u32 = STORE + FlashStorage::SECTOR_SIZE;
const STORE_MAGIC: u32 = 0x4652_4D31;

/// The signal cells (`g`), drawn at the panel's top-left: one per dozenal digit.
const CELL_W: u16 = 30;
const CELL_H: u16 = 13;
const CELL_BYTES: usize = CELL_W as usize * CELL_H as usize * 2;
const CELLS_BYTES: usize = 12 * CELL_BYTES;

#[repr(C, align(4))]
struct Cells([u8; CELLS_BYTES]);
static CELLS: ConstStaticCell<Cells> = ConstStaticCell::new(Cells([0; CELLS_BYTES]));

/// Kept in the `frame` partition after the frame, the same way: a header sector, then the data.
const CELLS_STORE: u32 = STORE_DATA + (FRAME_BYTES as u32).div_ceil(FlashStorage::SECTOR_SIZE) * FlashStorage::SECTOR_SIZE;
const CELLS_MAGIC: u32 = 0x5347_4E31;
const _: () = assert!(CELLS_STORE + FlashStorage::SECTOR_SIZE + CELLS_BYTES as u32 <= STORE + 0x3_0000, "the cells must fit the frame partition");

/// The signal digit for `rssi` (dBm): Zil at −80 or worse (or not connected) up to Stelor at −40 or better, evenly spaced in dB (about 3.6 dB a step).
fn signal_digit(rssi: Option<i32>) -> u8 {
    rssi.map_or(0, |r| (((r + 80).clamp(0, 40) * 11 + 20) / 40) as u8)
}

/// Wakes the signal task straight away, after a new frame has covered the cell.
static REDRAW: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Network, shared key and backlight, kept in the first sector of the `nvs` partition (nothing else here uses NVS).
#[derive(Clone)]
struct Settings {
    ssid: heapless::String<32>,
    password: heapless::String<64>,
    key: Option<[u8; 32]>,
    backlight: u8,
}

const SETTINGS_ADDR: u32 = 0x9000;
const SETTINGS_MAGIC: u32 = 0x4E45_5431;
/// Magic, lengths, SSID (32 bytes), password (64), key (32).
const SETTINGS_WORDS: usize = 34;

impl Settings {
    fn load(flash: &mut FlashStorage) -> Settings {
        let mut w = [0u32; SETTINGS_WORDS];
        // SAFETY: the u32 array viewed as its bytes, word-aligned as the ROM read needs.
        let bytes = unsafe { core::slice::from_raw_parts_mut(w.as_mut_ptr() as *mut u8, SETTINGS_WORDS * 4) };
        let empty = Settings { ssid: heapless::String::new(), password: heapless::String::new(), key: None, backlight: 255 };
        if flash.read_nor(SETTINGS_ADDR, bytes).is_err() || w[0] != SETTINGS_MAGIC {
            return empty;
        }
        let (ssid_len, pass_len, has_key) = ((w[1] & 0xFF) as usize, (w[1] >> 8 & 0xFF) as usize, w[1] >> 16 & 1 == 1);
        let ssid = core::str::from_utf8(&bytes[8..8 + ssid_len.min(32)]).ok().and_then(|s| s.try_into().ok());
        let password = core::str::from_utf8(&bytes[40..40 + pass_len.min(64)]).ok().and_then(|s| s.try_into().ok());
        let (Some(ssid), Some(password)) = (ssid, password) else {
            return empty;
        };
        let key = has_key.then(|| bytes[104..136].try_into().unwrap());
        // Bit 17 says the backlight was saved; without it, full.
        let backlight = if w[1] >> 17 & 1 == 1 { (w[1] >> 24) as u8 } else { 255 };
        Settings { ssid, password, key, backlight }
    }

    fn save(&self, flash: &mut FlashStorage) -> bool {
        let mut w = [0u32; SETTINGS_WORDS];
        // SAFETY: as in `load`.
        let bytes = unsafe { core::slice::from_raw_parts_mut(w.as_mut_ptr() as *mut u8, SETTINGS_WORDS * 4) };
        bytes[8..8 + self.ssid.len()].copy_from_slice(self.ssid.as_bytes());
        bytes[40..40 + self.password.len()].copy_from_slice(self.password.as_bytes());
        if let Some(k) = self.key {
            bytes[104..136].copy_from_slice(&k);
        }
        w[0] = SETTINGS_MAGIC;
        w[1] = self.ssid.len() as u32 | (self.password.len() as u32) << 8 | (self.key.is_some() as u32) << 16 | 1 << 17 | (self.backlight as u32) << 24;
        let bytes = unsafe { core::slice::from_raw_parts(w.as_ptr() as *const u8, SETTINGS_WORDS * 4) };
        flash.erase(SETTINGS_ADDR, SETTINGS_ADDR + FlashStorage::SECTOR_SIZE).is_ok() && flash.write_nor(SETTINGS_ADDR, bytes).is_ok()
    }
}

/// Everything the tasks share.
struct Board {
    lcd: St7789<'static>,
    io: Expander<'static>,
    flash: FlashStorage<'static>,
    frame: &'static mut [u8; FRAME_BYTES],
    settings: Settings,
    cells: &'static mut [u8; CELLS_BYTES],
    /// The cells' BLAKE3, once a whole set is in RAM.
    cells_hash: Option<[u8; 32]>,
    /// Whether the panel shows a frame (not a test patch), so the signal cell belongs on it.
    ribbon: bool,
    /// The signal digit drawn now, if any.
    cell: Option<u8>,
}

/// Keep `data` in flash at `at`: header sector (magic and BLAKE3) erased first, data written, header last, so a write cut short is never taken for whole.
fn keep(flash: &mut FlashStorage, at: u32, magic: u32, hash: [u8; 32], data: &[u8]) -> bool {
    let mut header = [0u32; 9];
    header[0] = magic;
    for (i, w) in hash.chunks_exact(4).enumerate() {
        header[1 + i] = u32::from_le_bytes(w.try_into().unwrap());
    }
    // SAFETY: the u32 array viewed as its bytes, word-aligned as the ROM write needs.
    let header = unsafe { core::slice::from_raw_parts(header.as_ptr() as *const u8, 36) };
    let data_at = at + FlashStorage::SECTOR_SIZE;
    let end = data_at + (data.len() as u32).div_ceil(FlashStorage::SECTOR_SIZE) * FlashStorage::SECTOR_SIZE;
    flash.erase(at, end).is_ok() && flash.write_nor(data_at, data).is_ok() && flash.write_nor(at, header).is_ok()
}

/// Read back what `keep` wrote at `at` into `data`: its BLAKE3, if the header is there and the data matches it.
fn load(flash: &mut FlashStorage, at: u32, magic: u32, data: &mut [u8]) -> Option<[u8; 32]> {
    let mut header = [0u32; 9];
    // SAFETY: as in `keep`.
    let bytes = unsafe { core::slice::from_raw_parts_mut(header.as_mut_ptr() as *mut u8, 36) };
    if flash.read_nor(at, bytes).is_err() || header[0] != magic || flash.read_nor(at + FlashStorage::SECTOR_SIZE, data).is_err() {
        return None;
    }
    let hash: [u8; 32] = bytes[4..36].try_into().unwrap();
    (blake3::hash(data).as_bytes() == &hash).then_some(hash)
}

type Shared = Mutex<CriticalSectionRawMutex, Board>;
static BOARD: StaticCell<Shared> = StaticCell::new();

impl Board {
    /// Draw the frame buffer, then have the signal task put its cell back on top.
    fn draw(&mut self) {
        self.lcd.begin(0, 0, WIDTH, HEIGHT);
        self.lcd.push(&self.frame[..]);
        self.cell = None;
        REDRAW.signal(());
    }

    /// Keep the frame in flash.
    fn persist(&mut self, hash: [u8; 32]) -> bool {
        keep(&mut self.flash, STORE, STORE_MAGIC, hash, &self.frame[..])
    }

    /// The kept frame, if it's there whole.
    fn restore(&mut self) -> bool {
        load(&mut self.flash, STORE, STORE_MAGIC, &mut self.frame[..]).is_some()
    }

    /// The kept signal cells, if they're there whole.
    fn restore_cells(&mut self) {
        self.cells_hash = load(&mut self.flash, CELLS_STORE, CELLS_MAGIC, &mut self.cells[..]);
    }

    /// Draw the cell for signal `digit` over the frame's top-left corner, unless it's there already (or there's no frame or no cells).
    fn show_signal(&mut self, digit: u8) {
        if !self.ribbon || self.cells_hash.is_none() || self.cell == Some(digit) {
            return;
        }
        let at = digit as usize * CELL_BYTES;
        self.lcd.begin(0, 0, CELL_W, CELL_H);
        self.lcd.push(&self.cells[at..at + CELL_BYTES]);
        self.cell = Some(digit);
    }

    /// Fill the panel with black and an s×s square of one colour at its centre (s = 0: all black; s ≥ 172: the whole panel).
    fn patch(&mut self, s: u16, (r, g, b): (u8, u8, u8)) {
        let c = ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3);
        let full = s >= WIDTH;
        let (x0, y0) = ((WIDTH - s.min(WIDTH)) / 2, (HEIGHT - s.min(HEIGHT)) / 2);
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                let on = full || (x >= x0 && x < x0 + s && y >= y0 && y < y0 + s);
                let p = if on { c } else { 0 };
                let i = 2 * (y as usize * WIDTH as usize + x as usize);
                self.frame[i] = (p >> 8) as u8;
                self.frame[i + 1] = p as u8;
            }
        }
        self.ribbon = false;
        self.draw();
    }
}

/// Keep the signal cell on the frame matching the board's own Wi-Fi signal: checked every second, and straight after each new frame.
#[embassy_executor::task]
async fn signal(board: &'static Shared) {
    loop {
        let digit = signal_digit(net::rssi());
        board.lock().await.show_signal(digit);
        let _ = with_timeout(Duration::from_secs(1), REDRAW.wait()).await;
    }
}


#[allow(clippy::large_stack_frames, reason = "it's not unusual to allocate larger buffers etc. in main")]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    // The Wi-Fi driver allocates; give it the RAM the bootloader used, plus a little more.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 48 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
    let delay = Delay::new();

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default().with_frequency(Rate::from_khz(400))).expect("I2C").with_sda(peripherals.GPIO4).with_scl(peripherals.GPIO3);
    let mut io = Expander::new(i2c);
    io.backlight(0);
    io.set(expander::SD_CS, true);
    io.set(expander::LCD_CS, false);
    io.set(expander::LCD_RST, false);
    delay.delay_millis(10);
    io.set(expander::LCD_RST, true);
    delay.delay_millis(120);
    let spi = Spi::new(peripherals.SPI2, SpiConfig::default().with_frequency(Rate::from_mhz(40)).with_mode(Mode::_0))
        .expect("SPI")
        .with_sck(peripherals.GPIO7)
        .with_mosi(peripherals.GPIO5)
        .with_dma(peripherals.DMA_CH0)
        .with_buffers(dma_rx_buffer!(64).expect("rx buf"), dma_tx_buffer!(4092).expect("tx buf"));
    let dc = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    let lcd = St7789::new(spi, dc, &delay);

    let mut flash = FlashStorage::new(peripherals.FLASH);
    let settings = Settings::load(&mut flash);
    let wifi = settings.clone();
    let board = BOARD.init(Mutex::new(Board { lcd, io, flash, frame: &mut FRAME.take().0, settings, cells: &mut CELLS.take().0, cells_hash: None, ribbon: true, cell: None }));
    {
        let mut b = board.lock().await;
        if !b.restore() {
            b.frame.fill(0);
        }
        b.restore_cells();
        b.draw();
        let duty = b.settings.backlight;
        b.io.backlight(duty);
    }
    println!("LCD up: {WIDTH}x{HEIGHT}");
    spawner.spawn(signal(board).expect("signal task"));
    if wifi.ssid.is_empty() {
        println!("no Wi-Fi network set: `wifi <ssid> <password>` over USB");
    } else {
        net::start(&spawner, peripherals.WIFI, &wifi.ssid, &wifi.password, board);
    }

    let (rx, _tx) = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async().split();
    let mut usb = Conn::new(Io::Usb(rx));
    println!("ready");
    loop {
        let _ = serve(&mut usb, board, true).await;
    }
}

/// A byte stream the command protocol runs over: the USB serial link or a TCP connection.
enum Io<'a, 'b> {
    Usb(UsbSerialJtagRx<'static, Async>),
    Tcp(&'a mut embassy_net::tcp::TcpSocket<'b>),
}

impl Io<'_, '_> {
    /// Read at least one byte; `Err` once the link is gone.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ()> {
        match self {
            Io::Usb(rx) => embedded_io_async::Read::read(rx, buf).await.map_err(|_| ()),
            Io::Tcp(socket) => socket.read(buf).await.map_err(|_| ()),
        }
    }

    async fn write(&mut self, mut data: &[u8]) -> Result<(), ()> {
        match self {
            Io::Usb(_) => {
                esp_println::Printer::write_bytes(data);
                Ok(())
            }
            Io::Tcp(socket) => {
                while !data.is_empty() {
                    let n = socket.write(data).await.map_err(|_| ())?;
                    data = &data[n..];
                }
                socket.flush().await.map_err(|_| ())
            }
        }
    }
}

/// Longest command line: `d` and a 64-digit hash, with room to spare.
const LINE: usize = 112;

struct Line {
    buf: [u8; LINE],
    len: usize,
}

/// A link with a small read buffer, so a command line and the frame data right behind it can arrive in any chunking.
struct Conn<'a, 'b> {
    io: Io<'a, 'b>,
    buf: [u8; 256],
    at: usize,
    end: usize,
}

impl<'a, 'b> Conn<'a, 'b> {
    fn new(io: Io<'a, 'b>) -> Self {
        Conn { io, buf: [0; 256], at: 0, end: 0 }
    }

    async fn byte(&mut self) -> Result<u8, ()> {
        if self.at == self.end {
            let n = self.io.read(&mut self.buf).await?;
            if n == 0 {
                return Err(());
            }
            (self.at, self.end) = (0, n);
        }
        self.at += 1;
        Ok(self.buf[self.at - 1])
    }

    /// The next non-empty line, without its line ending; overlong lines are cut at LINE bytes.
    async fn line(&mut self) -> Result<Line, ()> {
        let mut line = Line { buf: [0; LINE], len: 0 };
        loop {
            match self.byte().await? {
                b'\n' | b'\r' if line.len > 0 => return Ok(line),
                b'\n' | b'\r' => {}
                b if line.len < LINE => {
                    line.buf[line.len] = b;
                    line.len += 1;
                }
                _ => {}
            }
        }
    }

    /// Fill `out` exactly: whatever's buffered first, then straight from the link.
    async fn read_exact(&mut self, out: &mut [u8]) -> Result<(), ()> {
        let have = (self.end - self.at).min(out.len());
        out[..have].copy_from_slice(&self.buf[self.at..self.at + have]);
        self.at += have;
        let mut got = have;
        while got < out.len() {
            let n = self.io.read(&mut out[got..]).await?;
            if n == 0 {
                return Err(());
            }
            got += n;
        }
        Ok(())
    }

    async fn reply(&mut self, args: core::fmt::Arguments<'_>) -> Result<(), ()> {
        let mut s = heapless::String::<160>::new();
        let _ = s.write_fmt(args);
        let _ = s.push('\n');
        self.io.write(s.as_bytes()).await
    }
}

/// Hex digits, lowercase.
struct Hex<'a>(&'a [u8]);

impl core::fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.iter().try_for_each(|b| write!(f, "{b:02x}"))
    }
}

fn unhex32(hex: &str) -> Option<[u8; 32]> {
    let digits = hex.as_bytes();
    if digits.len() != 64 {
        return None;
    }
    let nibble = |c: u8| match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        _ => None,
    };
    let mut out = [0u8; 32];
    for (i, pair) in digits.chunks_exact(2).enumerate() {
        out[i] = nibble(pair[0])? << 4 | nibble(pair[1])?;
    }
    Some(out)
}

/// Up to N decimal arguments: the values and how many there were, or `None` if one isn't a number or there are too many.
fn numbers<'a, const N: usize>(words: impl Iterator<Item = &'a str>) -> Option<([u16; N], usize)> {
    let mut a = [0u16; N];
    let mut n = 0;
    for w in words {
        if n == N {
            return None;
        }
        a[n] = w.parse().ok()?;
        n += 1;
    }
    Some((a, n))
}

/// Answer commands on one link until it drops. `usb` is the trusted local link: no challenge, and the setup and patch commands are allowed.
async fn serve(conn: &mut Conn<'_, '_>, board: &'static Shared, usb: bool) -> Result<(), ()> {
    let mut nonce = [0u8; 32];
    let mut authed = usb;
    if !usb {
        esp_hal::rng::Rng::new().read(&mut nonce);
        conn.reply(format_args!("hello {}", Hex(&nonce))).await?;
    }
    loop {
        let line = conn.line().await?;
        let Ok(text) = core::str::from_utf8(&line.buf[..line.len]) else {
            conn.reply(format_args!("err utf8")).await?;
            continue;
        };
        let mut words = text.split_ascii_whitespace();
        let op = words.next().unwrap_or("");
        match op {
            "ping" => conn.reply(format_args!("ok ping")).await?,
            "auth" if !usb => {
                let key = board.lock().await.settings.key;
                let answer = words.next().and_then(unhex32);
                authed = matches!((key, answer), (Some(k), Some(a)) if blake3::keyed_hash(&k, &nonce) == blake3::Hash::from_bytes(a));
                conn.reply(format_args!("{} auth", if authed { "ok" } else { "err" })).await?;
            }
            _ if !authed => conn.reply(format_args!("err auth first")).await?,
            "d" => match words.next().and_then(unhex32) {
                Some(hash) => receive_frame(conn, board, hash).await?,
                None => conn.reply(format_args!("err d <blake3>")).await?,
            },
            "g" => match words.next().and_then(unhex32) {
                Some(hash) => receive_cells(conn, board, hash).await?,
                None => conn.reply(format_args!("err g <blake3>")).await?,
            },
            "b" => match numbers::<1>(words) {
                Some((a, 1)) if a[0] <= 255 => {
                    let mut b = board.lock().await;
                    b.io.backlight(a[0] as u8);
                    b.settings.backlight = a[0] as u8;
                    let s = b.settings.clone();
                    s.save(&mut b.flash);
                    drop(b);
                    conn.reply(format_args!("ok {text}")).await?;
                }
                _ => conn.reply(format_args!("err {text}")).await?,
            },
            "info" => {
                let (b, cells) = {
                    let b = board.lock().await;
                    (b.settings.backlight, b.cells_hash)
                };
                let mut c = heapless::String::<16>::new();
                match cells {
                    Some(h) => {
                        let _ = write!(c, "{}", Hex(&h[..8]));
                    }
                    None => {
                        let _ = c.push('-');
                    }
                }
                let ip = net::address();
                conn.reply(format_args!("ok info backlight {b} rssi {} cells {c} ip {}", net::rssi().unwrap_or(0), ip.map_or(heapless::String::<16>::new(), |a| {
                    let mut s = heapless::String::new();
                    let _ = write!(s, "{a}");
                    s
                })))
                .await?;
            }
            _ if !usb => conn.reply(format_args!("err {text}")).await?,
            "wifi" => {
                let (Some(ssid), Some(pass), None) = (words.next(), words.next(), words.next()) else {
                    conn.reply(format_args!("err wifi <ssid> <password>")).await?;
                    continue;
                };
                let mut b = board.lock().await;
                let (Ok(ssid), Ok(password)) = (ssid.try_into(), pass.try_into()) else {
                    conn.reply(format_args!("err wifi: ssid ≤ 32 bytes, password ≤ 64")).await?;
                    continue;
                };
                b.settings.ssid = ssid;
                b.settings.password = password;
                let ok = {
                    let s = b.settings.clone();
                    s.save(&mut b.flash)
                };
                conn.reply(format_args!("{} wifi; restarting", if ok { "ok" } else { "err" })).await?;
                Timer::after(Duration::from_millis(100)).await;
                esp_hal::system::software_reset();
            }
            "key" => match (words.next().and_then(unhex32), words.next()) {
                (Some(k), None) => {
                    let mut b = board.lock().await;
                    b.settings.key = Some(k);
                    let ok = {
                        let s = b.settings.clone();
                        s.save(&mut b.flash)
                    };
                    conn.reply(format_args!("{} key", if ok { "ok" } else { "err" })).await?;
                }
                _ => conn.reply(format_args!("err key <64 hex digits>")).await?,
            },
            "net" => conn.reply(format_args!("ok net {:?}", net::address())).await?,
            "f" | "w" => {
                let n = if op == "f" { 3 } else { 4 };
                match numbers::<4>(words) {
                    Some((a, c)) if c == n => {
                        let ch = |v: u16| v.min(255) as u8;
                        let (s, rgb) = if op == "f" { (WIDTH, (ch(a[0]), ch(a[1]), ch(a[2]))) } else { (a[0], (ch(a[1]), ch(a[2]), ch(a[3]))) };
                        board.lock().await.patch(s, rgb);
                        conn.reply(format_args!("ok {text}")).await?;
                    }
                    _ => conn.reply(format_args!("err {text}")).await?,
                }
            }
            _ => conn.reply(format_args!("err {text}")).await?,
        }
    }
}

/// `d hash`: read one whole frame into RAM, check its BLAKE3, then draw it and keep it in flash. A frame that arrives damaged or short is never drawn.
async fn receive_frame(conn: &mut Conn<'_, '_>, board: &'static Shared, hash: [u8; 32]) -> Result<(), ()> {
    let mut b = board.lock().await;
    match with_timeout(Duration::from_secs(20), conn.read_exact(&mut b.frame[..])).await {
        Ok(r) => r?,
        Err(_) => {
            drop(b);
            return conn.reply(format_args!("err d stalled")).await;
        }
    }
    if blake3::hash(&b.frame[..]) != blake3::Hash::from_bytes(hash) {
        // The frame buffer now holds damaged data: put the kept frame back.
        if !b.restore() {
            b.frame.fill(0);
        }
        drop(b);
        return conn.reply(format_args!("err d hash mismatch")).await;
    }
    b.ribbon = true;
    b.draw();
    let kept = b.persist(hash);
    drop(b);
    conn.reply(format_args!("{} d", if kept { "ok" } else { "err" })).await
}

/// `g hash`: read the signal cells into RAM, check their BLAKE3, keep them in flash and redraw the cell. Cells that arrive damaged or short are dropped for the kept ones.
async fn receive_cells(conn: &mut Conn<'_, '_>, board: &'static Shared, hash: [u8; 32]) -> Result<(), ()> {
    let mut guard = board.lock().await;
    let b = &mut *guard;
    b.cells_hash = None;
    let problem = match with_timeout(Duration::from_secs(20), conn.read_exact(&mut b.cells[..])).await {
        Ok(Ok(())) if blake3::hash(&b.cells[..]) == blake3::Hash::from_bytes(hash) => None,
        Ok(Ok(())) => Some("hash mismatch"),
        Ok(Err(())) => {
            b.restore_cells();
            return Err(());
        }
        Err(_) => Some("stalled"),
    };
    if let Some(problem) = problem {
        b.restore_cells();
        drop(guard);
        return conn.reply(format_args!("err g {problem}")).await;
    }
    b.cells_hash = Some(hash);
    let kept = keep(&mut b.flash, CELLS_STORE, CELLS_MAGIC, hash, &b.cells[..]);
    b.cell = None;
    drop(guard);
    REDRAW.signal(());
    conn.reply(format_args!("{} g", if kept { "ok" } else { "err" })).await
}
