//! The 1.47" 172×320 IPS LCD's ST7789 controller on SPI (data/command on its own pin; chip select and reset come from the I/O expander). The init sequence is Waveshare's; the panel sits at column 34 of the controller's 240.

use esp_hal::Blocking;
use esp_hal::delay::Delay;
use esp_hal::gpio::Output;
use esp_hal::spi::master::SpiDma;

pub const WIDTH: u16 = 172;
pub const HEIGHT: u16 = 320;
const X_OFFSET: u16 = 34;
const Y_OFFSET: u16 = 0;
/// Memory access control: column order mirrored, BGR, as Waveshare sets it.
const MADCTL: u8 = 0x48;

pub struct St7789<'d> {
    spi: SpiDma<'d, Blocking>,
    dc: Output<'d>,
}

impl<'d> St7789<'d> {
    /// Bring the controller up (it must already be out of reset, with chip select low).
    pub fn new(spi: SpiDma<'d, Blocking>, dc: Output<'d>, delay: &Delay) -> Self {
        let mut lcd = St7789 { spi, dc };
        lcd.command(0x11, &[]); // sleep out
        delay.delay_millis(120);
        lcd.command(0x36, &[MADCTL]);
        lcd.command(0x3A, &[0x55]); // 16 bits per pixel
        // RAM control: Waveshare sets 0xE8, whose bit 3 makes the panel take 16-bit pixels low byte first; frames here are high byte first.
        lcd.command(0xB0, &[0x00, 0xE0]);
        lcd.command(0xB2, &[0x0C, 0x0C, 0x00, 0x33, 0x33]);
        lcd.command(0xB7, &[0x75]);
        lcd.command(0xBB, &[0x1A]);
        lcd.command(0xC0, &[0x80]);
        lcd.command(0xC2, &[0x01, 0xFF]);
        lcd.command(0xC3, &[0x13]);
        lcd.command(0xC4, &[0x20]);
        lcd.command(0xC6, &[0x0F]);
        lcd.command(0xD0, &[0xA4, 0xA1]);
        lcd.command(0xE0, &[0xD0, 0x0D, 0x14, 0x0D, 0x0D, 0x09, 0x38, 0x44, 0x4E, 0x3A, 0x17, 0x18, 0x2F, 0x30]);
        lcd.command(0xE1, &[0xD0, 0x09, 0x0F, 0x08, 0x07, 0x14, 0x37, 0x44, 0x4D, 0x38, 0x15, 0x16, 0x2C, 0x2E]);
        lcd.command(0x21, &[]); // inversion on (this IPS panel needs it for true colours)
        lcd.command(0x29, &[]); // display on
        lcd
    }

    pub fn command(&mut self, cmd: u8, params: &[u8]) {
        self.dc.set_low();
        let _ = self.spi.write(&[cmd]);
        self.dc.set_high();
        if !params.is_empty() {
            let _ = self.spi.write(params);
        }
    }

    /// Start a w×h rectangle at (x, y); follow with `push` of exactly w·h·2 bytes (RGB565, high byte first).
    pub fn begin(&mut self, x: u16, y: u16, w: u16, h: u16) {
        let (x0, x1) = (x + X_OFFSET, x + X_OFFSET + w - 1);
        let (y0, y1) = (y + Y_OFFSET, y + Y_OFFSET + h - 1);
        self.command(0x2A, &[(x0 >> 8) as u8, x0 as u8, (x1 >> 8) as u8, x1 as u8]);
        self.command(0x2B, &[(y0 >> 8) as u8, y0 as u8, (y1 >> 8) as u8, y1 as u8]);
        self.command(0x2C, &[]);
    }

    pub fn push(&mut self, bytes: &[u8]) {
        let _ = self.spi.write(bytes);
    }
}
