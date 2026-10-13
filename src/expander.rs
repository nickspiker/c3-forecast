//! The board's I/O expander (I2C address 0x24): it drives the LCD's chip select (IO0) and reset (IO1), the SD card's chip select (IO2), and the backlight through its PWM output. Protocol from Waveshare's demo: two-byte writes of register then value.

use esp_hal::Blocking;
use esp_hal::i2c::master::I2c;

const ADDR: u8 = 0x24;
const MODE: u8 = 0x02;
const OUTPUT: u8 = 0x03;
const PWM: u8 = 0x05;

pub const LCD_CS: u8 = 0;
pub const LCD_RST: u8 = 1;
pub const SD_CS: u8 = 2;

pub struct Expander<'d> {
    i2c: I2c<'d, Blocking>,
    out: u8,
}

impl<'d> Expander<'d> {
    /// All eight pins as outputs, at Waveshare's power-on levels (everything high but IO3).
    pub fn new(i2c: I2c<'d, Blocking>) -> Self {
        let mut e = Expander { i2c, out: 0xF7 };
        e.write(MODE, 0xFF);
        e.write(OUTPUT, e.out);
        e
    }

    fn write(&mut self, reg: u8, value: u8) {
        let _ = self.i2c.write(ADDR, &[reg, value]);
    }

    pub fn set(&mut self, pin: u8, high: bool) {
        if high {
            self.out |= 1 << pin;
        } else {
            self.out &= !(1 << pin);
        }
        self.write(OUTPUT, self.out);
    }

    /// Backlight duty, 0 (off) to 255 (full).
    pub fn backlight(&mut self, duty: u8) {
        self.write(PWM, duty);
    }
}
