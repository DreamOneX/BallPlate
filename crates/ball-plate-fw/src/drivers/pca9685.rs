//! Minimal PCA9685 driver (async I2C).
//!
//! Only implements what we need initially:
//! - set PWM frequency
//! - set a channel pulse width in microseconds

use embassy_stm32::i2c::{Error as I2cError, I2c};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};

const REG_MODE1: u8 = 0x00;
const REG_MODE2: u8 = 0x01;
const REG_PRESCALE: u8 = 0xFE;
const REG_LED0_ON_L: u8 = 0x06;

const MODE1_RESTART: u8 = 0x80;
const MODE1_SLEEP: u8 = 0x10;
const MODE1_AI: u8 = 0x20;
const MODE1_ALLCALL: u8 = 0x01;

const MODE2_OUTDRV: u8 = 0x04;

const OSC_HZ: u32 = 27_000_000;

pub struct Pca9685<'d> {
    i2c: I2c<'d, Async>,
    addr: u8,
    pwm_freq_hz: u16,
}

impl<'d> Pca9685<'d> {
    pub fn new(i2c: I2c<'d, Async>, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            pwm_freq_hz: 50,
        }
    }

    pub fn free(self) -> I2c<'d, Async> {
        self.i2c
    }

    pub async fn init(&mut self, pwm_freq_hz: u16) -> Result<(), I2cError> {
        // Totem-pole outputs (recommended for servo driver boards)
        self.write_reg(REG_MODE2, MODE2_OUTDRV).await?;

        // Enable ALLCALL + auto-increment
        self.write_reg(REG_MODE1, MODE1_ALLCALL | MODE1_AI).await?;

        // Datasheet recommends a small delay after mode changes.
        Timer::after(Duration::from_millis(1)).await;

        self.set_pwm_freq(pwm_freq_hz).await
    }

    pub async fn set_pwm_freq(&mut self, freq_hz: u16) -> Result<(), I2cError> {
        // prescale = round(osc / (freq * 4096)) - 1
        let divisor = (freq_hz as u32).saturating_mul(4096);
        let prescale = ((OSC_HZ + divisor / 2) / divisor).saturating_sub(1);
        let prescale = prescale.clamp(3, 255) as u8;

        let oldmode = self.read_reg(REG_MODE1).await?;
        let sleep = (oldmode & !MODE1_RESTART) | MODE1_SLEEP;
        self.write_reg(REG_MODE1, sleep).await?;
        self.write_reg(REG_PRESCALE, prescale).await?;
        self.write_reg(REG_MODE1, oldmode).await?;

        // Wait for oscillator to stabilize.
        Timer::after(Duration::from_millis(5)).await;

        // Restart + auto-increment.
        self.write_reg(REG_MODE1, oldmode | MODE1_RESTART | MODE1_AI).await?;

        self.pwm_freq_hz = freq_hz;
        Ok(())
    }

    pub async fn set_channel_pwm(&mut self, channel: u8, on: u16, off: u16) -> Result<(), I2cError> {
        let reg = REG_LED0_ON_L.wrapping_add(channel.saturating_mul(4));
        let buf = [
            reg,
            (on & 0xFF) as u8,
            (on >> 8) as u8,
            (off & 0xFF) as u8,
            (off >> 8) as u8,
        ];
        self.i2c.write(self.addr, &buf).await
    }

    /// Set a servo pulse width in microseconds.
    pub async fn write_microseconds(&mut self, channel: u8, pulse_us: u16) -> Result<(), I2cError> {
        // ticks = pulse_us / (1e6 / (freq * 4096)) = pulse_us * freq * 4096 / 1e6
        let ticks = ((pulse_us as u64) * (self.pwm_freq_hz as u64) * 4096 + 500_000) / 1_000_000;
        let ticks = ticks.min(4095) as u16;
        self.set_channel_pwm(channel, 0, ticks).await
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), I2cError> {
        self.i2c.write(self.addr, &[reg, val]).await
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, I2cError> {
        let mut out = [0u8];
        self.i2c.write_read(self.addr, &[reg], &mut out).await?;
        Ok(out[0])
    }
}
