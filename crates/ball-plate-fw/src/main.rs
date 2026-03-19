#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Config as UartConfig, UartRx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use panic_probe as _;
use static_cell::StaticCell;

use ball_plate_fw::config;
use ball_plate_fw::drivers::pca9685::Pca9685;
use ball_plate_fw::shared::{ControlOutput, ControlRequest, Position, RunState};
use ball_plate_fw::tasks::prelude::*;

// ── Interrupt bindings ──────────────────────────────────────

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

// ── Shared state (static) ───────────────────────────────────

static POS_WATCH: StaticCell<Watch<CriticalSectionRawMutex, Position, 2>> = StaticCell::new();
static STATE_WATCH: StaticCell<Watch<CriticalSectionRawMutex, RunState, 1>> = StaticCell::new();
static OUT_WATCH: StaticCell<Watch<CriticalSectionRawMutex, ControlOutput, 1>> = StaticCell::new();
static REQ_CH: StaticCell<Channel<CriticalSectionRawMutex, ControlRequest, 4>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("ball-plate-fw boot");
    let p = embassy_stm32::init(Default::default());

    // ── LEDs (PF9 / PF10) ──────────────────────────────────
    let led1 = Output::new(p.PF9, Level::Low, Speed::Low);
    let led2 = Output::new(p.PF10, Level::Low, Speed::Low);

    // ── Shared state ────────────────────────────────────────
    let pos_watch: &'static _ = POS_WATCH.init(Watch::new());
    let state_watch: &'static _ = STATE_WATCH.init(Watch::new());
    let out_watch: &'static _ = OUT_WATCH.init(Watch::new());
    let req_ch: &'static _ = REQ_CH.init(Channel::new());

    // Start in Running state (matches C++ setup behaviour).
    state_watch.sender().send(RunState::Running);

    // ── I2C1 (PB6 SCL, PB7 SDA) + PCA9685 ─────────────────
    let i2c = I2c::new(
        p.I2C1,
        p.PB6,  // SCL
        p.PB7,  // SDA
        Irqs,
        p.DMA1_CH6, // TX DMA
        p.DMA1_CH0, // RX DMA
        Hertz(100_000),
        Default::default(),
    );

    let mut pca = Pca9685::new(i2c, config::PCA9685_ADDR);
    match pca.init(config::PCA9685_PWM_FREQ_HZ).await {
        Ok(()) => info!("PCA9685 init ok"),
        Err(e) => warn!("PCA9685 init err: {:?}", e),
    }

    // ── Camera UART: USART1 RX (PA10), DMA ring buffer ─────
    static CAMERA_DMA_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let camera_dma_buf = CAMERA_DMA_BUF.init([0u8; 64]);

    let mut cam_cfg = UartConfig::default();
    cam_cfg.baudrate = config::CAMERA_BAUD;

    let camera_rx = UartRx::new(p.USART1, Irqs, p.PA10, p.DMA2_CH2, cam_cfg)
        .unwrap()
        .into_ring_buffered(camera_dma_buf);

    // ── Host UART: USART2 (PA2 TX, PA3 RX) ─────────────────
    //     Used by telemetry (TX) and remote-uart (RX).
    #[cfg(any(feature = "remote-uart", feature = "telemetry"))]
    let (host_tx, host_rx) = {
        use embassy_stm32::usart::BufferedUart;

        static HOST_TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
        static HOST_RX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let host_tx_buf = HOST_TX_BUF.init([0u8; 256]);
        let host_rx_buf = HOST_RX_BUF.init([0u8; 64]);

        let mut host_cfg = UartConfig::default();
        host_cfg.baudrate = config::HOST_BAUD;

        let host_uart = BufferedUart::new(
            p.USART2, Irqs, p.PA3, p.PA2, host_tx_buf, host_rx_buf, host_cfg,
        )
        .unwrap();

        host_uart.split()
    };

    // ── Spawn tasks ─────────────────────────────────────────
    spawner.spawn(led_task(led1, led2, state_watch)).unwrap();
    spawner.spawn(camera_task(camera_rx, pos_watch)).unwrap();
    spawner
        .spawn(control_task(pca, pos_watch, state_watch, out_watch, req_ch))
        .unwrap();

    #[cfg(feature = "remote-uart")]
    spawner.spawn(remote_uart_task(host_rx, req_ch)).unwrap();

    #[cfg(feature = "telemetry")]
    spawner.spawn(telemetry_task(host_tx, out_watch)).unwrap();
}
