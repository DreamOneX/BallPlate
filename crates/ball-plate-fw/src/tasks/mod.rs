pub mod control;
pub mod camera;
pub mod led;

#[cfg(feature = "remote-uart")]
pub mod remote_uart;

#[cfg(feature = "telemetry")]
pub mod telemetry;

pub mod prelude {
    pub use super::{camera::camera_task, control::control_task, led::led_task};

    #[cfg(feature = "remote-uart")]
    pub use super::remote_uart::remote_uart_task;

    #[cfg(feature = "telemetry")]
    pub use super::telemetry::telemetry_task;
}
