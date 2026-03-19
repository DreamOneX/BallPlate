//! Types shared across tasks.

pub use ball_plate_protocol::command::StopMode;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Position {
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunState {
    Running,
    Stopped,
}

impl Default for RunState {
    fn default() -> Self {
        Self::Running
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ControlOutput {
    pub pos: Position,
    pub angle_x: f32,
    pub angle_y: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlRequest {
    Stop(StopMode),
    Resume,
    TargetUpdate { x: f32, y: f32 },
}
