use std::fmt::Display;

use vexide::controller::ControllerState;

pub mod copro;
pub mod drivetrain;
pub mod intake;
// pub mod replay;
pub mod trunk;

#[derive(Clone, Copy, Debug)]
pub enum ControllerConfiguration {
    Noah,
    Connor
}

impl ControllerConfiguration {
    pub fn as_str(&self) -> &'static str {
        match self {
            ControllerConfiguration::Noah =>   "Noah  ",
            ControllerConfiguration::Connor => "Connor"
        }
    }

    pub fn next(&self) -> Self {
        match self {
            ControllerConfiguration::Noah => ControllerConfiguration::Connor,
            ControllerConfiguration::Connor => ControllerConfiguration::Noah,
        }
    }
}

pub trait ControllableSubsystem {
    /// Update the subsystem based on controller input
    fn update(&mut self, controller: &ControllerState, configuration: ControllerConfiguration);
    /// Get the current subsystem state (for recording or similar)
    fn state(&self) -> Option<ciborium::Value> {
        None
    }
}
