use vexide::controller::ControllerState;

pub mod copro;
pub mod drivetrain;
pub mod intake;
pub mod replay;
pub mod trunk;

pub trait ControllableSubsystem {
    /// Update the subsystem based on controller input
    fn update(&mut self, controller: &ControllerState);
    /// Get the current subsystem state (for recording or similar)
    fn state(&self) -> Option<ciborium::Value> {
        None
    }
}
