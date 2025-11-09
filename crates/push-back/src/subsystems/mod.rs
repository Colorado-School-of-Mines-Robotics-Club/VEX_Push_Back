use vexide::controller::ControllerState;

pub mod copro;
pub mod drivetrain;
pub mod intake;

pub trait ControllableSubsystem {
    fn control(&mut self, controller: &ControllerState);
}
