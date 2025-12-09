use std::ops::{Deref, DerefMut};

use evian::{
    drivetrain::model::DrivetrainModel,
    math::desaturate,
    prelude::{Drivetrain, Tank},
    tracking::Tracking,
};
use vexide::controller::ControllerState;

use crate::subsystems::{ControllableSubsystem, ControllerConfiguration};

fn arcade(throttle: f64, steer: f64) -> (f64, f64) {
    desaturate([throttle + steer, throttle - steer], 1_f64).into()
}

pub struct DrivetrainSubsystem<M: Tank, T: Tracking> {
    pub drivetrain: Drivetrain<M, T>,
    /// The state of the drivetrain as (left, right), both should be desaturate before use
    pub state: (f64, f64),
}

impl<M: Tank, T: Tracking> DrivetrainSubsystem<M, T> {
    pub fn new(drivetrain: M, tracking: T) -> Self {
        Self {
            drivetrain: Drivetrain::new(drivetrain, tracking),
            state: Default::default(),
        }
    }

    pub fn run(&mut self) -> Result<(), <M as DrivetrainModel>::Error> {
        let [left, right] = desaturate(self.state.into(), 1_f64);

        self.drivetrain.model.drive_tank(left, right)
    }
}

impl<M: Tank, T: Tracking> ControllableSubsystem for DrivetrainSubsystem<M, T> {
    fn state(&self) -> Option<ciborium::Value> {
        Some(
            ciborium::Value::serialized(&self.state)
                .expect("Serializing drivetrain tuple should succeed"),
        )
    }

    fn direct(&mut self, state: &ciborium::Value) {
        if let Ok(parsed) = state.deserialized() {
            self.state = parsed;

            _ = self.run();
        }
    }

    fn control(&mut self, controller: &ControllerState, configuration: ControllerConfiguration) {
        let coefficient = if controller.button_down.is_pressed() {
            0.5
        } else {
            1.0
        };
        self.state = match configuration {
            ControllerConfiguration::Noah => arcade(
                // Split arcade
                coefficient * controller.left_stick.y(),
                coefficient * controller.right_stick.x(),
            ),
            ControllerConfiguration::Connor => (
                // Tank
                coefficient * controller.left_stick.y(),
                coefficient * controller.right_stick.y(),
            ),
        };

        _ = self.run();
    }
}

impl<M: Tank, T: Tracking> Deref for DrivetrainSubsystem<M, T> {
    type Target = Drivetrain<M, T>;

    fn deref(&self) -> &Self::Target {
        &self.drivetrain
    }
}

impl<M: Tank, T: Tracking> DerefMut for DrivetrainSubsystem<M, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.drivetrain
    }
}
