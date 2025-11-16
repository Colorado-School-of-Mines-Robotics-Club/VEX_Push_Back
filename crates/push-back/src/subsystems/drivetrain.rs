use std::ops::{Deref, DerefMut};

use evian::{
    drivetrain::model::DrivetrainModel,
    math::desaturate,
    prelude::{Drivetrain, Tank},
    tracking::Tracking,
};
use vexide::controller::ControllerState;

use crate::subsystems::ControllableSubsystem;

const fn arcade(throttle: i8, steer: i8) -> (i8, i8) {
    (throttle + steer, throttle - steer)
}

#[allow(dead_code)]
#[derive(Clone, Copy, Default, Debug)]
pub enum DrivetrainControlSystem {
    JointArcade,
    SplitArcade,
    #[default]
    Tank,
}

pub struct DrivetrainSubsystem<M: Tank, T: Tracking> {
    pub drivetrain: Drivetrain<M, T>,
    pub configuration: DrivetrainControlSystem,
    /// The state of the drivetrain as (left, right), both should be desaturate before use
    pub state: (i8, i8),
}

impl<M: Tank, T: Tracking> DrivetrainSubsystem<M, T> {
    pub fn new(drivetrain: M, tracking: T) -> Self {
        Self {
            drivetrain: Drivetrain::new(drivetrain, tracking),
            configuration: DrivetrainControlSystem::default(),
            state: Default::default(),
        }
    }

    pub fn run(&mut self, left: f64, right: f64) -> Result<(), <M as DrivetrainModel>::Error> {
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

    fn update(&mut self, controller: &ControllerState) {
        self.state = match self.configuration {
            DrivetrainControlSystem::JointArcade => {
                arcade(controller.left_stick.y_raw(), controller.left_stick.x_raw())
            }
            DrivetrainControlSystem::SplitArcade => arcade(
                controller.left_stick.y_raw(),
                controller.right_stick.x_raw(),
            ),
            DrivetrainControlSystem::Tank => (
                controller.left_stick.y_raw(),
                controller.right_stick.y_raw(),
            ),
        };

        let [left, right] = desaturate([self.state.0 as f64, self.state.1 as f64], 1_f64);
        _ = self.run(left, right);
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
