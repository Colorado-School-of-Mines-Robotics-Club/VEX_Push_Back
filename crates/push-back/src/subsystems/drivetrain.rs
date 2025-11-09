use std::ops::{Deref, DerefMut};

use evian::{
    prelude::{Arcade, Drivetrain, Tank},
    tracking::Tracking,
};
use vexide::controller::ControllerState;

use crate::subsystems::ControllableSubsystem;

#[allow(dead_code)]
#[derive(Clone, Copy, Default, Debug)]
pub enum DrivetrainControlSystem {
    JointArcade,
    SplitArcade,
    #[default]
    Tank,
}

pub struct DrivetrainSubsystem<M: Arcade + Tank, T: Tracking> {
    pub drivetrain: Drivetrain<M, T>,
    pub configuration: DrivetrainControlSystem,
}

impl<M: Arcade + Tank, T: Tracking> DrivetrainSubsystem<M, T> {
    pub fn new(drivetrain: M, tracking: T) -> Self {
        Self {
            drivetrain: Drivetrain::new(drivetrain, tracking),
            configuration: DrivetrainControlSystem::default(),
        }
    }
}

impl<M: Arcade + Tank, T: Tracking> ControllableSubsystem for DrivetrainSubsystem<M, T> {
    fn control(&mut self, controller: &ControllerState) {
        _ = match self.configuration {
            DrivetrainControlSystem::JointArcade => self
                .drivetrain
                .model
                .drive_arcade(controller.left_stick.y(), controller.left_stick.x()),
            DrivetrainControlSystem::SplitArcade => self
                .drivetrain
                .model
                .drive_arcade(controller.left_stick.y(), controller.right_stick.x()),
            DrivetrainControlSystem::Tank => self
                .drivetrain
                .model
                .drive_tank(controller.left_stick.y(), controller.right_stick.y()),
        };
    }
}

impl<M: Arcade + Tank, T: Tracking> Deref for DrivetrainSubsystem<M, T> {
    type Target = Drivetrain<M, T>;

    fn deref(&self) -> &Self::Target {
        &self.drivetrain
    }
}

impl<M: Arcade + Tank, T: Tracking> DerefMut for DrivetrainSubsystem<M, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.drivetrain
    }
}
