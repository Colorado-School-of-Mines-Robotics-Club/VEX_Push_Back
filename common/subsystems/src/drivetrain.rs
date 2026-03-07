use std::ops::{Deref, DerefMut};

use evian::{
	drivetrain::model::{Differential, DrivetrainModel},
	math::desaturate,
	prelude::{Drivetrain, Tank},
	tracking::Tracking,
};
use vexide::{controller::ControllerState, smart::motor::BrakeMode};

use crate::{ControllableSubsystem, ControllerConfiguration};

fn arcade(throttle: f64, steer: f64) -> (f64, f64) {
	desaturate([throttle + steer, throttle - steer], 1_f64).into()
}

pub struct DrivetrainSubsystem<M: Tank, T: Tracking> {
	pub drivetrain: Drivetrain<M, T>,
	pub reverse: bool,
	/// The state of the drivetrain as (left, right), both should be desaturated before use
	pub state: (f64, f64),
}

impl<M: Tank, T: Tracking> DrivetrainSubsystem<M, T> {
	pub fn new(drivetrain: M, tracking: T) -> Self {
		Self {
			drivetrain: Drivetrain::new(drivetrain, tracking),
			reverse: false,
			state: Default::default(),
		}
	}

	pub fn run(&mut self) -> Result<(), <M as DrivetrainModel>::Error> {
		let [left, right] = desaturate(self.state.into(), 1_f64);

		self.drivetrain.model.drive_tank(left, right)
	}
}

impl<T: Tracking> DrivetrainSubsystem<Differential, T> {
	pub fn brake(&mut self, mode: BrakeMode) {
		let mut left = self.model.left.borrow_mut();
		let mut right = self.model.right.borrow_mut();
		for motor in left.as_mut().iter_mut().chain(right.as_mut().iter_mut()) {
			_ = motor.brake(mode);
		}
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
		if controller.button_left.is_now_pressed() {
			self.reverse = !self.reverse;
		}

		self.state = match configuration {
			ControllerConfiguration::Noah => arcade(
				// Split arcade
				controller.left_stick.y() * if self.reverse { -1.0 } else { 1.0 },
				controller.right_stick.x(),
			),
			ControllerConfiguration::Connor => match self.reverse {
				false => (
					// Tank
					controller.left_stick.y(),
					controller.right_stick.y(),
				),
				true => (
					// Tank
					-controller.right_stick.y(),
					-controller.left_stick.y(),
				),
			},
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
