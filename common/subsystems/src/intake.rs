use std::ops::Mul;

use serde::{Deserialize, Serialize};
use vexide::controller::ControllerState;
use vexide_motorgroup::MotorGroup;

use crate::{ControllableSubsystem, ControllerConfiguration};

pub struct IntakeMotors {
	pub bottom: MotorGroup,
	pub middle: MotorGroup,
	pub top: MotorGroup,
}

#[derive(Clone, Copy, PartialEq, Debug, Serialize, Deserialize)]
/// Intake state, 1.0 = max forward, -1.0 = max reverse, 0.0 = stop
pub struct IntakeState {
	pub top: f64,
	pub middle: f64,
	pub bottom: f64,
}

impl IntakeState {
	pub const fn full_forward() -> Self {
		Self {
			top: 1.0,
			middle: 1.0,
			bottom: 1.0,
		}
	}

	pub const fn full_brake() -> Self {
		Self {
			top: 0.0,
			middle: 0.0,
			bottom: 0.0,
		}
	}

	/// Reverses all currently enabled motor groups. Any reversed motors will be un-reversed
	pub const fn reverse(self) -> Self {
		Self {
			top: -self.top,
			middle: -self.middle,
			bottom: -self.bottom,
		}
	}
}

impl Mul<f64> for IntakeState {
	type Output = Self;

	fn mul(mut self, rhs: f64) -> Self::Output {
		self.top *= rhs;
		self.middle *= rhs;
		self.bottom *= rhs;

		self
	}
}

pub struct IntakeSubsystem {
	motors: IntakeMotors,
	state: IntakeState,
}

impl IntakeSubsystem {
	pub fn new(motors: IntakeMotors) -> Self {
		Self {
			motors,
			state: IntakeState::full_brake(),
		}
	}

	pub fn run(&mut self, state: IntakeState) {
		_ = self.motors.top.set_voltage(state.top);
		_ = self.motors.middle.set_voltage(state.middle);
		_ = self.motors.bottom.set_voltage(state.bottom);
	}
}

impl ControllableSubsystem for IntakeSubsystem {
	fn direct(&mut self, state: &ciborium::Value) {
		if let Ok(parsed) = state.deserialized() {
			self.state = parsed;

			self.run(self.state);
		}
	}

	fn control(&mut self, controller: &ControllerState, _configuration: ControllerConfiguration) {
		self.state = if controller.button_r1.is_pressed() || controller.button_l1.is_pressed() {
			// Run at full speed for normal intake and long goal outtake
			IntakeState::full_forward()
		} else if controller.button_l2.is_pressed() {
			// Run at half speed for top center goal outtake
			IntakeState::full_forward() * 0.5
		} else if controller.button_r2.is_pressed() {
			// Run at half speed for bottom center goal outtake
			IntakeState::full_forward().reverse() * 0.5
		} else {
			IntakeState::full_brake()
		};

		self.run(self.state);
	}

	fn state(&self) -> Option<ciborium::Value> {
		Some(
			ciborium::Value::serialized(&self.state)
				.expect("Serializing state enum should succeed"),
		)
	}
}
