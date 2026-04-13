use std::{
	ops::Mul,
	time::{Duration, Instant},
};

use serde::{Deserialize, Serialize};
use vexide::{
	controller::ControllerState, prelude::OpticalSensor, smart::motor::BrakeMode, time::sleep,
};
use vexide_motorgroup::MotorGroup;

use crate::{
	ControllableSubsystem, ControllerConfiguration,
	pnemuatics::{AdiPneumatic, PneumaticState},
};

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

#[derive(PartialEq, Eq, PartialOrd, Ord, Debug)]
enum ParkState {
	Disabled,
	Outtaking,
	Parked,
	Manual,
}

pub struct IntakeSubsystem {
	motors: IntakeMotors,
	state: IntakeState,
	park_state: ParkState,
	park_sensor: Option<OpticalSensor>,
	park_piston: Option<AdiPneumatic>,
}

impl IntakeSubsystem {
	pub fn new(
		motors: IntakeMotors,
		park_sensor: Option<OpticalSensor>,
		park_piston: Option<AdiPneumatic>,
	) -> Self {
		Self {
			motors,
			state: IntakeState::full_brake(),
			park_state: ParkState::Disabled,
			park_sensor,
			park_piston,
		}
	}

	pub fn run(&mut self, state: IntakeState) {
		match self.park_state {
			ParkState::Disabled => {
				if let Some(p) = &mut self.park_piston {
					_ = p.set_state(PneumaticState::Contracted);
				}

				_ = self
					.motors
					.top
					.set_voltage(state.top * self.motors.top.max_voltage());
				_ = self
					.motors
					.middle
					.set_voltage(state.middle * self.motors.middle.max_voltage());
				_ = self
					.motors
					.bottom
					.set_voltage(state.bottom * self.motors.bottom.max_voltage());
			}
			ParkState::Outtaking => {
				if let Some(proximity) = self.park_sensor().and_then(|p| p.proximity().ok())
					&& proximity < 0.75
				{
					if let Some(p) = &mut self.park_piston {
						_ = p.set_state(PneumaticState::Contracted);
					}
					_ = self.motors.top.set_voltage(-self.motors.top.max_voltage());
					_ = self
						.motors
						.middle
						.set_voltage(-self.motors.middle.max_voltage());
					_ = self.motors.bottom.set_voltage(-0.25);
				} else {
					self.park_state = ParkState::Parked;
					_ = self
						.park_piston
						.as_mut()
						.and_then(|p| p.set_state(PneumaticState::Extended).ok());
					_ = self.motors.bottom.brake(BrakeMode::Hold);
				}
			}
			ParkState::Parked => {
				_ = self.motors.top.brake(BrakeMode::Coast);
				_ = self.motors.middle.brake(BrakeMode::Coast);
				_ = self.motors.bottom.brake(BrakeMode::Hold);
			}
			ParkState::Manual => {
				if let Some(p) = &mut self.park_piston {
					_ = p.set_state(PneumaticState::Extended);
				}
			}
		}
	}

	pub async fn run_unjamming(&mut self, state: IntakeState, duration: Duration) {
		const TOTAL_PERIOD: u128 = 1500;
		const UNJAM_PERIOD: u128 = 300;
		let start = Instant::now();
		while start.elapsed() < duration {
			let state =
				if (start.elapsed().as_millis() % TOTAL_PERIOD) >= (TOTAL_PERIOD - UNJAM_PERIOD) {
					state * -1.0
				} else {
					state
				};
			self.run(state);
			sleep(Duration::from_millis(25)).await;
		}
		self.run(IntakeState::full_brake());
	}

	pub fn park_sensor(&self) -> Option<&OpticalSensor> {
		self.park_sensor.as_ref()
	}

	pub fn park_piston(&mut self) -> Option<&mut AdiPneumatic> {
		self.park_piston.as_mut()
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
			IntakeState::full_forward() * 1.0
		} else if controller.button_r2.is_pressed() {
			// Run at half speed for bottom center goal outtake
			IntakeState {
				top: -1.0,
				middle: -1.0,
				bottom: -0.5,
			}
		} else {
			IntakeState::full_brake()
		};

		if self.park_sensor.is_some() {
			if controller.button_left.is_now_pressed() {
				self.park_state = match self.park_state {
					ParkState::Disabled | ParkState::Manual => ParkState::Outtaking,
					_ => ParkState::Disabled,
				}
			} else if controller.button_down.is_now_pressed() {
				self.park_state = match self.park_state {
					ParkState::Disabled => ParkState::Manual,
					_ => ParkState::Disabled,
				}
			}
		}

		self.run(self.state);
	}

	fn state(&self) -> Option<ciborium::Value> {
		Some(
			ciborium::Value::serialized(&self.state)
				.expect("Serializing state enum should succeed"),
		)
	}
}
