use std::{
	ops::Mul,
	time::{Duration, Instant},
};

use futures_util::pending;
use serde::{Deserialize, Serialize};
use vexide::{controller::ControllerState, prelude::OpticalSensor, smart::motor::BrakeMode};
use vexide_motorgroup::MotorGroup;

use crate::{
	ControllableSubsystem, ControllerConfiguration,
	pnemuatics::{AdiPneumatic, PneumaticState},
};

#[macro_export]
macro_rules! intake_unjamming {
	($robot:expr, $state:expr, $r:ident => async $block:block) => {{
		use ::subsystems::futures_util::{FutureExt, pending, pin_mut, select};

		async fn fut(__robot: &mut Robot) {
			let mut $r = __robot;
			$block
		}

		async fn unjam(__robot: *mut Robot) {
			loop {
				{
					// SAFETY: this isn't. i fucking hate this but i don't know a better way
					unsafe {
						(*__robot).intake.run($state);
					}
				}
				pending!()
			}
		}

		$robot.intake.enable_unjam();

		{
			let unjam = unjam($robot as *mut Robot).fuse();
			let fut = fut($robot).fuse();

			::subsystems::futures_util::pin_mut!(fut, unjam);

			::subsystems::futures_util::select_biased! {
				_ = fut => (),
				_ = unjam => ()
			};
		}

		$robot.intake.disable_unjam();
	}};
}

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
	park_sensor: OpticalSensor,
	park_piston: AdiPneumatic,
	last_jiggle: Option<Instant>,
}

impl IntakeSubsystem {
	pub fn new(
		motors: IntakeMotors,
		park_sensor: OpticalSensor,
		park_piston: AdiPneumatic,
	) -> Self {
		Self {
			motors,
			state: IntakeState::full_brake(),
			park_state: ParkState::Disabled,
			park_sensor,
			park_piston,
			last_jiggle: None,
		}
	}

	pub fn run(&mut self, state: IntakeState) {
		match self.park_state {
			ParkState::Disabled => {
				_ = self.park_piston.set_state(PneumaticState::Contracted);

				let coeff = match self.last_jiggle.map(|j| j.elapsed()) {
					Some(last_jiggle) if last_jiggle >= Duration::from_millis(1300) => {
						self.last_jiggle = Some(Instant::now());
						1.0
					}
					Some(last_jiggle) if last_jiggle >= Duration::from_secs(1) => -0.3,
					_ => 1.0,
				};

				_ = self
					.motors
					.top
					.set_voltage(coeff * state.top * self.motors.top.max_voltage());
				_ = self
					.motors
					.middle
					.set_voltage(coeff * state.middle * self.motors.middle.max_voltage());
				_ = self
					.motors
					.bottom
					.set_voltage(coeff * state.bottom * self.motors.bottom.max_voltage());
			}
			ParkState::Outtaking => {
				if let Some(proximity) = self.park_sensor().proximity().ok()
					&& proximity < 0.85
				{
					_ = self.park_piston.set_state(PneumaticState::Contracted);
					_ = self.motors.top.set_voltage(-self.motors.top.max_voltage());
					_ = self
						.motors
						.middle
						.set_voltage(-self.motors.middle.max_voltage());
					_ = self.motors.bottom.set_velocity(-100);
				} else {
					self.park_state = ParkState::Parked;
					_ = self.park_piston.set_state(PneumaticState::Extended).ok();
					_ = self.motors.bottom.brake(BrakeMode::Hold);
				}
			}
			ParkState::Parked => {
				_ = self.motors.top.brake(BrakeMode::Coast);
				_ = self.motors.middle.brake(BrakeMode::Coast);
				_ = self.motors.bottom.brake(BrakeMode::Hold);
			}
			ParkState::Manual => {
				_ = self.park_piston.set_state(PneumaticState::Extended);
			}
		}
	}

	pub fn enable_unjam(&mut self) {
		self.last_jiggle = Some(Instant::now());
	}

	pub fn disable_unjam(&mut self) {
		self.last_jiggle = None;
	}

	pub fn park_sensor(&self) -> &OpticalSensor {
		&self.park_sensor
	}

	pub fn park_piston(&mut self) -> &mut AdiPneumatic {
		&mut self.park_piston
	}

	pub async fn run_autopark(&mut self) {
		self.park_state = ParkState::Outtaking;
		while self.park_state != ParkState::Parked {
			self.run(IntakeState::full_brake()); // full_break is just a placeholder, it isn't used during park
			pending!()
		}
		self.run(IntakeState::full_brake());
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

		if controller.button_left.is_now_pressed() {
			self.park_state = match self.park_state {
				ParkState::Disabled | ParkState::Manual => ParkState::Outtaking,
				_ => ParkState::Disabled,
			}
		} else if controller.button_right.is_now_pressed() {
			self.park_state = match self.park_state {
				ParkState::Disabled => ParkState::Manual,
				_ => ParkState::Disabled,
			}
		}

		if controller.button_x.is_pressed() && self.last_jiggle.is_none() {
			self.enable_unjam();
		} else if controller.button_x.is_released() && self.last_jiggle.is_some() {
			self.disable_unjam();
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
