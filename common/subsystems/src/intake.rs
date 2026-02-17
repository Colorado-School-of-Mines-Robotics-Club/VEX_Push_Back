use std::{
	sync::{
		Arc,
		atomic::{AtomicU8, Ordering},
	},
	time::Duration,
};

use bitflags::bitflags;
use serde::{Deserialize, Serialize};
use vexide::{
	adi::digital::LogicLevel,
	controller::ControllerState,
	prelude::{AdiDigitalIn, Motor},
	smart::{PortError, motor::BrakeMode},
	task::Task,
	time::sleep,
};

use crate::{ControllableSubsystem, ControllerConfiguration};

const JAM_RPM_THRESHOLD: f64 = 3.0;

pub struct IntakeMotors {
	pub intake_wheels: Vec<Motor>,
	pub bottom_rollers: Vec<Motor>,
	pub elevator_intake_rollers: Vec<Motor>,
	pub elevator_rollers: Vec<Motor>,
	pub outtake_rollers: Vec<Motor>,
}

pub struct IntakeSensors {
	/// Just after the intake wheels
	pub intake: AdiDigitalIn,
	/// About 3/4 of the way up the elevator
	pub elevator: AdiDigitalIn,
	/// At the start of the trunk
	pub trunk: AdiDigitalIn,
	/// On top of the trunk exit wheels
	pub outtake: AdiDigitalIn,
}

impl From<&IntakeSensors> for LineBreakState {
	fn from(sensors: &IntakeSensors) -> Self {
		[
			(&sensors.intake, LineBreakState::INTAKE),
			(&sensors.elevator, LineBreakState::ELEVATOR),
			(&sensors.trunk, LineBreakState::TRUNK),
			(&sensors.outtake, LineBreakState::OUTTAKE),
		]
		.into_iter()
		.map(|(s, p)| {
			if p == LineBreakState::OUTTAKE {
				(!s.level().unwrap_or(LogicLevel::Low), p)
			} else {
				(s.level().unwrap_or(LogicLevel::Low), p)
			}
		})
		// NOTE: this should be checking for low, but vexide's logic appeaars to be reversed
		.filter(|(s, _)| matches!(s, LogicLevel::High))
		.fold(LineBreakState::empty(), |acc, (_, f)| acc.union(f))
	}
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
#[repr(transparent)]
/// Intake bitflags
pub struct IntakeState(u16);

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
#[repr(transparent)]
/// Intake sensor bitflags. A flag being enabled means the sensor is tripped.
pub struct LineBreakState(u8);

bitflags! {
	impl IntakeState: u16 {
		const INTAKE_WHEELS   = 1 << 0;
		const BOTTOM          = 1 << 1;
		const ELEVATOR_INTAKE = 1 << 2;
		const ELEVATOR        = 1 << 3;
		const TRUNK           = 1 << 4;
		const FULL            = (1 << (Self::TRUNK.bits().ilog2() + 1)) - 1;

		// All reverse flags are same order as the normal bits, higher order adjacent
		const FULL_REV            = Self::FULL.bits()            << Self::MOTOR_BITS;
		const INTAKE_WHEELS_REV   = Self::INTAKE_WHEELS.bits()   << Self::MOTOR_BITS;
		const BOTTOM_REV          = Self::BOTTOM.bits()          << Self::MOTOR_BITS;
		const ELEVATOR_INTAKE_REV = Self::ELEVATOR_INTAKE.bits() << Self::MOTOR_BITS;
		const ELEVATOR_REV        = Self::ELEVATOR.bits()        << Self::MOTOR_BITS;
		const TRUNK_REV           = Self::TRUNK.bits()           << Self::MOTOR_BITS;
	}

	impl LineBreakState: u8 {
		/// Just after the intake wheels
		const INTAKE   = 1 << 0;
		/// About 3/4 of the way up the elevator
		const ELEVATOR = 1 << 1;
		/// At the start of the trunk
		const TRUNK    = 1 << 2;
		/// On top of the trunk exit wheels
		const OUTTAKE  = 1 << 3;
	}
}

impl IntakeState {
	const MOTOR_BITS: u32 = Self::FULL.bits().count_ones();

	pub const fn full_forward() -> Self {
		Self::FULL
	}

	pub const fn full_reverse() -> Self {
		Self::from_bits_retain(Self::FULL.bits() | Self::FULL_REV.bits())
	}

	pub const fn full_brake() -> Self {
		Self::empty()
	}

	/// Reverses all currently enabled motor groups. Any reversed motors will be un-reversed
	pub const fn reverse(self) -> Self {
		Self::from_bits_retain(self.bits() ^ (self.bits() << Self::MOTOR_BITS))
	}
}

impl<'a> IntoIterator for &'a IntakeMotors {
	type Item = (IntakeState, std::slice::Iter<'a, Motor>);

	type IntoIter =
		std::array::IntoIter<(IntakeState, std::slice::Iter<'a, vexide::prelude::Motor>), 5>;

	fn into_iter(self) -> Self::IntoIter {
		[
			(IntakeState::INTAKE_WHEELS, self.intake_wheels.iter()),
			(IntakeState::BOTTOM, self.bottom_rollers.iter()),
			(
				IntakeState::ELEVATOR_INTAKE,
				self.elevator_intake_rollers.iter(),
			),
			(IntakeState::ELEVATOR, self.elevator_rollers.iter()),
			(IntakeState::TRUNK, self.outtake_rollers.iter()),
		]
		.into_iter()
	}
}

impl<'a> IntoIterator for &'a mut IntakeMotors {
	type Item = (IntakeState, std::slice::IterMut<'a, Motor>);

	type IntoIter =
		std::array::IntoIter<(IntakeState, std::slice::IterMut<'a, vexide::prelude::Motor>), 5>;

	fn into_iter(self) -> Self::IntoIter {
		[
			(IntakeState::INTAKE_WHEELS, self.intake_wheels.iter_mut()),
			(IntakeState::BOTTOM, self.bottom_rollers.iter_mut()),
			(
				IntakeState::ELEVATOR_INTAKE,
				self.elevator_intake_rollers.iter_mut(),
			),
			(IntakeState::ELEVATOR, self.elevator_rollers.iter_mut()),
			(IntakeState::TRUNK, self.outtake_rollers.iter_mut()),
		]
		.into_iter()
	}
}

pub struct IntakeSubsystem {
	motors: IntakeMotors,
	line_break_state: Arc<AtomicU8>,
	state: IntakeState,
	_task: Task<!>,
}

impl IntakeSubsystem {
	async fn task(state: Arc<AtomicU8>, sensors: IntakeSensors) -> ! {
		loop {
			let flags: LineBreakState = (&sensors).into();
			state.store(flags.bits(), Ordering::Relaxed);

			sleep(Duration::from_millis(5)).await
		}
	}

	pub fn new(motors: IntakeMotors, line_break_sensors: IntakeSensors) -> Self {
		let line_break_state = Arc::new(AtomicU8::new(LineBreakState::empty().bits()));
		Self {
			motors,
			state: IntakeState::empty(),
			_task: vexide::task::spawn(Self::task(line_break_state.clone(), line_break_sensors)),
			line_break_state,
		}
	}

	pub fn sensors(&self) -> LineBreakState {
		LineBreakState::from_bits_retain(self.line_break_state.load(Ordering::Relaxed))
	}

	pub fn jams(&self) -> IntakeState {
		let mut jams = IntakeState::empty();

		for (flag, motors) in self.motors.into_iter() {
			let len = motors.len();
			if motors.filter_map(|m| m.velocity().ok()).sum::<f64>() / (len as f64)
				< JAM_RPM_THRESHOLD
			{
				jams |= flag;
			}
		}

		jams
	}

	pub fn run(&mut self, state: IntakeState) -> Result<(), PortError> {
		for (flag, motors) in (&mut self.motors).into_iter() {
			const {
				// Make sure .contains works properly
				let rev_intake_wheels = IntakeState::from_bits_retain(
					IntakeState::INTAKE_WHEELS.bits()
						| IntakeState::INTAKE_WHEELS_REV.bits()
						| IntakeState::ELEVATOR.bits()
						| IntakeState::ELEVATOR_REV.bits(),
				);
				assert!(rev_intake_wheels.contains(IntakeState::INTAKE_WHEELS));
				assert!(rev_intake_wheels.contains(IntakeState::INTAKE_WHEELS.reverse()));
				assert!(rev_intake_wheels.contains(IntakeState::ELEVATOR));
				assert!(rev_intake_wheels.contains(IntakeState::ELEVATOR.reverse()));
			}

			// Match on (enabled, reversed)
			match (state.contains(flag), state.contains(flag.reverse())) {
				(true, false) => {
					for m in motors {
						let voltage = m.max_voltage();
						m.set_voltage(voltage)?
					}
				}
				(true, true) => {
					for m in motors {
						m.set_voltage(-m.max_voltage())?
					}
				}
				_ => {
					for m in motors {
						m.brake(BrakeMode::Brake)?
					}
				}
			}
		}

		Ok(())
	}
}

impl ControllableSubsystem for IntakeSubsystem {
	fn direct(&mut self, state: &ciborium::Value) {
		if let Ok(parsed) = state.deserialized() {
			self.state = parsed;

			_ = self.run(self.state);
		}
	}

	fn control(&mut self, controller: &ControllerState, _configuration: ControllerConfiguration) {
		self.state = if controller.button_b.is_pressed() {
			IntakeState::FULL | IntakeState::FULL_REV
		} else if controller.button_r2.is_pressed() {
			IntakeState::FULL
		} else if controller.button_r1.is_pressed() {
			IntakeState::FULL - IntakeState::TRUNK
		} else {
			IntakeState::full_brake()
		};

		_ = self.run(self.state);
	}

	fn state(&self) -> Option<ciborium::Value> {
		Some(
			ciborium::Value::serialized(&self.state)
				.expect("Serializing state enum should succeed"),
		)
	}
}
