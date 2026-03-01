use std::{ops::Not, time::Duration};

use serde::{Deserialize, Serialize};
use vexide::{
	adi::digital::LogicLevel, controller::ControllerState, prelude::AdiDigitalOut,
	smart::PortError, time::sleep,
};

use crate::{ControllableSubsystem, ControllerConfiguration};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PneumaticsSubsystemState {
	front_bar: PneumaticState,
	extender: PneumaticState,
	flap: PneumaticState,
	outtake_adjuster: PneumaticState,
}

pub struct PneumaticsSubsystem {
	pub front_bar: AdiPneumatic,
	pub extender: AdiPneumatic,
	pub flap: AdiPneumatic,
	pub outtake_adjuster: AdiPneumatic,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Ord, Eq, Serialize, Deserialize, Default)]
pub enum PneumaticState {
	Extended,
	#[default]
	Contracted,
}

impl Not for PneumaticState {
	type Output = Self;

	fn not(self) -> Self::Output {
		match self {
			Self::Contracted => Self::Extended,
			Self::Extended => Self::Contracted,
		}
	}
}

pub struct AdiPneumatic {
	pub port: AdiDigitalOut,
	pub high_mode: PneumaticState,
}

impl AdiPneumatic {
	pub fn state(&self) -> Result<PneumaticState, PortError> {
		let level = self.port.level()?;

		// vexide i swear to god
		// let level = !level;

		if self.high_mode == PneumaticState::Extended {
			if level == LogicLevel::High {
				Ok(PneumaticState::Extended)
			} else {
				Ok(PneumaticState::Contracted)
			}
		} else {
			#[allow(clippy::collapsible_else_if)] // shush its intentional
			if level == LogicLevel::High {
				Ok(PneumaticState::Contracted)
			} else {
				Ok(PneumaticState::Extended)
			}
		}
	}

	pub fn set_state(&mut self, state: PneumaticState) -> Result<(), PortError> {
		let level = if self.high_mode == PneumaticState::Contracted {
			match state {
				PneumaticState::Contracted => LogicLevel::High,
				PneumaticState::Extended => LogicLevel::Low,
			}
		} else {
			match state {
				PneumaticState::Contracted => LogicLevel::Low,
				PneumaticState::Extended => LogicLevel::High,
			}
		};

		// vexide i swear to god
		self.port.set_level(level)
	}
}

impl PneumaticsSubsystem {
	pub fn new(
		front_bar: AdiPneumatic,
		extender: AdiPneumatic,
		flap: AdiPneumatic,
		outtake_adjuster: AdiPneumatic,
	) -> Self {
		Self {
			front_bar,
			extender,
			flap,
			outtake_adjuster,
		}
	}

	pub fn set_state(&mut self, state: PneumaticsSubsystemState) -> Result<(), PortError> {
		self.front_bar.set_state(state.front_bar)?;
		self.extender.set_state(state.extender)?;
		self.flap.set_state(state.flap)?;
		self.outtake_adjuster.set_state(state.outtake_adjuster)?;

		Ok(())
	}

	pub async fn initialize(&mut self) {
		_ = self.front_bar.set_state(PneumaticState::Contracted);
		_ = self.extender.set_state(PneumaticState::Extended);
		_ = self.outtake_adjuster.set_state(PneumaticState::Extended);

		if self
			.flap
			.state()
			.is_ok_and(|s| s != PneumaticState::Contracted)
			|| self.flap.state().is_err()
		{
			println!("Adjusting flap");
			_ = self.flap.set_state(PneumaticState::Extended);

			sleep(Duration::from_millis(250)).await;

			_ = self.flap.set_state(PneumaticState::Contracted);
		} else {
			println!("Flap already adjusted");
		}
	}
}

impl ControllableSubsystem for PneumaticsSubsystem {
	fn state(&self) -> Option<ciborium::Value> {
		Some(
			ciborium::Value::serialized(&PneumaticsSubsystemState {
				front_bar: self.front_bar.state().unwrap_or(PneumaticState::Contracted),
				extender: self.extender.state().unwrap_or(PneumaticState::Contracted),
				flap: self.flap.state().unwrap_or(PneumaticState::Contracted),
				outtake_adjuster: self
					.outtake_adjuster
					.state()
					.unwrap_or(PneumaticState::Contracted),
			})
			.expect("Serializing state enum should succeed"),
		)
	}

	fn direct(&mut self, state: &ciborium::Value) {
		if let Ok(parsed) = state.deserialized() {
			_ = self.set_state(parsed);
		}
	}

	fn control(&mut self, controller: &ControllerState, _configuration: ControllerConfiguration) {
		if controller.button_l1.is_pressed() {
			_ = self.outtake_adjuster.set_state(PneumaticState::Extended);
			_ = self.flap.set_state(PneumaticState::Extended);
		} else if controller.button_l2.is_pressed() {
			_ = self.outtake_adjuster.set_state(PneumaticState::Contracted);
			_ = self.flap.set_state(PneumaticState::Extended);
		} else {
			_ = self.outtake_adjuster.set_state(PneumaticState::Extended);
			_ = self.flap.set_state(PneumaticState::Contracted);
		}

		if controller.button_b.is_now_pressed()
			&& let Ok(state) = self.front_bar.state()
		{
			_ = self.front_bar.set_state(!state);
		}
	}
}
