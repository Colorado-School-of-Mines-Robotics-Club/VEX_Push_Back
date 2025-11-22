use serde::{Deserialize, Serialize};
use vexide::{controller::ControllerState, prelude::AdiDigitalOut, smart::PortError};

use crate::subsystems::{ControllableSubsystem, ControllerConfiguration};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum TrunkState {
    Down,
    Lower,
    Upper,
}

pub struct TrunkSubsystem {
    lower_pneumatic: AdiDigitalOut,
    upper_pneumatic: AdiDigitalOut,
    state: TrunkState,
}

impl TrunkSubsystem {
    pub fn new(mut lower_pneumatic: AdiDigitalOut, mut upper_pneumatic: AdiDigitalOut) -> Self {
        _ = lower_pneumatic.set_low();
        _ = upper_pneumatic.set_low();

        Self {
            lower_pneumatic,
            upper_pneumatic,
            state: TrunkState::Down,
        }
    }

    pub fn set_state(&mut self, state: TrunkState) -> Result<(), PortError> {
        dbg!(state);
        match state {
            TrunkState::Down => {
                self.lower_pneumatic.set_low()?;
                self.upper_pneumatic.set_low()?;
            }
            TrunkState::Lower => {
                self.lower_pneumatic.set_high()?;
                self.upper_pneumatic.set_high()?;
            }
            TrunkState::Upper => {
                self.lower_pneumatic.set_high()?;
                self.upper_pneumatic.set_low()?;
            }
        }
        self.state = state;

        Ok(())
    }
}

impl ControllableSubsystem for TrunkSubsystem {
    fn state(&self) -> Option<ciborium::Value> {
        Some(
            ciborium::Value::serialized(&self.state)
                .expect("Serializing state enum should succeed"),
        )
    }

    fn update(&mut self, controller: &ControllerState, configuration: ControllerConfiguration) {
        match configuration {
            ControllerConfiguration::Noah => {
                let lower_btn = controller.button_l1.is_now_pressed();
                let upper_btn = controller.button_l2.is_now_pressed();
                match self.state {
                    TrunkState::Down => match (lower_btn, upper_btn) {
                        (true, false) => _ = self.set_state(TrunkState::Lower),
                        (false, true) | (true, true) => _ = self.set_state(TrunkState::Upper),
                        (false, false) => (),
                    },
                    TrunkState::Lower => match (lower_btn, upper_btn) {
                        (true, false) => _ = self.set_state(TrunkState::Down),
                        (false, true) | (true, true) => _ = self.set_state(TrunkState::Upper),
                        (false, false) => (),
                    },
                    TrunkState::Upper => match (lower_btn, upper_btn) {
                        (true, false) => _ = self.set_state(TrunkState::Lower),
                        (false, true) | (true, true) => _ = self.set_state(TrunkState::Down),
                        (false, false) => (),
                    },
                }
            },
            ControllerConfiguration::Connor => {
                if controller.button_l1.is_now_pressed() {
                    _ = self.set_state(TrunkState::Down);
                } else if controller.button_l2.is_now_released() {
                    let state = match self.state {
                        TrunkState::Down | TrunkState::Upper => TrunkState::Lower,
                        TrunkState::Lower => TrunkState::Upper,
                    };

                    _ = self.set_state(state);
                };
            },
        }
    }
}
