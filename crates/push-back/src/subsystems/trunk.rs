use serde::{Deserialize, Serialize};
use vexide::{adi::digital::LogicLevel, controller::ControllerState, prelude::AdiDigitalOut, smart::PortError};

use crate::subsystems::{ControllableSubsystem, ControllerConfiguration};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum TrunkState {
    Down,
    Lower,
    Upper,
}

pub struct TrunkSubsystem {
    pusher: AdiPneumatic,
    puller: AdiPneumatic,
    state: TrunkState,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Ord, Eq)]
pub enum PneumaticState {
    Extended,
    Contracted
}

pub struct AdiPneumatic {
    pub port: AdiDigitalOut,
    pub high_mode: PneumaticState
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
                PneumaticState::Extended => LogicLevel::Low
            }
        } else {
            match state {
                PneumaticState::Contracted => LogicLevel::Low,
                PneumaticState::Extended => LogicLevel::High
            }
        };

        // vexide i swear to god
        self.port.set_level(level)
    }
}

impl TrunkSubsystem {
    pub fn new(pusher: AdiPneumatic, puller: AdiPneumatic) -> Self {
        let mut sub = Self {
            pusher,
            puller,
            state: TrunkState::Down,
        };

        _ = sub.set_state(TrunkState::Down);

        sub
    }

    pub fn set_state(&mut self, state: TrunkState) -> Result<(), PortError> {
        // dbg!(state);
        match state {
            TrunkState::Down => {
                self.pusher.set_state(PneumaticState::Contracted)?;
                self.puller.set_state(PneumaticState::Extended)?;
            }
            TrunkState::Upper => {
                self.pusher.set_state(PneumaticState::Extended)?;
                self.puller.set_state(PneumaticState::Contracted)?;
            }
            TrunkState::Lower => {
                self.pusher.set_state(PneumaticState::Extended)?;
                self.puller.set_state(PneumaticState::Extended)?;
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

    fn direct(&mut self, state: &ciborium::Value) {
        if let Ok(parsed) = state.deserialized() {
            _ = self.set_state(parsed);
        }
    }

    fn control(&mut self, controller: &ControllerState, configuration: ControllerConfiguration) {
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
            }
            ControllerConfiguration::Connor => {
                if controller.button_l2.is_now_pressed() {
                    _ = self.set_state(TrunkState::Down);
                } else if controller.button_l1.is_now_released() {
                    let state = match self.state {
                        TrunkState::Down | TrunkState::Upper => TrunkState::Lower,
                        TrunkState::Lower => TrunkState::Upper,
                    };

                    _ = self.set_state(state);
                };
            }
        }
    }
}
