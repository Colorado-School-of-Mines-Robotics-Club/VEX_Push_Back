use bitflags::bitflags;
use serde::{Deserialize, Serialize};
use vexide::{
    controller::ControllerState,
    prelude::{AdiDigitalIn, Motor},
    smart::{PortError, motor::BrakeMode},
};

use crate::subsystems::{ControllableSubsystem, ControllerConfiguration};

pub struct IntakeMotors<M: AsMut<[Motor]>> {
    pub intake_wheels: M,
    pub bottom_rollers: M,
    pub elevator_intake_rollers: M,
    pub elevator_rollers: M,
    pub outtake_rollers: M,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
#[repr(transparent)]
/// Intake bitflags.
pub struct IntakeState(u16);

bitflags! {
    impl IntakeState: u16 {
        const FULL            = 0b11111;
        const INTAKE_WHEELS   = 1 << 0;
        const BOTTOM          = 1 << 1;
        const ELEVATOR_INTAKE = 1 << 2;
        const ELEVATOR        = 1 << 3;
        const TRUNK           = 1 << 4;

        // All reverse flags are same order as the normal bits, higher order adjacent
        const FULL_REV            = Self::FULL.bits()            << Self::MOTOR_BITS;
        const INTAKE_WHEELS_REV   = Self::INTAKE_WHEELS.bits()   << Self::MOTOR_BITS;
        const BOTTOM_REV          = Self::BOTTOM.bits()          << Self::MOTOR_BITS;
        const ELEVATOR_INTAKE_REV = Self::ELEVATOR_INTAKE.bits() << Self::MOTOR_BITS;
        const ELEVATOR_REV        = Self::ELEVATOR.bits()        << Self::MOTOR_BITS;
        const TRUNK_REV           = Self::TRUNK.bits()           << Self::MOTOR_BITS;
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

impl<'a, M: AsMut<[Motor]>> IntoIterator for &'a mut IntakeMotors<M> {
    type Item = (IntakeState, std::slice::IterMut<'a, Motor>);

    type IntoIter =
        std::array::IntoIter<(IntakeState, std::slice::IterMut<'a, vexide::prelude::Motor>), 5>;

    fn into_iter(self) -> Self::IntoIter {
        [
            (
                IntakeState::INTAKE_WHEELS,
                self.intake_wheels.as_mut().iter_mut(),
            ),
            (IntakeState::BOTTOM, self.bottom_rollers.as_mut().iter_mut()),
            (
                IntakeState::ELEVATOR_INTAKE,
                self.elevator_intake_rollers.as_mut().iter_mut(),
            ),
            (
                IntakeState::ELEVATOR,
                self.elevator_rollers.as_mut().iter_mut(),
            ),
            (IntakeState::TRUNK, self.outtake_rollers.as_mut().iter_mut()),
        ]
        .into_iter()
    }
}

pub struct IntakeSubsystem<M: AsMut<[Motor]>> {
    motors: IntakeMotors<M>,
    #[allow(dead_code)] // todo: use them lol
    line_break_sensors: [AdiDigitalIn; 4],
    state: IntakeState,
}

impl<M: AsMut<[Motor]>> IntakeSubsystem<M> {
    pub fn new(motors: IntakeMotors<M>, line_break_sensors: [AdiDigitalIn; 4]) -> Self {
        Self {
            motors,
            line_break_sensors,
            state: IntakeState::empty(),
        }
    }

    pub fn run(&mut self, state: IntakeState) -> Result<(), PortError> {
        for (flag, motors) in &mut self.motors.into_iter() {
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
                        m.set_current_limit(10.0)?;
                        m.set_voltage(m.max_voltage())?
                    }
                }
                (true, true) => {
                    for m in motors {
                        m.set_current_limit(10.0)?;
                        m.set_voltage(-m.max_voltage())?
                    }
                }
                _ => {
                    for m in motors {
                        m.set_current_limit(0.5)?;
                        m.brake(BrakeMode::Brake)?
                    }
                }
            }
        }

        Ok(())
    }
}

impl<M: AsMut<[Motor]>> ControllableSubsystem for IntakeSubsystem<M> {
    fn direct(&mut self, state: &ciborium::Value) {
        if let Ok(parsed) = state.deserialized() {
            self.state = parsed;

            _ = self.run(self.state);
        }
    }

    fn update(&mut self, controller: &ControllerState, _configuration: ControllerConfiguration) {
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
