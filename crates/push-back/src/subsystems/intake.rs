use vexide::{controller::ControllerState, prelude::Motor, smart::motor::BrakeMode};

use crate::subsystems::ControllableSubsystem;

const MOTOR_COUNT: usize = 3;

pub struct IntakeSubsystem {
    motors: [Motor; MOTOR_COUNT],
}

impl IntakeSubsystem {
    pub fn new(motors: [Motor; MOTOR_COUNT]) -> Self {
        motors.iter().reduce(|old, new| {
            if (old.is_v5() && !new.is_v5()) || (old.is_exp() && !new.is_exp()) {
                panic!("Intake motors must be a homogenous set of v5/exp");
            } else {
                new
            }
        });

        Self { motors }
    }

    fn set_voltage(&mut self, voltage: f64) {
        for motor in &mut self.motors {
            _ = motor.set_voltage(voltage);
        }
    }

    pub fn brake(&mut self) {
        for motor in &mut self.motors {
            _ = motor.brake(BrakeMode::Brake);
        }
    }

    pub fn set_intaking(&mut self) {
        self.set_voltage(self.motors[0].max_voltage());
    }

    pub fn set_outtaking(&mut self) {
        self.set_voltage(-self.motors[0].max_voltage());
    }
}

impl ControllableSubsystem for IntakeSubsystem {
    fn control(&mut self, controller: &ControllerState) {
        match (
            controller.button_r1.is_pressed(),
            controller.button_r2.is_pressed(),
        ) {
            (false, false) | (true, true) => self.brake(),
            (true, false) => self.set_intaking(),
            (false, true) => self.set_outtaking(),
        }
    }
}
