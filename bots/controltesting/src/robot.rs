use evian::drivetrain::model::Differential;
use push_back::subsystems::{
    copro::{CoproSubsystem, tracking::CoproTracking},
    drivetrain::DrivetrainSubsystem,
};
use vexide::{
    math::Direction,
    prelude::{CompeteExt as _, Controller, Gearset, Motor, Peripherals},
};

pub struct Robot {
    pub coprocessor: CoproSubsystem,
    pub controller: Controller,
    pub drivetrain: DrivetrainSubsystem<Differential, CoproTracking>,
}

impl Robot {
    pub async fn new(peripherals: Peripherals) -> Self {
        let coprocessor = CoproSubsystem::new(peripherals.port_15).await;
        let controller = peripherals.primary_controller;
        let drivetrain = DrivetrainSubsystem::new(
            Differential::new(
                [
                    Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
                ],
                [
                    Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
                    Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
                ],
            ),
            CoproTracking(coprocessor.data()),
        );

        Self {
            controller,
            drivetrain,
            coprocessor,
        }
    }

    pub async fn start(self) -> ! {
        self.compete().await
    }
}
