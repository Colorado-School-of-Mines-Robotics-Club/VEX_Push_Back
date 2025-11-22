use evian::drivetrain::model::Differential;
use push_back::subsystems::{
    // ControllableSubsystem,
    // copro::{CoproSubsystem, tracking::CoproTracking},
    drivetrain::DrivetrainSubsystem, intake::{IntakeMotors, IntakeSubsystem}, replay::ReplaySubsystem, trunk::TrunkSubsystem, ControllerConfiguration
};
use vexide::{
    math::Direction,
    prelude::{
        AdiDigitalIn, AdiDigitalOut, CompeteExt as _, Controller, Gearset, Motor, Peripherals,
    },
};

pub struct Robot {
    // pub display: RefCell<Display>,
    // pub coprocessor: CoproSubsystem,
    pub controller: Controller,
    pub configuration: ControllerConfiguration,
    pub drivetrain: DrivetrainSubsystem<Differential, /* CoproTracking */ ()>,
    pub intake: IntakeSubsystem<Vec<Motor>>,
    pub trunk: TrunkSubsystem,
    pub replay: ReplaySubsystem,
}

pub enum V1Bot {
    White,
    Black
}

impl Robot {
    pub async fn new(peripherals: Peripherals, bot: V1Bot) -> Self {
        // let coprocessor = CoproSubsystem::new(peripherals.port_15).await;
        let controller = peripherals.primary_controller;
        let configuration = match bot {
            V1Bot::White => ControllerConfiguration::Connor,
            V1Bot::Black => ControllerConfiguration::Noah
        };
        let drivetrain = DrivetrainSubsystem::new(
            match bot {
                V1Bot::White => Differential::new(
                    [
                        Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse), // white bot
                        Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
                        Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
                        Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
                    ],
                    [
                        Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward), // white bot
                        Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
                        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
                        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
                    ],
                ),
                V1Bot::Black => Differential::new(
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
            },
            // CoproTracking(coprocessor.data()),
            (),
        );
        let intake = IntakeSubsystem::new(
            IntakeMotors {
                intake_wheels: vec![
                    Motor::new_exp(peripherals.port_17, Direction::Reverse),
                    Motor::new_exp(peripherals.port_18, Direction::Forward),
                ],
                bottom_rollers: vec![
                    Motor::new(
                        peripherals.port_20,
                        Gearset::Blue,
                        Direction::Reverse,
                    )
                ],
                elevator_intake_rollers: vec![Motor::new(
                    peripherals.port_6,
                    Gearset::Blue,
                    Direction::Forward,
                )],
                elevator_rollers: vec![Motor::new(
                    peripherals.port_5,
                    Gearset::Blue,
                    Direction::Forward,
                )],
                outtake_rollers: vec![
                    Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
                    Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
                ],
            },
            [
                AdiDigitalIn::new(peripherals.adi_a),
                AdiDigitalIn::new(peripherals.adi_b),
                AdiDigitalIn::new(peripherals.adi_c),
                AdiDigitalIn::new(peripherals.adi_d),
            ],
        );
        let trunk = TrunkSubsystem::new(
            AdiDigitalOut::new(peripherals.adi_h),
            AdiDigitalOut::new(peripherals.adi_g),
        );
        let replay = ReplaySubsystem::new();

        Self {
            controller,
            configuration,
            drivetrain,
            intake,
            trunk,
            replay,
            // coprocessor,
        }
    }

    pub async fn start(self) -> ! {
        self.compete().await
    }
}
