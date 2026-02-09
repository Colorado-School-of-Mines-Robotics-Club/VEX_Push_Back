use coprocessor::requests::{PingRequest, SetOffsetsRequest};
use evian::drivetrain::model::Differential;
use push_back::subsystems::{
    ControllerConfiguration,
    copro::{CoproSubsystem, tracking::CoproTracking},
    drivetrain::DrivetrainSubsystem,
    intake::{IntakeMotors, IntakeSensors, IntakeSubsystem},
    replay::ReplaySubsystem,
    trunk::{AdiPneumatic, PneumaticState, TrunkSubsystem},
};
use shrewnit::{Degrees, Inches};
use slintui::{ui::RobotUi, *};
use vexide::{
    math::Direction,
    prelude::{
        AdiDigitalIn, AdiDigitalOut, CompeteExt as _, Controller, Gearset, Motor, Peripherals,
    },
};

pub struct Robot {
    pub ui: RobotUi,
    pub coprocessor: CoproSubsystem,
    pub controller: Controller,
    pub configuration: ControllerConfiguration,
    pub drivetrain: DrivetrainSubsystem<Differential, CoproTracking>,
    pub intake: IntakeSubsystem,
    pub trunk: TrunkSubsystem,
    pub replay: ReplaySubsystem,
}

impl Robot {
    pub async fn new(peripherals: Peripherals) -> Self {
        let ui = RobotUi::new(peripherals.display);
        ui.app().global::<OdometryPageState>().set_bot_size(15.0);

        let coprocessor = CoproSubsystem::new(peripherals.port_15).await;
        if let Ok(hash) = coprocessor.send_request(PingRequest).await {
            if PingRequest::verify_hash(hash) {
                ui.app().global::<OdometryPageState>().set_updated_status(UpdatedStatus::Updated);
            } else {
                ui.app().global::<OdometryPageState>().set_updated_status(UpdatedStatus::Outdated);
            }
        }

        let controller = peripherals.primary_controller;
        let configuration = ControllerConfiguration::Noah;
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
        let intake = IntakeSubsystem::new(
            IntakeMotors {
                intake_wheels: vec![
                    Motor::new_exp(peripherals.port_17, Direction::Reverse),
                    Motor::new_exp(peripherals.port_18, Direction::Forward),
                ],
                bottom_rollers: vec![Motor::new(
                    peripherals.port_20,
                    Gearset::Blue,
                    Direction::Reverse,
                )],
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
            IntakeSensors {
                intake: AdiDigitalIn::new(peripherals.adi_f),
                elevator: AdiDigitalIn::new(peripherals.adi_e),
                trunk: AdiDigitalIn::new(peripherals.adi_d),
                outtake: AdiDigitalIn::new(peripherals.adi_c),
            },
        );
        let trunk = TrunkSubsystem::new(
            AdiPneumatic {
                port: AdiDigitalOut::new(peripherals.adi_h),
                high_mode: PneumaticState::Extended,
            },
            AdiPneumatic {
                port: AdiDigitalOut::new(peripherals.adi_g),
                high_mode: PneumaticState::Contracted,
            },
        );
        let replay = ReplaySubsystem::new();

        Self {
            ui,
            controller,
            configuration,
            drivetrain,
            intake,
            trunk,
            replay,
            coprocessor,
        }
    }

    pub async fn start(self) -> ! {
        let ui = self.ui.clone();

        vexide::task::spawn(self.compete()).detach();

        ui.run_blocking();

        std::process::exit(0);
    }
}
