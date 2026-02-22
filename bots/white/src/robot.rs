use autons::{prelude::SelectCompeteExt as _, route};
use coprocessor::requests::{OtosPosition, PingRequest};
use evian::drivetrain::model::Differential;
use shrewnit::{Degrees, Inches};
use slintui::{
	OdometryPageState, UpdatedStatus, autons::SlintSelector, slint::ComponentHandle as _,
	ui::RobotUi,
};
use subsystems::{
	ControllerConfiguration,
	copro::{CoproSubsystem, tracking::CoproTracking},
	drivetrain::DrivetrainSubsystem,
	intake::{IntakeMotors, IntakeSubsystem},
	pnemuatics::{AdiPneumatic, PneumaticState, PneumaticsSubsystem},
	replay::ReplaySubsystem,
};
use vexide::prelude::*;
use vexide_motorgroup::MotorGroup;

pub struct Robot {
	pub ui: RobotUi,
	pub coprocessor: CoproSubsystem,
	pub controller: Controller,
	pub configuration: ControllerConfiguration,
	pub drivetrain: DrivetrainSubsystem<Differential, CoproTracking>,
	pub intake: IntakeSubsystem,
	pub pneumatics: PneumaticsSubsystem,
	pub replay: ReplaySubsystem,
}

impl Robot {
	pub async fn new(peripherals: Peripherals) -> Self {
		let coprocessor = CoproSubsystem::new(
			peripherals.port_6,
			OtosPosition {
				x: 0.0 * Inches,
				y: -1.25 * Inches,
				heading: 0.0 * Degrees,
			},
		)
		.await;

		let ui = RobotUi::new(peripherals.display, coprocessor.data().clone());
		ui.app().global::<OdometryPageState>().set_bot_size(15.0);

		if let Ok(hash) = coprocessor.send_request(PingRequest).await {
			if PingRequest::verify_hash(hash) {
				ui.app()
					.global::<OdometryPageState>()
					.set_updated_status(UpdatedStatus::Updated);
			} else {
				ui.app()
					.global::<OdometryPageState>()
					.set_updated_status(UpdatedStatus::Outdated);
			}
		}

		let controller = peripherals.primary_controller;
		let configuration = ControllerConfiguration::Connor;
		let drivetrain = DrivetrainSubsystem::new(
			Differential::new(
				[
					Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
					Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
					Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
					Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
				],
				[
					Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
					Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
					Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
					Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
				],
			),
			CoproTracking(coprocessor.data().clone()),
		);
		let intake = IntakeSubsystem::new(IntakeMotors {
			bottom: MotorGroup::new(vec![
				Motor::new(peripherals.port_18, Gearset::Blue, Direction::Forward),
				Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
			]),
			middle: MotorGroup::new(vec![Motor::new(
				peripherals.port_19,
				Gearset::Blue,
				Direction::Forward,
			)]),
			top: MotorGroup::new(vec![Motor::new(
				peripherals.port_12,
				Gearset::Blue,
				Direction::Reverse,
			)]),
		});
		let trunk = PneumaticsSubsystem::new(
			// Front bar
			AdiPneumatic {
				port: AdiDigitalOut::new(peripherals.adi_b),
				high_mode: PneumaticState::Extended,
			},
			// Extender
			AdiPneumatic {
				port: AdiDigitalOut::new(peripherals.adi_c),
				high_mode: PneumaticState::Extended,
			},
			// Flap
			AdiPneumatic {
				port: AdiDigitalOut::new(peripherals.adi_a),
				high_mode: PneumaticState::Contracted,
			},
			// Outtake adjuster
			AdiPneumatic {
				port: AdiDigitalOut::new(peripherals.adi_d),
				high_mode: PneumaticState::Extended,
			},
		);
		let replay = ReplaySubsystem::new();

		Self {
			ui,
			controller,
			configuration,
			drivetrain,
			intake,
			pneumatics: trunk,
			replay,
			coprocessor,
		}
	}

	pub async fn start(self) -> ! {
		let ui = self.ui.clone();

		vexide::task::spawn(self.compete(SlintSelector::new(
			ui.app(),
			[
				route!("Do nothing", crate::autons::do_nothing),
				route!("PID testing", crate::autons::pid_testing),
				// route!("PID Angle testing", crate::autons::pid_angle_testing),
			],
		)))
		.detach();

		ui.run_blocking();

		std::process::exit(0);
	}
}
