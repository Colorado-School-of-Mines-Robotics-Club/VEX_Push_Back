use std::{cell::RefCell, rc::Rc};

use autons::{prelude::SelectCompeteExt as _, route};
use coprocessor::requests::{CalibrateRequest, OtosPosition, PingRequest};
use evian::drivetrain::model::Differential;
use shrewnit::{Degrees, Inches};
#[cfg(feature = "ui")]
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

#[cfg(not(feature = "ui"))]
use crate::autons::StubSelector;

pub struct Robot {
	#[cfg(feature = "ui")]
	pub ui: RobotUi,
	pub coprocessor: CoproSubsystem,
	pub controller: Controller,
	pub configuration: ControllerConfiguration,
	pub drivetrain: DrivetrainSubsystem<Differential, CoproTracking>,
	pub intake: IntakeSubsystem,
	pub pneumatics: PneumaticsSubsystem,
	pub replay: ReplaySubsystem,
	pub imu: Rc<RefCell<InertialSensor>>,
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
		let imu = Rc::new(RefCell::new(InertialSensor::new(peripherals.port_5)));

		let imu_clone = imu.clone();
		vexide::task::spawn(async move {
			imu_clone.borrow_mut().calibrate().await;
			println!("IMU Calibrated")
		})
		.detach();

		#[cfg(feature = "ui")]
		let ui = {
			let coprocessor_clone = coprocessor.clone();
			let ui = RobotUi::new(peripherals.display, coprocessor.data().clone(), move || {
				vexide::task::spawn(coprocessor_clone.send_request(CalibrateRequest)).detach()
			});
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

			ui
		};

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
			CoproTracking::new(coprocessor.data().clone(), imu.clone()),
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
			#[cfg(feature = "ui")]
			ui,
			imu,
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
		let default_auton = "Match auton";
		let autons = [
			route!("Do nothing", crate::autons::do_nothing),
			route!("PID testing", crate::autons::pid_testing),
			route!("Match auton", crate::autons::match_auton),
			route!("Testing", crate::autons::testing),
		];

		#[cfg(feature = "ui")]
		{
			let ui = self.ui.clone();

			let selector = SlintSelector::new(ui.app(), default_auton, autons);
			vexide::task::spawn(self.compete(selector)).detach();
			ui.run_blocking();
		}
		#[cfg(not(feature = "ui"))]
		{
			self.compete(StubSelector::new(default_auton, autons)).await;
		};

		std::process::exit(0);
	}
}
