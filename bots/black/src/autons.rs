use std::time::Duration;

use autons::{Selector, simple::Route};
use evian::{
	math::Angle,
	prelude::{Arcade, TracksForwardTravel, TracksHeading, TracksPosition},
};
use evian_extra::{
	control::ltv_unicycle::LTVUnicycleController, motion::ltv_unicycle::LTVUnicycleMotion,
};
use subsystems::{intake::IntakeState, intake_unjamming, pnemuatics::PneumaticState};
use vexide::{smart::motor::BrakeMode, time::sleep};

use crate::robot::Robot;

pub async fn do_nothing(_robot: &mut Robot) {
	println!("Doing absolutely nothing!!!");
}

pub async fn match_auton(robot: &mut Robot) {
	// Basic setup.
	_ = robot
		.pneumatics
		.extender
		.set_state(PneumaticState::Extended);

	let mut _seeking = crate::control::SEEKING_CONTROLLER;

	let mut basic = crate::control::BASIC_CONTROLLER;

	// Move to center goal position
	basic.drive_distance(&mut robot.drivetrain, -43.5).await;

	// Turn towards center goal
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(45.0))
		.await;

	// Align with center goal
	_ = robot.drivetrain.model.drive_arcade(-0.25, 0.0);
	sleep(Duration::from_millis(900)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	_ = robot.drivetrain.model.drive_arcade(0.17, 0.0);
	sleep(Duration::from_millis(300)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Robot intake runs and ejects the balls into the center goal.

	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	robot.intake.run(IntakeState {
		top: 0.5,
		middle: 1.0,
		bottom: 1.0,
	});
	sleep(Duration::from_secs(2)).await;
	robot.intake.run(IntakeState::full_brake());
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);

	basic
		.drive_distance_at_heading(&mut robot.drivetrain, 25.0, Angle::from_degrees(45.0))
		.await;

	// Calculate distance from line
	let dist_x = (robot.drivetrain.tracking.position().x - 31.0).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, dist).await;

	// Turn to matchload
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 1.2);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(89.5))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 1.2);

	dbg!(robot.drivetrain.tracking.heading().as_degrees());

	// Load from matchload
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	sleep(Duration::from_millis(750)).await;
	_ = robot.drivetrain.model.drive_arcade(0.35, -0.01);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_secs(3)).await;

	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 1.0,
		bottom: 0.0,
	});
	sleep(Duration::from_millis(10)).await;
	robot.intake.run(IntakeState::full_brake());

	// Drive to long goal
	_ = robot.drivetrain.model.drive_arcade(-0.35, -0.007);
	sleep(Duration::from_millis(1500)).await;

	// Outtake into long goal
	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 0.0,
		bottom: 0.0,
	});
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	sleep(Duration::from_secs(2)).await;

	// Go back, outtake red balls
	basic.drive_distance(&mut robot.drivetrain, 12.0).await;
	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 1.0,
		bottom: 1.0,
	});
	sleep(Duration::from_secs(2)).await;
	robot.intake.run(IntakeState {
		top: -1.0,
		middle: -1.0,
		bottom: -1.0,
	});
	sleep(Duration::from_millis(100)).await;
	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 1.0,
		bottom: 1.0,
	});
	basic.drive_distance(&mut robot.drivetrain, 12.0).await;
	sleep(Duration::from_secs(1)).await;

	robot.intake.run(IntakeState::full_brake());

	while let error =
		(robot.drivetrain.tracking.heading().wrapped_full() - Angle::from_degrees(90.0))
		&& error.abs() > Angle::from_degrees(1.0)
	{
		_ = robot
			.drivetrain
			.model
			.drive_arcade(0.0, 0.12 * error.signum());
		sleep(Duration::from_millis(5)).await;
	}
	robot.drivetrain.brake(BrakeMode::Hold);

	// Leo's and Noah's test code, not perfect but will score a few points.
	// Once code is finalized the bot will go back to the tower and grab the balls again.

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	sleep(Duration::from_millis(800)).await;
	_ = robot.drivetrain.model.drive_arcade(0.30, 0.0);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_secs(3)).await;

	_ = robot.drivetrain.model.drive_arcade(0.0, 100.0);

	sleep(Duration::from_millis(10)).await;

	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.0);
	sleep(Duration::from_millis(1500)).await;

	// Robot drives up to the high beam and ejects its balls.

	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 1.0,
		bottom: 1.0,
	});
	sleep(Duration::from_secs(2)).await;
	robot.intake.run(IntakeState {
		top: -1.0,
		middle: -1.0,
		bottom: -1.0,
	});
	sleep(Duration::from_millis(100)).await;
	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 1.0,
		bottom: 1.0,
	});
	sleep(Duration::from_secs(2)).await;
}

pub async fn double_park(robot: &mut Robot) {
	// Drive back to be technically parked
	_ = robot.drivetrain.model.drive_arcade(-0.20, 0.0);
	sleep(Duration::from_millis(100)).await;
	_ = robot.drivetrain.brake(BrakeMode::Hold);
	sleep(Duration::from_millis(1000)).await;

	// Park
	_ = robot
		.intake
		.park_piston()
		.set_state(PneumaticState::Extended);
}

pub async fn skills_or_whatever(robot: &mut Robot) {
	println!("Lets a Goo!!!");
	let mut basic = crate::control::BASIC_CONTROLLER;

	basic.drive_distance(&mut robot.drivetrain, 18.0).await;
	basic
		.drive_distance_at_heading(&mut robot.drivetrain, 31.0, Angle::from_degrees(180.0))
		.await;
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(-90.0))
		.await;
	// sould be allined in frot of match load

	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 1.0,
		bottom: 1.0,
	});
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);
	_ = robot
		.pneumatics
		.extender
		.set_state(PneumaticState::Extended);
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	_ = robot.drivetrain.model.drive_arcade(0.25, 0.0);
	sleep(Duration::from_secs(3)).await;

	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.0);
	sleep(Duration::from_millis(300)).await;
	_ = robot.drivetrain.model.drive_arcade(0.25, 0.0);
	sleep(Duration::from_millis(1000)).await;

	robot.intake.run(IntakeState {
		top: 0.0,
		middle: 0.0,
		bottom: 1.0,
	});

	basic.drive_distance(&mut robot.drivetrain, -7.0).await;

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);

	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(180.0))
		.await;

	_ = robot.drivetrain.model.drive_arcade(0.35, 0.0);
	sleep(Duration::from_secs(1)).await;

	// Put bar down and bring balls back from wall
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);

	_ = robot.drivetrain.model.drive_arcade(-0.1, 0.0);
	sleep(Duration::from_secs(1)).await;
	_ = robot.drivetrain.model.drive_arcade(-0.5, 0.0);
	sleep(Duration::from_millis(200)).await;

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);

	let gooo = -43.0;
	let dist_y = (robot.drivetrain.tracking.position().x - gooo).abs();
	let dist = (dist_y / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, dist).await;

	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0))
		.await;

	let gooo = 98.0;
	let dist_y = (robot.drivetrain.tracking.position().y - gooo).abs();
	let dist = (dist_y / (robot.drivetrain.tracking.heading()).sin()).abs();

	basic.drive_distance(&mut robot.drivetrain, dist).await;

	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(0.0))
		.await;

	let gooo = -31.0;
	let dist_y = (robot.drivetrain.tracking.position().x - gooo).abs();
	let dist = (dist_y / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, dist).await;

	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0))
		.await;

	_ = robot.drivetrain.model.drive_arcade(-1.0, 0.0);
	sleep(Duration::from_millis(300)).await;
	_ = robot.drivetrain.model.drive_arcade(-0.3, 0.0);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);

	intake_unjamming!(robot, IntakeState::full_forward(), r => async {
		sleep(Duration::from_millis(3500)).await;
	});
	// Move to matchload and intake balls
	_ = robot.drivetrain.model.drive_arcade(0.35, 0.01);
	sleep(Duration::from_secs(1)).await;
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	sleep(Duration::from_secs(1)).await;

	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 1.0,
		bottom: 1.0,
	});

	_ = robot.drivetrain.model.drive_arcade(0.25, 0.01);
	sleep(Duration::from_secs(4)).await;
	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.0);
	sleep(Duration::from_millis(300)).await;
	_ = robot.drivetrain.model.drive_arcade(0.25, 0.0);
	sleep(Duration::from_millis(1000)).await;
	_ = robot.drivetrain.model.drive_arcade(-0.15, 0.01);
	sleep(Duration::from_millis(350)).await;

	// Move to long goal and outtake balls

	let gooo: f64 = Angle::atan2(
		robot.drivetrain.tracking.position().y - 94.0,
		robot.drivetrain.tracking.position().x + 29.0,
	)
	.as_degrees();

	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(gooo))
		.await;
	_ = robot.drivetrain.model.drive_arcade(-0.5, 0.0);
	sleep(Duration::from_secs(2)).await;
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);

	intake_unjamming!(robot, IntakeState::full_forward(), r => async {
		sleep(Duration::from_secs(1)).await;
	});
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	intake_unjamming!(
		robot,
		IntakeState {
			top: 0.5,
			middle: 1.0,
			bottom: 1.0,
		},
		r => async {
			// Slow down front for the rest to get 15 balls
			sleep(Duration::from_secs(4)).await;
		}
	);
	basic.drive_distance(&mut robot.drivetrain, 0.0).await;
	basic.drive_distance(&mut robot.drivetrain, 15.0).await;
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(180.0))
		.await;

	_ = robot.drivetrain.model.drive_arcade(0.35, 0.0);
	sleep(Duration::from_secs(1)).await;

	// Put bar down and bring balls back from wall
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);
	sleep(Duration::from_millis(750)).await;

	_ = robot.drivetrain.model.drive_arcade(-0.1, 0.0);
	sleep(Duration::from_secs(1)).await;

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);

	_ = robot.drivetrain.model.drive_arcade(0.3, 0.0);
	sleep(Duration::from_millis(200)).await;
	let gooo = 17.0;
	let dist_y = (robot.drivetrain.tracking.position().x.abs() + gooo);
	let dist = (dist_y / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, -dist).await;

	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0))
		.await;
	_ = robot.drivetrain.model.drive_arcade(0.3, 0.0);
	sleep(Duration::from_millis(500)).await;

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);
	sleep(Duration::from_millis(500)).await;

	_ = robot.drivetrain.model.drive_arcade(-0.3, 0.0);
	sleep(Duration::from_millis(500)).await;
	_ = robot.drivetrain.model.drive_arcade(0.3, 0.0);
	sleep(Duration::from_millis(300)).await;
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);

	//go to point code
	let x = -7.0;
	let y = 94.0;
	let gooo: f64 = Angle::atan2(
		robot.drivetrain.tracking.position().y - y,
		robot.drivetrain.tracking.position().x - x,
	)
	.as_degrees();
	let gooo2: f64 = ((robot.drivetrain.tracking.position().y - y)
		* (robot.drivetrain.tracking.position().y - y)
		+ (robot.drivetrain.tracking.position().x - x)
			* (robot.drivetrain.tracking.position().x - x))
		.sqrt();
	println!("Gooo: {gooo}, {gooo2}");
	println!("Current pos: {:?}", robot.drivetrain.tracking.position());
	println!("Current heading: {:?}", robot.drivetrain.tracking.heading());
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(gooo))
		.await;
	basic.drive_distance(&mut robot.drivetrain, -gooo2).await;
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(-45.0))
		.await;
	_ = robot.drivetrain.model.drive_arcade(0.3, 0.0);
	sleep(Duration::from_millis(1300)).await;
	basic.drive_distance(&mut robot.drivetrain, -4.0).await;
	intake_unjamming!(
		robot,
		IntakeState {
			top: -1.0,
			middle: -1.0,
			bottom: -0.4,
		},
		r => async {
			// Slow down front for the rest to get 15 balls
			sleep(Duration::from_secs(4)).await;
		}
	);
	_ = robot.drivetrain.model.drive_arcade(-0.3, 0.0);
	sleep(Duration::from_millis(1000)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
}

pub async fn pid_testing(robot: &mut Robot) {
	println!("Pid testing!!!");

	let mut basic = crate::control::BASIC_CONTROLLER;

	let start = robot.drivetrain.tracking.forward_travel();

	let target = start + 24.0;
	basic.drive_distance(&mut robot.drivetrain, target).await;

	let end = robot.drivetrain.tracking.forward_travel();

	println!(
		"---------------------------\nStart: {start:.03}\nEnd:   {end:.03}\nDiff:  {diff}\nError: {error}",
		start = start,
		end = end,
		diff = (end - start),
		error = (end - target)
	)
}

pub async fn motion_profile(robot: &mut Robot) {
	let mut ltv = LTVUnicycleMotion {
		controller: LTVUnicycleController::new_with_frc_defaults(),
		linear_tolerances: crate::control::LINEAR_TOLERANCES,
		angular_tolerances: crate::control::ANGULAR_TOLERANCES,
	};

	ltv.drive_distance(&mut robot.drivetrain, 48.0, 24.0, 2.0, 3.25, 0.75, 10.5)
		.await;

	println!("Finished");
}

#[allow(dead_code)]
pub struct StubSelector<R> {
	route: Option<Route<R>>,
}

#[allow(dead_code)]
impl<R> StubSelector<R> {
	pub fn new<const N: usize>(selected: &'static str, routes: [Route<R>; N]) -> Self {
		Self {
			route: routes.into_iter().find(|r| r.name == selected),
		}
	}
}

impl<R> Selector<R> for StubSelector<R> {
	async fn run(&self, robot: &mut R) {
		if let Some(route) = &self.route {
			(route.callback)(robot).await;
		}
	}
}
