use std::time::Duration;

use autons::{Selector, simple::Route};
use evian::{
	math::Angle,
	prelude::{Arcade, TracksHeading, TracksPosition},
};
use subsystems::{intake::IntakeState, pnemuatics::PneumaticState};
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
	basic.drive_distance(&mut robot.drivetrain, -45.0).await;

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
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0))
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
	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.00);
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
	basic.drive_distance(&mut robot.drivetrain, 12.0).await;
	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 1.0,
		bottom: 1.0,
	});
	sleep(Duration::from_secs(1)).await;

	robot.intake.run(IntakeState::full_brake());
	println!("Done");

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

pub async fn skills_auton(robot: &mut Robot) {
	// Drive back to be technically parked
	_ = robot.drivetrain.model.drive_arcade(-0.20, 0.0);
	sleep(Duration::from_millis(100)).await;
	_ = robot.drivetrain.brake(BrakeMode::Hold);
	sleep(Duration::from_millis(1000)).await;

	// Park
	_ = robot
		.intake
		.park_piston()
		.unwrap()
		.set_state(PneumaticState::Extended);
}

pub async fn pid_testing(robot: &mut Robot) {
	println!("Pid testing!!!");

	let mut basic = crate::control::BASIC_CONTROLLER;
	let mut _seeking = crate::control::SEEKING_CONTROLLER;

	let start = robot.drivetrain.tracking.heading();

	let target = start + Angle::from_degrees(180.0);
	basic.turn_to_heading(&mut robot.drivetrain, target).await;

	let end = robot.drivetrain.tracking.heading();

	println!(
		"---------------------------\nStart: {start:.03}\nEnd:   {end:.03}\nDiff:  {diff}\nError: {error}",
		start = start.as_degrees(),
		end = end.as_degrees(),
		diff = (end - start).as_degrees(),
		error = (end - target).wrapped_full().as_degrees()
	)
}

pub async fn park_test(robot: &mut Robot) {
	loop {
		dbg!(robot.intake.park_sensor().unwrap().proximity().unwrap());
		sleep(Duration::from_millis(250)).await;
	}
}

pub async fn unjam_test(robot: &mut Robot) {
	robot
		.intake
		.run_unjamming(IntakeState::full_forward(), Duration::from_secs(5))
		.await;
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
