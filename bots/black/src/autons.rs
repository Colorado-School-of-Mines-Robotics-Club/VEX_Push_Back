use std::time::Duration;

use autons::{Selector, simple::Route};
use evian::{
	math::{Angle, Vec2}, motion::basic, prelude::{Arcade, TracksHeading, TracksPosition}
};
use slintui::slint::platform::duration_until_next_timer_update;
use subsystems::{intake::IntakeState, pnemuatics::PneumaticState};
use vexide::time::sleep;

use crate::robot::Robot;

pub async fn do_nothing(_robot: &mut Robot) {
	println!("Doing absolutely nothing!!!");
}

// Time based backup code, only use if the sensor breaks.

/*

pub async fn match_auton(robot: &mut Robot) {
	// Robot goes to post and ejects its balls into the top beam.

	robot.drivetrain.model.drive_arcade(-0.5, 0.0);
	sleep(Duration::from_secs(2)).await;
	robot.drivetrain.model.drive_arcade(0.0, 0.25);
	sleep(Duration::from_millis(700)).await;
	robot.intake.run(IntakeState {
		top: 0.35,
		middle: 1.0,
		bottom: 1.0,
	});
	sleep(Duration::from_secs(2));
	robot.intake.run(IntakeState::full_brake());

	// Robot drives to the tower and takes the tower balls.

	robot.drivetrain.model.drive_arcade(-0.50, 0.0);
	sleep(Duration::from_secs(2)).await;
	robot.drivetrain.model.drive_arcade(0.0, -0.25);
	sleep(Duration::from_millis(700)).await;

	// Robot drives back to the higher beam and ejects its balls.

	robot.drivetrain.model.drive_arcade(-0.25, 0.0);
	sleep(Duration::from_secs(2));
	robot.intake.run(IntakeState {
		top: 0.35,
		middle: 1.0,
		bottom: 1.0,
	});
	sleep(Duration::from_secs(2));
	robot.intake.run(IntakeState::full_brake());

}
*/

// Standard code from Leo.

pub async fn match_auton(robot: &mut Robot) {
	// Basic setup.
	_ = robot
		.pneumatics
		.extender
		.set_state(PneumaticState::Extended);

	let mut seeking = crate::control::SEEKING_CONTROLLER;

	let mut basic = crate::control::BASIC_CONTROLLER;

	basic.drive_distance(&mut robot.drivetrain, -40.5).await;

	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 1.5);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(45.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 1.5);

	_ = robot.drivetrain.model.drive_arcade(-0.25, 0.0);
	sleep(Duration::from_millis(900)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Robot intake runs and ejects the balls into the middle beam.

	robot.intake.run(IntakeState {
		top: 0.35,
		middle: 1.0,
		bottom: 1.0,
	});
	sleep(Duration::from_secs(2)).await;
	robot.intake.run(IntakeState::full_brake());

	basic
		.drive_distance_at_heading(&mut robot.drivetrain, 25.0, Angle::from_degrees(45.0))
		.await;

	// Calculate distance from line
	let dist_x = (robot.drivetrain.tracking.position().x + 31.25).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic
		.drive_distance(&mut robot.drivetrain, -dbg!(dist))
		.await;

	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 1.05);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(-90.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 1.05);

	dbg!(robot.drivetrain.tracking.heading().as_degrees());

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);
	sleep(Duration::from_millis(500)).await;
	_ = robot.drivetrain.model.drive_arcade(0.5, 0.0);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_secs(2)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	sleep(Duration::from_secs(3)).await;

	_ = robot.drivetrain.model.drive_arcade(-0.25, 0.0);
	sleep(Duration::from_millis(1500)).await;
	_ = robot.drivetrain.model.drive_arcade(0.25, 0.0);
	sleep(Duration::from_secs(1)).await;

	robot.intake.run(IntakeState::full_brake());

	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 1.2);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(180.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 1.2);

	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Contracted);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_secs(2)).await;
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);

	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 1.1);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(270.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 1.1);

	println!("WOWOWOWOOWOWOW");
}

pub async fn pid_testing(robot: &mut Robot) {
	println!("Pid testing!!!");

	let mut basic = crate::control::BASIC_CONTROLLER;
	let mut seeking = crate::control::SEEKING_CONTROLLER;
	// seeking.linear_controller = Pid::new(0.0, 0.0, 0.0, None);

	let start = robot.drivetrain.tracking.heading();

	let target = Angle::from_degrees(-45.0);
	seeking
		.move_to_point(&mut robot.drivetrain, Vec2::new(24.0, 24.0))
		.await;

	let end = robot.drivetrain.tracking.heading();

	println!(
		"---------------------------\nStart: {start:.03}\nEnd:   {end:.03}\nDiff:  {diff}\nError: {error}",
		start = start.as_degrees(),
		end = end.as_degrees(),
		diff = (end - start).as_degrees(),
		error = (target - (end - start)).as_degrees()
	)
}

#[allow(dead_code)]
pub async fn testing(robot: &mut Robot) {
	robot.imu.borrow_mut().calibrate().await.unwrap();
	robot.coprocessor.calibrate().await.unwrap();
	sleep(Duration::from_millis(500)).await;

	loop {
		let copro = robot.drivetrain.tracking.heading();
		let imu = -robot.imu.borrow().heading().unwrap() + Angle::from_degrees(90.0);
		println!(
			"{}, {}, {}",
			copro.wrapped_full().as_degrees(),
			imu.wrapped_full().as_degrees(),
			(copro - imu).as_degrees()
		);
		sleep(Duration::from_millis(500)).await;
	}
}

pub struct StubSelector<R> {
	route: Option<Route<R>>,
}

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
