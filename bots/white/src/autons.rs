use std::time::Duration;

use autons::{Selector, simple::Route};
use evian::{
	math::{Angle, Vec2},
	prelude::{Arcade, TracksHeading, TracksPosition},
};
use subsystems::{intake::IntakeState, pnemuatics::PneumaticState};
use vexide::time::sleep;

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

	let mut seeking = crate::control::SEEKING_CONTROLLER;

	let mut basic = crate::control::BASIC_CONTROLLER;

	// Robot drives to balls under the goal.

	println!("Drive out of start");
	basic.drive_distance(&mut robot.drivetrain, 20.0).await;

	println!("Move to balls");
	let start = robot.drivetrain.tracking.position();
	let target = Vec2::new(18.0, 55.5);
	println!("{:?}", start);
	seeking.move_to_point(&mut robot.drivetrain, target).await;
	let end = robot.drivetrain.tracking.position();
	println!("{:?}", end);
	println!("{:?}", end - target);

	println!("Turn towards balls");
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 2.0);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(10.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 2.0);
	println!(
		"{:.02} deg",
		robot.drivetrain.tracking.heading().as_degrees()
	);

	println!("Move to position to grab balls");
	basic
		.linear_controller
		.set_kp(basic.linear_controller.kp() * 3.0);
	basic.drive_distance(&mut robot.drivetrain, 3.0).await;
	basic
		.linear_controller
		.set_kp(basic.linear_controller.kp() / 3.0);

	println!("Pull down bar");
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);

	sleep(Duration::from_secs(1)).await;

	return;
	// Robot intakes balls.
	robot.intake.run(IntakeState::full_forward());

	_ = robot.drivetrain.model.drive_arcade(0.15, 0.0);

	sleep(Duration::from_secs(3)).await;

	robot.intake.run(IntakeState::full_brake());

	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Robot positions itself towards middle bar and ejects balls.

	let target = Vec2::new(0.0, 46.7);
	basic.turn_to_point(&mut robot.drivetrain, target).await;
	seeking.move_to_point(&mut robot.drivetrain, target).await;

	// Robot positions itself near the tower and intakes the balls in the tower.

	// Robot turns 180 degress and positions itself toward the upper side beam, then ejects its balls.
}

// End of test code from Leo.

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
