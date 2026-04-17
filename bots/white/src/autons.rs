use std::time::{Duration, Instant};

use autons::{Selector, simple::Route};
use coprocessor::requests::GetStdDevRequest;
use evian::{
	math::{Angle, Vec2},
	prelude::{Arcade, TracksHeading, TracksPosition},
};
use shrewnit::{Degrees, Inches};
use subsystems::{intake::IntakeState, intake_unjamming, pnemuatics::PneumaticState};
use vexide::{smart::motor::BrakeMode, time::sleep};

use crate::robot::Robot;

macro_rules! boost_kp {
	($basic:ident, $factor:literal, $code:expr) => {
		$basic
			.angular_controller
			.set_kp($basic.angular_controller.kp() * $factor);
		$code;
		$basic
			.angular_controller
			.set_kp($basic.angular_controller.kp() / $factor);
	};
}

const FIELD_TILE_LENGTH: f64 = 24.0;

pub async fn do_nothing(_robot: &mut Robot) {
	println!("Doing absolutely nothing!!!");
}

pub async fn match_auton(robot: &mut Robot) {
	let matchload_line = 30.0;

	let start_time = Instant::now();
	// Basic setup.
	_ = robot
		.pneumatics
		.extender
		.set_state(PneumaticState::Extended);

	let mut basic = crate::control::BASIC_CONTROLLER;
	let mut _seeking = crate::control::SEEKING_CONTROLLER;

	// Line up with the center goal
	basic.drive_distance(&mut robot.drivetrain, 45.5).await;

	// Turn towards center goal
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 1.5);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0 + 45.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 1.5);

	// Align with center
	_ = robot.drivetrain.model.drive_arcade(0.25, 0.0);
	sleep(Duration::from_millis(1000)).await;
	_ = robot.drivetrain.model.drive_arcade(-0.15, 0.0);
	sleep(Duration::from_millis(350)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Outtake preload ball into center goal
	robot.intake.run(IntakeState {
		top: -1.0,
		middle: -1.0,
		bottom: -0.50,
	});
	sleep(Duration::from_secs(1)).await;
	robot.intake.run(IntakeState::full_brake());

	// Go generally towards aligned with the matchload
	basic
		.drive_distance_at_heading(
			&mut robot.drivetrain,
			-25.0,
			Angle::from_degrees(-45.0 + 180.0),
		)
		.await;

	// Move to matchload line
	let dist_x = (robot.drivetrain.tracking.position().x - matchload_line).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, -dist).await;

	// Turn towards matchload
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 1.1);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(-90.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 1.1);

	// Back up as a precaution for the front bar
	_ = robot.drivetrain.model.drive_arcade(-0.17, 0.0);
	sleep(Duration::from_millis(350)).await;

	// Intake from matchload
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	sleep(Duration::from_millis(500)).await;
	_ = robot.drivetrain.model.drive_arcade(0.35, 0.0);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_millis(2500)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
	sleep(Duration::from_secs(2)).await;

	// Move to long goal position
	robot.intake.run(IntakeState::full_brake());

	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.01);
	sleep(Duration::from_secs(2)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Eject 3 balls into long goal
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	robot.intake.run(IntakeState {
		top: 1.0,
		middle: 0.0,
		bottom: 0.0,
	});
	sleep(Duration::from_secs(2)).await;
	robot.intake.run(IntakeState::full_brake());

	// Back up and eject balls out of the intake
	basic.drive_distance(&mut robot.drivetrain, 12.0).await;
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Contracted);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_secs(2)).await;

	while let error =
		(robot.drivetrain.tracking.heading().wrapped_full() - Angle::from_degrees(90.0 + 180.0))
		&& error.abs() > Angle::from_degrees(1.0)
	{
		_ = robot
			.drivetrain
			.model
			.drive_arcade(0.0, 0.12 * error.signum());
		sleep(Duration::from_millis(5)).await;
	}
	robot.drivetrain.brake(BrakeMode::Hold);

	// Go back to matchload and grab balls
	_ = robot
		.pneumatics
		.outtake_adjuster
		.set_state(PneumaticState::Extended);
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	_ = robot.drivetrain.model.drive_arcade(0.35, 0.0);
	sleep(Duration::from_secs(2)).await;

	// Score the balls on long goal
	basic.drive_distance(&mut robot.drivetrain, -36.0).await;
	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.0);
	sleep(Duration::from_millis(500)).await;

	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_secs(2)).await;

	return;

	// Move back a bit and eject all balls (wrong color)
	_ = robot.drivetrain.model.drive_arcade(0.40, 0.0);
	sleep(Duration::from_millis(500)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() * 3.05);
	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(75.0 + 180.0))
		.await;
	robot.intake.run(IntakeState::full_forward());
	sleep(Duration::from_millis(1500)).await;

	let dist_x = (robot.drivetrain.tracking.position().x - matchload_line).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, dist).await;

	basic
		.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0 + 180.0))
		.await;
	basic
		.angular_controller
		.set_kp(basic.angular_controller.kp() / 3.05);

	if robot.drivetrain.tracking.position().y() >= 15.0 {
		// Back up as a precaution for the front bar
		_ = robot.drivetrain.model.drive_arcade(-0.17, 0.0);
		sleep(Duration::from_millis(350)).await;
	}

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);

	// Move the rest of the distance and intake balls
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	_ = robot.drivetrain.model.drive_arcade(0.35, 0.0);
	sleep(Duration::from_secs(3)).await;

	// Drive to long goal
	basic.timeout = Some(Duration::from_secs(1));
	basic
		.drive_distance_at_heading(
			&mut robot.drivetrain,
			-40.0,
			Angle::from_degrees(180.0 + 90.0),
		)
		.await;
	basic.timeout = crate::control::BASIC_CONTROLLER.timeout;

	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);

	// Outtake the balls!!!!!!!!!
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	sleep(Duration::from_secs(2)).await;
	robot.intake.run(IntakeState::full_brake());

	// Move back from goal
	_ = robot.drivetrain.model.drive_arcade(0.30, 0.0);
	sleep(Duration::from_millis(250)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
	sleep(Duration::from_millis(250)).await;

	// Go to park zone
	// basic
	// 	.angular_controller
	// 	.set_kp(basic.angular_controller.kp() * 2.3);
	// basic
	// 	.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(180.0 + 45.0))
	// 	.await;
	// basic
	// 	.angular_controller
	// 	.set_kp(basic.angular_controller.kp() / 2.3);

	println!("Done: {}s", start_time.elapsed().as_secs_f32());
}

pub async fn skills_doublepark(robot: &mut Robot) {
	robot.intake.run(IntakeState::full_forward());
	_ = robot.drivetrain.model.drive_arcade(0.4, 0.0);
	sleep(Duration::from_millis(1250)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
	sleep(Duration::from_secs(3)).await;
}

pub async fn skills_main(robot: &mut Robot) {
	let start_time = Instant::now();

	let mut basic = crate::control::BASIC_CONTROLLER;

	// Drive out of starting position, now top-right of park zone
	basic.drive_distance(&mut robot.drivetrain, 18.0).await;

	// Drive to right side of right long goal
	let target = Vec2::new(42.0, FIELD_TILE_LENGTH);
	boost_kp!(
		basic,
		1.1,
		basic.turn_to_point(&mut robot.drivetrain, target).await
	);

	let pos = robot.drivetrain.tracking.position();
	basic
		.drive_distance(&mut robot.drivetrain, target.distance(pos))
		.await;

	// Drive to other side of field
	boost_kp!(
		basic,
		1.2,
		basic
			.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(87.0))
			.await
	);
	basic
		.drive_distance_at_heading(
			&mut robot.drivetrain,
			FIELD_TILE_LENGTH * 3.0 - 2.0,
			Angle::from_degrees(90.0),
		)
		.await;

	// Angle towards matchload align position
	boost_kp!(
		basic,
		1.5,
		basic
			.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0 + 45.0))
			.await
	);

	// Drive to matchload alignment
	let matchload_line = 27.2;
	let dist_x = (robot.drivetrain.tracking.position().x - matchload_line).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, dist).await;

	// Turn to matchload
	boost_kp!(
		basic,
		1.37,
		basic
			.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0))
			.await
	);

	// Matchload
	sleep(Duration::from_millis(500)).await;
	intake_unjamming!(robot.intake, IntakeState::full_forward(), async {
		_ = robot
			.pneumatics
			.front_bar
			.set_state(PneumaticState::Extended);
		_ = robot
			.pneumatics
			.extender
			.set_state(PneumaticState::Extended);
		_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
		_ = robot
			.pneumatics
			.outtake_adjuster
			.set_state(PneumaticState::Extended);
		sleep(Duration::from_secs(1)).await;

		_ = robot.drivetrain.model.drive_arcade(0.35, 0.0);
		sleep(Duration::from_secs(1)).await;
		_ = robot.drivetrain.model.drive_arcade(0.5, 0.0);
		sleep(Duration::from_secs(2)).await;
	});

	robot.intake.run(IntakeState {
		top: 0.0,
		middle: 0.0,
		bottom: 1.0,
	});

	// Drive to side balls alignment
	let red_balls_y = 108.9;
	let dist_y = (robot.drivetrain.tracking.position().y - red_balls_y).abs();
	let dist = (dist_y / (robot.drivetrain.tracking.heading()).sin()).abs();

	basic.drive_distance(&mut robot.drivetrain, -dist).await;

	// Pull up bar before turning
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Contracted);

	// Turn towards balls
	boost_kp!(
		basic,
		1.25,
		basic
			.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(0.0))
			.await
	);

	// Drive up to balls
	let red_balls_x = 41.5;
	let dist_x = (robot.drivetrain.tracking.position().x - red_balls_x).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, dist).await;

	robot.intake.run(IntakeState::full_brake());

	// Put bar down and bring balls back from wall
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);
	sleep(Duration::from_secs(1)).await;
	_ = robot.drivetrain.model.drive_arcade(-0.1, 0.0);
	sleep(Duration::from_secs(1)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Back up to get room for intaking balls
	let back_up_x = FIELD_TILE_LENGTH * 0.75;
	let dist_x = (robot.drivetrain.tracking.position().x - back_up_x).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, -dist).await;

	// Run intake while driving forward to grab balls
	robot.intake.run(IntakeState {
		top: 0.0,
		middle: 0.5,
		bottom: 1.0,
	});
	boost_kp!(
		basic,
		1.2,
		basic
			.drive_distance_at_heading(&mut robot.drivetrain, 26.0, Angle::from_degrees(-60.0))
			.await
	);

	_ = robot.drivetrain.model.drive_arcade(0.15, 0.08);
	sleep(Duration::from_millis(750)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Drive to original side of field
	let heading = Angle::from_degrees(90.0 + 180.0);
	boost_kp!(
		basic,
		1.3,
		basic.turn_to_heading(&mut robot.drivetrain, heading).await
	);
	basic
		.drive_distance_at_heading(&mut robot.drivetrain, FIELD_TILE_LENGTH * 2.0, heading)
		.await;

	// Turn towards machload line
	boost_kp!(
		basic,
		1.4,
		basic
			.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(225.0))
			.await
	);

	// Move to machload line
	let matchload_line = 27.9;
	let dist_x = (robot.drivetrain.tracking.position().x - matchload_line).abs();
	let dist = (dist_x / (robot.drivetrain.tracking.heading()).cos()).abs();

	basic.drive_distance(&mut robot.drivetrain, dist).await;

	// Turn towards matchload
	boost_kp!(
		basic,
		1.4,
		basic
			.turn_to_heading(&mut robot.drivetrain, Angle::from_degrees(90.0 + 180.0))
			.await
	);

	// Move to long goal
	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.01);
	sleep(Duration::from_secs(2)).await;
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);

	// Outtake balls
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);
	_ = robot
		.pneumatics
		.front_bar
		.set_state(PneumaticState::Extended);

	intake_unjamming!(robot.intake, IntakeState::full_forward(), async {
		sleep(Duration::from_secs(3)).await;
	});

	// Move to matchload and intake balls
	_ = robot.drivetrain.model.drive_arcade(0.35, 0.01);
	sleep(Duration::from_secs(1)).await;
	_ = robot.pneumatics.flap.set_state(PneumaticState::Contracted);
	sleep(Duration::from_secs(1)).await;

	intake_unjamming!(robot.intake, IntakeState::full_forward(), async {
		_ = robot.drivetrain.model.drive_arcade(0.5, 0.01);
		sleep(Duration::from_secs(2)).await;
	});

	// Move to long goal and outtake balls
	_ = robot.drivetrain.model.drive_arcade(-0.35, 0.01);
	sleep(Duration::from_secs(2)).await;
	_ = robot.pneumatics.flap.set_state(PneumaticState::Extended);

	intake_unjamming!(robot.intake, IntakeState::full_forward(), async {
		sleep(Duration::from_secs(1)).await;
	});

	intake_unjamming!(
		robot.intake,
		IntakeState {
			top: 0.5,
			middle: 1.0,
			bottom: 1.0,
		},
		async {
			// Slow down front for the rest to get 15 balls
			sleep(Duration::from_secs(1)).await;
		}
	);

	println!("Time elapsed: {}s", start_time.elapsed().as_secs_f64());
	_ = robot.drivetrain.model.drive_arcade(0.0, 0.0);
	robot.intake.run(IntakeState::full_brake());

	let stddev = robot.coprocessor.send_request(GetStdDevRequest).await;
	if let Ok(stddev) = stddev {
		println!(
			"StdDev: X: {}in, Y: {}in, H: {}deg",
			stddev.x.to::<Inches>(),
			stddev.y.to::<Inches>(),
			stddev.heading.to::<Degrees>()
		)
	} else {
		println!("StdDev request failure")
	}
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
