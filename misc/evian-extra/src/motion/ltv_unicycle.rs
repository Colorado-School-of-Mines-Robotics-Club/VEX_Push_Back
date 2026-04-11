use std::time::{Duration, Instant};

use evian::{
	control::loops::Feedback,
	math::Vec2,
	prelude::{Drivetrain, Tolerances, TracksHeading, TracksPosition, TracksVelocity},
};
use nalgebra::vector;
use vexide::time::sleep;

use crate::{
	TankVelocity,
	control::ltv_unicycle::{LTVState, LTVUnicycleController},
};

pub struct LTVUnicycleMotion {
	pub controller: LTVUnicycleController,
	pub linear_tolerances: Tolerances,
	pub angular_tolerances: Tolerances,
}

pub fn trapezoid_profile(
	current_time: f64,
	start_pos: Vec2<f64>,
	start_heading: f64,
	distance: f64,
	peak_velocity: f64,
	accel_time: f64,
) -> LTVState {
	if current_time < accel_time {
		let pos = 0.5 * (peak_velocity / accel_time) * current_time.powi(2);

		LTVState {
			position: vector![
				start_pos.x + pos * start_heading.cos(),
				start_pos.y + pos * start_heading.sin()
			],
			heading: start_heading,
			velocity: (peak_velocity / accel_time) * current_time,
		}
	} else if current_time < distance / peak_velocity {
		let stage1_pos = 0.5 * (peak_velocity / accel_time) * accel_time.powi(2);
		let pos = stage1_pos + peak_velocity * (current_time - accel_time);
		LTVState {
			position: vector![
				start_pos.x + pos * start_heading.cos(),
				start_pos.y + pos * start_heading.sin()
			],
			heading: start_heading,
			velocity: peak_velocity,
		}
	} else {
		let stage3_time = current_time - distance / peak_velocity;
		let stage1_pos = 0.5 * (peak_velocity / accel_time) * accel_time.powi(2);
		let stage2_pos = stage1_pos + peak_velocity * (distance / peak_velocity - accel_time);
		let stage3_pos = stage2_pos + peak_velocity * stage3_time
			- 0.5 * (peak_velocity / accel_time) * stage3_time.powi(2);

		LTVState {
			position: vector![
				start_pos.x + stage3_pos * start_heading.cos(),
				start_pos.y + stage3_pos * start_heading.sin()
			],
			heading: start_heading,
			velocity: peak_velocity - stage3_time * (peak_velocity / accel_time),
		}
	}
}

impl LTVUnicycleMotion {
	pub async fn drive_distance<
		M: TankVelocity,
		T: TracksPosition + TracksVelocity + TracksHeading,
	>(
		&mut self,
		drivetrain: &mut Drivetrain<M, T>,
		distance: f64,
		peak_velocity: f64,
		accel_time: f64,
		wheel_diameter: f64,
		gear_ratio: f64,
		track_width: f64,
	) {
		let start_time = Instant::now();
		let mut last_update = Instant::now();
		let mut controller = self.controller.clone();

		let start = drivetrain.tracking.position();
		let start_heading = drivetrain.tracking.heading();
		let target = Vec2::new(
			start.x + distance * start_heading.cos(),
			start.y + distance * start_heading.sin(),
		);

		loop {
			let current_pose = drivetrain.tracking.position();
			let current_heading = drivetrain.tracking.heading();
			let current_linear_velocity = drivetrain.tracking.linear_velocity();
			let current_angular_velocity = drivetrain.tracking.angular_velocity();

			if self
				.linear_tolerances
				.check(target.distance(current_pose), current_linear_velocity)
				&& self.angular_tolerances.check(
					(start_heading - current_heading)
						.wrapped_half()
						.as_radians(),
					current_angular_velocity,
				) {
				return;
			}

			// Calculate trapezoidal profile position & velocity target
			let setpoint = trapezoid_profile(
				start_time.elapsed().as_secs_f64(),
				start,
				start_heading.as_radians(),
				distance,
				peak_velocity,
				accel_time,
			);

			let signal = controller.update(
				LTVState {
					position: vector![current_pose.x, current_pose.y],
					heading: current_heading.as_radians(),
					velocity: current_linear_velocity,
				},
				setpoint,
				last_update.elapsed(),
			);
			let rpms = vector![
				signal.x - signal.y * (track_width / 2.0),
				signal.x + signal.y * (track_width / 2.0)
			] / (std::f64::consts::PI * wheel_diameter)
				/ gear_ratio;

			_ = drivetrain
				.model
				.drive_tank_velocity(rpms.x as i32, rpms.y as i32);

			last_update = Instant::now();
			sleep(Duration::from_millis(10)).await;
		}
	}
}
