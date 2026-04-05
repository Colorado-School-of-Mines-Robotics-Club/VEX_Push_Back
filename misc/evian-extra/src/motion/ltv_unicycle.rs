use std::{
	pin::Pin,
	task::Poll,
	time::{Duration, Instant},
};

use evian::{
	control::loops::Feedback,
	math::{Angle, Vec2},
	prelude::{Drivetrain, Tolerances, TracksHeading, TracksPosition, TracksVelocity},
};
use nalgebra::{Vector3, vector};
use vexide::time::{Sleep, sleep};

use crate::{TankVelocity, control::ltv_unicycle::LTVUnicycleController};

pub struct LTVUnicycleMotion {
	pub controller: LTVUnicycleController,
	pub linear_tolerances: Tolerances,
	pub angular_tolerances: Tolerances,
}

impl LTVUnicycleMotion {
	pub async fn move_to_point<
		M: TankVelocity,
		T: TracksPosition + TracksVelocity + TracksHeading,
	>(
		&mut self,
		drivetrain: &mut Drivetrain<M, T>,
		point: Vec2<f64>,
		heading: Angle,
	) {
		let mut last_update = Instant::now();
		let mut controller = self.controller.clone();

		loop {
			let current_pose = drivetrain.tracking.position();
			let current_heading = drivetrain.tracking.heading();
			let current_linear_velocity = drivetrain.tracking.linear_velocity();
			let current_angular_velocity = drivetrain.tracking.angular_velocity();

			if self
				.linear_tolerances
				.check(point.distance(current_pose), current_linear_velocity)
				&& self.angular_tolerances.check(
					(heading - current_heading).wrapped_half().as_radians(),
					current_angular_velocity,
				) {
				return;
			}

			let signal = controller.update(
				vector![current_pose.x, current_pose.y, current_heading.as_radians()],
				vector![point.x, point.y, heading.as_radians()],
				last_update.elapsed(),
			);
			let rpms = signal / std::f64::consts::TAU * 60.0;

			_ = drivetrain
				.model
				.drive_tank_velocity(rpms.x as i32, rpms.y as i32);

			last_update = Instant::now();
			sleep(Duration::from_millis(10)).await;
		}
	}
}
