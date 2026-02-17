use std::{
	task::Poll,
	time::{Duration, Instant},
};

use evian::{
	control::loops::Feedback,
	math::{Angle, Vec2},
	prelude::{Arcade, Drivetrain, Tolerances, TracksHeading, TracksPosition, TracksVelocity},
};

pub struct BoomerangMotion<
	L: Feedback<State = f64, Signal = f64> + Clone,
	A: Feedback<State = Angle, Signal = f64> + Clone,
> {
	pub linear_controller: L,
	pub angular_controller: A,
	pub linear_tolerances: Tolerances,
	pub angular_tolerances: Tolerances,
}

impl<
	L: Feedback<State = f64, Signal = f64> + Clone,
	A: Feedback<State = Angle, Signal = f64> + Clone,
> BoomerangMotion<L, A>
{
	pub fn boomerang<'a, M: Arcade, T: TracksPosition + TracksVelocity + TracksHeading>(
		&self,
		drivetrain: &'a mut Drivetrain<M, T>,
		target: Vec2<f64>,
		theta: Angle,
		dlead: f64,
	) -> BoomerangFuture<'a, L, A, M, T> {
		BoomerangFuture {
			linear_controller: self.linear_controller.clone(),
			angular_controller: self.angular_controller.clone(),
			linear_tolerances: self.linear_tolerances,
			angular_tolerances: self.angular_tolerances,
			drivetrain,
			target,
			theta,
			dlead,
			last_poll: None,
		}
	}
}

pub struct BoomerangFuture<
	'a,
	L: Feedback<State = f64, Signal = f64>,
	A: Feedback<State = Angle, Signal = f64>,
	M: Arcade,
	T: TracksPosition + TracksVelocity + TracksHeading,
> {
	linear_controller: L,
	angular_controller: A,
	linear_tolerances: Tolerances,
	angular_tolerances: Tolerances,
	drivetrain: &'a mut Drivetrain<M, T>,
	target: Vec2<f64>,
	theta: Angle,
	dlead: f64,
	last_poll: Option<Instant>,
}

impl<
	'a,
	L: Feedback<State = f64, Signal = f64> + Unpin,
	A: Feedback<State = Angle, Signal = f64> + Unpin,
	M: Arcade,
	T: TracksPosition + TracksVelocity + TracksHeading,
> Future for BoomerangFuture<'a, L, A, M, T>
{
	type Output = ();

	fn poll(
		self: std::pin::Pin<&mut Self>,
		_cx: &mut std::task::Context<'_>,
	) -> std::task::Poll<Self::Output> {
		let this = self.get_mut();
		let current_pose = this.drivetrain.tracking.position();
		let current_heading = this.drivetrain.tracking.heading();
		let current_linear_velocity = this.drivetrain.tracking.linear_velocity();
		let current_angular_velocity = this.drivetrain.tracking.angular_velocity();

		if this
			.linear_tolerances
			.check(this.target.distance(current_pose), current_linear_velocity)
			&& this.angular_tolerances.check(
				(this.theta - current_heading).wrapped_half().as_radians(),
				current_angular_velocity,
			) {
			return Poll::Ready(());
		}

		let distance = current_pose.distance(this.target);
		let close = distance < 7.5;
		// let carrot = Vec2::new(
		//     this.target.x - distance * this.theta.cos() * this.dlead,
		//     this.target.y - distance * this.theta.sin() * this.dlead,
		// );
		let carrot = if close {
			// this.target
			return Poll::Ready(());
		} else {
			this.target - Vec2::from_polar(distance * this.dlead, this.theta.as_radians())
		};

		let linear_error = current_pose.distance(carrot);
		let angular_error = (current_heading - this.theta).wrapped_full();
		let dt = this
			.last_poll
			.map(|i| i.elapsed())
			.unwrap_or(Duration::ZERO);

		let throttle = this.linear_controller.update(0.0, linear_error, dt);
		let steer = this
			.angular_controller
			.update(angular_error, this.theta, dt);
		this.last_poll = Some(Instant::now());

		_ = this.drivetrain.model.drive_arcade(throttle, steer);

		Poll::Pending
	}
}
