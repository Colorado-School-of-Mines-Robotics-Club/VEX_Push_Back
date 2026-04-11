use std::time::Duration;

use bitcode::{Decode, Encode};
use evian::{
	math::{Angle, Vec2},
	prelude::{TracksHeading, TracksPosition, TracksVelocity},
	tracking::Tracking,
};

#[derive(Encode, Decode)]
pub struct TelemetryLog {
	/// Time on the high precision clock in microseconds
	pub time: u64,
	/// The current position of the robot
	pub position: (f64, f64),
	/// The current velocity of the robot
	pub velocity: f64,
	/// The current heading of the robot (in radians)
	pub heading: f64,
	/// The current angular velocity of the robot
	pub angular_velocity: f64,
}

impl TelemetryLog {
	pub fn from_tracking<T>(time: Duration, tracking: T) -> Self
	where
		T: Tracking + TracksPosition + TracksVelocity + TracksHeading,
	{
		let pos = tracking.position();

		Self {
			time: time.as_micros() as u64,
			position: (pos.x(), pos.y()),
			velocity: tracking.linear_velocity(),
			angular_velocity: tracking.angular_velocity(),
			heading: tracking.heading().as_radians(),
		}
	}
}
