use std::{cell::RefCell, ops::Deref, rc::Rc};

use evian::{
	math::{Angle, Vec2},
	prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity},
	tracking::{Gyro, Tracking},
};
use shrewnit::{Degrees, FeetPerSecond, Inches, Radians, RadiansPerSecond};
use vexide::prelude::InertialSensor;

use crate::copro::CoproData;

/// A struct that, given a reference to updated coprocessor data,
/// implements standard methods for recieving odometry information.
pub struct CoproTracking {
	copro_data: Rc<RefCell<CoproData>>,
	imu: Rc<RefCell<InertialSensor>>,
}

impl CoproTracking {
	pub fn new(copro_data: Rc<RefCell<CoproData>>, imu: Rc<RefCell<InertialSensor>>) -> Self {
		Self { copro_data, imu }
	}
}

impl Tracking for CoproTracking {}

impl TracksForwardTravel for CoproTracking {
	fn forward_travel(&self) -> f64 {
		self.copro_data.borrow().forward_travel.to::<Inches>()
	}
}

impl TracksHeading for CoproTracking {
	// OTOS heading:
	// Clockwise is negative, until 180
	// Counterclockwise is positive, until 180
	fn heading(&self) -> vexide::math::Angle {
		// let heading = self.copro_data.borrow().position.heading + 90.0 * Degrees; // The sensor uses 0.0 as forward, so adjust it to cartesian-style

		// Angle::from_radians(heading.to::<Radians>()).wrapped_full()
		-self.imu.borrow().heading().unwrap_or_default() + Angle::from_degrees(90.0)
	}
}

impl TracksPosition for CoproTracking {
	fn position(&self) -> evian::math::Vec2<f64> {
		let pos = self.copro_data.borrow().position;

		let x = pos.x.to::<Inches>();
		let y = pos.y.to::<Inches>();

		Vec2::new(x, y)
	}
}

impl TracksVelocity for CoproTracking {
	fn linear_velocity(&self) -> f64 {
		let velocity = self.copro_data.borrow().velocity;

		Vec2::new(
			velocity.x.to::<FeetPerSecond>() * 12.0,
			velocity.y.to::<FeetPerSecond>() * 12.0,
		)
		.length()
	}

	fn angular_velocity(&self) -> f64 {
		// self.copro_data
		// 	.borrow()
		// 	.velocity
		// 	.heading
		// 	.to::<RadiansPerSecond>()
		self.imu.borrow().angular_velocity().unwrap_or_default()
	}
}
