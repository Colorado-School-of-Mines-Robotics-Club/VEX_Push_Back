use std::{cell::RefCell, ops::Deref, rc::Rc};

use evian::{
	math::{Angle, Vec2},
	prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity},
	tracking::Tracking,
};
use shrewnit::{Degrees, FeetPerSecond, Inches, Radians, RadiansPerSecond};

use crate::copro::CoproData;

/// A struct that, given a reference to updated coprocessor data,
/// implements standard methods for recieving odometry information.
pub struct CoproTracking(pub Rc<RefCell<CoproData>>);

impl Deref for CoproTracking {
	type Target = RefCell<CoproData>;

	fn deref(&self) -> &Self::Target {
		&self.0
	}
}

impl Tracking for CoproTracking {}

impl TracksForwardTravel for CoproTracking {
	fn forward_travel(&self) -> f64 {
		self.borrow().forward_travel.to::<Inches>()
	}
}

impl TracksHeading for CoproTracking {
	// OTOS heading:
	// Clockwise is negative, until 180
	// Counterclockwise is positive, until 180
	fn heading(&self) -> vexide::math::Angle {
		let heading = self.borrow().position.heading + 90.0 * Degrees; // The sensor uses 0.0 as forward, so adjust it to cartesian-style

		Angle::from_radians(heading.to::<Radians>()).wrapped_full()
	}
}

impl TracksPosition for CoproTracking {
	fn position(&self) -> evian::math::Vec2<f64> {
		let pos = self.borrow().position;

		let x = pos.x.to::<Inches>();
		let y = pos.y.to::<Inches>();

		Vec2::new(x, y)
	}
}

impl TracksVelocity for CoproTracking {
	fn linear_velocity(&self) -> f64 {
		let velocity = self.borrow().velocity;

		Vec2::new(
			velocity.x.to::<FeetPerSecond>() * 12.0,
			velocity.y.to::<FeetPerSecond>() * 12.0,
		)
		.length()
	}

	fn angular_velocity(&self) -> f64 {
		self.borrow().velocity.heading.to::<RadiansPerSecond>()
	}
}
