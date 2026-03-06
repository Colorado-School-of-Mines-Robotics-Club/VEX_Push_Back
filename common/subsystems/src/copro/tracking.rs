use std::{
	cell::{Cell, RefCell},
	rc::Rc,
};

use evian::{
	math::{Angle, Vec2},
	prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity},
	tracking::{Gyro, Tracking},
};
use shrewnit::{FeetPerSecond, Inches};
use vexide::{prelude::InertialSensor, sync::Mutex};

use crate::copro::CoproData;

fn imu_to_cartesian_angle(angle: Angle) -> Angle {
	(-angle + Angle::from_degrees(90.0)).wrapped_full()
}

struct TrackingCache {
	heading: Cell<Angle>,
	angular_velocity: Cell<f64>,
}

impl Default for TrackingCache {
	fn default() -> Self {
		Self {
			heading: imu_to_cartesian_angle(Angle::default()).into(),
			angular_velocity: f64::default().into(),
		}
	}
}

/// A struct that, given a reference to updated coprocessor data,
/// implements standard methods for recieving odometry information.
pub struct CoproTracking {
	copro_data: Rc<RefCell<CoproData>>,
	imu: Rc<Mutex<InertialSensor>>,
	cache: TrackingCache,
}

impl CoproTracking {
	pub fn new(copro_data: Rc<RefCell<CoproData>>, imu: Rc<Mutex<InertialSensor>>) -> Self {
		Self {
			copro_data,
			cache: imu
				.try_lock()
				.map(|imu| TrackingCache {
					heading: imu_to_cartesian_angle(imu.heading().ok().unwrap_or_default()).into(),
					angular_velocity: imu.angular_velocity().unwrap_or_default().into(),
				})
				.unwrap_or_default(),
			imu,
		}
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
		self.imu
			.try_lock()
			.and_then(|i| i.heading().ok())
			.map(imu_to_cartesian_angle)
			.inspect(|h| self.cache.heading.set(*h))
			.unwrap_or(self.cache.heading.get())
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
		self.imu
			.try_lock()
			.and_then(|i| i.angular_velocity().ok())
			.inspect(|v| self.cache.angular_velocity.set(*v))
			.unwrap_or(self.cache.angular_velocity.get())
	}
}
