use std::{cell::RefCell, f64::consts::PI, ops::Deref, rc::Rc};

use coprocessor::vexide::CoprocessorData;
use evian::{
    math::{Angle, Vec2},
    prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity},
    tracking::Tracking,
};
use shrewnit::{Degrees, FeetPerSecond, Inches, Radians, RadiansPerSecond};

/// A struct that, given a reference to updated coprocessor data,
/// implements standard methods for recieving odometry information.
pub struct CoproTracking(pub Rc<RefCell<CoprocessorData>>);

impl Deref for CoproTracking {
    type Target = RefCell<CoprocessorData>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Tracking for CoproTracking {}

impl TracksForwardTravel for CoproTracking {
    fn forward_travel(&self) -> f64 {
        let mut data = self.borrow_mut();
        data.forward_travel.read().to::<Inches>()
    }
}

impl TracksHeading for CoproTracking {
    // OTOS heading:
    // Clockwise is negative, until 180
    // Counterclockwise is positive, until 180
    fn heading(&self) -> vexide::math::Angle {
        let mut data = self.borrow_mut();
        let mut heading = data.position.read().heading;

        // Correct 0 as Y to 0 as X
        heading += 90.0 * Degrees;

        Angle::from_radians(heading.to::<Radians>()).wrapped_full()
    }
}

impl TracksPosition for CoproTracking {
    fn position(&self) -> evian::math::Vec2<f64> {
        let mut data = self.borrow_mut();
        let pos = data.position.read();

        let x = pos.x.to::<Inches>();
        let y = pos.y.to::<Inches>();

        Vec2::new(x, y)
    }
}

impl TracksVelocity for CoproTracking {
    fn linear_velocity(&self) -> f64 {
        let mut data = self.borrow_mut();
        let velocity = data.velocity.read();

        Vec2::new(
            velocity.x.to::<FeetPerSecond>() * 12.0,
            velocity.y.to::<FeetPerSecond>() * 12.0,
        )
        .length()
    }

    fn angular_velocity(&self) -> f64 {
        let mut data = self.borrow_mut();

        data.velocity.read().heading.to::<RadiansPerSecond>()
    }
}
