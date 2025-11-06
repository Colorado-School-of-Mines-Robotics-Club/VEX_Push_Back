use std::{
    cell::RefCell,
    {f64::consts::PI, ops::Deref},
};

use coprocessor::vexide::CoprocessorData;
use evian::{
    math::{Angle, Vec2},
    prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity},
    tracking::Tracking,
};
use shrewnit::{Degrees, FeetPerSecond, Inches, Radians, RadiansPerSecond};

/// A struct that, given a reference to updated coprocessor data,
/// implements standard methods for recieving odometry information.
pub struct CoproTracking(pub RefCell<CoprocessorData>);

impl CoproTracking {
    pub fn new(data: CoprocessorData) -> Self {
        Self(RefCell::new(data))
    }
}

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
    fn heading(&self) -> evian::prelude::Angle {
        let mut data = self.borrow_mut();
        let heading = data.position.read().heading;

        // +- pi
        //     0 = +y
        // -pi/2 = +x
        //  pi/2 = -x
        let radians = (heading - 45.0 * Degrees).to::<Radians>();

        Angle::from_radians(if radians.is_sign_positive() {
            radians
        } else {
            radians + 2.0 * PI
        })
    }
}

impl TracksPosition for CoproTracking {
    fn position(&self) -> evian::prelude::Vec2<f64> {
        let mut data = self.borrow_mut();
        let pos = data.position.read();
        Vec2::new(pos.x.to::<Inches>(), pos.y.to::<Inches>())
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
