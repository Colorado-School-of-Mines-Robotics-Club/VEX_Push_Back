use core::{f64::consts::PI, ops::Deref, sync::atomic::Ordering};

use alloc::sync::Arc;
use coprocessor::vexide::CoprocessorData;
use evian::{math::{Angle, Vec2}, prelude::{TracksForwardTravel, TracksHeading, TracksPosition, TracksVelocity}, tracking::Tracking};
use shrewnit::{Degrees, FeetPerSecond, Inches, Radians, RadiansPerSecond};
use vexide::float::Float;

/// A struct that, given a reference to updated coprocessor data,
/// implements standard methods for recieving odometry information.
#[derive(Clone)]
pub struct CoproTracking(pub Arc<CoprocessorData>);

impl Deref for CoproTracking {
    type Target = Arc<CoprocessorData>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Tracking for CoproTracking {}

impl TracksForwardTravel for CoproTracking {
    fn forward_travel(&self) -> f64 {
        let data = self.forward_travel.load(Ordering::Relaxed);
        data.to::<Inches>()
    }
}

impl TracksHeading for CoproTracking {
    fn heading(&self) -> evian::prelude::Angle {
        let data = self.position.load(Ordering::Relaxed);

        // +- pi
        //     0 = +y
        // -pi/2 = +x
        //  pi/2 = -x
        let radians = (data.heading - 45.0 * Degrees).to::<Radians>();

        Angle::from_radians(if radians.is_sign_positive() {
            radians
        } else {
            radians + 2.0 * PI
        })
    }
}

impl TracksPosition for CoproTracking {
    fn position(&self) -> evian::prelude::Vec2<f64> {
        let data = self.position.load(Ordering::Relaxed);
        Vec2::new(
            data.x.to::<Inches>(),
            data.y.to::<Inches>()
        )
    }
}

impl TracksVelocity for CoproTracking {
    fn linear_velocity(&self) -> f64 {
        let data = self.velocity.load(Ordering::Relaxed);
        let velocity = Vec2::new(
            data.x.to::<FeetPerSecond>() * 12.0,
            data.y.to::<FeetPerSecond>() * 12.0
        );
        (velocity.x.powi(2) + velocity.y.powi(2)).sqrt()
    }

    fn angular_velocity(&self) -> f64 {
        let data = self.velocity.load(Ordering::Relaxed);
        data.heading.to::<RadiansPerSecond>()
    }
}
