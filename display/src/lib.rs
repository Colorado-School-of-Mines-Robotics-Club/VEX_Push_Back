#![cfg_attr(feature = "vexide", no_std)]
#![cfg_attr(feature = "vexide", feature(never_type))]

extern crate alloc;

mod autons;

#[cfg(feature = "vexide")]
mod vexide;

pub use autons::*;
pub use embedded_graphics::pixelcolor::{Rgb888, RgbColor};
use embedded_graphics::prelude::{DrawTarget, Point};
pub use u8g2_fonts::fonts::*;

pub trait RobotDisplayScreen<T: DrawTarget<Color = Rgb888>> {
    /// Draws this screen using the provided target and touch status
    fn draw(&self, target: &mut T, touch_status: RobotDisplayTouchStatus) -> Result<(), T::Error>;
}

#[derive(Default, Clone, Copy)]
pub struct RobotDisplayTouchStatus {
    pub point: Point,
    pub state: RobotDisplayTouchState,
}

#[derive(Default, Clone, Copy)]
pub enum RobotDisplayTouchState {
    #[default]
    Default,
    Pressed,
    Released,
    Held,
}

#[derive(Clone)]
pub struct RobotDisplay<T: DrawTarget<Color = Rgb888>> {
    target: T,
    touch_status: RobotDisplayTouchStatus,
}

impl<T: DrawTarget<Color = Rgb888>> RobotDisplay<T> {
    pub fn new(target: T) -> Self {
        Self {
            target,
            touch_status: Default::default(),
        }
    }

    pub fn draw(&mut self, screen: &mut impl RobotDisplayScreen<T>) -> Result<(), T::Error> {
        screen.draw(&mut self.target, self.touch_status)
    }

    pub fn target(&self) -> &T {
        &self.target
    }

    pub fn touch_status(&self) -> &RobotDisplayTouchStatus {
        &self.touch_status
    }

    pub fn set_touch_status(&mut self, status: RobotDisplayTouchStatus) {
        self.touch_status = status;
    }
}
