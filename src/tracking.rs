use core::ops::Deref;

use evian::{prelude::{TracksForwardTravel, TracksPosition}, tracking::Tracking};
use vexide::prelude::{Motor, Position};
use vexide_motorgroup::{MotorGroupError, SharedMotors};

use crate::robot::Robot;

impl Tracking for Robot {}
