use evian::tracking::Tracking;

use crate::robot::Robot;

impl Tracking for Robot {}

// TODO: All the tracking impls are non-async, so we need to make a background task keep updated data in mem
