use core::time::Duration;

use alloc::vec;
use coprocessor::vexide::CoprocessorSmartPort;
use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::Differential,
    prelude::{Drivetrain, Tolerances},
};
// use autons::prelude::SelectCompeteExt as _;
use vexide::prelude::{CompeteExt as _, Controller, Direction, Gearset, Motor, Peripherals};
use vexide_motorgroup::MotorGroup;

// use crate::display::{RobotDisplay, RobotDisplaySelector};

pub struct Robot {
    // display: Arc<RobotDisplay>
    pub coprocessor: CoprocessorSmartPort,
    pub controller: Controller,
    pub drivetrain: Drivetrain<Differential, ()>,
    pub intake: MotorGroup,
}

impl Robot {
    pub const WHEEL_DIAMETER: f64 = 3.25;

    // TODO: tune
    pub const LINEAR_PID: Pid = Pid::new(0.0, 0.0, 0.0, None);
    pub const ANGULAR_PID: AngularPid = AngularPid::new(0.0, 0.0, 0.0, None);

    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(0.5)
        .velocity(0.1)
        .duration(Duration::from_millis(15));
    pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.05)
        .duration(Duration::from_millis(15));

    pub async fn new(peripherals: Peripherals) -> Self {
        Self {
            // display: Arc::new(RobotDisplay::new(peripherals.display))
            controller: peripherals.primary_controller,
            drivetrain: Drivetrain::new(
                Differential::new(
                    [
                        Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
                        Motor::new(peripherals.port_10, Gearset::Green, Direction::Forward),
                    ],
                    [
                        Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse),
                        Motor::new(peripherals.port_9, Gearset::Green, Direction::Reverse),
                    ],
                ),
                (),
            ),
            intake: MotorGroup::new(vec![
                Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
            ]),
            coprocessor: CoprocessorSmartPort::new(peripherals.port_20).await,
        }
    }

    pub async fn start(self) -> ! {
        self.compete().await
        // self.compete(RobotDisplaySelector::new(self.display.clone())).await
    }
}
