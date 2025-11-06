use core::{cell::RefCell, time::Duration};

use coprocessor::vexide::CoprocessorSmartPort;
use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::Differential,
    prelude::{Drivetrain, Tolerances},
};
use vexide::prelude::{
    CompeteExt as _, Controller, Direction, Display, Gearset, Motor, Peripherals,
};
use vexide_motorgroup::MotorGroup;

use crate::tracking::CoproTracking;

pub struct Robot {
    pub display: RefCell<Display>,
    pub coprocessor: CoprocessorSmartPort,
    pub controller: Controller,
    pub drivetrain: Drivetrain<Differential, CoproTracking>,
    // pub intake: MotorGroup,
}

impl Robot {
    pub async fn new(peripherals: Peripherals) -> Self {
        let coprocessor = CoprocessorSmartPort::new(peripherals.port_11).await;
        Self {
            display: RefCell::new(peripherals.display),
            controller: peripherals.primary_controller,
            drivetrain: Drivetrain::new(
                Differential::new(
                    [
                        Motor::new(peripherals.port_17, Gearset::Green, Direction::Forward),
                        Motor::new(peripherals.port_18, Gearset::Green, Direction::Reverse),
                        Motor::new(peripherals.port_19, Gearset::Green, Direction::Forward),
                        Motor::new(peripherals.port_20, Gearset::Green, Direction::Reverse),
                    ],
                    [
                        Motor::new(peripherals.port_7, Gearset::Green, Direction::Reverse),
                        Motor::new(peripherals.port_8, Gearset::Green, Direction::Forward),
                        Motor::new(peripherals.port_9, Gearset::Green, Direction::Reverse),
                        Motor::new(peripherals.port_10, Gearset::Green, Direction::Forward),
                    ],
                ),
                CoproTracking(coprocessor.latest_data()),
            ),
            // intake: MotorGroup::new(vec![]),
            coprocessor,
        }
    }

    pub async fn start(self) -> ! {
        self.compete().await
    }
}
