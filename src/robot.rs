use core::{cell::{RefCell, RefMut}, time::Duration};

use alloc::{sync::Arc, vec};
use evian::{drivetrain::model::Differential, prelude::Drivetrain};
// use autons::prelude::SelectCompeteExt as _;
use vexide::{prelude::{CompeteExt as _, Controller, Direction, Gearset, Motor, Peripherals, SerialPort}, task::{self, spawn}, time::sleep};
use vexide_motorgroup::MotorGroup;

// use crate::display::{RobotDisplay, RobotDisplaySelector};

pub struct Robot {
    // display: Arc<RobotDisplay>
    pub pi: Option<SerialPort>,
    pub controller: Controller,
    pub drivetrain: Drivetrain<Differential, ()>,
    pub intake: MotorGroup,
}

impl Robot {
    pub async fn new(peripherals: Peripherals) -> Self {
        Self {
            pi: Some(SerialPort::open(
                peripherals.port_20,921600
            ).await),
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
        }
    }

    pub async fn start(mut self) -> ! {
        let mut pi = self.pi.take().unwrap();
        task::spawn(async move {
            loop {
            let _ = pi.write_byte(4);
            sleep(Duration::from_millis(10)).await;
            }
        }).detach();

        self.compete().await
        // self.compete(RobotDisplaySelector::new(self.display.clone())).await
    }
}
