use core::time::Duration;

use alloc::{boxed::Box, string::String, vec};
use coprocessor::vexide::CoprocessorSmartPort;
use display::{AutonRoute, AutonSelector, Rgb888, RgbColor as _, RobotDisplay};
use evian::{
    control::loops::{AngularPid, Pid},
    drivetrain::model::Differential,
    prelude::{Drivetrain, Tolerances},
};
// use autons::prelude::SelectCompeteExt as _;
use vexide::{competition::CompetitionMode, io::println, prelude::{CompeteExt as _, Controller, Direction, Display, Gearset, Motor, Peripherals}, task, time::sleep};
use vexide_motorgroup::MotorGroup;

use crate::tracking::CoproTracking;

pub struct Robot {
    // display: Arc<RobotDisplay>
    pub coprocessor: CoprocessorSmartPort,
    pub controller: Controller,
    pub drivetrain: Drivetrain<Differential, CoproTracking>,
    pub intake: MotorGroup
}

impl Robot {
    // pub const WHEEL_DIAMETER: f64 = 3.25;

    // Partially tuned for the dev bot
    // P = 0.09 causes oscillaion
    pub const LINEAR_PID: Pid = Pid::new(0.08, 0.0, 0.0, None);
    pub const ANGULAR_PID: AngularPid = AngularPid::new(0.00, 0.00, 0.00, None);

    // TODO: verify these tolerances, these are completely untested
    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(0.5)
        .velocity(0.1)
        .duration(Duration::from_millis(15));
    pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.05)
        .duration(Duration::from_millis(15));

    pub async fn new(peripherals: Peripherals) -> Self {
        Self::setup_display(peripherals.display);

        let coprocessor = CoprocessorSmartPort::new(peripherals.port_20).await;
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
                CoproTracking(coprocessor.latest_data()),
            ),
            intake: MotorGroup::new(vec![
                Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
                Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
            ]),
            coprocessor,
        }
    }

    pub async fn start(self) -> ! {
        self.compete().await
    }

    // TODO re-organize this to use SelectCompete instead and actually do something
    // unfinished!
    fn setup_display(display: Display) {
        let mut display = RobotDisplay::from_display(display);
        let mut selector = AutonSelector::<4, 2, Robot>::new([
            AutonRoute::new(
                String::from("hi"),
                Rgb888::BLUE,
                |_robot| Box::pin(async { println!("Hi!!!") })
            ),
            AutonRoute::new(
                String::from("bye"),
                Rgb888::RED,
                |_robot| Box::pin(async { println!("Bye!!!") })
            ),
        ]);

        task::spawn(async move {
            loop {
                match vexide::competition::mode() {
                    CompetitionMode::Disabled | CompetitionMode::Autonomous => _ = display.draw(&mut selector),
                    CompetitionMode::Driver => (),
                }

                sleep(Display::REFRESH_INTERVAL).await
            }
        }).detach();
    }
}
