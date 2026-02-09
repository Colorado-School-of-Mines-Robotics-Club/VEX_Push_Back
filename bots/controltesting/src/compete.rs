use std::time::{Duration, Instant};

use coprocessor::requests::CalibrateRequest;
use evian::{
    control::loops::{AngularPid, Pid},
    math::{Angle, Vec2},
    motion::{Basic, Seeking},
    prelude::*,
};
use push_back::subsystems::{
    ControllableSubsystem, ControllerConfiguration, intake::IntakeState, trunk::TrunkState,
};
use vexide::prelude::*;

use crate::robot::Robot;

impl Compete for Robot {
    async fn driver(&mut self) {
        let configuration = ControllerConfiguration::Noah;
        println!("Driver!");

        let mut i: usize = 0;
        loop {
            if let Ok(controller) = self.controller.state() {
                // Run subsystems
                self.drivetrain.control(&controller, configuration);
                self.coprocessor.control(&controller, configuration);
            } else if i.is_multiple_of(100) {
                // Only print every second
                println!("Warning: controller not connected");
            }

            i = i.wrapping_add(1);
            sleep(Duration::from_millis(10)).await;
        }
    }

    async fn autonomous(&mut self) {
        println!("Autonomous!");

        push_back::autons::wait_calibration(&mut self.coprocessor).await;

        let mut basic = Basic {
            linear_controller: crate::control::basic::LINEAR_PID,
            angular_controller: crate::control::basic::ANGULAR_PID,
            linear_tolerances: crate::control::basic::LINEAR_TOLERANCES,
            angular_tolerances: crate::control::basic::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };
        let mut seeking = Seeking {
            linear_controller: crate::control::basic::LINEAR_PID,
            lateral_controller: crate::control::seeking::LATERAL_PID,
            tolerances: crate::control::basic::LINEAR_TOLERANCES,
            timeout: Some(Duration::from_secs(5)),
        };

        seeking.move_to_point(&mut self.drivetrain, Vec2::new(12.0, 24.0)).await;

        dbg!(self.drivetrain.tracking.position());

        basic.turn_to_heading(&mut self.drivetrain, Angle::from_degrees(0.0)).await;

        println!("Auton done");
    }

    async fn disabled(&mut self) {
        println!("Disabled!")
    }
}
