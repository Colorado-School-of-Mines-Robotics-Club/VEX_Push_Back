use std::time::{Duration, Instant};

use ::autons::prelude::SelectCompete;
use anyhow::Context;
use subsystems::ControllableSubsystem;
use vexide::prelude::*;

use crate::robot::Robot;

const AUTON_IN_DRIVER: bool = option_env!("DO_NOT_USE_AT_COMP_AUTON_TEST").is_some();

impl SelectCompete for Robot {
	async fn driver(&mut self) {
		if AUTON_IN_DRIVER && let Some(auton) = &self.default_auton {
			let attempted_auton: anyhow::Result<Duration> = try {
				self.coprocessor
					.calibrate()
					.await
					.context("OTOS calibration failed")?;
				self.imu
					.lock()
					.await
					.calibrate()
					.await
					.context("IMU calibration failed")?;

				sleep(Duration::from_millis(500)).await;

				let start = Instant::now();
				(auton.callback)(self).await;
				start.elapsed()
			};

			match attempted_auton {
				Ok(duration) => println!("Auton finished in {:.02}s", duration.as_secs_f64()),
				Err(e) => println!("Auton failed: {:?}", e),
			}
			std::process::exit(0);
		}

		sleep(Duration::from_millis(100)).await;
		_ = self.pneumatics.initialize().await;

		println!("Driver!");
		_ = self
			.controller
			.set_text(self.configuration.as_str(), 1, 1)
			.await;

		let mut i: usize = 0;
		loop {
			if let Ok(controller) = self.controller.state() {
				if controller.button_up.is_now_pressed() {
					self.configuration = self.configuration.next();
					_ = self
						.controller
						.set_text(self.configuration.as_str(), 1, 1)
						.await
				}

				if controller.button_y.is_now_pressed() {
					let imu = self.imu.clone();
					vexide::task::spawn(async move {
						_ = imu.lock().await.calibrate().await;
						println!("IMU calibrated");
					})
					.detach();
				}

				// Run subsystems
				self.drivetrain.control(&controller, self.configuration);
				self.coprocessor.control(&controller, self.configuration);
				self.intake.control(&controller, self.configuration);
				self.pneumatics.control(&controller, self.configuration);

				self.replay.record(
					&controller,
					&[
						("drivetrain", &self.drivetrain),
						("intake", &self.intake),
						("pneumatics", &self.pneumatics),
					],
				);
			} else if i.is_multiple_of(100) {
				// Only print every second
				println!("Warning: controller not connected");
			}

			i = i.wrapping_add(1);
			sleep(Duration::from_millis(10)).await;
		}
	}

	async fn disabled(&mut self) {
		println!("Disabled!")
	}
}
