use std::time::Duration;

use ::autons::prelude::SelectCompete;
use subsystems::ControllableSubsystem;
use vexide::prelude::*;

use crate::robot::Robot;

impl SelectCompete for Robot {
	async fn driver(&mut self) {
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
