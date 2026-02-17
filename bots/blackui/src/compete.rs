use std::time::{Duration, Instant};

use coprocessor::requests::{
	CalibrateRequest, GetPositionRequest, OtosPosition, SetOffsetsRequest, SetPositionRequest,
};
use evian::{
	control::loops::{AngularPid, Pid},
	math::Angle,
	motion::Basic,
	prelude::*,
};
use subsystems::{ControllableSubsystem, intake::IntakeState, trunk::TrunkState};
use shrewnit::{Degrees, Inches, Radians};
use slintui::*;
use vexide::prelude::*;

use crate::robot::Robot;

impl Compete for Robot {
	async fn driver(&mut self) {
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
				self.trunk.control(&controller, self.configuration);

				if controller.button_a.is_now_pressed() {
					// vexide::task::spawn(self.coprocessor.send_request(SetPositionRequest(OtosPosition {
					//     x: 48.0 * Inches,
					//     y: -12.0 * Inches,
					//     heading: 90.0 * Degrees
					// }))).detach();
					_ = dbg!(
						self.coprocessor
							.send_request(SetOffsetsRequest(OtosPosition {
								x: 0.0 * Inches,
								y: 0.0 * Inches,
								heading: 90.0 * Degrees,
							}))
							.await
					);
				}

				self.replay.record(
					&controller,
					&[
						("drivetrain", &self.drivetrain),
						("intake", &self.intake),
						("trunk", &self.trunk),
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

	async fn autonomous(&mut self) {
		loop {
			let position = self.coprocessor.send_request(GetPositionRequest).await;
			if let Ok(position) = position {
				self.ui
					.app()
					.global::<OdometryPageState>()
					.set_position(Pose {
						x: position.x.to::<Inches>() as f32,
						y: position.y.to::<Inches>() as f32,
						h: position.heading.to::<Degrees>() as f32,
					});
			}
			sleep(Duration::from_millis(500)).await
		}
	}

	async fn disabled(&mut self) {
		println!("Disabled!")
	}
}
