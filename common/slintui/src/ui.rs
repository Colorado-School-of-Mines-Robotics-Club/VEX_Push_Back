use std::{cell::RefCell, rc::Rc};

use plotters::style::FontStyle;
use shrewnit::{Degrees, DegreesPerSecond, FeetPerSecond, Inches};
use slint::ComponentHandle as _;
use subsystems::copro::CoproData;
use vexide::{prelude::*, task::Task};

use crate::{App, FONT, OdometryPageState, Pose};

pub struct RobotUi {
	app: App,
	_task: Rc<Task<!>>,
}

impl Clone for RobotUi {
	fn clone(&self) -> Self {
		Self {
			app: self.app.clone_strong(),
			_task: self._task.clone(),
		}
	}
}

impl RobotUi {
	pub fn new(display: Display, otos_data: Rc<RefCell<CoproData>>) -> Self {
		// Initialize slint using the V5 display
		vexide_slint::initialize_slint_platform(display);

		// Initialize the slint application, render images
		let app = App::new().expect("Failed to create application");

		_ = plotters::style::register_font("sans-serif", FontStyle::Normal, FONT);

		Self {
			_task: Rc::new(vexide::task::spawn(Self::background_task(
				app.clone_strong(),
				otos_data,
			))),
			app,
		}
	}

	pub fn app(&self) -> &App {
		&self.app
	}

	pub fn run_blocking(&self) {
		self.app.run().expect("Failed to run application");
	}

	async fn background_task(app: App, otos_data: Rc<RefCell<CoproData>>) -> ! {
		loop {
			{
				let data = otos_data.borrow();
				let odom_page_state = app.global::<OdometryPageState>();

				odom_page_state.set_position(Pose {
					x: data.position.x.to::<Inches>() as f32,
					y: data.position.y.to::<Inches>() as f32,
					h: data.position.heading.to::<Degrees>() as f32,
				});
				odom_page_state.set_velocity(Pose {
					x: data.velocity.x.to::<FeetPerSecond>() as f32 * 12.0,
					y: data.velocity.y.to::<FeetPerSecond>() as f32 * 12.0,
					h: data.velocity.heading.to::<DegreesPerSecond>() as f32,
				});
				// TODO: acceleration? or is it just not worth spamming the pi for it
			}

			sleep(Display::REFRESH_INTERVAL).await
		}
	}
}
