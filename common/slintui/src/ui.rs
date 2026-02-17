use plotters::style::FontStyle;
use slint::ComponentHandle as _;
use vexide::prelude::*;

use crate::{App, FONT, plotting::draw_plot};

pub struct RobotUi {
	app: App,
}

impl Clone for RobotUi {
	fn clone(&self) -> Self {
		Self {
			app: self.app.clone_strong(),
		}
	}
}

impl RobotUi {
	pub fn new(display: Display) -> Self {
		// Initialize slint using the V5 display
		vexide_slint::initialize_slint_platform(display);

		// Initialize the slint application, render images
		let app = App::new().expect("Failed to create application");

		_ = plotters::style::register_font("sans-serif", FontStyle::Normal, FONT);
		app.set_graph_source(
			draw_plot(app.get_graph_width() as u32, app.get_graph_height() as u32).unwrap(),
		);

		Self { app }
	}

	pub fn app(&self) -> &App {
		&self.app
	}

	pub fn run_blocking(&self) {
		self.app.run().expect("Failed to run application");
	}
}
