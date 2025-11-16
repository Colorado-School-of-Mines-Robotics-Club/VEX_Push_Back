use vexide::devices::display::{Display, RenderMode};
use vexide_embedded_graphics::DisplayDriver;

use crate::{AutonSelector, RobotDisplay};

impl RobotDisplay<DisplayDriver> {
    pub fn from_display(mut display: Display) -> Self {
        display.set_render_mode(RenderMode::DoubleBuffered);
        Self::new(DisplayDriver::new(display))
    }
}
