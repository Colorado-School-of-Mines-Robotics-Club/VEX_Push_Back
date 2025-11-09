use vexide::devices::display::{Display, RenderMode};
use vexide_embedded_graphics::DisplayDriver;

use crate::{AutonSelector, RobotDisplay};

impl RobotDisplay<DisplayDriver> {
    pub fn from_display(mut display: Display) -> Self {
        display.set_render_mode(RenderMode::DoubleBuffered);
        Self::new(DisplayDriver::new(display))
    }
}

impl<const ROWS: u32, const COLS: u32, R> autons::Selector<R>
    for AutonSelector<{ ROWS }, { COLS }, R>
{
    async fn run(&self, robot: &mut R) {
        self.start_auton(robot).await
    }
}
