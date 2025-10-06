use core::time::Duration;

use vexide::{competition::CompetitionMode, devices::display::Display, task, time::sleep};
use vexide_embedded_graphics::DisplayDriver;

use crate::{AutonRoute, AutonSelector, RobotDisplay};

impl RobotDisplay<DisplayDriver> {
    pub fn from_display(display: Display) -> Self {
        Self::new(DisplayDriver::new(display))
    }
}

impl<'a, const ROWS: u32, const COLS: u32, A, R> AutonSelector<'a, { ROWS }, { COLS }, A, R>
where
    A: Iterator<Item = AutonRoute<'a, R>> + ExactSizeIterator + Clone,
{
    pub fn setup(routes: A) {
        task::spawn(async {
            loop {
                if matches!(vexide::competition::mode(), CompetitionMode::Disabled | CompetitionMode::Autonomous) {

                }
                sleep(Display::REFRESH_INTERVAL).await;
            }
        }).detach();

    }
}

impl<'a, const ROWS: u32, const COLS: u32, A, R> autons::Selector<R>
    for AutonSelector<'a, { ROWS }, { COLS }, A, R>
where
    A: Iterator<Item = AutonRoute<'a, R>> + ExactSizeIterator + Clone,
{
    async fn run(&self, robot: &mut R) {
        if let Some(route) = self.routes.clone().nth(self.selected) {
            (route.callback)(robot).await;
        }
    }
}
