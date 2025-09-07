use core::{
    cell::{RefCell, RefMut},
    time::Duration,
};

use alloc::sync::Arc;
use autons::Selector;
use vexide::{
    competition::{self, CompetitionMode},
    devices::display::{Line, Rect},
    prelude::Display,
    task,
};

pub struct RobotDisplay<const W: u8 = 2, const H: u8 = 4> {}

impl<const W: u8, const H: u8> RobotDisplay<W, H> {
    pub fn new(display: Display) -> Self {
        task::spawn(Self::display_task(display)).detach();

        Self {}
    }

    async fn display_task(mut display: Display) {
        loop {
            match competition::mode() {
                CompetitionMode::Disabled | CompetitionMode::Autonomous => {
                    Self::auton_selector(&mut display).await
                }
                CompetitionMode::Driver => (),
            }

            vexide::time::sleep(Duration::from_millis(50)).await
        }
    }

    async fn auton_selector(display: &mut Display) {
        let width: i16 = W.into();
        let height: i16 = H.into();

        display.fill(
            &Rect::new(
                [0, 0],
                [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
            ),
            (0, 0, 0),
        );

        // Draw outlines
        for i in 0..width {
            let pos = Display::HORIZONTAL_RESOLUTION / width * (i + 1);

            display.fill(
                &Line::new([0, pos], [Display::VERTICAL_RESOLUTION, pos]),
                (255, 255, 255),
            );
        }
    }
}

pub struct RobotDisplaySelector {
    routes: (),
    display: Arc<RobotDisplay>,
}

impl RobotDisplaySelector {
    pub fn new(display: Arc<RobotDisplay>) -> Self {
        Self {
            display,
            routes: (),
        }
    }
}

impl<R> Selector<R> for RobotDisplaySelector {
    async fn run(&self, robot: &mut R) {
        todo!()
    }
}
