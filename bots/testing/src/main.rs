#![feature(never_type, future_join)]

use std::time::Duration;

use vexide::prelude::*;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut i = 0;
    let mut a = 0;
    let mut controller = peripherals.primary_controller;
    let text = [
        "Hello there!",
        "Number 2",
        "hehehehehehhehe",
        "lol"
    ];
    loop {
        if i % 10 == 0 {
            _ = controller.screen.set_text(text[a], 1, 2).await;
        }

        if controller.state().unwrap().button_right.is_now_pressed() {
            _ = controller.screen.clear_line(1).await;
            a = (a + 1) % text.len();
        }

        i += 1;
        sleep(Duration::from_millis(10)).await
    }
}
