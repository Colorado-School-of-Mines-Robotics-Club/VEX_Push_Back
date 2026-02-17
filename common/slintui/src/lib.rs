#![feature(never_type)]
pub mod autons;
pub mod plotting;
pub mod ui;

pub use slint;

const FONT: &[u8] = include_bytes!("../ui/static/NotoSans-Regular-Small.ttf");

slint::include_modules!();
