#![feature(never_type)]
pub mod plotting;
pub mod ui;

use plotters::style::FontStyle;
use vexide::{prelude::Display, time::sleep};

use crate::plotting::draw_plot;

const FONT: &[u8] = include_bytes!("../ui/static/NotoSans-Regular-Small.ttf");

slint::include_modules!();
