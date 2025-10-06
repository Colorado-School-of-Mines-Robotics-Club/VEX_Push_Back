use core::pin::Pin;

use alloc::boxed::Box;
use embedded_graphics::{
    Drawable,
    pixelcolor::Rgb888,
    prelude::{DrawTarget, Point, RgbColor, Size},
    primitives::{Line, PrimitiveStyle, Rectangle, StyledDrawable},
};
use embedded_text::TextBox;
use embedded_text::alignment::{HorizontalAlignment, VerticalAlignment};
use embedded_text::style::TextBoxStyleBuilder;
use u8g2_fonts::fonts::u8g2_font_Pixellari_tr;
use u8g2_fonts::{Font, U8g2TextStyle};

#[cfg(feature = "vexide")]
use vexide::float::Float as _;

use crate::{RobotDisplayScreen, RobotDisplayTouchState, RobotDisplayTouchStatus};

fn adjust_component(component: f64) -> f64 {
    if component <= 0.04045 {
        component / 12.92
    } else {
        ((component + 0.055) / 1.055).powf(2.4)
    }
}

fn calculate_luminance<C: RgbColor>(color: C) -> f64 {
    let r = adjust_component(color.r() as f64 / C::MAX_R as f64);
    let g = adjust_component(color.g() as f64 / C::MAX_G as f64);
    let b = adjust_component(color.b() as f64 / C::MAX_B as f64);

    0.2126 * r + 0.7152 * g + 0.0722 * b
}

pub type RouteFn<R> = for<'s> fn(&'s mut R) -> Pin<Box<dyn Future<Output = ()> + 's>>;

#[derive(Clone, Copy)]
pub struct AutonRoute<'a, R> {
    pub text: &'a str,
    pub color: Rgb888,
    pub callback: RouteFn<R>,
}

impl<'a, R> AutonRoute<'a, R> {
    pub fn new(text: &'a str, color: Rgb888, callback: RouteFn<R>) -> Self {
        Self {
            text,
            color,
            callback
        }
    }
}

pub struct AutonSelector<'a, const ROWS: u32, const COLS: u32, A, R>
where
    A: Iterator<Item = AutonRoute<'a, R>> + ExactSizeIterator + Clone,
{
    pub(crate) routes: A,
    pub(crate) selected: usize,
    character_style: U8g2TextStyle<Rgb888>,
}

impl<'a, const ROWS: u32, const COLS: u32, A, R> AutonSelector<'a, { ROWS }, { COLS }, A, R>
where
    A: Iterator<Item = AutonRoute<'a, R>> + ExactSizeIterator + Clone,
{
    pub fn new(autons: impl IntoIterator<IntoIter = A>) -> Self {
        Self::new_with_font(autons, u8g2_font_Pixellari_tr)
    }

    pub fn new_with_font<F: Font>(autons: impl IntoIterator<IntoIter = A>, font: F) -> Self {
        Self {
            routes: autons.into_iter(),
            character_style: U8g2TextStyle::new(font, Rgb888::default()),
            selected: 0,
        }
    }
}

impl<'a, const ROWS: u32, const COLS: u32, A, T, R> RobotDisplayScreen<T>
    for AutonSelector<'a, { ROWS }, { COLS }, A, R>
where
    T: DrawTarget<Color = Rgb888>,
    A: Iterator<Item = AutonRoute<'a, R>> + ExactSizeIterator + Clone,
{
    fn draw(
        &mut self,
        target: &mut T,
        touch_status: crate::RobotDisplayTouchStatus,
    ) -> Result<(), T::Error> {
        let size = target.bounding_box().size;
        let thin_stroke = PrimitiveStyle::with_stroke(Rgb888::WHITE, 1);

        let cell_size = size.component_div(Size::new(COLS, ROWS));

        target.clear(Rgb888::BLACK)?;

        // Handle touch
        // TODO: don't allow selecting autons which don't exist
        if let RobotDisplayTouchStatus {
            point: position,
            state: RobotDisplayTouchState::Pressed | RobotDisplayTouchState::Held,
        } = touch_status
        {
            let index = (
                position.x / (cell_size.width as i32),
                position.y / (cell_size.height as i32),
            );
            let selected = (index.0 + index.1 * (COLS as i32)) as usize;
            if selected < self.routes.len() {
                self.selected = selected;
            }
        }

        // Draw each auton
        for (i, auton) in self.routes.clone().enumerate() {
            let i = i as u32;

            let row = i / (COLS);
            let col = i % COLS;

            let top_left = Point::new(
                (col * (cell_size.width)) as i32,
                (row * (cell_size.height)) as i32,
            );
            let cell_bounds = Rectangle::with_corners(top_left, top_left + cell_size);

            // If selected, highlight background
            self.character_style.text_color = if (i as usize) == self.selected {
                cell_bounds.draw_styled(&PrimitiveStyle::with_fill(auton.color), target)?;

                if calculate_luminance(auton.color) > f64::sqrt(0.0525) {
                    Some(Rgb888::BLACK)
                } else {
                    Some(Rgb888::WHITE)
                }
            } else {
                Some(Rgb888::WHITE)
            };

            // Draw text
            TextBox::with_textbox_style(
                auton.text,
                cell_bounds.offset(-3),
                self.character_style.clone(),
                TextBoxStyleBuilder::new()
                    .alignment(HorizontalAlignment::Center)
                    .vertical_alignment(VerticalAlignment::Middle)
                    .build(),
            )
            .draw(target)?;
        }

        // Draw grid
        for col in 0..COLS {
            Line::with_delta(
                Point::new((size.width / COLS * (col + 1)) as i32, 0),
                Point::new(0, size.height as i32),
            )
            .draw_styled(&thin_stroke, target)?;
        }

        for row in 0..ROWS {
            Line::with_delta(
                Point::new(0, (size.height / ROWS * (row + 1)) as i32),
                Point::new(size.width as i32, 0),
            )
            .draw_styled(&thin_stroke, target)?;
        }

        Ok(())
    }
}
