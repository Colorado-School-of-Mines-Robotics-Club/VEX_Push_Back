use core::{pin::Pin, sync::atomic::{AtomicUsize, Ordering}};

use alloc::{boxed::Box, string::String, vec::Vec};
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

pub struct AutonRoute<R> {
    pub text: String,
    pub color: Rgb888,
    pub callback: RouteFn<R>,
}

impl<R> AutonRoute<R> {
    pub fn new(text: String, color: Rgb888, callback: RouteFn<R>) -> Self {
        Self {
            text,
            color,
            callback
        }
    }
}

pub struct AutonSelector<const ROWS: u32, const COLS: u32, R>
{
    pub(crate) routes: Vec<AutonRoute<R>>,
    pub(crate) selected: AtomicUsize,
    character_style: U8g2TextStyle<Rgb888>,
}

impl<const ROWS: u32, const COLS: u32, R> AutonSelector<{ ROWS }, { COLS }, R>
{
    pub fn new(autons: impl IntoIterator<Item = AutonRoute<R>>) -> Self {
        Self::new_with_font(autons, u8g2_font_Pixellari_tr)
    }

    pub fn new_with_font<F: Font>(autons: impl IntoIterator<Item = AutonRoute<R>>, font: F) -> Self {
        Self {
            routes: autons.into_iter().collect(),
            character_style: U8g2TextStyle::new(font, Rgb888::default()),
            selected: Default::default(),
        }
    }

    pub async fn start_auton(&self, robot: &mut R) {
        if let Some(route) = self.routes.get(self.selected.load(Ordering::Relaxed)) {
            (route.callback)(robot).await
        }
    }

    pub fn select_next(&self) {
        self.selected.fetch_add(1, Ordering::Relaxed);
        _ = self.selected.compare_exchange(
            self.routes.len(),
            self.routes.len() - 1,
            Ordering::Relaxed,
            Ordering::Relaxed
        ); // Handle overflow
    }

    pub fn select_prev(&self) {
        self.selected.fetch_sub(1, Ordering::Relaxed);
        _ = self.selected.compare_exchange(
            usize::MAX,
            0,
            Ordering::Relaxed,
            Ordering::Relaxed
        ); // Handle overflow
    }
}

impl<const ROWS: u32, const COLS: u32, T, R> RobotDisplayScreen<T>
    for AutonSelector<{ ROWS }, { COLS }, R>
where
    T: DrawTarget<Color = Rgb888>,
{
    fn draw(
        &self,
        target: &mut T,
        touch_status: crate::RobotDisplayTouchStatus,
    ) -> Result<(), T::Error> {
        let size = target.bounding_box().size;
        let thin_stroke = PrimitiveStyle::with_stroke(Rgb888::WHITE, 1);
        let mut character_style = self.character_style.clone();
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
                self.selected.store(selected, Ordering::Relaxed);
            }
        }

        // Draw each auton
        let selected = self.selected.load(Ordering::Relaxed);
        for (i, auton) in self.routes.iter().enumerate() {
            let i = i as u32;

            let row = i / (COLS);
            let col = i % COLS;

            let top_left = Point::new(
                (col * (cell_size.width)) as i32,
                (row * (cell_size.height)) as i32,
            );
            let cell_bounds = Rectangle::with_corners(top_left, top_left + cell_size);

            // If selected, highlight background
            character_style.text_color = if (i as usize) == selected {
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
                &auton.text,
                cell_bounds.offset(-3),
                character_style.clone(),
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
