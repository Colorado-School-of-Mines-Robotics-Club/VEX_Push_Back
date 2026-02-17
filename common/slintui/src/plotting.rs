use std::{
	rc::Rc,
	time::{Duration, Instant},
};

use evian::control::loops::Feedback;
use plotters::{backend::RGBPixel, prelude::*};
use slint::{ComponentHandle, Image, Rgb8Pixel, SharedPixelBuffer};
use vexide::prelude::Display;

use crate::App;

pub struct TrackedPid<T: Feedback> {
	inner: T,
	measurements: Vec<PidState<T::State>>,
	app: Rc<App>,
	last_draw: Instant,
}

impl<T: Feedback + Clone> Clone for TrackedPid<T> {
	fn clone(&self) -> Self {
		Self {
			inner: self.inner.clone(),
			app: self.app.clone(),
			measurements: Vec::new(),
			last_draw: self.last_draw,
		}
	}
}

#[derive(Clone, Debug)]
pub(crate) struct PidState<T> {
	time: f64,
	measurement: T,
	setpoint: T,
}

impl<T: Feedback> TrackedPid<T>
where
	T::State: Clone + Into<f64>,
{
	pub fn new(inner: T, app: &App) -> Self {
		let this = Self {
			inner,
			measurements: Vec::with_capacity(2_097_152 / std::mem::size_of::<PidState<T::State>>()),
			app: Rc::new(app.clone_strong()),
			last_draw: Instant::now(),
		};

		if let Ok(plot) = draw_plot(
			app.get_graph_width() as u32,
			app.get_graph_height() as u32,
			&this.measurements,
		) {
			app.set_graph_source(plot);
		}

		this
	}
}

impl<T: Feedback> Feedback for TrackedPid<T>
where
	T::State: Clone + Into<f64>,
{
	type State = <T as Feedback>::State;

	type Signal = <T as Feedback>::Signal;

	fn update(
		&mut self,
		measurement: Self::State,
		setpoint: Self::State,
		dt: std::time::Duration,
	) -> Self::Signal {
		let last_time = self.measurements.last().map(|m| m.time).unwrap_or_default();
		self.measurements.push(PidState {
			time: last_time + dt.as_secs_f64(),
			measurement: measurement.clone(),
			setpoint: setpoint.clone(),
		});

		if self.last_draw.elapsed() >= Duration::from_millis(100)
			&& let Ok(plot) = draw_plot(
				self.app.get_graph_width() as u32,
				self.app.get_graph_height() as u32,
				&self.measurements,
			) {
			self.last_draw = Instant::now();
			self.app.set_graph_source(plot);
		}

		self.inner.update(measurement, setpoint, dt)
	}
}

pub(crate) fn draw_plot<T: Clone + Into<f64>>(
	width: u32,
	height: u32,
	measurements: &[PidState<T>],
) -> anyhow::Result<Image> {
	let graph_size = (width, height);
	let mut buf = SharedPixelBuffer::<Rgb8Pixel>::new(graph_size.0, graph_size.1);

	{
		let root =
			BitMapBackend::<RGBPixel>::with_buffer_and_format(buf.make_mut_bytes(), graph_size)?
				.into_drawing_area();

		root.fill(&WHITE)?;

		let mut chart = ChartBuilder::on(&root)
			.caption("PID", ("sans-serif", 25).into_font())
			.margin(5)
			.x_label_area_size(30)
			.y_label_area_size(30)
			.build_cartesian_2d(
				0.0..measurements.last().map(|m| m.time).unwrap_or_default(),
				(measurements
					.iter()
					.flat_map(|m| [m.measurement.clone().into(), m.setpoint.clone().into()])
					.min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
					.unwrap_or_default())
					..(measurements
						.iter()
						.flat_map(|m| [m.measurement.clone().into(), m.setpoint.clone().into()])
						.max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
						.unwrap_or_default()
						+ 2.0),
			)?;

		chart.configure_mesh().draw()?;

		chart.draw_series(LineSeries::new(
			measurements
				.iter()
				.map(|s| (s.time, s.measurement.clone().into())),
			&RED,
		))?;
		chart.draw_series(LineSeries::new(
			measurements
				.iter()
				.map(|s| (s.time, s.setpoint.clone().into())),
			&BLUE,
		))?;

		root.present()?;
	}

	Ok(Image::from_rgb8(buf))
}
