use plotters::{backend::RGBPixel, prelude::*};
use slint::{Image, Rgb8Pixel, Rgba8Pixel, SharedPixelBuffer};

pub fn draw_plot(width: u32, height: u32) -> anyhow::Result<Image> {
	let graph_size = (width, height);
	let mut buf = SharedPixelBuffer::<Rgb8Pixel>::new(graph_size.0, graph_size.1);

	{
		let root =
			BitMapBackend::<RGBPixel>::with_buffer_and_format(buf.make_mut_bytes(), graph_size)?
				.into_drawing_area();

		root.fill(&WHITE)?;

		let mut chart = ChartBuilder::on(&root)
			.caption("y=x^2", ("sans-serif", 25).into_font())
			.margin(5)
			.x_label_area_size(30)
			.y_label_area_size(30)
			.build_cartesian_2d(-1f32..1f32, -0.1f32..1f32)?;

		chart.configure_mesh().draw()?;

		chart.draw_series(LineSeries::new(
			(-50..=50).map(|x| x as f32 / 50.0).map(|x| (x, x * x)),
			&RED,
		))?;

		root.present()?;
	}

	Ok(Image::from_rgb8(buf))
}
