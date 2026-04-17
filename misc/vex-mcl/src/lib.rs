#![feature(portable_simd, iter_array_chunks)]
use std::simd::{Simd, num::SimdFloat};

use rand_distr::{Distribution, Normal, Uniform};
use shrewnit::{Angle, Length};

// TODO: attempt to make this f16 because woooooo more math per math
type ParticleFloat = f32;
type ParticleSimd = Simd<ParticleFloat, LANES>;
const NEON_BITS: usize = 128;
const LANES: usize = NEON_BITS / (size_of::<ParticleFloat>() * 8);

#[derive(Clone, Default, Debug)]
pub struct MonteCarloLocalizaition<const P: usize> {
	particles: Particles<P>,
}

#[derive(Clone, Copy, Debug)]
pub struct Particles<const P: usize> {
	x: [ParticleSimd; P],
	y: [ParticleSimd; P],
	h: [ParticleSimd; P],
	weight: [ParticleSimd; P],
}

impl<const P: usize> Default for Particles<P> {
	fn default() -> Self {
		Self {
			x: [ParticleSimd::splat(0.0); P],
			y: [ParticleSimd::splat(0.0); P],
			h: [ParticleSimd::splat(0.0); P],
			weight: [ParticleSimd::splat(1.0); P],
		}
	}
}

impl<const P: usize> MonteCarloLocalizaition<P> {
	pub fn new(
		initial_x: Length<ParticleFloat>,
		initial_y: Length<ParticleFloat>,
		initial_heading: Angle<ParticleFloat>,
		pos_std_dev: Length<ParticleFloat>,
		angle_std_dev: Angle<ParticleFloat>,
	) -> Self {
		let mut rng = rand::rng();
		let mut data = Self::default();

		let x_distr = Normal::new(initial_x.canonical(), pos_std_dev.canonical())
			.expect("pos_std_dev not finite");
		let y_distr = Normal::new(initial_y.canonical(), pos_std_dev.canonical())
			.expect("pos_std_dev not finite");
		let h_distr = Normal::new(initial_heading.canonical(), angle_std_dev.canonical())
			.expect("angle_std_dev not finite");

		// Fill x
		data.particles
			.x
			.iter_mut()
			.for_each(|xs| xs.as_mut_array().fill_with(|| x_distr.sample(&mut rng)));
		// Fill y
		data.particles
			.y
			.iter_mut()
			.for_each(|ys| ys.as_mut_array().fill_with(|| y_distr.sample(&mut rng)));
		// Fill heading
		data.particles
			.h
			.iter_mut()
			.for_each(|hs| hs.as_mut_array().fill_with(|| h_distr.sample(&mut rng)));
		// Default impl for Particles initializes weights to 1

		data
	}

	/// Resample using stochastic universal sampling
	pub fn resample(&mut self) {
		let sum = self
			.particles
			.weight
			.into_iter()
			.sum::<ParticleSimd>()
			.reduce_sum();
		let distance = sum / P as ParticleFloat;
		let offset = Uniform::new(0.0, distance)
			.unwrap()
			.sample(&mut rand::rng());

		let mut weight_sum = 0.0;
		let mut line_iter = (0..P).map(|n| (n as ParticleFloat) + offset);
		let old_particles = self.particles;
		let mut i = 0;
		for (((x, y), h), weight) in old_particles
			.x
			.into_iter()
			.flat_map(|s| *s.as_array())
			.zip(old_particles.y.into_iter().flat_map(|s| *s.as_array()))
			.zip(old_particles.h.into_iter().flat_map(|s| *s.as_array()))
			.zip(old_particles.weight.into_iter().flat_map(|s| *s.as_array()))
		{
			weight_sum += weight;
			while let Some(line) = line_iter.next()
				&& line < weight_sum
			{
				self.particles.x[i / LANES][i % LANES] = x;
				self.particles.y[i / LANES][i % LANES] = y;
				self.particles.h[i / LANES][i % LANES] = h;
				self.particles.weight[i / LANES][i % LANES] = weight;
				i += 1;
			}
		}
	}
}
