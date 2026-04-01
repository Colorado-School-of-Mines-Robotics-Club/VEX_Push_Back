use rand_distr::{Distribution, Normal, Uniform};
use shrewnit::{Angle, Length};

pub struct MonteCarloLocalizaition<const P: usize> {
	particles: [Particle; P],
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Pose {
	x: Length<f64>,
	y: Length<f64>,
	h: Angle<f64>,
}

#[derive(Debug, Default, Clone, Copy)]
struct Particle {
	pose: Pose,
	weight: f64,
}

impl<const P: usize> MonteCarloLocalizaition<P> {
	pub fn new(initial: Pose, pos_std_dev: f64, angle_std_dev: f64) -> Self {
		let mut rng = rand::rng();
		let mut particles = [Particle::default(); P];

		let x_distr =
			Normal::new(initial.x.canonical(), pos_std_dev).expect("pos_std_dev not finite");
		let y_distr =
			Normal::new(initial.y.canonical(), pos_std_dev).expect("pos_std_dev not finite");
		let h_distr =
			Normal::new(initial.h.canonical(), angle_std_dev).expect("angle_std_dev not finite");

		for particle in &mut particles {
			particle.pose = Pose {
				x: Length::from_canonical(x_distr.sample(&mut rng)),
				y: Length::from_canonical(y_distr.sample(&mut rng)),
				h: Angle::from_canonical(h_distr.sample(&mut rng)),
			};
			particle.weight = 1.0; // TODO: do we even store this or do we just calculate weight on-demand in a combo sense-resample function
		}

		Self { particles }
	}

	/// Resample using stochastic universal sampling
	pub fn resample(&mut self) {
		let sum = self.particles.iter().map(|p| p.weight).sum::<f64>();
		let distance = sum / P as f64;
		let offset = Uniform::new(0.0, distance)
			.unwrap()
			.sample(&mut rand::rng());

		let mut weight_sum = 0.0;
		let mut line_iter = (0..P).map(|n| (n as f64) + offset);
		let old_particles_iter = self.particles.into_iter();
		let mut new_particles_iter = &mut self.particles[..];
		for particle in old_particles_iter {
			weight_sum += particle.weight;
			while let Some(line) = line_iter.next()
				&& line < weight_sum
			{
				new_particles_iter[0] = particle;
				new_particles_iter[0].weight /= sum; // Normalize weights
				new_particles_iter = &mut new_particles_iter[1..];
			}
		}
	}
}
