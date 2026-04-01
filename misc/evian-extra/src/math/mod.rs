use std::{ops::Mul, time::Duration};

use nalgebra::{
	Cholesky, DefaultAllocator, DimMin, DimName, DimNameAdd, DimNameSum, Matrix, OMatrix, OVector,
	RealField, Storage, U1, Vector, allocator::Allocator,
};

/// Solves the DARE for a set of matrices A, B, Q, R, and a precision bound eps
/// A good value of eps is about 1e-10
pub fn solve_dare<T, States, Inputs, S1, S2, S3, S4>(
	a: &Matrix<T, States, States, S1>,
	b: &Matrix<T, States, Inputs, S2>,
	q: &Matrix<T, States, States, S3>,
	r: &Matrix<T, Inputs, Inputs, S4>,
	eps: T,
) -> OMatrix<T, States, States>
where
	T: RealField,
	States: DimName,
	Inputs: DimName,
	States: DimMin<States, Output = States>,
	S1: Storage<T, States, States>,
	S2: Storage<T, States, Inputs>,
	S3: Storage<T, States, States>,
	S4: Storage<T, Inputs, Inputs>,
	nalgebra::DefaultAllocator: Allocator<Inputs, Inputs>
		+ Allocator<States, States>
		+ Allocator<Inputs, States>
		+ Allocator<States>,
{
	let identity = OMatrix::<T, States, States>::identity();

	// Initialize starting conditions for approximation
	let mut a_k = a.clone_owned();
	let mut g_k = b * r.clone_owned().cholesky().unwrap().solve(&b.transpose());
	let mut h_k = q.clone_owned();

	// Iterate, returning when we get to the desired precision
	loop {
		let w = &identity + &g_k * &h_k;
		let w_solver = w.lu();

		let v1 = w_solver.solve(&a_k).unwrap();
		let v2 = w_solver.solve(&g_k.transpose()).unwrap().transpose();

		g_k += &a_k * v2 * a_k.transpose();
		let h_k_next = &h_k + v1.transpose() * &h_k * &a_k;
		a_k *= v1;

		if (&h_k_next - h_k).norm() < (eps.clone() * h_k_next.norm()) {
			return h_k_next;
		}

		h_k = h_k_next;
	}
}

/// Solves for the optimal LQR gains K given A, B, Q, R, N, and an eps to use for DARE solver precision
/// A good value of eps is about 1e-10
pub fn lqr<T, States, Inputs, S1, S2, S3, S4, S5>(
	a: &Matrix<T, States, States, S1>,
	b: &Matrix<T, States, Inputs, S2>,
	q: &Matrix<T, States, States, S3>,
	r: &Matrix<T, Inputs, Inputs, S4>,
	n: &Matrix<T, States, Inputs, S5>,
	eps: T,
) -> OMatrix<T, Inputs, States>
where
	T: RealField,
	States: DimName,
	Inputs: DimName,
	States: DimMin<States, Output = States>,
	S1: Storage<T, States, States>,
	S2: Storage<T, States, Inputs>,
	S3: Storage<T, States, States>,
	S4: Storage<T, Inputs, Inputs>,
	S5: Storage<T, States, Inputs>,
	nalgebra::DefaultAllocator: Allocator<Inputs, Inputs>
		+ Allocator<States, States>
		+ Allocator<States, Inputs>
		+ Allocator<Inputs, States>
		+ Allocator<States>,
{
	let r_llt: Cholesky<T, Inputs> = r.clone_owned().cholesky().unwrap();
	let rinv_nt = r_llt.solve(&n.transpose());

	let a2 = a - b * &rinv_nt;
	let q2 = q - n * &rinv_nt;

	let p = solve_dare(&a2, b, &q2, r, eps);
	let bt_p = b.transpose() * p;

	(r + &bt_p * b)
		.cholesky()
		.unwrap()
		.solve(&(&bt_p * a + n.transpose()))
}

pub fn brysons_rule<T, D, S>(max: &Vector<T, D, S>) -> OMatrix<T, D, D>
where
	T: RealField,
	D: DimName,
	S: Storage<T, D, U1>,
	nalgebra::DefaultAllocator: Allocator<D, D> + Allocator<D>,
{
	OMatrix::<T, D, D>::from_diagonal(&OVector::<T, D>::from_iterator(
		max.iter().map(|e| e.clone().powi(-2)),
	))
}

pub fn discretize_ab<T, S, I, S1, S2>(
	a: &Matrix<T, S, S, S1>,
	b: &Matrix<T, S, I, S2>,
	dt: T,
) -> (OMatrix<T, S, S>, OMatrix<T, S, I>)
where
	T: RealField,
	S: DimName,
	I: DimName,
	S: DimNameAdd<I>,
	S1: Storage<T, S, S>,
	S2: Storage<T, S, I>,
	DimNameSum<S, I>: DimMin<DimNameSum<S, I>, Output = DimNameSum<S, I>>,
	DefaultAllocator: Allocator<S, S>
		+ Allocator<S, I>
		+ Allocator<DimNameSum<S, I>, DimNameSum<S, I>>
		+ Allocator<DimNameSum<S, I>>,
{
	let mut m = OMatrix::<T, DimNameSum<S, I>, DimNameSum<S, I>>::zeros();
	m.view_range_mut(0..S::DIM, 0..S::DIM).copy_from(a);
	m.view_range_mut(0..S::DIM, S::DIM..).copy_from(b);

	let phi = m.scale(dt).exp();

	(
		phi.generic_view((0, 0), (S::name(), S::name()))
			.clone_owned(),
		phi.generic_view((0, S::DIM), (S::name(), I::name()))
			.clone_owned(),
	)
}
