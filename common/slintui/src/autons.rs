use std::rc::Rc;

use autons::{Selector, simple::Route};
use slint::{ComponentHandle, SharedString, VecModel};

use crate::{App, AutonsPageState};

pub struct SlintSelector<const N: usize, R> {
	app: App,
	routes: [Route<R>; N],
}

impl<const N: usize, R> SlintSelector<N, R> {
	pub fn new(app: &App, routes: [Route<R>; N]) -> Self {
		app.global::<AutonsPageState>().set_autons(
			Rc::new(
				routes
					.iter()
					.map(|r| SharedString::from(r.name))
					.collect::<VecModel<_>>(),
			)
			.into(),
		);

		Self {
			app: app.clone_strong(),
			routes,
		}
	}
}

impl<const N: usize, R> Selector<R> for SlintSelector<N, R> {
	async fn run(&self, robot: &mut R) {
		if let Some(route) = self
			.routes
			.get(self.app.global::<AutonsPageState>().get_selected() as usize)
		{
			(route.callback)(robot).await;
		}
	}
}
