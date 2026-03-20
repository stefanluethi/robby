pub mod task_manager;
pub mod plot;
pub mod heatmap;

mod colormap;

pub trait Viewable {
    fn view(&mut self, ui: &mut egui::Ui);
}
