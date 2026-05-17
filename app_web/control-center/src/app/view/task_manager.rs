use crate::app::model::Model;
use super::Viewable;

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct TaskManager {
    slider_value: f32,
}

impl Default for TaskManager {
    fn default() -> Self {
        Self::new()
    }
}

impl TaskManager {
    pub fn new() -> Self {
        Self {
            slider_value: 0.0,
        }
    }

    pub fn value(&self) -> f32 {
        self.slider_value
    }
}

impl Viewable for TaskManager {
    fn view(&mut self, ui: &mut egui::Ui, model: &mut Model) {
        ui.vertical(|ui| {
            let available_height = ui.available_height();
            let table = egui_extras::TableBuilder::new(ui)
                .striped(true)
                .resizable(true)
                .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
                .column(egui_extras::Column::auto())
                .column(egui_extras::Column::auto())
                .column(egui_extras::Column::auto())
                .column(egui_extras::Column::auto())
                .column(egui_extras::Column::auto())
                .column(egui_extras::Column::remainder())
                .min_scrolled_height(0.0)
                .max_scroll_height(available_height);

            table
                .header(20.0, |mut header| {
                    header.col(|ui| {
                        ui.strong("Name");
                    });
                    header.col(|ui| {
                        ui.strong("Priority");
                    });
                    header.col(|ui| {
                        ui.strong("Stack Usage %");
                    });
                    header.col(|ui| {
                        ui.strong("Stack Size kB");
                    });
                    header.col(|ui| {
                        ui.strong("CPU Load %");
                    });
                })
                .body(|mut body| {
                    model.thread_table.threads.sort_by(|a, b| {
                        a.name.to_lowercase().cmp(&b.name.to_lowercase())
                    });
                    for thread in model.thread_table.threads.iter() {
                        body.row(20.0_f32, |mut row| {
                            row.col(|ui| {
                                ui.label(thread.name.clone());
                            });
                            row.col(|ui| {
                                ui.label(format!("{}", thread.priority));
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.1}", thread.stack_usage));
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.2}", thread.stack_size as f32 / 1024.0));
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.1}", thread.runtime));
                            });
                        });
                    }
                });

            ui.add_space(10.0);
            ui.spacing_mut().slider_width = 300.0;
            let _ = ui.add(
                egui::Slider::new(&mut self.slider_value, 0_f32..=100_f32)
                    .orientation(egui::SliderOrientation::Horizontal)
                    .text("Left Motor")
                    .step_by(0.1)
            );
        });
    }
}
