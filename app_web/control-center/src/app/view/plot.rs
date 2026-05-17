use crate::app::model::{Model, streams::Stream};
use super::Viewable;

#[derive(serde::Deserialize, serde::Serialize, PartialEq, Debug, Clone, Copy)]
pub enum ScopeMode {
    Full,
    Scan,
}

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct Plot
{
    scope_mode: ScopeMode,
}

impl Default for Plot {
    fn default() -> Self {
        Self::new()
    }
}

impl Plot {
    pub fn new() -> Self {
        Self {
            scope_mode: ScopeMode::Full,
        }
    }

    fn plot_stream(stream: &Stream, plot_ui: &mut egui_plot::PlotUi<'_>, scope_mode: ScopeMode) {
        let stride_length = stream.values_raw.len() / 10_000 + 1;

        let subset = match scope_mode {
            ScopeMode::Full => stream
                .values_raw
                .chunks(stride_length)
                .map(|c| c
                    .iter()
                    .map(|v| (*v as f64) * stream.descriptor.scale_factor as f64)
                    .sum::<f64>() / f64::from(c.len() as u32))
                .enumerate()
                .map(|(i, v)| {
                    [
                        f64::from((i * stride_length) as u32)
                            * stream.descriptor.sampling_time as f64,
                        v,
                    ]
                })
                .collect::<Vec<[f64; 2]>>(),
            ScopeMode::Scan => {
                let n_samples = stream.values_raw.len();
                stream.values_raw[(n_samples - n_samples)..]
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        [
                            f64::from((i * stride_length) as u32)
                                * stream.descriptor.sampling_time as f64,
                            *v as f64,
                        ]
                    })
                    .collect::<Vec<[f64; 2]>>()
            }
        };

        plot_ui.line(egui_plot::Line::new(
            format!("{} / {}", stream.descriptor.name, stream.descriptor.unit),
            egui_plot::PlotPoints::from(subset),
        ));
    }
}

impl Viewable for Plot {
    fn view(&mut self, ui: &mut egui::Ui, model: &mut Model) {
        ui.vertical(|ui| {
            ui.set_height(ui.available_height());
            ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                // we need to layout the plot last so that it can fill the remaining space
                ui.set_height(ui.available_height());

                ui.horizontal(|ui| {
                    if ui.button("Clear Data").clicked() {
                        model.streams.clear();
                    }
                    egui::ComboBox::from_label("Mode")
                        .selected_text(format!("{:?}", self.scope_mode))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.scope_mode, ScopeMode::Full, "Full");
                            ui.selectable_value(&mut self.scope_mode, ScopeMode::Scan, "Scan");
                        });
                });
                egui_plot::Plot::new("Stream View")
                    .legend(egui_plot::Legend::default().follow_insertion_order(true))
                    .x_axis_label("t / s")
                    .y_axis_label("value")
                    .show(ui, |plot_ui| {
                        for stream in model.streams.iter() {
                            Plot::plot_stream(&stream, plot_ui, self.scope_mode);
                        }
                    });
            });
        });
    }
}
