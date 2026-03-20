use super::Viewable;

#[derive(serde::Deserialize, serde::Serialize, PartialEq, Debug, Clone, Copy)]
pub enum ScopeMode {
    Full,
    Scan,
}

const ESP_SAMPLING_FREQUENCY: f32 = 20e3;

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct Plot
{
    samples: Vec<f32>,
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
            samples: vec![],
            scope_mode: ScopeMode::Full,
        }
    }

    pub fn update(&mut self, mut samples: Vec<f32>)
    {
        self.samples.append(&mut samples);
    }
}

impl Viewable for Plot {
    fn view(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.set_height(ui.available_height());
            ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                // we need to layout the plot last so that it can fill the remaining space
                ui.set_height(ui.available_height());

                ui.horizontal(|ui| {
                    if ui.button("Clear Data").clicked() {
                        self.samples.clear();
                    }
                    egui::ComboBox::from_label("Mode")
                        .selected_text(format!("{:?}", self.scope_mode))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.scope_mode, ScopeMode::Full, "Full");
                            ui.selectable_value(&mut self.scope_mode, ScopeMode::Scan, "Scan");
                        });
                });
                egui_plot::Plot::new("some plot")
                    .legend(egui_plot::Legend::default().follow_insertion_order(true))
                    .x_axis_label("t / s")
                    .y_axis_label("adc")
                    .show(ui, |plot_ui| {
                        let stride_length = self.samples.len() / 10_000 + 1;

                        let subset = match self.scope_mode {
                            ScopeMode::Full => self
                                .samples
                                .chunks(stride_length)
                                .map(|c| c.iter().sum::<f32>() as f64 / f64::from(c.len() as u32))
                                .enumerate()
                                .map(|(i, v)| {
                                    [
                                        f64::from((i * stride_length) as u32)
                                            / ESP_SAMPLING_FREQUENCY as f64,
                                        v,
                                    ]
                                })
                                .collect::<Vec<[f64; 2]>>(),
                            ScopeMode::Scan => {
                                let n_samples = self.samples.len();
                                self.samples[(self.samples.len() - n_samples)..]
                                    .iter()
                                    .enumerate()
                                    .map(|(i, v)| {
                                        [
                                            f64::from((i * stride_length) as u32)
                                                / ESP_SAMPLING_FREQUENCY as f64,
                                            *v as f64,
                                        ]
                                    })
                                    .collect::<Vec<[f64; 2]>>()
                            }
                        };

                        plot_ui.line(egui_plot::Line::new(
                            "Some plot",
                            egui_plot::PlotPoints::from(subset),
                        ));
                    });
            });
        });
    }
}
