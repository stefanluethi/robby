use super::Viewable;
use crate::app::model::{Model, streams::Stream};
use std::cmp::min;
use egui::ahash::HashMap;
use egui::Ui;

const MAX_PLOT_COUNT: usize = 6;

#[derive(serde::Deserialize, serde::Serialize, PartialEq, Debug, Clone, Copy)]
pub enum ScopeMode {
    Full,
    Scan,
}

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct Plot {
    number_of_plots: usize,
    selected_plot: usize,
    selected_streams: [HashMap<u16, bool>; MAX_PLOT_COUNT],
    scope_mode: ScopeMode,
    scan_window: f32,
}

impl Default for Plot {
    fn default() -> Self {
        Self::new()
    }
}

impl Plot {
    pub fn new() -> Self {
        Self {
            number_of_plots: 1,
            selected_plot: 0,
            selected_streams: Self::init_selected_streams(),
            scope_mode: ScopeMode::Full,
            scan_window: 20.0,
        }
    }

    fn init_selected_streams() -> [HashMap<u16, bool>; MAX_PLOT_COUNT] {
        let mut array: [HashMap<u16, bool>; MAX_PLOT_COUNT] =
            Default::default();
        for item in &mut array {
            *item = HashMap::default();
        }
        array
    }

    fn show_plot_settings(&mut self, model: &mut Model, ui: &mut egui::Ui) {
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

            if self.scope_mode == ScopeMode::Scan {
                ui.separator();
                ui.label("Scan window:");
                ui.add(
                    egui::DragValue::new(&mut self.scan_window)
                        .range(0.0..=1.0e6)
                        .suffix("s")
                        .speed(0.01),
                );
            }
        });
    }

    fn show_stream_selection(&mut self, model: &Model, ui: &mut egui::Ui) {
        egui::Frame::default()
            .fill(ui.visuals().faint_bg_color)
            .corner_radius(4.0)
            .inner_margin(egui::Margin::same(8))
            .show(ui, |ui| {
                ui.set_height(ui.available_height());
                ui.vertical(|ui| {
                    ui.set_width(100.0);
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.number_of_plots)
                                .speed(1.0)
                                .range(1..=MAX_PLOT_COUNT),
                        );
                        ui.label("plots");
                    });

                    ui.add_space(10.0);

                    ui.label("Streams:");
                    let active_selectors = &mut self.selected_streams[self.selected_plot];

                    ui.set_width(ui.available_width());
                    ui.vertical(|ui| {
                        let mut streams = model.streams.iter().collect::<Vec<_>>();
                        streams.sort_by_key(|stream| stream.descriptor.id);

                        for stream in streams {
                            let stream_selected = *active_selectors
                                .entry(stream.descriptor.id)
                                .or_insert(false);
                            let button_name = format!("{}: {}", stream.descriptor.id, stream.descriptor.name);
                            if ui.add_sized(
                                egui::Vec2::new(ui.available_width(), 20.0),
                                egui::Button::new(button_name).selected(stream_selected),
                            ).clicked() {
                                *active_selectors.get_mut(&stream.descriptor.id).unwrap() = !stream_selected;
                            }
                        }
                    });
                });
            });
    }

    fn plot_stream_series(&self, stream: &Stream, plot_ui: &mut egui_plot::PlotUi<'_>) {
        let stride_length = stream.values_raw.len() / 10_000 + 1;

        let subset = match self.scope_mode {
            ScopeMode::Full => stream
                .values_raw
                .chunks(stride_length)
                .map(|c| c.iter().map(|v| *v as f64).sum::<f64>() / f64::from(c.len() as u32))
                .enumerate()
                .map(|(i, v)| {
                    [
                        f64::from((i * stride_length) as u32)
                            * stream.descriptor.sampling_time as f64,
                        v * stream.descriptor.scale_factor as f64,
                    ]
                })
                .collect::<Vec<[f64; 2]>>(),
            ScopeMode::Scan => {
                let window_length = (self.scan_window / stream.descriptor.sampling_time) as usize;
                let n_samples = min(stream.values_raw.len(), window_length);
                stream.values_raw[(stream.values_raw.len() - n_samples)..]
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        [
                            f64::from(i as u32) * stream.descriptor.sampling_time as f64,
                            *v as f64 * stream.descriptor.scale_factor as f64,
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

    fn calculate_plot_height(&mut self, ui: &mut Ui) -> f32 {
        let plot_count = self.number_of_plots as f32;
        let frame_stroke_width = 1.0;
        let frame_inner_margin = 4.0;
        let spacing_between_plots = ui.spacing().item_spacing.y * (plot_count - 1.0);
        let frame_vertical_overhead = plot_count * (2.0 * frame_stroke_width + 2.0 * frame_inner_margin);

        let plot_height =
            ((ui.available_height() - spacing_between_plots - frame_vertical_overhead) / plot_count)
                .max(0.0);
        plot_height
    }

    fn show_plot_stack(&mut self, model: &Model, ui: &mut egui::Ui) {
        let plot_height = self.calculate_plot_height(ui);

        let link_group_id = ui.id().with("stream_plot_group");
        let link_x = egui::Vec2b::new(true, false);

        self.selected_plot = min(self.selected_plot, self.number_of_plots - 1);
        for i in 0..self.number_of_plots {
            self.show_single_plot(model, ui, i, plot_height, link_group_id, link_x);
        }
    }

    fn show_single_plot(
        &mut self,
        model: &Model,
        ui: &mut egui::Ui,
        index: usize,
        plot_height: f32,
        link_group_id: egui::Id,
        link_x: egui::Vec2b,
    ) {
        let stroke = if self.selected_plot == index {
            egui::Stroke::new(1.0, ui.visuals().widgets.active.bg_fill)
        } else {
            egui::Stroke::new(1.0, ui.visuals().window_fill())
        };
        let fille = if self.selected_plot == index {
            ui.visuals().faint_bg_color
        } else {
            ui.visuals().window_fill()
        };

        let frame = egui::Frame::default()
            .stroke(stroke)
            .fill(fille)
            .corner_radius(4.0)
            .inner_margin(egui::Margin::same(4));

        frame.show(ui, |ui| {
            ui.set_height(plot_height);
            let plot_response = self.show_plot_contents(model, ui, index, link_group_id, link_x);

            if plot_response.response.clicked() {
                self.selected_plot = index;
            }
        });
    }

    fn show_plot_contents(
        &self,
        model: &Model,
        ui: &mut egui::Ui,
        plot_index: usize,
        link_group_id: egui::Id,
        link_x: egui::Vec2b,
    ) -> egui_plot::PlotResponse<()> {
        let mut plot = egui_plot::Plot::new(format!("Stream View {}", plot_index))
            .link_axis(link_group_id, link_x)
            .link_cursor(link_group_id, link_x)
            .legend(egui_plot::Legend::default())
            .custom_y_axes(vec![
                egui_plot::AxisHints::new_y()
                    .placement(egui_plot::HPlacement::Left)
                    .min_thickness(50.0),
            ]);

        if plot_index == self.number_of_plots {
            plot = plot.x_axis_label("t / s");
        }

        plot.show(ui, |plot_ui| {
            for stream in model.streams.iter() {
                if *self.selected_streams[plot_index].get(&stream.descriptor.id).unwrap_or(&false) {
                    self.plot_stream_series(stream, plot_ui);
                }
            }
        })
    }
}

impl Viewable for Plot {
    fn view(&mut self, ui: &mut egui::Ui, model: &mut Model) {
        ui.vertical(|ui| {
            let available_size = ui.available_size();

            ui.horizontal(|ui| {
                ui.set_height(available_size.y);

                self.show_stream_selection(model, ui);

                ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                    self.show_plot_settings(model, ui);
                    ui.vertical(|ui| {
                        self.show_plot_stack(model, ui);
                    })
                });
            });
        });
    }
}
