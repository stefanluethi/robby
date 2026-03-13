use std::collections::BTreeMap;

const ESP_SAMPLING_FREQUENCY: f32 = 20e3;

#[derive(serde::Deserialize, serde::Serialize, PartialEq, Debug, Clone, Copy)]
enum ScopeMode {
    Full,
    Scan,
}

#[derive(serde::Deserialize, serde::Serialize, PartialEq, Clone, Copy)]
#[repr(u32)]
enum FFTSamples {
    _512 = 512,
    _1024 = 1024,
    _2048 = 2048,
    _4096 = 4096,
    _8192 = 8192,
    _16384 = 16384,
    _32768 = 32768,
}

impl Into<String> for FFTSamples {
    fn into(self) -> String {
        format!("{}", self as u32)
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
enum Pane {
    Plots,
    Logs,
}

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct AppState {
    esp_address: String,
    samples: Vec<f32>,
    scope_mode: ScopeMode,
    fft_samples: FFTSamples,
    fft_frequence_end: f32,

    #[serde(skip)]
    ws_receiver: Option<ewebsock::WsReceiver>,
    #[serde(skip)]
    ws_sender: Option<ewebsock::WsSender>,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            esp_address: "ws://192.168.1.70/ws/adc".to_owned(),
            samples: vec![],
            scope_mode: ScopeMode::Full,
            fft_samples: FFTSamples::_1024,
            fft_frequence_end: ESP_SAMPLING_FREQUENCY / 2.0,
            ws_receiver: None,
            ws_sender: None,
        }
    }
}

impl AppState {
    fn websocket_connect(&mut self, ctx: egui::Context) {
        let wakeup = move || ctx.request_repaint();
        let mut options = ewebsock::Options::default();
        options.max_incoming_frame_size = 32;
        let (sender, receiver) =
            ewebsock::connect_with_wakeup(self.esp_address.clone(), options, wakeup).unwrap();
        self.ws_receiver = Some(receiver);
        self.ws_sender = Some(sender);
    }

    fn websocket_disconnect(&mut self) {
        if let Some(sender) = self.ws_sender.as_mut() {
            sender.close();
        }
        self.ws_receiver = None;
        self.ws_sender = None;
        log::log!(target: "robby", log::Level::Info, "ws disconnected")
    }

    fn websocket_receive(&mut self) {
        if let Some(receiver) = self.ws_receiver.as_ref() {
            while let Some(event) = receiver.try_recv() {
                match event {
                    ewebsock::WsEvent::Opened => {
                        log::log!(target: "robby", log::Level::Info, "ws connected")
                    }
                    ewebsock::WsEvent::Message(message) => match message {
                        ewebsock::WsMessage::Text(msg) => {
                            log::log!(target: "robby", log::Level::Info, "{}", msg);
                        }
                        ewebsock::WsMessage::Binary(msg) => {
                            let message: proto::AdcMessage = postcard::from_bytes(&msg).unwrap();

                            self.samples.append(
                                &mut message
                                    .samples
                                    .iter()
                                    .map(|v| f32::from(*v))
                                    .collect::<Vec<f32>>(),
                            );
                            log::log!(target: "robby", log::Level::Info, "ws received {} samples", message.samples.len())
                        }
                        _ => {
                            log::log!(target: "robby", log::Level::Info, "ws unknown message received")
                        }
                    },
                    ewebsock::WsEvent::Error(_) => {
                        log::log!(target: "robby", log::Level::Warn, "ws error")
                    }
                    ewebsock::WsEvent::Closed => {
                        log::log!(target: "robby", log::Level::Info, "ws disconnected")
                    }
                };
            }
        }
    }

    fn plots_ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("esp32scope");

        ui.horizontal(|ui| {
            ui.label("websocket address:");
            ui.text_edit_singleline(&mut self.esp_address);
            if ui
                .button(if self.ws_receiver.is_none() {
                    "Connect"
                } else {
                    "Disconnect"
                })
                .clicked()
            {
                if self.ws_receiver.is_none() {
                    self.websocket_connect(ui.ctx().clone());
                } else {
                    self.websocket_disconnect();
                }
            }
            if ui.button("Clear Data").clicked() {
                self.samples.clear();
            }
        });
        ui.separator();

        //-----------------------------------------------------------------
        ui.vertical(|ui| {
            ui.set_height(ui.available_height() / 2.0);
            ui.vertical_centered(|ui| {
                ui.strong("Time Domain");
            });
            ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                // we need to layout the plot last so that it can fill the remaining space
                ui.set_height(ui.available_height());

                ui.horizontal(|ui| {
                    egui::ComboBox::from_label("Mode")
                        .selected_text(format!("{:?}", self.scope_mode))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.scope_mode, ScopeMode::Full, "Full");
                            ui.selectable_value(&mut self.scope_mode, ScopeMode::Scan, "Scan");
                        });
                });
                egui_plot::Plot::new("eps32 adc plot")
                    .legend(egui_plot::Legend::default().follow_insertion_order(true))
                    .x_axis_label("t / s")
                    .y_axis_label("adc")
                    .show(ui, |plot_ui| {
                        let stride_length = self.samples.len() / 10_000 + 1;

                        let subset = match self.scope_mode {
                            ScopeMode::Full => self
                                .samples
                                .chunks(stride_length)
                                .map(|c| {
                                    c.iter().sum::<f32>() as f64 / f64::from(c.len() as u32)
                                })
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
                                let n_samples = std::cmp::min(
                                    self.fft_samples as usize,
                                    self.samples.len(),
                                );
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
                            "ESP ADC",
                            egui_plot::PlotPoints::from(subset),
                        ));
                    });
            });
        });
        ui.separator();

        //-----------------------------------------------------------------
        ui.vertical_centered(|ui| {
            ui.strong("Frequency Domain");
        });
        let fft_length = self.fft_samples as usize;
        let n_samples = if self.samples.len() < fft_length {
            0
        } else {
            fft_length
        };
        let hann_window = spectrum_analyzer::windows::hann_window(
            &self.samples[(self.samples.len() - n_samples)..],
        );
        // calc spectrum
        let spectrum = spectrum_analyzer::samples_fft_to_spectrum(
            // (windowed) samples
            &hann_window,
            // sampling rate
            ESP_SAMPLING_FREQUENCY as u32,
            // optional frequency limit: e.g. only interested in frequencies 50 <= f <= 150?
            spectrum_analyzer::FrequencyLimit::All,
            // optional scale
            Some(&spectrum_analyzer::scaling::divide_by_N),
        );

        ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
            // we need to layout the plot last so that it can fill the remaining space
            ui.set_height(ui.available_height());

            ui.horizontal(|ui| {
                egui::ComboBox::from_label("Samples")
                    .selected_text(format!("{}", Into::<String>::into(self.fft_samples)))
                    .show_ui(ui, |ui| {
                        for item in [
                            FFTSamples::_512,
                            FFTSamples::_1024,
                            FFTSamples::_2048,
                            FFTSamples::_4096,
                            FFTSamples::_8192,
                            FFTSamples::_16384,
                            FFTSamples::_32768,
                        ]
                        .iter()
                        {
                            ui.selectable_value(
                                &mut self.fft_samples,
                                *item,
                                Into::<String>::into(*item),
                            );
                        }
                    });

                ui.separator();
                ui.label("End frequency");
                ui.add(
                    egui::DragValue::new(&mut self.fft_frequence_end)
                        .range(0.0..=ESP_SAMPLING_FREQUENCY / 2.0)
                        .speed(100),
                );
            });
            egui_plot::Plot::new("eps32 fft plot")
                .legend(egui_plot::Legend::default().follow_insertion_order(true))
                .x_axis_label("f / Hz")
                .y_axis_label("dB")
                .show(ui, |plot_fft| {
                    let fft_line = if let Ok(s) = spectrum {
                        s.data()
                            .iter()
                            .map(|(fr, fr_val)| {
                                [fr.val() as f64, 20.0 * fr_val.val().log10() as f64]
                            })
                            .collect::<Vec<[f64; 2]>>()
                    } else {
                        Vec::new()
                    };

                    plot_fft.line(egui_plot::Line::new(
                        "FFT",
                        egui_plot::PlotPoints::from(fft_line),
                    ));

                    plot_fft.set_plot_bounds_y(std::ops::RangeInclusive::new(-60_f64, 40_f64));
                    plot_fft.set_plot_bounds_x(0.0..=self.fft_frequence_end as f64);
                });
        });
    }
}

impl egui_tiles::Behavior<Pane> for AppState {
    fn pane_ui(
        &mut self,
        ui: &mut egui::Ui,
        _tile_id: egui_tiles::TileId,
        pane: &mut Pane,
    ) -> egui_tiles::UiResponse {
        let title = match pane {
            Pane::Plots => "Plots",
            Pane::Logs => "Logs",
        };

        let mut drag_response = egui_tiles::UiResponse::None;

        let title_bar_response = ui.horizontal(|ui| {
            ui.add_space(4.0);
            ui.strong(title);
            ui.allocate_space(ui.available_size())
        }).response;

        if title_bar_response.interact(egui::Sense::click_and_drag()).dragged() {
            drag_response = egui_tiles::UiResponse::DragStarted;
        }

        ui.separator();

        match pane {
            Pane::Plots => {
                self.plots_ui(ui);
            }
            Pane::Logs => {
                egui_logger::logger_ui()
                    .enable_category("robby", true)
                    .include_target(false)
                    .show(ui);
            }
        }

        drag_response
    }

    fn tab_title_for_pane(&mut self, pane: &Pane) -> egui::WidgetText {
        match pane {
            Pane::Plots => "Plots".into(),
            Pane::Logs => "Logs".into(),
        }
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct EspApp {
    state: AppState,
    tree: egui_tiles::Tree<Pane>,
}

impl Default for EspApp {
    fn default() -> Self {
        let mut tiles = egui_tiles::Tiles::default();
        let plots = tiles.insert_pane(Pane::Plots);
        let logs = tiles.insert_pane(Pane::Logs);
        let root = tiles.insert_horizontal_tile(vec![plots, logs]);
        let tree = egui_tiles::Tree::new("main_tree", root, tiles);

        Self {
            state: Default::default(),
            tree,
        }
    }
}

impl EspApp {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.
        
        configure_text_styles(&cc.egui_ctx);

        // Load previous app state (if any).
        // Note that you must enable the `persistence` feature for this to work.
        let mut app: Self = if let Some(storage) = cc.storage {
            eframe::get_value(storage, eframe::APP_KEY).unwrap_or_default()
        } else {
            Default::default()
        };
        
        if app.tree.root.is_none() {
            let mut tiles = egui_tiles::Tiles::default();
            let plots = tiles.insert_pane(Pane::Plots);
            let logs = tiles.insert_pane(Pane::Logs);
            let root = tiles.insert_horizontal_tile(vec![plots, logs]);
            app.tree = egui_tiles::Tree::new("main_tree", root, tiles);
        }
        
        app
    }
}

impl eframe::App for EspApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.state.websocket_receive();

        egui::TopBottomPanel::bottom("bottom panel").show(ctx, |ui| {
            ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                egui::widgets::global_theme_preference_buttons(ui);
                egui::warn_if_debug_build(ui);
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.tree.ui(&mut self.state, ui);
        });
    }
}

fn configure_text_styles(ctx: &egui::Context) {
    use egui::FontFamily::{Monospace, Proportional};
    use egui::style::TextStyle;
    use egui::FontId;

    let text_styles: BTreeMap<TextStyle, FontId> = [
        (TextStyle::Heading, FontId::new(24.0, Proportional)),
        (TextStyle::Body, FontId::new(14.0, Proportional)),
        (TextStyle::Monospace, FontId::new(10.0, Monospace)),
        (TextStyle::Button, FontId::new(14.0, Proportional)),
        (TextStyle::Small, FontId::new(10.0, Proportional)),
    ]
    .into();
    ctx.all_styles_mut(move |style| style.text_styles = text_styles.clone());
}