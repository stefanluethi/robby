use std::collections::BTreeMap;

use proto::TaskManager;

const ESP_SAMPLING_FREQUENCY: f32 = 20e3;

#[derive(serde::Deserialize, serde::Serialize, PartialEq, Debug, Clone, Copy)]
enum ScopeMode {
    Full,
    Scan,
}

#[derive(serde::Deserialize, serde::Serialize)]
enum Pane {
    Plots,
    Logs,
    TaskManager,
}

impl From<&Pane> for &str {
    fn from(value: &Pane) -> Self {
        match value {
            Pane::Plots => "Plots",
            Pane::Logs => "Logs",
            Pane::TaskManager => "Task Manager",
        }
    }
}

impl From<&mut Pane> for &str {
    fn from(value: &mut Pane) -> Self {
        (value as &Pane).into()
    }
}

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct AppState {
    esp_address: String,
    samples: Vec<f32>,
    scope_mode: ScopeMode,
    task_manager: proto::TaskManager,

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
            task_manager: TaskManager {
                threads: Vec::new(),
            },
            ws_receiver: None,
            ws_sender: None,
        }
    }
}

impl AppState {
    fn websocket_connect(&mut self, ctx: egui::Context) {
        let wakeup = move || ctx.request_repaint();
        let mut options = ewebsock::Options::default();
        options.max_incoming_frame_size = 2048;
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
                            // todo: find better solution!
                            if let Ok(message) =
                                serde_cbor_2::from_slice::<proto::TaskManager>(&msg)
                            {
                                for thread in message.threads.iter() {
                                    log::log!(target: "robby", log::Level::Info, "thread info \"{}\" stack usage: {}, cpu usage {}", 
                                    thread.name, thread.stack_usage, thread.runtime);
                                }
                                self.task_manager = message;
                            } else if let Ok(message) =
                                serde_cbor_2::from_slice::<proto::AdcMessage>(&msg)
                            {
                                self.samples.append(
                                    &mut message
                                        .samples
                                        .iter()
                                        .map(|v| f32::from(*v))
                                        .collect::<Vec<f32>>(),
                                );
                                log::log!(target: "robby", log::Level::Trace, "ws received {} samples", message.samples.len());
                            }
                        }
                        _ => {
                            log::log!(target: "robby", log::Level::Info, "ws unknown message received")
                        }
                    },
                    ewebsock::WsEvent::Error(e) => {
                        log::log!(target: "robby", log::Level::Warn, "ws error: {}", e)
                    }
                    ewebsock::WsEvent::Closed => {
                        log::log!(target: "robby", log::Level::Info, "ws disconnected")
                    }
                };
            }
        }
    }

    fn settings_ui(&mut self, ui: &mut egui::Ui) {
        ui.heading("robby control center");

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
    }

    fn plots_ui(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.set_height(ui.available_height());
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
                            "ESP ADC",
                            egui_plot::PlotPoints::from(subset),
                        ));
                    });
            });
        });
    }

    fn task_manager_ui(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            let available_height = ui.available_height();
            let table = egui_extras::TableBuilder::new(ui)
                .striped(true)
                .resizable(true)
                .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
                .column(egui_extras::Column::auto())
                .column(egui_extras::Column::auto())
                .column(egui_extras::Column::auto())
                .column(egui_extras::Column::remainder())
                .column(egui_extras::Column::remainder())
                .min_scrolled_height(0.0)
                .max_scroll_height(available_height);

            table
                .header(20.0, |mut header| {
                    header.col(|ui| {
                        ui.strong("Name");
                    });
                    header.col(|ui| {
                        ui.strong("Stack Usage %");
                    });
                    header.col(|ui| {
                        ui.strong("Stack Usage B");
                    });
                    header.col(|ui| {
                        ui.strong("CPU Load %");
                    });
                })
                .body(|mut body| {
                    for thread in self.task_manager.threads.iter() {
                        body.row(20.0_f32, |mut row| {
                            row.col(|ui| {
                                ui.label(thread.name.clone());
                            });
                            row.col(|ui| {
                                ui.label(format!("{}%", thread.stack_usage));
                            });
                            row.col(|ui| {
                                ui.label("1200");
                            });
                            row.col(|ui| {
                                ui.label(format!("{:.1}%", thread.runtime));
                            });
                        });
                    }
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
        let title: &str = pane.into();
        let mut drag_response = egui_tiles::UiResponse::None;

        let title_bar_response = ui
            .horizontal(|ui| {
                ui.add_space(4.0);
                ui.strong(title);
                ui.allocate_space(ui.available_size())
            })
            .response;

        if title_bar_response
            .interact(egui::Sense::click_and_drag())
            .dragged()
        {
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
            Pane::TaskManager => {
                self.task_manager_ui(ui);
            }
        }

        drag_response
    }

    fn tab_title_for_pane(&mut self, pane: &Pane) -> egui::WidgetText {
        Into::<&str>::into(pane).into()
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
        Self {
            state: Default::default(),
            tree: EspApp::create_tree(),
        }
    }
}

impl EspApp {
    fn create_tree() -> egui_tiles::Tree<Pane> {
        let mut tiles = egui_tiles::Tiles::default();
        let plots = tiles.insert_pane(Pane::Plots);
        let logs = tiles.insert_pane(Pane::Logs);
        let task_manager = tiles.insert_pane(Pane::TaskManager);
        let root = tiles.insert_horizontal_tile(vec![plots, logs, task_manager]);
        egui_tiles::Tree::new("main_tree", root, tiles)
    }

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
            app.tree = EspApp::create_tree();
        }

        // renew tree if new panes have been added
        if !app.tree.inactive_tiles().is_empty() {
            app.tree = EspApp::create_tree();
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

        egui::TopBottomPanel::top("top panel").show(ctx, |ui| {
            ui.with_layout(egui::Layout::top_down(egui::Align::LEFT), |ui| {
                self.state.settings_ui(ui);
            });
        });

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
    use egui::FontId;
    use egui::style::TextStyle;

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
