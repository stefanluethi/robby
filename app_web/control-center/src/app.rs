mod proto_parser;
mod view;
mod model;

use std::collections::BTreeMap;
use view::Viewable;

#[derive(serde::Deserialize, serde::Serialize)]
enum Pane {
    Plots,
    Logs,
    TaskManager,
    RoomView,
}

impl From<&Pane> for &str {
    fn from(value: &Pane) -> Self {
        match value {
            Pane::Plots => "Plots",
            Pane::Logs => "Logs",
            Pane::TaskManager => "Task Manager",
            Pane::RoomView => "Room View",
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
pub struct Views {
    esp_address: String,
    stream_view: view::plot::Plot,
    task_manager: view::task_manager::TaskManager,
    room_view: view::heatmap::Heatmap,

    model: model::Model,

    #[serde(skip)]
    ws_receiver: Option<ewebsock::WsReceiver>,
    #[serde(skip)]
    ws_sender: Option<ewebsock::WsSender>,
}

impl Default for Views {
    fn default() -> Self {
        Self {
            esp_address: "ws://192.168.1.70/ws/robby".to_owned(),
            stream_view: view::plot::Plot::new(),
            task_manager: view::task_manager::TaskManager::new(),
            room_view: view::heatmap::Heatmap::new(),
            model: model::Model::default(),
            ws_receiver: None,
            ws_sender: None,
        }
    }
}

impl Views {
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
                            proto_parser::parse_message(msg, &mut self.model);
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

    fn websocket_send(&mut self, command: proto::Command) {
        if let Some(sender) = self.ws_sender.as_mut() {
            let command_packet = serde_cbor_2::to_vec(&command).unwrap();
            log::log!(target: "robby", log::Level::Info, "send command {}", hex_string::HexString::from_bytes(&command_packet).as_string());
            sender.send(ewebsock::WsMessage::Binary(command_packet));
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
        });
    }
}

impl egui_tiles::Behavior<Pane> for Views {
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
                self.stream_view.view(ui, &mut self.model);
            }
            Pane::Logs => {
                egui_logger::logger_ui()
                    .enable_category("robby", true)
                    .include_target(false)
                    .show(ui);
            }
            Pane::TaskManager => {
                self.task_manager.view(ui, &mut self.model);
            }
            Pane::RoomView => {
                self.room_view.view(ui, &mut self.model);
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
pub struct App {
    state: Views,
    tree: egui_tiles::Tree<Pane>,
}

impl Default for App {
    fn default() -> Self {
        Self {
            state: Default::default(),
            tree: App::create_tree(),
        }
    }
}

impl App {
    fn create_tree() -> egui_tiles::Tree<Pane> {
        let mut tiles = egui_tiles::Tiles::default();
        let plots = tiles.insert_pane(Pane::Plots);
        let logs = tiles.insert_pane(Pane::Logs);
        let task_manager = tiles.insert_pane(Pane::TaskManager);
        let room_view = tiles.insert_pane(Pane::RoomView);
        let root = tiles.insert_horizontal_tile(vec![plots, logs, task_manager, room_view]);
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
            app.tree = App::create_tree();
        }

        // renew tree if new panes have been added
        // if !app.tree.inactive_tiles().is_empty() {
        //     app.tree = EspApp::create_tree();
        // }

        app
    }
}

impl eframe::App for App {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.state.websocket_receive();

        // todo: find viable solution!!!
        let old_value = self.state.task_manager.value();

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

        let new_value = self.state.task_manager.value();
        if new_value != old_value  {
            self.state.websocket_send(proto::Command::SetMotorSpeed((new_value * 10.0_f32) as u16));
        }

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
