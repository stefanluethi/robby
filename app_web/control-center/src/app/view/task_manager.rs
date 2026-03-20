use super::Viewable;

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct TaskManager {
    thread_table: proto::ThreadTable,
}

impl Default for TaskManager {
    fn default() -> Self {
        Self::new()
    }
}

impl TaskManager {
    pub fn new() -> Self {
        Self {
            thread_table: proto::ThreadTable {
                threads: Vec::new(),
            },
        }
    }

    pub fn update(&mut self, thread_table: proto::ThreadTable) {
        self.thread_table = thread_table;
    }
}

impl Viewable for TaskManager {
    fn view(&mut self, ui: &mut egui::Ui) {
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
                    for thread in self.thread_table.threads.iter() {
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
