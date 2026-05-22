use crate::app::model::Model;
use super::Viewable;

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct Heatmap {
}

impl Default for Heatmap {
    fn default() -> Self {
        Self::new()
    }
}

impl Heatmap {
    pub fn new() -> Self {
        Self {
        }
    }
}

impl Viewable for Heatmap {
    fn view(&mut self, ui: &mut egui::Ui, model: &mut Model) {
        let size = (model.distance_map.distances_mm.len(), model.distance_map.distances_mm[0].len());

        let remainder = ui.available_rect_before_wrap();
        let rect_size: f32 = (remainder.size().x / (size.0 - 1) as f32).min(remainder.size().y / (size.1 - 1) as f32);
        for x in 0..size.0 - 1 {
            for y in 0..size.1 - 1 {
                let distance = model.distance_map.distances_mm[x][y];
                let color = map_distance_color(distance).unwrap_or(ui.visuals().panel_fill);
                let top_left = remainder.left_top() + egui::vec2(rect_size * x as f32, rect_size * y as f32); 
                let bottom_right = remainder.left_top() + egui::vec2(rect_size * (x + 1) as f32, rect_size * (y + 1) as f32);
                // colored box
                ui.painter().rect_filled(
                    egui::Rect {
                        min: top_left,
                        max: bottom_right,
                    },
                    0.0,
                    color,
                );

                // value
                if rect_size < 35.0 {
                    continue;
                }
                if distance != u16::MAX {
                    let text_color = if color.intensity() < 0.5 {
                        egui::Color32::WHITE
                    } else {
                        egui::Color32::DARK_GRAY
                    };

                    ui.painter().text(
                        (top_left + bottom_right.to_vec2()) / 2.0_f32, 
                        egui::Align2::CENTER_CENTER, 
                        format!("{}", model.distance_map.distances_mm[x][y]),
                        Default::default(), 
                        text_color
                    );
                }
            }
        }
    }
}

fn map_distance_color(distance: u16) -> Option<egui::Color32> {
    const MAX_DISTANCE: u16 = 4_000;
    if distance > MAX_DISTANCE || distance == 0 {
        None
    } else {
        Some(super::colormap::map_color(distance as f32 / MAX_DISTANCE as f32))
    }
}