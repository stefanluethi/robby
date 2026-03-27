use super::Viewable;

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct Heatmap {
    distance_map: proto::DistanceMap,
}

impl Default for Heatmap {
    fn default() -> Self {
        Self::new()
    }
}

impl Heatmap {
    pub fn new() -> Self {
        Self {
            distance_map: proto::DistanceMap { distances_mm: [[0;_];_] }
        }
    }

    pub fn update(&mut self, distances: proto::DistanceMap) {
        self.distance_map = distances;
    }

}

impl Viewable for Heatmap {
    fn view(&mut self, ui: &mut egui::Ui) {
        let size = (self.distance_map.distances_mm.len(), self.distance_map.distances_mm[0].len());

        let remainder = ui.available_rect_before_wrap();
        let rect_size: f32 = (remainder.size().x / (size.0 - 1) as f32).min(remainder.size().y / (size.1 - 1) as f32);
        for x in 0..size.0 - 1 {
            for y in 0..size.1 - 1 {
                ui.painter().rect_filled(
                    egui::Rect {
                        min: remainder.left_top()
                            + egui::vec2(rect_size * x as f32, rect_size * y as f32),
                        max: remainder.left_top()
                            + egui::vec2(rect_size * (x + 1) as f32, rect_size * (y + 1) as f32),
                    },
                    0.0,
                    map_distance_color(self.distance_map.distances_mm[x][y]),
                );
            }
        }
    }
}

fn map_distance_color(distance: u16) -> egui::Color32 {
    const MAX_DISTANCE: u16 = 4_000;
    if distance > MAX_DISTANCE {
        egui::Color32::BLACK
    } else {
        super::colormap::map_color(distance as f32 / MAX_DISTANCE as f32)
    }
}