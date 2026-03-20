use super::Viewable;

#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct Heatmap {}

impl Default for Heatmap {
    fn default() -> Self {
        Self::new()
    }
}

impl Heatmap {
    pub fn new() -> Self {
        Self {}
    }

}

impl Viewable for Heatmap {
    fn view(&mut self, ui: &mut egui::Ui) {
        let remainder = ui.available_rect_before_wrap();
        let min_size = remainder.size().x.min(remainder.size().y);
        let rect_size: f32 = min_size / 7.0;
        for x in 0..7 {
            for y in 0..7 {
                ui.painter().rect_filled(
                    egui::Rect {
                        min: remainder.left_top()
                            + egui::vec2(rect_size * x as f32, rect_size * y as f32),
                        max: remainder.left_top()
                            + egui::vec2(rect_size * (x + 1) as f32, rect_size * (y + 1) as f32),
                    },
                    0.0,
                    super::colormap::map_color((x + y) as f32 / 14.0),
                );
            }
        }
    }
}
