// Source - https://stackoverflow.com/a
// Posted by Joshua Fraser, modified by community. See post 'Timeline' for change history
// Retrieved 2026-01-11, License - CC BY-SA 4.0

fn clamp(v: f32) -> f32
{
  let t = if v < 0.0_f32 { 0.0_f32 } else { v };
  return if t > 1.0_f32 { 1.0_f32 } else { t };
}


pub fn map_color(value: f32) -> egui::Color32
{
    let t = value * 2.0_f32 - 1.0_f32;
    let red = clamp(1.5 - (2.0 * t - 1.0).abs()) * 255.0;
    let green = clamp(1.5 - (2.0 * t).abs()) * 255.0;
    let blue = clamp(1.5 - (2.0 * t + 1.0).abs()) * 255.0;

    return egui::Color32::from_rgb(red as u8, green as u8, blue as u8);
}