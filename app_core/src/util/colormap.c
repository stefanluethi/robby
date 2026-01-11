// Source - https://stackoverflow.com/a
// Posted by Joshua Fraser, modified by community. See post 'Timeline' for change history
// Retrieved 2026-01-11, License - CC BY-SA 4.0

#include "colormap.h"
#include <stdint.h>
#include <math.h>

float clamp(float v)
{
  const float t = v < 0.0F ? 0.0F : v;
  return t > 1.0F ? 1.0F : t;
}


uint16_t map_color_rgb565(float input)
{
    float t = input * 2.0F - 1.0F;
    uint16_t red   = clamp(1.5 - fabsf(2.0 * t - 1.0)) * 31;
    uint16_t green = clamp(1.5 - fabsf(2.0 * t)) * 63;
    uint16_t blue  = clamp(1.5 - fabsf(2.0 * t + 1.0)) * 31;

    return red << 11 | green << 5 | blue;
}