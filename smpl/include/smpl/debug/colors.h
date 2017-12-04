#ifndef SMPL_COLORS_H
#define SMPL_COLORS_H

#include <smpl/debug/marker.h>

namespace sbpl {
namespace visual {

Color MakeColorHSV(float h, float s = 1.0, float v = 1.0, float a = 1.0);

void hsv_to_rgb(float* r, float* g, float* b, float h, float s, float v);
void rgb_to_hsv(float* h, float* s, float* v, float r, float g, float b);

void hsv_to_rgb(double* r, double* g, double* b, double h, double s, double v);
void rgb_to_hsv(double* h, double* s, double* v, double r, double g, double b);

} // namespace visual
} // namespace sbpl

#endif
