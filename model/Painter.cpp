#ifdef LOCAL
#include <model/Painter.h>
#else
#include "Painter.h"
#endif

std::vector<Painter::Line> Painter::lines_to_draw;
std::vector<Painter::Sphere> Painter::spheres_to_draw;