#ifdef LOCAL
#include <model/P.h>
#else
#include "Painter.h"
#endif

std::vector<P::Line> P::lines_to_draw;
std::vector<P::Sphere> P::spheres_to_draw;
std::vector<std::string>  P::logs;