#ifdef LOCAL
#include <model/C.h>
#else
#include "Constants.h"
#endif

model::Rules C::rules;
std::mt19937_64 C::rd;