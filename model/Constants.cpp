#ifdef LOCAL
#include <model/Constants.h>
#else
#include "Constants.h"
#endif

model::Rules Constants::rules;
std::mt19937_64 Constants::rd;