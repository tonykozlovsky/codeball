#ifdef LOCAL
#include <model/C.h>
#else
#include "C.h"
#endif

model::Rules C::rules;
std::mt19937_64 C::rd;
int C::unique_plan_id = 1;