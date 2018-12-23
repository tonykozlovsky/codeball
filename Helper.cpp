#ifdef LOCAL

#include <Helper.h>

#else

#include "Helper.h"

#endif


model::Game Helper::game;

int Helper::tick = -1;
model::Action Helper::actions[3];
int Helper::global_id;
int Helper::id;
Plan Helper::best_plan[2];

MyTimer Helper::t[100];