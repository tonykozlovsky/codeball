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
MyTimer Helper::global_timer;
int Helper::player_score[2];
int Helper::waiting_ticks;
double Helper::time_limit;
double Helper::half_time;