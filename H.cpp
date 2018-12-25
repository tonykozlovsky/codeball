#ifdef LOCAL

#include <H.h>

#else

#include "Helper.h"

#endif


model::Game H::game;

int H::tick = -1;
model::Action H::actions[3];
int H::global_id;
int H::id;
Plan H::best_plan[2];

MyTimer H::t[100];
MyTimer H::global_timer;
int H::player_score[2];
int H::waiting_ticks;
double H::time_limit;
double H::half_time;