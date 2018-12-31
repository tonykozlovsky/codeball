#ifdef LOCAL

#include <H.h>

#else

#include "H.h"

#endif


model::Game H::game;

int H::tick = -1;
model::Action H::actions[3];
int H::global_id;
int H::id;
int H::my_id;
Plan H::best_plan[2];

MyTimer H::t[100];
MyTimer H::global_timer;
int H::player_score[2];
int H::waiting_ticks;
double H::time_limit;
double H::half_time;
double H::sum_bushes_near_the_road = 0;
double H::bushes_near_the_road_k = 0;