#ifdef LOCAL

#include <H.h>

#else

#include "H.h"

#endif


model::Game H::game;

int H::tick = -1;
model::Action H::actions[7];
int H::global_id;
int H::my_id;
Plan H::best_plan[4];

MyTimer H::t[100];
MyTimer H::global_timer;
int H::player_score[2];
int H::waiting_ticks;
double H::time_limit;
double H::half_time;
double H::sum_asserts_failed = 0;
double H::asserts_failed_k = 0;
int H::danger_grid[80][20][60][50];
DGState H::used_cells[1000007];
int H::used_cells_size = 0;