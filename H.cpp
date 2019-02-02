#ifdef LOCAL

#include <H.h>

#else

#include "H.h"

#endif


model::Game H::game;

int H::cur_round_tick = -1;
int H::tick = -1;
model::Action H::actions[7];
int H::global_id;
int H::my_id;
Plan H::best_plan[6];
Plan H::last_action_plan[6];
Plan H::last_action0_plan[6];

MyTimer H::t[100];
MyTimer H::c[100];
MyTimer H::global_timer;
MyTimer H::cur_tick_timer;
int H::player_score[2];
int H::waiting_ticks;
double H::time_limit;
double H::cur_tick_remaining_time;
double H::min_iterations = 1e9;
double H::max_iterations = 0;
double H::sum_iterations = 0;
double H::iterations_k = 0;
Point H::prev_velocity[7];
Point H::prev_position[7];
int H::danger_grid[60][20][100][C::MAX_SIMULATION_DEPTH];
DGState H::used_cells[1000007];
int H::used_cells_size = 0;

std::map<int, int> H::best_plan_type;


H::ROLE H::role[6];
bool H::flag;

Point2d H::prev_last_action[6];

#ifndef LOCAL
namespace Frozen {

model::Game H::game;

int H::tick = -1;
model::Action H::actions[7];
int H::global_id;
int H::my_id;
Plan H::best_plan[6];
Plan H::last_action_plan[6];

MyTimer H::t[100];
MyTimer H::c[100];
MyTimer H::global_timer;
int H::player_score[2];
int H::waiting_ticks;
double H::time_limit;
double H::half_time;
double H::sum_asserts_failed = 0;
double H::asserts_failed_k = 0;
Point H::prev_velocity[7];
Point H::prev_position[7];
int H::danger_grid[80][20][60][100];
DGState H::used_cells[1000007];
int H::used_cells_size = 0;

Point2d H::prev_last_action[6];

}
#endif