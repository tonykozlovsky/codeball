#include <Helper.h>

model::Game Helper::game;
model::Arena Helper::arena;

int Helper::tick = -1;
std::map<int, model::Action> Helper::actions;
int Helper::current_id;

MyTimer Helper::t[100];