#ifndef CODEBALL_ENTITY_H
#define CODEBALL_ENTITY_H


#ifdef LOCAL
#include <model/C.h>
#else
#include "C.h"
#endif

struct EntityState {
  Point position;
  Point velocity;
  double radius;
  bool touch;
  Point touch_normal;
};

struct Entity {
  EntityState state;

  MyAction action;

  double arena_e;
  double mass;
  double radius_change_speed;

  int id;
  bool is_teammate;
  bool is_dynamic;

  std::vector<EntityState> states;

  bool operator <(const Entity& other) const {
    return id < other.id;
  }

  Entity() {}

  Entity(const model::Ball& ball) {
    state.position = {ball.x, ball.y, ball.z};
    state.velocity = {ball.velocity_x, ball.velocity_y, ball.velocity_z};
    state.radius = ball.radius;
    state.touch = false;
    state.touch_normal = {0, 0, 0};

    arena_e = C::rules.BALL_ARENA_E;
    mass = C::rules.BALL_MASS;
    radius_change_speed = 0;

    id = 0;
    is_teammate = false;
  }

  Entity(const model::Robot& robot) {
    state.position = {robot.x, robot.y, robot.z};
    state.velocity = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
    state.radius = robot.radius;
    state.touch = robot.touch;
    state.touch_normal = {robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z};

    arena_e = C::rules.ROBOT_ARENA_E;
    mass = C::rules.ROBOT_MASS;
    radius_change_speed = 0;

    id = robot.id;
    is_teammate = robot.is_teammate;
  }

  void saveState() {
    states.push_back(state);
  }

  void fromState(const int tick_number) {
    state = states[tick_number];
  }

};

#endif //CODEBALL_ENTITY_H
