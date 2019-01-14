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
  double nitro;
  bool touch;
  Point touch_normal;
  int touch_surface_id;
  int respawn_ticks;
  bool alive;
};

struct Entity {

  struct Collision {
    Entity* e;
    int tick;
  };

  EntityState state;

  MyAction action;

  double arena_e;
  double mass;
  double radius_change_speed;

  int id;
  bool is_teammate;
  bool is_dynamic;
  bool want_to_become_dynamic;
  int want_to_become_dynamic_on_tick;

  EntityState prev_state;
  EntityState prev_micro_state;
  EntityState states[101];
  Collision* collisions = nullptr;
  int collisions_size = 0;

  bool collide_with_ball_in_air;
  bool additional_jump;

  bool is_robot, is_ball, is_pack;

  bool operator<(const Entity& other) const {
    return id < other.id;
  }

  Entity() {}

  void fromPack(const model::NitroPack& _pack) {
    is_pack = true;
    is_ball = is_robot = false;
    state.respawn_ticks = _pack.respawn_ticks;
    state.alive = _pack.alive;
    id = _pack.id;
    state.radius = _pack.radius;
    state.position = {_pack.x, _pack.y, _pack.z};
  }

  void fromBall(const model::Ball& ball) {
    is_ball = true;
    is_pack = is_robot = false;
    state.position = {ball.x, ball.y, ball.z};
    state.velocity = {ball.velocity_x, ball.velocity_y, ball.velocity_z};
    state.radius = ball.radius;
    state.nitro = 0;
    state.touch = false;
    state.touch_normal = {0, 0, 0};

    arena_e = C::rules.BALL_ARENA_E;
    mass = C::rules.BALL_MASS;
    radius_change_speed = 0;

    id = 0;
    is_teammate = false;

    collisions = new Collision[7];

  }

  void fromRobot(const model::Robot& robot) {
    is_robot = true;
    is_pack = is_ball = false;
    state.position = {robot.x, robot.y, robot.z};
    state.velocity = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
    state.radius = robot.radius;
    state.nitro = robot.nitro_amount;
    state.touch = robot.touch;
    state.touch_normal = {robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z};
    state.touch_surface_id = 1; //TODO set right surface ids

    arena_e = C::rules.ROBOT_ARENA_E;
    mass = C::rules.ROBOT_MASS;
    radius_change_speed = 0;

    id = robot.id;
    is_teammate = robot.is_teammate;

    collisions = new Collision[7];

    action = {state.velocity.normalize() * C::rules.ROBOT_MAX_GROUND_SPEED, 0};

  }

  ~Entity() {
    delete[] collisions;
  }

  void saveState(const int tick_number) {
    states[tick_number] = state;
  }

  void savePrevState() {
    prev_state = state;
  }

  void fromPrevState() {
    state = prev_state;
  }

  void savePrevMicroState() {
    prev_micro_state = state;
  }

  void fromPrevMicroState() {
    state = prev_micro_state;
  }

  void fromState(const int tick_number) {
    state = states[tick_number];
  }

  void wantToBecomeDynamic(int tick_number) {
    want_to_become_dynamic = true; // todo min tick
    want_to_become_dynamic_on_tick = tick_number;
    /*for (auto& collision : collisions) { // TODO do and test
      if (collision.tick >= tick_number && !collision.e->want_to_become_dynamic) {
        collision.e->wantToBecomeDynamic(collision.tick);
      }
    }*/
  }

};

#endif //CODEBALL_ENTITY_H
