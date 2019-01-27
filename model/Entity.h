#ifndef CODEBALL_ENTITY_H
#define CODEBALL_ENTITY_H

#ifdef LOCAL
#include <model/C.h>
#include <model/Plan.h>
#else
#include "C.h"
#include "Plan.h"
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

  bool operator!=(const EntityState& other) const {
    return !(*this == other);
  }
  bool operator==(const EntityState& other) const {
    return
        position == other.position
            && velocity == other.velocity
            && radius == other.radius
            && nitro == other.nitro
            && touch == other.touch
            && touch_normal == other.touch_normal
            && touch_surface_id == other.touch_surface_id
            && respawn_ticks == other.respawn_ticks
            && alive == other.alive;

  }
};

struct StaticEvent {
  bool collide_with_ball;
  void clear() {
    collide_with_ball = false;
  }
};

struct Entity {

  struct Collision {
    Entity* me;
    Entity* e;
    int tick;
  };

  EntityState state;
  EntityState* state_ptr; // only for static for not copying state

  EntityState& getState() { // use outside simulator, if we dont know static or dynamic
    if (is_dynamic) {
      return state;
    } else {
      return *state_ptr;
    }
  }

  MyAction action;
  Plan plan;

  double arena_e;
  double mass;
  double radius_change_speed;

  int id;
  bool is_teammate;
  bool is_dynamic;
  bool want_to_become_dynamic;
  int want_to_become_dynamic_on_tick;

  bool did_not_touch_on_prefix;

  EntityState prev_state;
  EntityState prev_micro_state;
  EntityState states[C::MAX_SIMULATION_DEPTH + 1];
  StaticEvent static_events[C::MAX_SIMULATION_DEPTH + 1];
  StaticEvent* static_event_ptr;

  static constexpr int max_collisions = 14;
  Collision collisions[14];
  int collisions_size = 0;

  double taken_nitro;
  bool collide_with_ball;
  bool collide_with_entity_in_air;
  bool additional_jump;

  bool is_robot, is_ball, is_pack;

  bool operator<(const Entity& other) const {
    return id < other.id;
  }

  Entity() {}

  void addCollision(const Collision& collision) {
    if (collisions_size == max_collisions) {
      return;
    }
    collisions[collisions_size++] = collision;
  }

  void fromPack(const model::NitroPack& _pack) {
    is_pack = true;
    is_ball = is_robot = false;
    state.respawn_ticks = _pack.respawn_ticks;
    state.alive = _pack.alive;
    id = _pack.id;
    state.radius = _pack.radius;
    state.position = {_pack.x, _pack.y, _pack.z};
    collisions_size = 0;
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
    state.alive = true;

    arena_e = C::rules.BALL_ARENA_E;
    mass = C::rules.BALL_MASS;
    radius_change_speed = 0;

    id = 0;
    is_teammate = false;

    collisions_size = 0;

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
    state.alive = true;

    arena_e = C::rules.ROBOT_ARENA_E;
    mass = C::rules.ROBOT_MASS;
    radius_change_speed = 0;

    id = robot.id;
    is_teammate = robot.is_teammate;

    collisions_size = 0;

    action = {{0, 0, 0}, 0, 0, false};

    did_not_touch_on_prefix = !robot.touch;

  }

  ~Entity() {
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

  void fromStateStatic(const int tick_number) {
    state_ptr = states + tick_number;
    static_event_ptr = static_events + tick_number;
  }

  void fromState(const int tick_number) {
    state = states[tick_number];
  }

  void wantToBecomeDynamic(int tick_number) {
    want_to_become_dynamic = true;
    want_to_become_dynamic_on_tick = tick_number;
    for (int i = 0; i < collisions_size; ++i) {
      if (collisions[i].tick >= tick_number && (!collisions[i].e->want_to_become_dynamic
          || collisions[i].e->want_to_become_dynamic_on_tick > tick_number)) {
        collisions[i].e->wantToBecomeDynamic(collisions[i].tick);
      }
    }
  }

  void nitroCheck() {
    if (!action.use_nitro) {
      return;
    }
    if (state.touch) { //work for dynamic and for initial static
      action.use_nitro = false;
      return;
    }
    if (state.nitro == 0) {
      action.use_nitro = false;
      return;
    }
    bool restrict_nitro = false;
    const auto& target_velocity_change = (action.target_velocity - state.velocity);
    const auto& tvc_length_sq = target_velocity_change.length_sq();
    if (tvc_length_sq > 0) {
      const auto& max_nitro_change = state.nitro * C::rules.NITRO_POINT_VELOCITY_CHANGE;
      const auto& ac_per_dt = C::rules.ROBOT_NITRO_ACCELERATION / 60. * C::TPT;
      if (max_nitro_change < ac_per_dt && tvc_length_sq > max_nitro_change * max_nitro_change) {
        restrict_nitro = true;
      } else if (ac_per_dt * ac_per_dt > tvc_length_sq) {
        restrict_nitro = true;
      }
    }
    if (restrict_nitro) {
      action.use_nitro = false;
    }
  }
};

#endif //CODEBALL_ENTITY_H
