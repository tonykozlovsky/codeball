*** SmartSimulator.h ***
---
#ifndef CODEBALL_SMARTSIMULATOR_H
#define CODEBALL_SMARTSIMULATOR_H

#ifdef LOCAL
#include <model/Entity.h>
#include <model/P.h>
#include <model/Dan.h>
#include <H.h>
#else
#include "model/Entity.h"
#include "model/P.h"
#include "model/Dan.h"
#include "H.h"
#endif

struct SmartSimulator {

  struct GoalInfo {
    bool goal_to_me;
    bool goal_to_enemy;
    int goal_tick;

    void operator|=(const GoalInfo& other) {
      goal_to_me |= other.goal_to_me;
      goal_to_enemy |= other.goal_to_enemy;
    }

  } goal_info;

  Entity initial_static_entities[11];
  int initial_static_entities_size = 0;

  Entity initial_dynamic_entities[11];
  int initial_dynamic_entities_size = 0;

  Entity* initial_static_robots[6];
  int initial_static_robots_size = 0;

  Entity* initial_dynamic_robots[6];
  int initial_dynamic_robots_size = 0;

  Entity* initial_static_packs[4];
  int initial_static_packs_size = 0;

  Entity* static_entities[11];
  int static_entities_size = 0;

  Entity* dynamic_entities[11];
  int dynamic_entities_size = 0;

  Entity* static_robots[6];
  int static_robots_size = 0;

  Entity* dynamic_robots[6];
  int dynamic_robots_size = 0;

  Entity* static_packs[4];
  int static_packs_size = 0;

  Entity* dynamic_packs[4];
  int dynamic_packs_size = 0;

  Entity* main_robot;
  Entity* ball;

  bool acceleration_trigger;
  int acceleration_trigger_fires;
  const int acceleration_trigger_limit = 2;

  bool entity_entity_collision_trigger;
  int entity_entity_collision_trigger_fires;
  const int entity_entity_collision_limit = 2;

  bool entity_ball_collision_trigger;
  int entity_ball_collision_trigger_fires;
  const int entity_ball_collision_limit = 2;

  bool entity_arena_collision_trigger;
  int entity_arena_collision_trigger_fires;
  const int entity_arena_collision_limit = 2;

  bool ball_arena_collision_trigger;
  int ball_arena_collision_trigger_fires;
  const int ball_arena_collision_limit = 2;

  int simulation_depth, tpt;

  bool accurate, unaccurate;

  double hit_e = (C::rules.MIN_HIT_E + C::rules.MAX_HIT_E) / 2;
  // double hit_e = C::rules.MAX_HIT_E;

  bool collided_entities[7][7];

  bool static_goal_to_me;

  // maybe we can have 4x-5x performance boost, and more when 3x3
  SmartSimulator(
      const bool unaccurate,
      const int tpt,
      const int simulation_depth,
      const int main_robot_id,
      const int plans_configuration,
      const std::vector<model::Robot>& _robots,
      const model::Ball& _ball,
      const std::vector<model::NitroPack>& _packs,
      bool accurate = false,
      int viz_id = -1)
      : unaccurate(unaccurate), tpt(tpt), simulation_depth(simulation_depth), accurate(accurate) {

    initial_static_entities[initial_static_entities_size].fromBall(_ball);
    ball = &initial_static_entities[initial_static_entities_size++];
    ball->is_dynamic = false;

    for (auto& robot : _robots) {
      if (robot.id == main_robot_id) {
        initial_dynamic_entities[initial_dynamic_entities_size].fromRobot(robot);
        auto new_robot = &initial_dynamic_entities[initial_dynamic_entities_size++];
        new_robot->is_dynamic = true;
        main_robot = new_robot;
        initial_dynamic_robots[initial_dynamic_robots_size++] = new_robot;
        new_robot->saveState(0);
      } else {
        if (unaccurate) { // todo check if need my guys maybe no
          continue;
        }
        initial_static_entities[initial_static_entities_size].fromRobot(robot);
        auto new_robot = &initial_static_entities[initial_static_entities_size++];
        new_robot->is_dynamic = false;
        initial_static_robots[initial_static_robots_size++] = new_robot;
      }
    }

    for (auto& pack : _packs) {
      if (unaccurate) {
        continue;
      }
      initial_static_entities[initial_static_entities_size].fromPack(pack);
      auto new_pack = &initial_static_entities[initial_static_entities_size++];
      new_pack->is_dynamic = false;
      initial_static_packs[initial_static_packs_size++] = new_pack;
    }

    //todo check what better last_action nothing, best action and for my or enemy
    if (plans_configuration == 1) { // best action all for minimax
      for (int i = 0; i < initial_static_robots_size; ++i) {
        initial_static_robots[i]->plan = H::best_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
      }
    } else if (plans_configuration == 2) { // best action my, last action 15 enemy
      for (int i = 0; i < initial_static_robots_size; ++i) {
        if (initial_static_robots[i]->is_teammate) {
          initial_static_robots[i]->plan = H::best_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
        } else {
          initial_static_robots[i]->plan = H::last_action_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
        }
      }
    } else if (plans_configuration == 3) { // last action all for minimax
      for (int i = 0; i < initial_static_robots_size; ++i) {
        initial_static_robots[i]->plan = H::last_action_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
      }
    } if (plans_configuration == 7) { // best action my, last action 0 enemy
      for (int i = 0; i < initial_static_robots_size; ++i) {
        if (initial_static_robots[i]->is_teammate) {
          initial_static_robots[i]->plan = H::best_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
        } else {
          initial_static_robots[i]->plan = H::last_action0_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
        }
      }
    }

    static_goal_to_me = false;

    for (int sim_tick = 0; sim_tick < simulation_depth + 1; ++sim_tick) {
      tickWithJumpsStatic(sim_tick, true);
    }

#ifdef DEBUG
    if (main_robot_id == viz_id) {
      for (int i = 0; i < initial_static_entities_size; ++i) {
        auto& e = initial_static_entities[i];
        for (int j = 1; j < simulation_depth; ++j) {
          if (!e.states[j].alive) {
            break;
          }
          P::drawLine(e.states[j - 1].position, e.states[j].position, accurate ? 0xFFFFFF : 0x000000);
        }
      }
    }
#endif

    // init
    // calculate static trajectories and build collision-time dependencies tree
    // ? fair 100 microtick calculation of maybe ball only, maybe on sphere collisions ?
    // add main to dynamic
    // start simulation
    // if any static want to become a dynamic - do it
    // simulate tick
    // if any dynamic has a collision with static, add this static to dynamic
    // and any son of this static from subtree now want to become a dynamic
    // on collision tick with its parent
    //
  }

  void clearAdditionalJumpsStatic() {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      initial_static_robots[i]->collide_with_entity_in_air = false;
      initial_static_robots[i]->collide_with_ball = false;
      initial_static_robots[i]->additional_jump = false;
      initial_static_robots[i]->taken_nitro = 0;
    }
  }

  bool somebodyJumpThisTickStatic() {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      if (initial_static_robots[i]->action.jump_speed > 0) {
        return true;
      }
    }
    return false;
  }

  inline void clearTriggers() {
    acceleration_trigger = false;
    entity_entity_collision_trigger = false;
    entity_ball_collision_trigger = false;
    entity_arena_collision_trigger = false;
    ball_arena_collision_trigger = false;
  }

  inline void clearTriggerFires() {
    acceleration_trigger_fires = 0;
    entity_entity_collision_trigger_fires = 0;
    entity_ball_collision_trigger_fires = 0;
    entity_arena_collision_trigger_fires = 0;
    ball_arena_collision_trigger_fires = 0;
  }

  inline void tickMicroticksStatic(const int& number_of_tick, const int& number_of_microticks) {
    clearTriggers();
    if (number_of_microticks == 0) {
      return;
    }
    updateStatic((double) number_of_microticks / C::rules.TICKS_PER_SECOND / C::rules.MICROTICKS_PER_TICK, number_of_tick, number_of_microticks);
  }

  void clearCollisionsAndStaticEvents(const int tick) {
    for (int i = 0; i < 7; ++i) {
      for (int j = 0; j < 7; ++j) {
        collided_entities[i][j] = false;
      }
    }
    for (int i = 0; i < initial_static_entities_size; ++i) {
      initial_static_entities[i].static_events[tick + 1].clear();
    }
  }

  void tickWithJumpsStatic(const int tick_number, bool with_jumps) {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& robot = initial_static_robots[i];
      if (!robot->state.alive) {
        continue;
      }
      if (!unaccurate && !robot->is_teammate) {
        if (robot->did_not_touch_on_prefix && robot->state.touch && robot->state.touch_surface_id == 1) {
          robot->did_not_touch_on_prefix = false;
        }
        if (tick_number > C::ENEMY_LIVE_TICKS && !robot->did_not_touch_on_prefix) {
          robot->state.alive = false;
        }
      }
    }
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& robot = initial_static_robots[i];
      if (!robot->state.alive) {
        continue;
      }
      if (!robot->state.touch) {
        robot->action = robot->plan.toMyAction(tick_number, true, true, robot->state.position, robot->state.velocity);
        robot->nitroCheck();
        if (!robot->action.use_nitro) {
          robot->action = robot->plan.toMyAction(tick_number, true, false, robot->state.position, robot->state.velocity);
        }
      } else {
        robot->action = robot->plan.toMyAction(tick_number, true, false, robot->state.position, robot->state.velocity);
      }
    }
    for (int i = 0; i < initial_static_entities_size; ++i) { // save state
      auto& e = initial_static_entities[i];
      e.saveState(tick_number);
    }
    if (with_jumps) {
      clearAdditionalJumpsStatic();
    }
    clearCollisionsAndStaticEvents(tick_number);
    tickStatic(tick_number);
    if (with_jumps) {
      bool needs_rollback = false;
      for (int i = 0; i < initial_static_robots_size; ++i) {
        auto& e = initial_static_robots[i];
        if (!e->state.alive) {
          continue;
        }
        if (e->collide_with_entity_in_air || e->collide_with_ball || e->additional_jump) {
          needs_rollback = true;
          e->action.jump_speed = e->additional_jump ? std::max(C::MIN_WALL_JUMP, e->action.max_jump_speed) : e->action.max_jump_speed;
        }
      }
      if (needs_rollback) {
        for (int i = 0; i < initial_static_entities_size; ++i) {
          auto& e = initial_static_entities[i];
          if (!e.state.alive) {
            continue;
          }
          e.fromState(tick_number);
        }
        clearCollisionsAndStaticEvents(tick_number);
        tickStatic(tick_number);
      }
    }

    for (int i = 0; i < initial_static_packs_size; ++i) {
      auto& pack = initial_static_packs[i];
      if (pack->state.alive) {
        continue;
      }
      pack->state.respawn_ticks--;
      if (pack->state.respawn_ticks == 0) {
        pack->state.alive = true;
      }
    }

    for (int i = 0; i < 7; ++i) {
      for (int j = 0; j < 7; ++j) {
        if (collided_entities[i][j]) {
          Entity* e = initialStaticEntityById(i);
          Entity* me = initialStaticEntityById(j);
          if (e && me) {
            me->addCollision({me, e, tick_number});
            e->addCollision({e, me, tick_number});
          }
        }
      }
    }

    if (ball->state.position.z < -42) {
      static_goal_to_me = true;
    }
  }

  Entity* initialStaticEntityById(const int id) {
    for (int i = 0; i < initial_static_entities_size; ++i) {
      if (initial_static_entities[i].id == id) {
        return initial_static_entities + i;
      }
    }
    return 0;
  }

  inline bool anyTriggersActive() {
    return
        (acceleration_trigger && acceleration_trigger_fires < acceleration_trigger_limit) ||
            (entity_arena_collision_trigger && entity_arena_collision_trigger_fires < entity_arena_collision_limit) ||
            (ball_arena_collision_trigger && ball_arena_collision_trigger_fires < ball_arena_collision_limit) ||
            (entity_ball_collision_trigger && entity_ball_collision_trigger_fires < entity_ball_collision_limit) ||
            (entity_entity_collision_trigger && entity_entity_collision_trigger_fires < entity_entity_collision_limit);

  }

  void setTriggersFired() {
    if (acceleration_trigger) {
      acceleration_trigger_fires++;
    }
    if (entity_arena_collision_trigger) {
      entity_arena_collision_trigger_fires++;
    }

    if (ball_arena_collision_trigger) {
      ball_arena_collision_trigger_fires++;
    }
    if (entity_entity_collision_trigger) {
      entity_entity_collision_trigger_fires++;
    }
    if (entity_ball_collision_trigger) {
      entity_ball_collision_trigger_fires++;
    }
  }

  inline void saveMicrostatesStatic() {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& e = initial_static_robots[i];
      if (!e->state.alive) {
        continue;
      }
      e->savePrevMicroState();
    }
    ball->savePrevMicroState();
  }

  void fromMictostatesStatic() {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& e = initial_static_robots[i];
      if (!e->state.alive) {
        continue;
      }
      e->fromPrevMicroState();
    }
    ball->fromPrevMicroState();
  }

  void tickStatic(const int tick_number) {
    if (accurate) {
      for (int i = 0; i < 100 * tpt; ++i) {
        tickMicroticksStatic(tick_number, 1);
      }
      return;
    } else if (unaccurate) {
      tickMicroticksStatic(tick_number, 100 * tpt);
      return;
    }
    int remaining_microticks = 100 * tpt;
    if (somebodyJumpThisTickStatic()) {
      tickMicroticksStatic(tick_number, 1);
      remaining_microticks--;
      tickMicroticksStatic(tick_number, 1);
      remaining_microticks--;
    } else if (tick_number == 0) {
      tickMicroticksStatic(tick_number, 1);
      remaining_microticks--;
    }

    clearTriggerFires();

    while (true) {

      saveMicrostatesStatic();
      tickMicroticksStatic(tick_number, remaining_microticks);

      if (anyTriggersActive() && remaining_microticks > 1) { // todo as dynamic
        int l = 0;
        int r = remaining_microticks;
        while (r - l > 1) {
          int mid = (r + l) / 2;
          fromMictostatesStatic();
          tickMicroticksStatic(tick_number, mid);
          if (anyTriggersActive()) {
            r = mid;
          } else {
            l = mid;
          }
        }
        fromMictostatesStatic();
        if (l > 0) {
          tickMicroticksStatic(tick_number, l);
          remaining_microticks -= l;
        }
        tickMicroticksStatic(tick_number, 1);
        setTriggersFired();
        remaining_microticks--;
      } else {
        break;
      }
    }
  }

  bool collideEntitiesStatic(const int tick_number, Entity* a, Entity* b, bool check_with_ball) {
    const Point& delta_position = b->state.position - a->state.position;
    const double distance_sq = delta_position.length_sq();
    const double sum_r = a->state.radius + b->state.radius;
    if (check_with_ball) {
      if ((3 + jr * a->action.max_jump_speed) * (3 + jr * a->action.max_jump_speed) > distance_sq) {
        a->collide_with_ball = true;
        a->static_events[tick_number + 1].collide_with_ball = true;
      }
    } else {
      if (a->is_teammate && !b->state.touch && (2 + jr * a->action.max_jump_speed) * (2 + jr * a->action.max_jump_speed) > distance_sq) {
        a->collide_with_entity_in_air = true;
      } else if (b->is_teammate && !a->state.touch && (2 + jr * b->action.max_jump_speed) * (2 + jr * b->action.max_jump_speed) > distance_sq) {
        b->collide_with_entity_in_air = true;
      }
    }
    if (sum_r * sum_r > distance_sq) {
      collided_entities[a->id][b->id] = true;
      const double penetration = sum_r - sqrt(distance_sq);
      if (check_with_ball) {
        entity_ball_collision_trigger = true;
      } else if (!a->state.touch && !b->state.touch) {
        entity_entity_collision_trigger = true;
      }
      const double k_a = 1. / (a->mass * ((1 / a->mass) + (1 / b->mass)));
      const double k_b = 1. / (b->mass * ((1 / a->mass) + (1 / b->mass)));
      const Point& normal = delta_position.normalize();
      a->state.position -= normal * (penetration * k_a);
      b->state.position += normal * (penetration * k_b);
      const double delta_velocity = (b->state.velocity - a->state.velocity).dot(normal) - (b->radius_change_speed + a->radius_change_speed);
      if (delta_velocity < 0) {
        const Point& impulse = normal * ((1. + hit_e) * delta_velocity);
        a->state.velocity += impulse * k_a;
        b->state.velocity -= impulse * k_b;
        return true;
      }
    }
    return false;
  }

  bool collideWithArenaStatic(Entity* e, Point& result, int& collision_surface_id) {
    const Dan& dan = Dan::dan_to_arena(e->state.position, e->state.radius);
    const double distance = dan.distance;
    if (e->state.radius > distance) {
      const Point& normal = dan.normal.normalize();
      const double penetration = e->state.radius - distance;
      e->state.position += normal * penetration;
      const double velocity = e->state.velocity.dot(normal) - e->radius_change_speed;
      if (velocity < 0) {
        e->state.velocity -= normal * ((1. + e->arena_e) * velocity);
        result = normal;
        collision_surface_id = dan.collision_surface_id;
        return true;
      }
    }
    return false;
  }

  void moveStatic(Entity* e, const double delta_time) {
    e->state.velocity = e->state.velocity.clamp(C::rules.MAX_ENTITY_SPEED);
    e->state.position += e->state.velocity * delta_time;
    e->state.position.y -= C::rules.GRAVITY * delta_time * delta_time / 2;
    e->state.velocity.y -= C::rules.GRAVITY * delta_time;
  }

  static constexpr double jr = 0.0033333333333333333333333333333;

  inline bool updateDynamic(const double& delta_time, const int& number_of_tick, const int& number_of_microticks, GoalInfo& cur_goal_info) {
    bool has_collision_with_static = false;
    cur_goal_info = {false, false, -1};
    for (int i = 0; i < dynamic_robots_size; ++i) { // 1/4 time !
      auto& robot = dynamic_robots[i];
      if (robot->state.touch) {
        const Point& target_velocity = (robot->state.touch_surface_id == 1) ? robot->action.target_velocity :
            robot->action.target_velocity - robot->state.touch_normal * robot->state.touch_normal.dot(robot->action.target_velocity);
        const Point& target_velocity_change = target_velocity - robot->state.velocity;
        double length = target_velocity_change.length_sq();
        if (length > 0) {
          const double& acceleration = (robot->state.touch_normal.y > 0) ? robot->state.touch_normal.y * 100 : 0;
          length = sqrt(length);
          const double& delta = length - acceleration * delta_time;
          if (delta > 0) {
            const auto& robot_acceleration = target_velocity_change * (acceleration * delta_time / length);
            robot->state.velocity += robot_acceleration;
            const double& coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / (2. * number_of_microticks)) : 0.; // todo optimise ?
            robot->state.position -= robot_acceleration * (coef * delta_time);
          } else {
            if (robot->state.touch_surface_id == 1 && robot->is_teammate) {
              if (robot->accelerate_trigger_on_prev_tick == false) {
                acceleration_trigger = true;
              }
              robot->accelerate_trigger_on_cur_tick = true;
            }
            robot->state.velocity += target_velocity_change;
          }
        }
      } else {
        if (robot->action.use_nitro && robot->state.nitro > 0) {
          const auto& target_velocity_change = (robot->action.target_velocity - robot->state.velocity);
          const auto& tvc_length_sq = target_velocity_change.length_sq();
          if (tvc_length_sq > 0) {
            const auto& ac_per_dt = C::rules.ROBOT_NITRO_ACCELERATION * delta_time;
            const auto& robot_acceleration = target_velocity_change * (ac_per_dt / sqrt(tvc_length_sq));
            robot->state.velocity += robot_acceleration;
            robot->state.nitro -= ac_per_dt / C::rules.NITRO_POINT_VELOCITY_CHANGE;
            const double& coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / 2. / number_of_microticks) : 0.;
            robot->state.position -= robot_acceleration * (coef * delta_time);
          }
        }
      }
      moveDynamic(robot, delta_time);
      robot->state.radius = 1. + jr * robot->action.jump_speed;
      robot->radius_change_speed = robot->action.jump_speed;
    }

    if (ball->is_dynamic) {
      moveDynamic(ball, delta_time);
    }

    for (int i = 0; i < dynamic_robots_size; i++) { // 1/4 time !
      for (int j = 0; j < static_robots_size; j++) {
        if (collideEntitiesCheckDynamic(static_robots[j], dynamic_robots[i])) {
          static_robots[j]->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
        }
      }
    }

    for (int i = 0; i < dynamic_robots_size; i++) {
      for (int j = 0; j < i; j++) {
        collideEntitiesDynamic(number_of_tick, number_of_microticks, dynamic_robots[i], dynamic_robots[j], false);
      }
    }

    Point collision_normal;
    int touch_surface_id;

    for (int i = 0; i < dynamic_robots_size; i++) { // 1/2 time !
      auto& robot = dynamic_robots[i];
      if (ball->is_dynamic) {
        collideEntitiesDynamic(number_of_tick, number_of_microticks, robot, ball, true);
      } else {
        if (collideEntitiesCheckDynamic(ball, robot)) {
          ball->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
        }
      }
      if (!collideWithArenaDynamic(robot, collision_normal, touch_surface_id)) {
        if (robot->is_teammate && robot->state.touch) {
          //H::c[7].call();
          entity_arena_collision_trigger = true;
        }
        robot->state.touch = false;
      } else {
        if (!robot->state.touch || robot->state.touch_surface_id != touch_surface_id) {
          entity_arena_collision_trigger = true;
        }
        if (robot->is_teammate && touch_surface_id != 1) {
          robot->additional_jump = true;
        }
        robot->state.touch_surface_id = touch_surface_id;
        robot->state.touch = true;
        robot->state.touch_normal = collision_normal;
      }
    }

    if (ball->is_dynamic) {
      for (int i = 0; i < static_robots_size; ++i) {
        if (collideEntitiesCheckDynamic(static_robots[i], ball)) {
          static_robots[i]->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
        }
      }
    }

    for (int i = 0; i < dynamic_robots_size; ++i) {
      const auto& robot = dynamic_robots[i];
      if (robot->is_teammate && !(robot->state.nitro < C::rules.MAX_NITRO_AMOUNT)) {
        continue;
      }
      const double& x = robot->state.position.x > 0 ? robot->state.position.x : -robot->state.position.x;
      const double& y = robot->state.position.y;
      const double& z = robot->state.position.z > 0 ? robot->state.position.z : -robot->state.position.z;
      const double& r = robot->state.radius;
      if (y > 1.5 + r || x > 20.5 + r || x < 19.5 - r || z > 30.5 + r || z < 29.5 - r) {
        continue;
      }
      for (int j = 0; j < static_packs_size; ++j) {
        const auto& pack = static_packs[j];
        if (!pack->state_ptr->alive) {
          continue;
        }
        if (collideEntitiesCheckDynamic(pack, robot)) {
          pack->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
          break;
        }
      }
    }

    if (has_collision_with_static) {
      return true;
    }

    if (ball->is_dynamic) {
      if (!collideWithArenaDynamic(ball, collision_normal, touch_surface_id)) {
        if (ball->state.touch) {
          if (ball->state.touch_surface_id != 1 || ball->state.velocity.y > C::ball_antiflap) {
            //H::c[9].call();
            ball_arena_collision_trigger = true;
            ball->state.touch = false;
          }
        }
      } else {
        if (!ball->state.touch || ball->state.touch_surface_id != touch_surface_id) {
          //H::c[10].call();
          ball_arena_collision_trigger = true;
        }
        ball->state.touch_surface_id = touch_surface_id;
        ball->state.touch = true;
      }
    }

    for (int i = 0; i < dynamic_robots_size; i++) {
      auto& robot = dynamic_robots[i];
      if (robot->state.nitro == C::rules.MAX_NITRO_AMOUNT) {
        continue;
      }
      const double& x = robot->state.position.x > 0 ? robot->state.position.x : -robot->state.position.x;
      const double& y = robot->state.position.y;
      const double& z = robot->state.position.z > 0 ? robot->state.position.z : -robot->state.position.z;
      const double& r = robot->state.radius;
      if (y > 1.5 + r || x > 20.5 + r || x < 19.5 - r || z > 30.5 + r || z < 29.5 - r) {
        continue;
      }
      for (int j = 0; j < dynamic_packs_size; ++j) {
        auto& pack = dynamic_packs[j];
        if (!pack->state.alive) {
          continue;
        }
        const double& sum_r = robot->state.radius + pack->state.radius;
        if ((robot->state.position - pack->state.position).length_sq() <= sum_r * sum_r) {
          robot->taken_nitro = C::rules.MAX_NITRO_AMOUNT - robot->state.nitro;
          robot->state.nitro = C::rules.MAX_NITRO_AMOUNT;
          pack->state.alive = false;
          pack->state.respawn_ticks = C::rules.NITRO_PACK_RESPAWN_TICKS;
        }
      }
    }

    if (ball->is_dynamic) {
      if (ball->state.position.z > 42) {
        cur_goal_info.goal_to_enemy = true;
      } else if (ball->state.position.z < -42) {
        cur_goal_info.goal_to_me = true;
      }
    } else {
      if (ball->state_ptr->position.z > 42 || ball->state_ptr->position.z < -42) {
        ball->wantToBecomeDynamic(number_of_tick);
        has_collision_with_static = true;
      }
    }

    return has_collision_with_static;
  }

  void initIteration(const int iteration, const Plan& main_robot_plan) {
    goal_info.goal_to_me = false;
    goal_info.goal_to_enemy = false;

    main_robot->plan = main_robot_plan;

    static_entities_size = 0;
    for (int i = 0; i < initial_static_entities_size; ++i) {
      static_entities[static_entities_size++] = &initial_static_entities[i];
    }

    dynamic_entities_size = 0;
    for (int i = 0; i < initial_dynamic_entities_size; ++i) {
      dynamic_entities[dynamic_entities_size++] = &initial_dynamic_entities[i];
    }

    static_robots_size = 0;
    for (int i = 0; i < initial_static_robots_size; ++i) {
      static_robots[static_robots_size++] = initial_static_robots[i];
    }

    dynamic_robots_size = 0;
    for (int i = 0; i < initial_dynamic_robots_size; ++i) {
      dynamic_robots[dynamic_robots_size++] = initial_dynamic_robots[i];
    }

    static_packs_size = 0;
    for (int i = 0; i < initial_static_packs_size; ++i) {
      static_packs[static_packs_size++] = initial_static_packs[i];
    }

    dynamic_packs_size = 0;

    for (int i = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      e->is_dynamic = false;
      e->want_to_become_dynamic = false;
    }
    for (int i = 0; i < dynamic_entities_size; ++i) {
      auto& e = dynamic_entities[i];
      e->is_dynamic = true;
    }
    main_robot->fromState(0);
  }

  inline void wantedStaticGoToDynamic(const int& tick_number) {
    bool smth_chandes = false;
    for (int i = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      if (e->want_to_become_dynamic && e->want_to_become_dynamic_on_tick == tick_number) {
        smth_chandes = true;
        e->fromState(tick_number);
        e->is_dynamic = true;
        dynamic_entities[dynamic_entities_size++] = e;
        if (e->is_robot) {
          dynamic_robots[dynamic_robots_size++] = e;
        } else if (e->is_pack) {
          dynamic_packs[dynamic_packs_size++] = e;
        }
      }
    }
    if (smth_chandes) {
      static_packs_size = 0;
      static_robots_size = 0;
      int new_static_entities_size = 0;
      for (int i = 0; i < static_entities_size; ++i) {
        auto& e = static_entities[i];
        if (!e->want_to_become_dynamic || e->want_to_become_dynamic_on_tick != tick_number) {
          static_entities[new_static_entities_size++] = e;
          if (e->is_robot) {
            static_robots[static_robots_size++] = e;
          } else if (e->is_pack) {
            static_packs[static_packs_size++] = e;
          }
        }
      }
      static_entities_size = new_static_entities_size;
    }
  }

  inline void clearCollideWithBallInAirDynamic() {
    for (int i = 0; i < dynamic_robots_size; ++i) {
      dynamic_robots[i]->collide_with_entity_in_air = false;
      dynamic_robots[i]->collide_with_ball = false;
      dynamic_robots[i]->additional_jump = false;
      dynamic_robots[i]->taken_nitro = 0;
      dynamic_robots[i]->accelerate_trigger_on_cur_tick = false;
    }
  }

  inline bool checkCollideWithBallInAirDynamic() {
    for (int i = 0; i < dynamic_robots_size; ++i) {
      if (dynamic_robots[i]->collide_with_entity_in_air ||
          dynamic_robots[i]->collide_with_ball ||
          dynamic_robots[i]->additional_jump) {
        return true;
      }
    }
    return false;
  }

  inline bool tryTickWithJumpsDynamic(const int& tick_number, int& main_robot_additional_jump_type, GoalInfo& cur_goal_info) {
    //H::c[0].call();
    clearCollideWithBallInAirDynamic();
    main_robot_additional_jump_type = 0;
    bool sbd_become_dynamic = tickDihaDynamic(tick_number, cur_goal_info); // todo check need return here
    bool needs_rollback = false;
    for (int i = 0; i < dynamic_robots_size; ++i) {
      const auto& e = dynamic_robots[i];
      if (e->collide_with_entity_in_air || e->collide_with_ball || e->additional_jump) {
        needs_rollback = true;
        e->action.jump_speed = e->additional_jump ? std::max(C::MIN_WALL_JUMP, e->action.max_jump_speed) : e->action.max_jump_speed;
        if (e == main_robot) {
          if (e->collide_with_ball) {
            main_robot_additional_jump_type = 1;
          } else if (e->collide_with_entity_in_air) {
            main_robot_additional_jump_type = 2;
          } else if (e->additional_jump) {
            main_robot_additional_jump_type = 3;
          }
        }
      }
    }
    if (needs_rollback) {
      clearCollideWithBallInAirDynamic();
      for (int i = 0; i < dynamic_entities_size; ++i) {
        dynamic_entities[i]->fromPrevState();
      }
      sbd_become_dynamic = tickDihaDynamic(tick_number, cur_goal_info, true);
    }
    return sbd_become_dynamic;
  }

  inline bool tryDoTickWithoutAnybodyBecomingDynamic(const int tick_number, int& main_robot_additional_jump_type, GoalInfo& cur_goal_info) {
    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& robot = dynamic_robots[i];
      if (!robot->state.touch) {
        robot->action = robot->plan.toMyAction(tick_number, true, true, robot->state.position, robot->state.velocity);
        robot->nitroCheck();
        if (!robot->action.use_nitro) {
          robot->action = robot->plan.toMyAction(tick_number, true, false, robot->state.position, robot->state.velocity);
        }
      } else {
        robot->action = robot->plan.toMyAction(tick_number, true, false, robot->state.position, robot->state.velocity);
      }
    }

    const bool& flag = tryTickWithJumpsDynamic(tick_number, main_robot_additional_jump_type, cur_goal_info);
    return flag;
  }

  inline bool somebodyJumpThisTickDynamic() {
    for (int i = 0; i < dynamic_robots_size; ++i) {
      if (dynamic_robots[i]->action.jump_speed > 0) {
        return true;
      }
    }
    return false;
  }

  inline bool tickMicroticksDynamic(const int& number_of_tick, const int& number_of_microticks, GoalInfo& cur_goal_info, const bool& after_rollback) {
    clearTriggers();
    if (number_of_microticks == 0) {
      return false;
    }
    if (!after_rollback && checkCollideWithBallInAirDynamic()) {
      return false;
    }
    return updateDynamic((double) number_of_microticks / C::rules.TICKS_PER_SECOND / C::rules.MICROTICKS_PER_TICK, number_of_tick, number_of_microticks, cur_goal_info);
  }

  inline bool tickDihaDynamic(const int& tick_number, GoalInfo& cur_goal_info, const bool after_rollback = false) {
    bool sbd_wants_to_become_dynamic = false;
    GoalInfo goal_info = {false, false, -1};
    cur_goal_info = {false, false, -1};
#ifdef LOCAL
    if (accurate) {
      for (int i = 0; i < 100 * tpt; ++i) {
        sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1, goal_info, after_rollback);
        cur_goal_info |= goal_info;
      }
      return sbd_wants_to_become_dynamic;
    } else if (unaccurate) {
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 100 * tpt, goal_info, after_rollback);
      cur_goal_info |= goal_info;
      return sbd_wants_to_become_dynamic;
    }
#endif
    int remaining_microticks = 100 * tpt;
    const bool& flag = somebodyJumpThisTickDynamic();
    if (flag) {
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1, goal_info, after_rollback);
      remaining_microticks--;
      cur_goal_info |= goal_info;
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1, goal_info, after_rollback);
      remaining_microticks--;
      cur_goal_info |= goal_info;
    } else if (tick_number == 0) {
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1, goal_info, after_rollback); //todo do single initialisation (low prior)
      remaining_microticks--;
      cur_goal_info |= goal_info;
    }
    if (sbd_wants_to_become_dynamic) {
      return true;
    }

    clearTriggerFires();

    int iteration = 0;
    //H::c[1].call();
    while (true) {
      iteration++;
      for (int i = 0; i < dynamic_entities_size; ++i) { // 3/2 time of diha!!!!
        dynamic_entities[i]->savePrevMicroState();
      }
      sbd_wants_to_become_dynamic = tickMicroticksDynamic(tick_number, remaining_microticks, goal_info, after_rollback);
      //H::c[2].call();
      if (iteration == 1 && sbd_wants_to_become_dynamic) {
        return true;
      }
      if (anyTriggersActive() && remaining_microticks > 1) {
        for (int i = 0; i < dynamic_entities_size; ++i) {
          dynamic_entities[i]->fromPrevMicroState();
        }
        //H::c[3].call();
        tickMicroticksDynamic(tick_number, 1, goal_info, after_rollback);
        int l;
        int r;
        if (anyTriggersActive()) {
          l = 0;
        } else {
          l = 1;
          r = remaining_microticks;
          while (r - l > 1) {
            const int& mid = (r + l) / 2;
            for (int i = 0; i < dynamic_entities_size; ++i) {
              dynamic_entities[i]->fromPrevMicroState();
            }
            //H::c[3].call();
            tickMicroticksDynamic(tick_number, mid, goal_info, after_rollback);
            if (anyTriggersActive()) {
              r = mid;
            } else {
              l = mid;
            }
          }
        }
        for (int i = 0; i < dynamic_entities_size; ++i) {
          dynamic_entities[i]->fromPrevMicroState();
        }
        if (l > 0) {
          //H::c[3].call();
          tickMicroticksDynamic(tick_number, l, goal_info, after_rollback);
          cur_goal_info |= goal_info;
          remaining_microticks -= l;
        }
        //H::c[3].call();
        tickMicroticksDynamic(tick_number, 1, goal_info, after_rollback); // todo maybe not need
        cur_goal_info |= goal_info;
        setTriggersFired();
        remaining_microticks--;
      } else {
        cur_goal_info |= goal_info;
        break;
      }
    }
    return false;
  }

  void removeSleepingEntitiesDynamic(int simulation_tick) {


    int new_static_entities_size = 0;
    for (int i = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      if (e->state_ptr->alive) {
        static_entities[new_static_entities_size++] = e;
      }
    }
    static_entities_size = new_static_entities_size;


    int new_dynamic_entities_size = 0;
    for (int i = 0; i < dynamic_entities_size; ++i) {
      auto& e = dynamic_entities[i];
      if (!(e->is_robot && !e->is_teammate && simulation_tick > C::ENEMY_LIVE_TICKS && e->state.touch && e->state.touch_surface_id == 1)) {
        dynamic_entities[new_dynamic_entities_size++] = e;
      }
    }
    dynamic_entities_size = new_dynamic_entities_size;


    int new_static_robots_size = 0;
    for (int i = 0; i < static_robots_size; ++i) {
      auto& e = static_robots[i];
      if (e->state_ptr->alive) {
        static_robots[new_static_robots_size++] = e;
      }
    }
    static_robots_size = new_static_robots_size;

    int new_dynamic_robots_size = 0;
    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& e = dynamic_robots[i];
      if (!(!e->is_teammate && simulation_tick > C::ENEMY_LIVE_TICKS && e->state.touch && e->state.touch_surface_id == 1)) {
        dynamic_robots[new_dynamic_robots_size++] = e;
      }
    }
    dynamic_robots_size = new_dynamic_robots_size;


  }

  inline int tickDynamic(const int tick_number, int viz_id = -1, bool viz = false) {
    if (!unaccurate && (goal_info.goal_to_me || goal_info.goal_to_enemy)) {
      return 0;
    }
    int main_robot_additional_jump_type = 0;

    for (int i = 0; i < static_entities_size; ++i) { // a lot of time, but sleep will help
      if (static_entities[i]->is_pack) {
        static_entities[i]->fromStateStatic(tick_number);
      } else {
        static_entities[i]->fromStateStatic(tick_number + 1);
      }
    }

    if (!unaccurate) {
      removeSleepingEntitiesDynamic(tick_number);
    }

    wantedStaticGoToDynamic(tick_number);

    for (int i = 0; i < dynamic_entities_size; ++i) {
      dynamic_entities[i]->savePrevState();
    }

    GoalInfo cur_goal_info;

    bool flag = tryDoTickWithoutAnybodyBecomingDynamic(tick_number, main_robot_additional_jump_type, cur_goal_info);

    if (flag) {
      for (int i = 0; i < dynamic_entities_size; ++i) {
        dynamic_entities[i]->fromPrevState();
      }
      wantedStaticGoToDynamic(tick_number);
      for (int i = 0; i < dynamic_entities_size; ++i) {
        dynamic_entities[i]->savePrevState();
      }
      tryDoTickWithoutAnybodyBecomingDynamic(tick_number, main_robot_additional_jump_type, cur_goal_info);
    }

    for (int i = 0; i < dynamic_robots_size; ++i) {
      dynamic_robots[i]->accelerate_trigger_on_prev_tick = dynamic_robots[i]->accelerate_trigger_on_cur_tick;
    }

    for (int i = 0; i < dynamic_packs_size; ++i) {
      auto& pack = dynamic_packs[i];
      if (pack->state.alive) {
        continue;
      }
      pack->state.respawn_ticks--;
      if (pack->state.respawn_ticks == 0) {
        pack->state.alive = true;
      }
    }

    if (cur_goal_info.goal_to_me || cur_goal_info.goal_to_enemy) {
      goal_info = cur_goal_info;
      goal_info.goal_tick = tick_number;
    }

#ifdef DEBUG
    //if (!unaccurate && ball->is_dynamic) {
    //  P::drawLine(ball->state.position, ball->prev_state.position, 0x00FFFF);
    //}
    if (main_robot->id == viz_id && viz) {
      for (int i = 0; i < dynamic_entities_size; ++i) {
        auto& e = dynamic_entities[i];
        P::drawLine(e->state.position, e->prev_state.position, accurate ? 0x00FF00 : 0x0000FF);
      }
      for (int i = 0; i < static_entities_size; ++i) {
        auto& e = static_entities[i];
        P::drawLine(
            e->states[tick_number].position,
            e->states[tick_number + 1].position, accurate ? 0x00FFFF : 0xFF00FF);
      }
    }
#endif
    return main_robot_additional_jump_type;
  }

  inline bool collideEntitiesDynamic(const int& number_of_tick, const int& number_of_microticks, Entity* a, Entity* b, const bool& check_with_ball) {
    const Point& delta_position = b->state.position - a->state.position;
    const double& distance_sq = delta_position.length_sq();
    const double& sum_r = a->state.radius + b->state.radius;
    if (check_with_ball) {
      if ((3 + jr * a->action.max_jump_speed) * (3 + jr * a->action.max_jump_speed) > distance_sq) {
        a->collide_with_ball = true;
      }
    } else {
      if (a->is_teammate && !b->state.touch && (2 + jr * a->action.max_jump_speed) * (2 + jr * a->action.max_jump_speed) > distance_sq) {
        a->collide_with_entity_in_air = true;
      } else if (b->is_teammate && !a->state.touch && (2 + jr * b->action.max_jump_speed) * (2 + jr * b->action.max_jump_speed) > distance_sq) {
        b->collide_with_entity_in_air = true;
      }
    }
    if (sum_r * sum_r > distance_sq) {
      const double& penetration = sum_r - sqrt(distance_sq);
      const double& k_a = 1. / (a->mass * ((1 / a->mass) + (1 / b->mass)));
      const double& k_b = 1. / (b->mass * ((1 / a->mass) + (1 / b->mass)));
      const Point& normal = delta_position.normalize();
      a->state.position -= normal * (penetration * k_a);
      b->state.position += normal * (penetration * k_b);
      const double& delta_velocity = (b->state.velocity - a->state.velocity).dot(normal) - (b->radius_change_speed + a->radius_change_speed);
      if (check_with_ball && a->is_teammate) {
        //if (!accurate && main_robot->id == 4) {
        //  //H::t[18].call();
        //}
        entity_ball_collision_trigger = true;
        //H::c[5].call();
      } else if (a->radius_change_speed > 0 || b->radius_change_speed > 0) { // todo my on ground accurate if radius change speed > 0
        entity_entity_collision_trigger = true;
        //H::c[6].call();
      }
      if (delta_velocity < 0) {
        const Point& impulse = normal * ((1. + hit_e) * delta_velocity);
        a->state.velocity += impulse * k_a;
        b->state.velocity -= impulse * k_b;
        return true;
      }
    }
    return false;
  }

  inline bool collideEntitiesCheckDynamic(Entity* a_static, Entity* b_dynamic) {
    const double& sum_r = a_static->state_ptr->radius + b_dynamic->state.radius;
    const double& dz = a_static->state_ptr->position.z - b_dynamic->state.position.z;
    if (fabs(dz) > sum_r) {
      return false;
    }
    const double& dx = a_static->state_ptr->position.x - b_dynamic->state.position.x;
    if (fabs(dx) > sum_r) {
      return false;
    }
    const double& dy = a_static->state_ptr->position.y - b_dynamic->state.position.y;
    if (fabs(dy) > sum_r) {
      return false;
    }
    return sum_r * sum_r > dx * dx + dy * dy + dz * dz;
  }

  /*inline bool collideEntitiesCheckDynamic(const Entity* a_static, const Entity* b_dynamic) {
    return
        (a_static->state_ptr->radius + b_dynamic->state.radius)
            * (a_static->state_ptr->radius + b_dynamic->state.radius) >
            (b_dynamic->state.position - a_static->state_ptr->position).length_sq();
  }*/

  inline bool collideWithArenaDynamic(Entity* e, Point& result, int& collision_surface_id) {
    //todo optimise in goal
    const double& x = e->state.position.x > 0 ? e->state.position.x : -e->state.position.x;
    const double& z = e->state.position.z > 0 ? e->state.position.z : -e->state.position.z;
    if (((x < 22 && z < 32 && e->state.position.y < 18) || (z < 45 && x < 10&& e->state.position.y < 8)) ) {
      if (e->state.position.y < e->state.radius) {
        e->state.position.y = e->state.radius;
        e->state.velocity.y -= (1. + e->arena_e) * (e->state.velocity.y - e->radius_change_speed);
        result = {0, 1, 0};
        collision_surface_id = 1;
        return true;
      }
      return false;
    } else {

      const Dan& dan = Dan::dan_to_arena(e->state.position, e->state.radius);
      const double& distance = dan.distance;
      if (e->state.radius > distance) {
        const Point& normal = dan.normal.normalize();
        const double& penetration = e->state.radius - distance;
        e->state.position += normal * penetration;
        const double& velocity = e->state.velocity.dot(normal) - e->radius_change_speed;
        if (velocity < 0) {
          e->state.velocity -= normal * ((1. + e->arena_e) * velocity);
          result = normal;
          collision_surface_id = dan.collision_surface_id;
          return true;
        }
      }
      return false;
    }
  }

  inline void moveDynamic(Entity* e, const double& delta_time) {
    const double& length_sq = e->state.velocity.length_sq();
    if (length_sq > 10000.) {
      e->state.velocity *= (100. / sqrt(length_sq));
    }
    e->state.position += e->state.velocity * delta_time;
    e->state.position.y -= 15. * delta_time * delta_time;
    e->state.velocity.y -= 30. * delta_time;
  }

  void updateStatic(const double& delta_time, const int& number_of_tick, const int& number_of_microticks) {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& robot = initial_static_robots[i];
      if (!robot->state.alive) {
        continue;
      }
      if (robot->state.touch) { //todo nitro max speed clamp
        const Point& target_velocity = robot->action.target_velocity - robot->state.touch_normal * robot->state.touch_normal.dot(robot->action.target_velocity);
        const Point& target_velocity_change = target_velocity - robot->state.velocity;
        double length = target_velocity_change.length_sq();
        if (length > 0) {
          const double& acceleration = C::rules.ROBOT_ACCELERATION * fmax(0., robot->state.touch_normal.y);
          length = sqrt(length);
          const double& delta = length - acceleration * delta_time;
          if (delta > 0) {
            const auto& robot_acceleration = target_velocity_change * (acceleration * delta_time / length);
            robot->state.velocity += robot_acceleration;
            const double& coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / 2. / number_of_microticks) : 0.;
            robot->state.position -= robot_acceleration * (coef * delta_time);
          } else {
            if (robot->state.touch_surface_id == 1) {
              acceleration_trigger = true;
            }
            robot->state.velocity += target_velocity_change;
          }
        }
      } else {
        if (robot->action.use_nitro && robot->state.nitro > 0) {
          const auto& target_velocity_change = (robot->action.target_velocity - robot->state.velocity);
          const auto& tvc_length_sq = target_velocity_change.length_sq();
          if (tvc_length_sq > 0) {
            //const auto& max_nitro_change = robot->state.nitro * C::rules.NITRO_POINT_VELOCITY_CHANGE;
            const auto& ac_per_dt = C::rules.ROBOT_NITRO_ACCELERATION * delta_time;
            const auto& robot_acceleration = target_velocity_change * (ac_per_dt / sqrt(tvc_length_sq));
            robot->state.velocity += robot_acceleration;
            robot->state.nitro -= ac_per_dt / C::rules.NITRO_POINT_VELOCITY_CHANGE;
            const double coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / 2. / number_of_microticks) : 0.;
            robot->state.position -= robot_acceleration * (coef * delta_time);
          }
        }
      }

      moveStatic(robot, delta_time);

      robot->state.radius = C::rules.ROBOT_MIN_RADIUS + (C::rules.ROBOT_MAX_RADIUS - C::rules.ROBOT_MIN_RADIUS) * robot->action.jump_speed / C::rules.ROBOT_MAX_JUMP_SPEED;
      robot->radius_change_speed = robot->action.jump_speed;
    }
    moveStatic(ball, delta_time);

    for (int i = 0; i < initial_static_robots_size; i++) {
      if (!initial_static_robots[i]->state.alive) {
        continue;
      }
      for (int j = 0; j < i; j++) {
        if (!initial_static_robots[j]->state.alive) {
          continue;
        }
        collideEntitiesStatic(number_of_tick, initial_static_robots[i], initial_static_robots[j], false);
      }
    }

    Point collision_normal;
    int touch_surface_id;
    for (int i = 0; i < initial_static_robots_size; i++) {
      auto& robot = initial_static_robots[i];
      if (!robot->state.alive) {
        continue;
      }
      collideEntitiesStatic(number_of_tick, robot, ball, true);
      if (!collideWithArenaStatic(robot, collision_normal, touch_surface_id)) {
        if (robot->state.touch) {
          entity_arena_collision_trigger = true;
        }
        robot->state.touch = false;
      } else {
        if (!robot->state.touch || robot->state.touch_surface_id != touch_surface_id) {
          entity_arena_collision_trigger = true;
        }
        if (robot->is_teammate && touch_surface_id != 1) {
          robot->additional_jump = true;
        }
        robot->state.touch_surface_id = touch_surface_id;
        robot->state.touch = true;
        robot->state.touch_normal = collision_normal;
      }
    }
    if (!collideWithArenaStatic(ball, collision_normal, touch_surface_id)) {
      if (ball->state.touch) {
        if (ball->state.touch_surface_id != 1 || ball->state.velocity.y > C::ball_antiflap) {
          ball_arena_collision_trigger = true;
          ball->state.touch = false;
        }
      }
    } else {
      if (!ball->state.touch || ball->state.touch_surface_id != touch_surface_id) {
        ball_arena_collision_trigger = true;
      }
      ball->state.touch_surface_id = touch_surface_id;
      ball->state.touch = true;
    }

    for (int i = 0; i < initial_static_robots_size; i++) {
      auto& robot = initial_static_robots[i];
      if (!robot->state.alive) {
        continue;
      }
      if (robot->state.nitro == C::rules.MAX_NITRO_AMOUNT) {
        continue;
      }
      for (int j = 0; j < initial_static_packs_size; ++j) {
        auto& pack = initial_static_packs[j];
        if (!pack->state.alive) {
          continue;
        }
        const double& sum_r = robot->state.radius + pack->state.radius;
        if ((robot->state.position - pack->state.position).length_sq() <= sum_r * sum_r) {
          robot->state.nitro = C::rules.MAX_NITRO_AMOUNT;
          pack->state.alive = false;
          pack->state.respawn_ticks = C::rules.NITRO_PACK_RESPAWN_TICKS;
        }
      }
    }

  }


  // trajectory finding

  // 1. gall keeper:

  // 1.1 if has no nitro try safely swap with somebody with nitro

  // 1.2 if has no trajectory
  // 1.2.1 try find one vector2d trajectory without nitro
  //       this trajectory should be a:
  //       pass, a long ball flight, or any save from gall

  // 1.2.2 same as 1.2.1 with nitro vector3d

  // 1.2.3 same as 1.2.1 but stay N ticks, then one vector2d

  // 1.2.4 same as 1.2.2 but stay N ticks, then one vector3d

  // 1.2.5 try two vectors2d without nitro

  // 1.2.6 try two vectors3d without nitro


  double getSumScoreFighter(const int tick_number, const double goal_multiplier, const bool ball_on_my_side) {
    double score = 0;

    if (H::cur_round_tick >= 50) {
      if (goal_info.goal_to_me) {
        score += tick_number == goal_info.goal_tick ? -1e9 : 0;
      } else if (goal_info.goal_to_enemy) {
        //const double& height = ball->getState().position.y;
        //const double& height_score = 1e3 + 1e3 * ((height - 2) / 6.);
        //score += tick_number == goal_info.goal_tick ? height_score : 0;
        score += (tick_number == goal_info.goal_tick) ?  1e9 * goal_multiplier : 0;
        //score += tick_number == goal_info.goal_tick ? 1e3 : 0;
      }
      if (!(goal_info.goal_to_me || goal_info.goal_to_enemy) || tick_number <= goal_info.goal_tick) {

        if (!main_robot->state.touch) {
          score -= 1 * C::TPT;
        }

        //if (main_robot->collide_with_ball) {
        //  score += 1;
        //}
        //if (main_robot->action.use_nitro) {
        //  //score -= 0.1 * C::TPT;
        //}
        //if (!ball_on_my_side) {
        double delta_nitro =
            main_robot->taken_nitro > 0 ? main_robot->state.nitro - main_robot->states[0].nitro : 0;
        score += 1 * delta_nitro;
        //}


        /*for (int i = 0; i < static_robots_size; ++i) {
          auto& e = static_robots[i];
          if (!e->is_teammate && e->static_event_ptr->collide_with_ball) {
            score -= 10;
          }
        }
        for (int i = 0; i < dynamic_robots_size; ++i) {
          auto& e = dynamic_robots[i];
          if (!e->is_teammate && e->collide_with_ball) {
            score -= 10;
          }
        }*/

        if (tick_number < C::ENEMY_SIMULATION_DEPTH) {
          const int cell_x = std::clamp((int) ((ball->getState().position.x + 30. - 1.) / 2.), 0, 58);
          const int cell_y = std::clamp((int) ((ball->getState().position.y - 1.) / 2.), 0, 18);
          const int cell_z = std::clamp((int) ((ball->getState().position.z + 50. - 1.) / 2.), 0, 98);
          const int sum = H::danger_grid[cell_x][cell_y][cell_z][tick_number]
              + H::danger_grid[cell_x + 1][cell_y][cell_z][tick_number]
              + H::danger_grid[cell_x][cell_y + 1][cell_z][tick_number]
              + H::danger_grid[cell_x][cell_y][cell_z + 1][tick_number]
              + H::danger_grid[cell_x + 1][cell_y + 1][cell_z][tick_number]
              + H::danger_grid[cell_x + 1][cell_y][cell_z + 1][tick_number]
              + H::danger_grid[cell_x][cell_y + 1][cell_z + 1][tick_number]
              + H::danger_grid[cell_x + 1][cell_y + 1][cell_z + 1][tick_number];
          score -= 1e4 * sum;
        }
        //score -= 10 * (std::max(0., main_robot->state.position.z - ball->getState().position.z));
        //score += 1e3 * ball->getState().position.z;
        score -= 10 * (std::max(0., main_robot->state.position.z - ball->getState().position.z));
      }

    } else {
      score += 1e9 * ball->getState().position.z;
      if (main_robot->collide_with_ball) {
        score += 1e9;
      }
    }

    return score;
  }

  double getMinDistToGoalScoreFighter() {
    if (H::cur_round_tick >= 50) {
      const double& d1 = (Point{
          -C::rules.arena.goal_width / 2 + 2,
          C::rules.arena.goal_height - 2,
          C::rules.arena.depth / 2 + 2} - ball->getState().position).length_sq();
      const double& d2 = (Point{
          0,
          C::rules.arena.goal_height - 2,
          C::rules.arena.depth / 2 + 2} - ball->getState().position).length_sq();
      const double& d3 = (Point{
          C::rules.arena.goal_width / 2 - 2,
          C::rules.arena.goal_height - 2,
          C::rules.arena.depth / 2 + 2} - ball->getState().position).length_sq();
      return 10 * sqrt(std::min(d1, std::min(d2, d3)));
    } else {
      return 0;
    }
  }

  double getMinDistToBallScoreFighter() {
    if (H::cur_round_tick >= 50) {
      return (main_robot->state.position - ball->getState().position).length();
    } else {
      return 0;
    }
  }

  double getSumScoreEnemy(const int tick_number) {
    double score = 0;
    if (goal_info.goal_to_me) {
      score += tick_number == goal_info.goal_tick ? 1e3 : 0;
    } else if (goal_info.goal_to_enemy) {
      score += tick_number == goal_info.goal_tick ? -1e3 : 0;
    }
    if (!(goal_info.goal_to_me || goal_info.goal_to_enemy) || tick_number <= goal_info.goal_tick) {
      if (!main_robot->state.touch) {
        score -= 0.5 * C::TPT;
      }
      if (main_robot->collide_with_ball) {
        score += 0 * 20;
      }
    }
    return score;
  }

  double getMinDistToGoalScoreEnemy() {
    const double& d1 = (Point{
        -C::rules.arena.goal_width / 2 + 2,
        C::rules.arena.goal_height - 2,
        -C::rules.arena.depth / 2 - 2} - ball->getState().position).length_sq();
    const double& d2 = (Point{
        0,
        C::rules.arena.goal_height - 2,
        -C::rules.arena.depth / 2 - 2} - ball->getState().position).length_sq();
    const double& d3 = (Point{
        C::rules.arena.goal_width / 2 - 2,
        C::rules.arena.goal_height - 2,
        -C::rules.arena.depth / 2 - 2} - ball->getState().position).length_sq();
    return sqrt(std::min(d1, std::min(d2, d3)));
  }

  double getMinDistToBallScoreEnemy() {
    return (main_robot->state.position - ball->getState().position).length();
  }

  double goalInFuture() {
    if (!goal_info.goal_to_me && !goal_info.goal_to_enemy) {
      auto p = ball->getState().position;
      auto v = ball->getState().velocity;
      if (v.z == 0) {
        return 0;
      }
      double t = (40 - p.z) / v.z;
      if (t < 0) {
        return 0;
      }
      double D = v.y * v.y + 4 * 15 * (p.y - 18);
      if (D > 0) {
        double t1 = (-v.y + sqrt(D)) / (-30);
        double t2 = (-v.y - sqrt(D)) / (-30);
        if (t1 > 0 && t1 < t) {
          return 0;
        }
        if (t2 > 0 && t2 < t) {
          return 0;
        }
      }
      p.y += v.y * t - 15 * t * t;
      p.x += v.x * t;
      if (p.y > 2 && p.y < 8 && p.x > -13 && p.x < 13) {
        const double& height = p.y;
        const double& height_score = 1e3 + 1e3 * ((height - 2) / 6.);
        auto state = ball->getState();
        state.position.z = 40;
        state.position.x = p.x;
        state.position.y = p.y;
        P::drawEntities(ball->getState(), 0, 0x00FFFF);
        P::drawEntities(state, 0, 0xFF00FF);
        return height_score;
      }
    }
    return 0;
  }

  double getSumScoreDefender(const int tick_number, const bool ball_on_my_side) {
    double score = 0;
    if (H::cur_round_tick >= 45) {
      if (goal_info.goal_to_me) {
        score += tick_number == goal_info.goal_tick ? -1e9 : 0;
      } else if (goal_info.goal_to_enemy) {
        score += tick_number == goal_info.goal_tick ? 1e3 : 0;
      }
      if (!(goal_info.goal_to_me || goal_info.goal_to_enemy) || tick_number <= goal_info.goal_tick) {
        if (!main_robot->state.touch) {
          score -= 1 * C::TPT;
        }
        if (!ball_on_my_side && main_robot->state.position.z < 0) {
          double delta_nitro =
              main_robot->taken_nitro > 0 ? main_robot->state.nitro - main_robot->states[0].nitro : 0;
          score += 1e9 * delta_nitro;
        }

        //if (main_robot->collide_with_ball) {
        //  score += 1;
        //}

        /*for (int i = 0; i < static_robots_size; ++i) {
          auto& e = static_robots[i];
          if (!e->is_teammate && e->static_event_ptr->collide_with_ball) {
            score -= 10;
          }
        }
        for (int i = 0; i < dynamic_robots_size; ++i) {
          auto& e = dynamic_robots[i];
          if (!e->is_teammate && e->collide_with_ball) {
            score -= 10;
          }
        }*/

        if (tick_number < C::ENEMY_SIMULATION_DEPTH) {
          const int cell_x = std::clamp((int) ((ball->getState().position.x + 30. - 1.) / 2.), 0, 58);
          const int cell_y = std::clamp((int) ((ball->getState().position.y - 1.) / 2.), 0, 18);
          const int cell_z = std::clamp((int) ((ball->getState().position.z + 50. - 1.) / 2.), 0, 98);
          const int sum = H::danger_grid[cell_x][cell_y][cell_z][tick_number]
              + H::danger_grid[cell_x + 1][cell_y][cell_z][tick_number]
              + H::danger_grid[cell_x][cell_y + 1][cell_z][tick_number]
              + H::danger_grid[cell_x][cell_y][cell_z + 1][tick_number]
              + H::danger_grid[cell_x + 1][cell_y + 1][cell_z][tick_number]
              + H::danger_grid[cell_x + 1][cell_y][cell_z + 1][tick_number]
              + H::danger_grid[cell_x][cell_y + 1][cell_z + 1][tick_number]
              + H::danger_grid[cell_x + 1][cell_y + 1][cell_z + 1][tick_number];
          score -= 1e4 * sum;
        }
      }
    }

    double where_x = 0;
    if (ball->states[0].velocity.z != 0) {
      double t = (-40 - ball->states[0].position.x) / ball->states[0].velocity.z;
      if (t > 0) {
        where_x = ball->states[0].position.x + ball->states[0].velocity.x * t;
        if (where_x > 8) {
          where_x = 8;
        } else if (where_x < -8) {
          where_x = -8;
        }
      }
    }
    score -= (0.0025 * C::TPT) * (main_robot->state.position - Point{
        where_x,
        1,
        -C::rules.arena.depth / 2 - 2}).length();
    return score;
  }

  double getMinDistToEnemyScore() {
    return 0;
    if (H::cur_round_tick >= 50) {
      double min_dist = 1e9;
      for (int i = 0; i < static_robots_size; ++i) {
        auto& e = static_robots[i];
        if (!e->is_teammate) {
          min_dist = std::min(min_dist, (e->getState().position - ball->getState().position).length_sq());
        }
      }
      for (int i = 0; i < dynamic_robots_size; ++i) {
        auto& e = dynamic_robots[i];
        if (!e->is_teammate) {
          min_dist = std::min(min_dist, (e->getState().position - ball->getState().position).length_sq());
        }
      }
      return min_dist;
    } else {
      return 0;
    }
  }

  double getMinDistFromGoalScoreDefender() {
    if (H::cur_round_tick >= 45) {
      return ball->getState().position.z;
    } else {
      return 0;
    }
  }

  double getMinDistToBallScoreDefender() {
    if (H::cur_round_tick >= 45) {
      return 0.1 * (main_robot->state.position - ball->getState().position).length();
    } else {
      return 0;
    }
  }

};

#ifndef LOCAL
namespace Frozen {

struct SmartSimulator {

  struct GoalInfo {
    bool goal_to_me;
    bool goal_to_enemy;
    int goal_tick;

    void operator|=(const GoalInfo& other) {
      goal_to_me |= other.goal_to_me;
      goal_to_enemy |= other.goal_to_enemy;
    }

  } goal_info;

  Entity initial_static_entities[11];
  int initial_static_entities_size = 0;

  Entity initial_dynamic_entities[11];
  int initial_dynamic_entities_size = 0;

  Entity* initial_static_robots[6];
  int initial_static_robots_size = 0;

  Entity* initial_dynamic_robots[6];
  int initial_dynamic_robots_size = 0;

  Entity* initial_static_packs[4];
  int initial_static_packs_size = 0;

  Entity* static_entities[11];
  int static_entities_size = 0;

  Entity* dynamic_entities[11];
  int dynamic_entities_size = 0;

  Entity* static_robots[6];
  int static_robots_size = 0;

  Entity* dynamic_robots[6];
  int dynamic_robots_size = 0;

  Entity* static_packs[4];
  int static_packs_size = 0;

  Entity* dynamic_packs[4];
  int dynamic_packs_size = 0;

  Entity* main_robot;
  Entity* ball;

  bool acceleration_trigger;
  int acceleration_trigger_fires;
  const int acceleration_trigger_limit = 2;

  bool entity_entity_collision_trigger;
  int entity_entity_collision_trigger_fires;
  const int entity_entity_collision_limit = 2;

  bool entity_ball_collision_trigger;
  int entity_ball_collision_trigger_fires;
  const int entity_ball_collision_limit = 2;

  bool entity_arena_collision_trigger;
  int entity_arena_collision_trigger_fires;
  const int entity_arena_collision_limit = 2;

  bool ball_arena_collision_trigger;
  int ball_arena_collision_trigger_fires;
  const int ball_arena_collision_limit = 2;

  int simulation_depth;

  bool accurate;

  double hit_e = (C::rules.MIN_HIT_E + C::rules.MAX_HIT_E) / 2;
  // double hit_e = C::rules.MAX_HIT_E;

  bool collided_entities[7][7];

  // maybe we can have 4x-5x performance boost, and more when 3x3
  SmartSimulator(
      const int simulation_depth,
      const int main_robot_id,
      const int plans_configuration,
      const std::vector<model::Robot>& _robots,
      const model::Ball& _ball,
      const std::vector<model::NitroPack>& _packs,
      bool accurate = false,
      int viz_id = -1)
      : simulation_depth(simulation_depth), accurate(accurate) {

    initial_static_entities[initial_static_entities_size].fromBall(_ball);
    ball = &initial_static_entities[initial_static_entities_size++];
    ball->is_dynamic = false;

    for (auto& robot : _robots) {
      if (robot.id == main_robot_id) {
        initial_dynamic_entities[initial_dynamic_entities_size].fromRobot(robot);
        auto new_robot = &initial_dynamic_entities[initial_dynamic_entities_size++];
        new_robot->is_dynamic = true;
        main_robot = new_robot;
        initial_dynamic_robots[initial_dynamic_robots_size++] = new_robot;
        new_robot->saveState(0);
      } else {
        initial_static_entities[initial_static_entities_size].fromRobot(robot);
        auto new_robot = &initial_static_entities[initial_static_entities_size++];
        new_robot->is_dynamic = false;
        initial_static_robots[initial_static_robots_size++] = new_robot;
      }
    }

    for (auto& pack : _packs) {
      initial_static_entities[initial_static_entities_size].fromPack(pack);
      auto new_pack = &initial_static_entities[initial_static_entities_size++];
      new_pack->is_dynamic = false;
      initial_static_packs[initial_static_packs_size++] = new_pack;
    }

    //todo check what better last_action nothing, best action and for my or enemy
    if (plans_configuration == 1) { // best action all for minimax
      for (int i = 0; i < initial_static_robots_size; ++i) {
        initial_static_robots[i]->plan = H::best_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
      }
    } else if (plans_configuration == 2) { // best action my, last action enemy
      for (int i = 0; i < initial_static_robots_size; ++i) {
        if (initial_static_robots[i]->is_teammate) {
          initial_static_robots[i]->plan = H::best_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
        } else {
          initial_static_robots[i]->plan = H::last_action_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
        }
      }
    } else if (plans_configuration == 3) { // last action all for minimax
      for (int i = 0; i < initial_static_robots_size; ++i) {
        initial_static_robots[i]->plan = H::last_action_plan[H::getRobotLocalIdByGlobal(initial_static_robots[i]->id)];
      }
    }

    for (int sim_tick = 0; sim_tick < simulation_depth + 1; ++sim_tick) {
      tickWithJumpsStatic(sim_tick, true);
    }

#ifdef DEBUG
    if (main_robot_id == viz_id) {
      for (int i = 0; i < initial_static_entities_size; ++i) {
        auto& e = initial_static_entities[i];
        for (int j = 1; j < simulation_depth; ++j) {
          P::drawLine(e.states[j - 1].position, e.states[j].position, accurate ? 0xFFFFFF : 0x000000);
        }
      }
    }
#endif

    // init
    // calculate static trajectories and build collision-time dependencies tree
    // ? fair 100 microtick calculation of maybe ball only, maybe on sphere collisions ?
    // add main to dynamic
    // start simulation
    // if any static want to become a dynamic - do it
    // simulate tick
    // if any dynamic has a collision with static, add this static to dynamic
    // and any son of this static from subtree now want to become a dynamic
    // on collision tick with its parent
    //
  }

  void clearAdditionalJumpsStatic() {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      initial_static_robots[i]->collide_with_entity_in_air = false;
      initial_static_robots[i]->collide_with_ball = false;
      initial_static_robots[i]->additional_jump = false;
      initial_static_robots[i]->taken_nitro = 0;
    }
  }

  bool somebodyJumpThisTickStatic() {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      if (initial_static_robots[i]->action.jump_speed > 0) {
        return true;
      }
    }
    return false;
  }

  void clearTriggers() {
    acceleration_trigger = false;
    entity_entity_collision_trigger = false;
    entity_ball_collision_trigger = false;
    entity_arena_collision_trigger = false;
    ball_arena_collision_trigger = false;
  }

  void clearTriggerFires() {
    acceleration_trigger_fires = 0;
    entity_entity_collision_trigger_fires = 0;
    entity_ball_collision_trigger_fires = 0;
    entity_arena_collision_trigger_fires = 0;
    ball_arena_collision_trigger_fires = 0;
  }

  void tickMicroticksStatic(const int number_of_tick, const int number_of_microticks) {
    clearTriggers();
    if (number_of_microticks == 0) {
      return;
    }
    updateStatic((double) number_of_microticks / C::rules.TICKS_PER_SECOND / C::rules.MICROTICKS_PER_TICK, number_of_tick, number_of_microticks);
  }

  void clearCollisionsAndStaticEvents(const int tick) {
    for (int i = 0; i < 7; ++i) {
      for (int j = 0; j < 7; ++j) {
        collided_entities[i][j] = false;
      }
    }
    for (int i = 0; i < initial_static_entities_size; ++i) {
      initial_static_entities[i].static_events[tick + 1].clear();
    }
  }

  void tickWithJumpsStatic(const int tick_number, bool with_jumps) {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& robot = initial_static_robots[i];
      robot->action = robot->plan.toMyAction(tick_number, true, true, robot->state.position);
      robot->nitroCheck();
      if (!robot->action.use_nitro) {
        robot->action = robot->plan.toMyAction(tick_number, true, false, robot->state.position);
      }
    }
    for (int i = 0; i < initial_static_entities_size; ++i) { // save state
      auto& e = initial_static_entities[i];
      e.saveState(tick_number);
    }
    if (with_jumps) {
      clearAdditionalJumpsStatic();
    }
    clearCollisionsAndStaticEvents(tick_number);
    tickStatic(tick_number);
    if (with_jumps) {
      bool needs_rollback = false;
      for (int i = 0; i < initial_static_robots_size; ++i) {
        auto& e = initial_static_robots[i];
        if (e->collide_with_entity_in_air || e->collide_with_ball || e->additional_jump) {
          needs_rollback = true;
          e->action.jump_speed = e->additional_jump ? std::max(C::MIN_WALL_JUMP, e->action.max_jump_speed) : e->action.max_jump_speed;
        }
      }
      if (needs_rollback) {
        for (int i = 0; i < initial_static_entities_size; ++i) {
          auto& e = initial_static_entities[i];
          e.fromState(tick_number);
        }
        clearCollisionsAndStaticEvents(tick_number);
        tickStatic(tick_number);
      }
    }

    for (int i = 0; i < initial_static_packs_size; ++i) {
      auto& pack = initial_static_packs[i];
      if (pack->state.alive) {
        continue;
      }
      pack->state.respawn_ticks--;
      if (pack->state.respawn_ticks == 0) {
        pack->state.alive = true;
      }
    }

    for (int i = 0; i < 7; ++i) {
      for (int j = 0; j < 7; ++j) {
        if (collided_entities[i][j]) {
          Entity* e = initialStaticEntityById(i);
          Entity* me = initialStaticEntityById(j);
          if (e && me) {
            me->addCollision({me, e, tick_number});
            e->addCollision({e, me, tick_number});
          }
        }
      }
    }
  }

  Entity* initialStaticEntityById(const int id) {
    for (int i = 0; i < initial_static_entities_size; ++i) {
      if (initial_static_entities[i].id == id) {
        return initial_static_entities + i;
      }
    }
    return 0;
  }

  bool anyTriggersActive() {
    return
        (acceleration_trigger && acceleration_trigger_fires < acceleration_trigger_limit) ||
            (entity_arena_collision_trigger && entity_arena_collision_trigger_fires < entity_arena_collision_limit) ||
            (ball_arena_collision_trigger && ball_arena_collision_trigger_fires < ball_arena_collision_limit) ||
            (entity_ball_collision_trigger && entity_ball_collision_trigger_fires < entity_ball_collision_limit) ||
            (entity_entity_collision_trigger && entity_entity_collision_trigger_fires < entity_entity_collision_limit);

  }

  void setTriggersFired() {
    if (acceleration_trigger) {
      acceleration_trigger_fires++;
    }
    if (entity_arena_collision_trigger) {
      entity_arena_collision_trigger_fires++;
    }

    if (ball_arena_collision_trigger) {
      ball_arena_collision_trigger_fires++;
    }
    if (entity_entity_collision_trigger) {
      entity_entity_collision_trigger_fires++;
    }
    if (entity_ball_collision_trigger) {
      entity_ball_collision_trigger_fires++;
    }
  }

  void saveMicrostatesStatic() {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& e = initial_static_robots[i];
      e->savePrevMicroState();
    }
    ball->savePrevMicroState();
  }

  void fromMictostatesStatic() {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& e = initial_static_robots[i];
      e->fromPrevMicroState();
    }
    ball->fromPrevMicroState();
  }

  void tickStatic(const int tick_number) {
    if (accurate) {
      for (int i = 0; i < C::MICROTICKS_PER_TICK; ++i) {
        tickMicroticksStatic(tick_number, 1);
      }
      return;
    }
    int remaining_microticks = C::MICROTICKS_PER_TICK;
    if (somebodyJumpThisTickStatic()) {
      tickMicroticksStatic(tick_number, 1);
      remaining_microticks--;
      tickMicroticksStatic(tick_number, 1);
      remaining_microticks--;
    } else if (tick_number == 0) {
      tickMicroticksStatic(tick_number, 1);
      remaining_microticks--;
    }

    clearTriggerFires();

    while (true) {

      saveMicrostatesStatic();
      tickMicroticksStatic(tick_number, remaining_microticks);

      if (anyTriggersActive() && remaining_microticks > 1) { // todo as dynamic
        int l = 0;
        int r = remaining_microticks;
        while (r - l > 1) {
          int mid = (r + l) / 2;
          fromMictostatesStatic();
          tickMicroticksStatic(tick_number, mid);
          if (anyTriggersActive()) {
            r = mid;
          } else {
            l = mid;
          }
        }
        fromMictostatesStatic();
        if (l > 0) {
          tickMicroticksStatic(tick_number, l);
          remaining_microticks -= l;
        }
        tickMicroticksStatic(tick_number, 1);
        setTriggersFired();
        remaining_microticks--;
      } else {
        break;
      }
    }
  }

  bool collideEntitiesStatic(const int tick_number, Entity* a, Entity* b, bool check_with_ball) {
    const Point& delta_position = b->state.position - a->state.position;
    const double distance_sq = delta_position.length_sq();
    const double sum_r = a->state.radius + b->state.radius;
    if (check_with_ball) {
      if ((3 + jr * a->action.max_jump_speed) * (3 + jr * a->action.max_jump_speed) > distance_sq) {
        a->collide_with_ball = true;
        a->static_events[tick_number + 1].collide_with_ball = true;
      }
    } else {
      if (a->is_teammate && !b->state.touch && (2 + jr * a->action.max_jump_speed) * (2 + jr * a->action.max_jump_speed) > distance_sq) {
        a->collide_with_entity_in_air = true;
      } else if (b->is_teammate && !a->state.touch && (2 + jr * b->action.max_jump_speed) * (2 + jr * b->action.max_jump_speed) > distance_sq) {
        b->collide_with_entity_in_air = true;
      }
    }
    if (sum_r * sum_r > distance_sq) {
      collided_entities[a->id][b->id] = true;
      const double penetration = sum_r - sqrt(distance_sq);
      if (check_with_ball) {
        entity_ball_collision_trigger = true;
      } else if (!a->state.touch && !b->state.touch) {
        entity_entity_collision_trigger = true;
      }
      const double k_a = 1. / (a->mass * ((1 / a->mass) + (1 / b->mass)));
      const double k_b = 1. / (b->mass * ((1 / a->mass) + (1 / b->mass)));
      const Point& normal = delta_position.normalize();
      a->state.position -= normal * (penetration * k_a);
      b->state.position += normal * (penetration * k_b);
      const double delta_velocity = (b->state.velocity - a->state.velocity).dot(normal) - (b->radius_change_speed + a->radius_change_speed);
      if (delta_velocity < 0) {
        const Point& impulse = normal * ((1. + hit_e) * delta_velocity);
        a->state.velocity += impulse * k_a;
        b->state.velocity -= impulse * k_b;
        return true;
      }
    }
    return false;
  }

  bool collideWithArenaStatic(Entity* e, Point& result, int& collision_surface_id) {
    const Dan& dan = Dan::dan_to_arena(e->state.position, e->state.radius);
    const double distance = dan.distance;
    if (e->state.radius > distance) {
      const Point& normal = dan.normal.normalize();
      const double penetration = e->state.radius - distance;
      e->state.position += normal * penetration;
      const double velocity = e->state.velocity.dot(normal) - e->radius_change_speed;
      if (velocity < 0) {
        e->state.velocity -= normal * ((1. + e->arena_e) * velocity);
        result = normal;
        collision_surface_id = dan.collision_surface_id;
        return true;
      }
    }
    return false;
  }

  void moveStatic(Entity* e, const double delta_time) {
    e->state.velocity = e->state.velocity.clamp(C::rules.MAX_ENTITY_SPEED);
    e->state.position += e->state.velocity * delta_time;
    e->state.position.y -= C::rules.GRAVITY * delta_time * delta_time / 2;
    e->state.velocity.y -= C::rules.GRAVITY * delta_time;
  }

  static constexpr double jr = 0.0033333333333333333333333333333;

  bool updateDynamic(const double delta_time, const int number_of_tick, const int number_of_microticks, GoalInfo& cur_goal_info) {

    //H::t[0].start();
    bool has_collision_with_static = false;
    cur_goal_info = {false, false, -1};
    //H::t[0].cur(true);

    //H::t[1].start();
    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& robot = dynamic_robots[i];
      if (robot->state.touch) {
        ////H::t[12].start();
        const Point& target_velocity = (robot->state.touch_surface_id == 1) ? robot->action.target_velocity :
            robot->action.target_velocity - robot->state.touch_normal * robot->state.touch_normal.dot(robot->action.target_velocity);
        ////H::t[12].cur(true);
        ////H::t[13].start();
        const Point& target_velocity_change = target_velocity - robot->state.velocity;
        ////H::t[13].cur(true);
        ////H::t[14].start();
        double length = target_velocity_change.length_sq();
        ////H::t[14].cur(true);
        if (length > 0) {
          ////H::t[15].start();
          const double& acceleration = (robot->state.touch_normal.y > 0) ? robot->state.touch_normal.y * 100 : 0;
          ////H::t[15].cur(true);
          ////H::t[16].start();

          length = sqrt(length);
          ////H::t[16].cur(true);
          ////H::t[17].start();

          const double& delta = length - acceleration * delta_time;
          ////H::t[17].cur(true);

          if (delta > 0) {
            ////H::t[18].start();

            const auto& robot_acceleration = target_velocity_change * (acceleration * delta_time / length);
            ////H::t[18].cur(true);
            ////H::t[19].start();

            robot->state.velocity += robot_acceleration;
            ////H::t[19].cur(true);
            ////H::t[20].start();

            const double& coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / (2. * number_of_microticks)) : 0.; // todo optimise ?
            ////H::t[20].cur(true);
            ////H::t[21].start();
            robot->state.position -= robot_acceleration * (coef * delta_time);
            ////H::t[21].cur(true);

          } else {
            if (robot->state.touch_surface_id == 1) {
              acceleration_trigger = true;
            }
            ////H::t[22].start();

            robot->state.velocity += target_velocity_change;
            ////H::t[22].cur(true);

          }
        }
      } else {
        if (robot->is_teammate && robot->action.use_nitro && robot->state.nitro > 0) {
          const auto& target_velocity_change = (robot->action.target_velocity - robot->state.velocity);
          const auto& tvc_length_sq = target_velocity_change.length_sq();
          if (tvc_length_sq > 0) {
            //const auto& max_nitro_change = robot->state.nitro * C::rules.NITRO_POINT_VELOCITY_CHANGE;
            const auto& ac_per_dt = C::rules.ROBOT_NITRO_ACCELERATION * delta_time;
            const auto& robot_acceleration = target_velocity_change * (ac_per_dt / sqrt(tvc_length_sq));
            robot->state.velocity += robot_acceleration;
            robot->state.nitro -= ac_per_dt / C::rules.NITRO_POINT_VELOCITY_CHANGE;
            const double coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / 2. / number_of_microticks) : 0.;
            robot->state.position -= robot_acceleration * (coef * delta_time);
          }
        }
      }

      ////H::t[10].start();
      moveDynamic(robot, delta_time);

      ////H::t[10].cur(true);
      ////H::t[23].start();
      robot->state.radius = 1. + jr * robot->action.jump_speed;
      ////H::t[23].cur(true);
      ////H::t[24].start();

      robot->radius_change_speed = robot->action.jump_speed;
      ////H::t[24].cur(true);
    }
    //H::t[1].cur(true);
    //H::t[2].start();
    if (ball->is_dynamic) {
      moveDynamic(ball, delta_time);
    }
    //H::t[2].cur(true);
    //H::t[3].start();

    for (int i = 0; i < static_robots_size; i++) {
      for (int j = 0; j < dynamic_robots_size; j++) {
        if (collideEntitiesCheckDynamic(static_robots[i], dynamic_robots[j])) {
          static_robots[i]->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
          break;
        }
      }
    }

    //H::t[3].cur(true);
    //H::t[4].start();

    for (int i = 0; i < dynamic_robots_size; i++) {
      for (int j = 0; j < i; j++) {
        collideEntitiesDynamic(number_of_tick, number_of_microticks, dynamic_robots[i], dynamic_robots[j], false);
      }
    }
    //H::t[4].cur(true);

    ////H::t[5].start();

    Point collision_normal;
    int touch_surface_id;

    for (int i = 0; i < dynamic_robots_size; i++) {
      auto& robot = dynamic_robots[i];
      if (ball->is_dynamic) {
        //H::t[9].start();
        collideEntitiesDynamic(number_of_tick, number_of_microticks, robot, ball, true);
        //H::t[9].cur(true);
      } else {
        //H::t[10].start();
        if (collideEntitiesCheckDynamic(ball, robot)) {
          ball->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
        }
        //H::t[10].cur(true);
      }
      //H::t[11].start();
      if (!collideWithArenaDynamic(robot, collision_normal, touch_surface_id)) {
        //H::t[11].cur(true);
        if (robot->state.touch) {
          entity_arena_collision_trigger = true;
        }
        robot->state.touch = false;
      } else {
        //H::t[11].cur(true);
        //H::t[12].start();
        if (!robot->state.touch || robot->state.touch_surface_id != touch_surface_id) {
          entity_arena_collision_trigger = true;
        }
        if (robot->is_teammate && touch_surface_id != 1) {
          robot->additional_jump = true;
        }
        robot->state.touch_surface_id = touch_surface_id;
        robot->state.touch = true;
        robot->state.touch_normal = collision_normal;
        //H::t[12].cur(true);
      }
    }


    ////H::t[5].cur(true);
    //H::t[6].start();

    if (ball->is_dynamic) {
      for (int i = 0; i < static_robots_size; ++i) {
        if (collideEntitiesCheckDynamic(static_robots[i], ball)) {
          static_robots[i]->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
        }
      }
    }
    ////H::t[6].cur(true);
    //H::t[1].cur(true);
    //H::t[3].start();
    for (int i = 0; i < dynamic_robots_size; ++i) {
      const auto& robot = dynamic_robots[i];
      if (robot->is_teammate && !(robot->state.nitro < C::rules.MAX_NITRO_AMOUNT)) {
        continue;
      }
      const double& x = robot->state.position.x > 0 ? robot->state.position.x : -robot->state.position.x;
      const double& y = robot->state.position.y;
      const double& z = robot->state.position.z > 0 ? robot->state.position.z : -robot->state.position.z;
      const double& r = robot->state.radius;
      if (y > 1.5 + r || x > 20.5 + r || x < 19.5 - r || z > 30.5 + r || z < 29.5 - r) {
        continue;
      }
      for (int j = 0; j < static_packs_size; ++j) {
        const auto& pack = static_packs[j];
        if (!pack->state_ptr->alive) {
          continue;
        }
        if (collideEntitiesCheckDynamic(pack, robot)) {
          pack->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
          break;
        }
      }
    }

    //H::t[3].cur(true);

    if (has_collision_with_static) {
      return true;
    }

    //H::t[7].start();
    if (ball->is_dynamic) {
      if (!collideWithArenaDynamic(ball, collision_normal, touch_surface_id)) {
        if (ball->state.touch) {
          if (ball->state.touch_surface_id != 1 || ball->state.velocity.y > C::ball_antiflap) {
            ball_arena_collision_trigger = true;
            ball->state.touch = false;
          }
        }
      } else {
        if (!ball->state.touch || ball->state.touch_surface_id != touch_surface_id) {
          ball_arena_collision_trigger = true;
        }
        ball->state.touch_surface_id = touch_surface_id;
        ball->state.touch = true;
      }
    }
    //H::t[7].cur(true);

    //H::t[5].cur(true);
    //H::t[4].start();
    for (int i = 0; i < dynamic_robots_size; i++) {
      auto& robot = dynamic_robots[i];
      if (robot->state.nitro == C::rules.MAX_NITRO_AMOUNT) {
        continue;
      }
      for (int j = 0; j < dynamic_packs_size; ++j) {
        auto& pack = dynamic_packs[j];
        if (!pack->state.alive) {
          continue;
        }
        const double& sum_r = robot->state.radius + pack->state.radius;
        if ((robot->state.position - pack->state.position).length_sq() <= sum_r * sum_r) {
          robot->taken_nitro = C::rules.MAX_NITRO_AMOUNT - robot->state.nitro;
          robot->state.nitro = C::rules.MAX_NITRO_AMOUNT;
          pack->state.alive = false;
          pack->state.respawn_ticks = C::rules.NITRO_PACK_RESPAWN_TICKS;
        }
      }
    }
    //H::t[4].cur(true);

    //H::t[8].start();
    if (ball->is_dynamic) {
      if (ball->state.position.z > C::rules.arena.depth / 2 + 2) {
        cur_goal_info.goal_to_enemy = true;
      } else if (ball->state.position.z < -C::rules.arena.depth / 2 - 2) {
        cur_goal_info.goal_to_me = true;
      }
    } else {
      if (ball->state_ptr->position.z > C::rules.arena.depth / 2 + 2 || ball->state_ptr->position.z < -C::rules.arena.depth / 2 - 2) {
        ball->wantToBecomeDynamic(number_of_tick);
        has_collision_with_static = true;
      }
    }
    //H::t[8].cur(true);

    return has_collision_with_static;
  }

  void initIteration(const int iteration, const Plan& main_robot_plan) {
    goal_info.goal_to_me = false;
    goal_info.goal_to_enemy = false;

    main_robot->plan = main_robot_plan;

    static_entities_size = 0;
    for (int i = 0; i < initial_static_entities_size; ++i) {
      static_entities[static_entities_size++] = &initial_static_entities[i];
    }

    dynamic_entities_size = 0;
    for (int i = 0; i < initial_dynamic_entities_size; ++i) {
      dynamic_entities[dynamic_entities_size++] = &initial_dynamic_entities[i];
    }

    static_robots_size = 0;
    for (int i = 0; i < initial_static_robots_size; ++i) {
      static_robots[static_robots_size++] = initial_static_robots[i];
    }

    dynamic_robots_size = 0;
    for (int i = 0; i < initial_dynamic_robots_size; ++i) {
      dynamic_robots[dynamic_robots_size++] = initial_dynamic_robots[i];
    }

    static_packs_size = 0;
    for (int i = 0; i < initial_static_packs_size; ++i) {
      static_packs[static_packs_size++] = initial_static_packs[i];
    }

    dynamic_packs_size = 0;

    for (int i = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      e->is_dynamic = false;
      e->want_to_become_dynamic = false;
    }
    for (int i = 0; i < dynamic_entities_size; ++i) {
      auto& e = dynamic_entities[i];
      e->is_dynamic = true;
    }
    main_robot->fromState(0);
  }

  void wantedStaticGoToDynamic(const int tick_number) {
    static_packs_size = 0;
    static_robots_size = 0;
    int new_static_entities_size = 0;
    for (int i = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      if (e->want_to_become_dynamic && e->want_to_become_dynamic_on_tick == tick_number) {
        e->fromState(tick_number);
        e->is_dynamic = true;
        dynamic_entities[dynamic_entities_size++] = e;
        if (e->is_robot) {
          dynamic_robots[dynamic_robots_size++] = e;
        } else if (e->is_pack) {
          dynamic_packs[dynamic_packs_size++] = e;
        }
      } else {
        static_entities[new_static_entities_size++] = e;
        if (e->is_robot) {
          static_robots[static_robots_size++] = e;
        } else if (e->is_pack) {
          static_packs[static_packs_size++] = e;
        }
      }
    }
    static_entities_size = new_static_entities_size;
  }

  void clearCollideWithBallInAirDynamic() {
    for (int i = 0; i < dynamic_robots_size; ++i) {
      dynamic_robots[i]->collide_with_entity_in_air = false;
      dynamic_robots[i]->collide_with_ball = false;
      dynamic_robots[i]->additional_jump = false;
      dynamic_robots[i]->taken_nitro = 0;
    }
  }

  bool tryTickWithJumpsDynamic(const int tick_number, const bool with_jumps, int& main_robot_additional_jump_type, GoalInfo& cur_goal_info) {
    if (with_jumps) {
      clearCollideWithBallInAirDynamic();
      main_robot_additional_jump_type = 0;
    }
    bool sbd_become_dynamic = tickDihaDynamic(tick_number, cur_goal_info); // todo check need return here
    if (with_jumps) {
      bool needs_rollback = false;
      for (int i = 0; i < dynamic_robots_size; ++i) {
        const auto& e = dynamic_robots[i];
        if (e->collide_with_entity_in_air || e->collide_with_ball || e->additional_jump) {
          needs_rollback = true;
          e->action.jump_speed = e->additional_jump ? std::max(C::MIN_WALL_JUMP, e->action.max_jump_speed) : e->action.max_jump_speed;
          if (e == main_robot) {
            if (e->collide_with_ball) {
              main_robot_additional_jump_type = 1;
            } else if (e->collide_with_entity_in_air) {
              main_robot_additional_jump_type = 2;
            } else if (e->additional_jump) {
              main_robot_additional_jump_type = 3;
            }
          }
        }
      }
      if (needs_rollback) {
        for (int i = 0; i < dynamic_entities_size; ++i) {
          dynamic_entities[i]->fromPrevState();
        }
        sbd_become_dynamic = tickDihaDynamic(tick_number, cur_goal_info);
      }
    }
    return sbd_become_dynamic;
  }

  bool tryDoTickWithoutAnybodyBecomingDynamic(const int tick_number, int& main_robot_additional_jump_type, GoalInfo& cur_goal_info) {
    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& robot = dynamic_robots[i];
      robot->action = robot->plan.toMyAction(tick_number, true, true, robot->state.position);
      robot->nitroCheck();
      if (!robot->action.use_nitro) {
        robot->action = robot->plan.toMyAction(tick_number, true, false, robot->state.position);
      }
    }
    return tryTickWithJumpsDynamic(tick_number, true, main_robot_additional_jump_type, cur_goal_info);
  }

  bool somebodyJumpThisTickDynamic() {
    for (int i = 0; i < dynamic_robots_size; ++i) {
      if (dynamic_robots[i]->action.jump_speed > 0) {
        return true;
      }
    }
    return false;
  }

  bool tickMicroticksDynamic(const int number_of_tick, const int number_of_microticks, GoalInfo& cur_goal_info) {
    clearTriggers();
    if (number_of_microticks == 0) {
      return false;
    }
    return updateDynamic((double) number_of_microticks / C::rules.TICKS_PER_SECOND / C::rules.MICROTICKS_PER_TICK, number_of_tick, number_of_microticks, cur_goal_info);
  }

  bool tickDihaDynamic(const int tick_number, GoalInfo& cur_goal_info) {
    bool sbd_wants_to_become_dynamic = false;
    GoalInfo goal_info = {false, false, -1};
    cur_goal_info = {false, false, -1};

    if (accurate) {
      for (int i = 0; i < C::MICROTICKS_PER_TICK; ++i) {
        sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1, goal_info);
        cur_goal_info |= goal_info;
      }
      return sbd_wants_to_become_dynamic;
    }

    int remaining_microticks = C::MICROTICKS_PER_TICK;
    const bool& flag = somebodyJumpThisTickDynamic();
    if (flag) {
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1, goal_info);
      remaining_microticks--;
      cur_goal_info |= goal_info;
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1, goal_info);
      remaining_microticks--;
      cur_goal_info |= goal_info;
    } else if (tick_number == 0) {
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1, goal_info); //todo do single initialisation (low prior)
      remaining_microticks--;
      cur_goal_info |= goal_info;
    }
    if (sbd_wants_to_become_dynamic) {
      return true;
    }

    clearTriggerFires();

    int iteration = 0;
    while (true) {
      iteration++;

      for (int i = 0; i < dynamic_entities_size; ++i) { // 3/2 time of diha!!!!
        dynamic_entities[i]->savePrevMicroState();
      }
      sbd_wants_to_become_dynamic = tickMicroticksDynamic(tick_number, remaining_microticks, goal_info);

      if (iteration == 1 && sbd_wants_to_become_dynamic) {
        return true;
      }

      if (anyTriggersActive() && remaining_microticks > 1) {
        for (int i = 0; i < dynamic_entities_size; ++i) {
          dynamic_entities[i]->fromPrevMicroState();
        }
        tickMicroticksDynamic(tick_number, 1, goal_info);
        int l;
        int r;
        if (anyTriggersActive()) {
          l = 0;
        } else {
          l = 1;
          r = remaining_microticks;
          while (r - l > 1) {
            const int& mid = (r + l) / 2;
            for (int i = 0; i < dynamic_entities_size; ++i) {
              dynamic_entities[i]->fromPrevMicroState();
            }
            tickMicroticksDynamic(tick_number, mid, goal_info);
            if (anyTriggersActive()) {
              r = mid;
            } else {
              l = mid;
            }
          }
        }
        for (int i = 0; i < dynamic_entities_size; ++i) {
          dynamic_entities[i]->fromPrevMicroState();
        }
        if (l > 0) {
          tickMicroticksDynamic(tick_number, l, goal_info);
          cur_goal_info |= goal_info;
          remaining_microticks -= l;
        }
        tickMicroticksDynamic(tick_number, 1, goal_info);
        cur_goal_info |= goal_info;
        setTriggersFired();
        remaining_microticks--;
      } else {
        cur_goal_info |= goal_info;
        break;
      }
    }
    return false;
  }

  int tickDynamic(const int tick_number, int viz_id = -1, bool viz = false) {
    if (goal_info.goal_to_me || goal_info.goal_to_enemy) {
      return 0;
    }
    int main_robot_additional_jump_type = 0;
    wantedStaticGoToDynamic(tick_number);
    for (int i = 0; i < static_entities_size; ++i) {
      static_entities[i]->fromStateStatic(tick_number + 1);
    }
    for (int i = 0; i < dynamic_entities_size; ++i) {
      dynamic_entities[i]->savePrevState();
    }

    GoalInfo cur_goal_info;

    bool flag = tryDoTickWithoutAnybodyBecomingDynamic(tick_number, main_robot_additional_jump_type, cur_goal_info);

    if (flag) {
      for (int i = 0; i < dynamic_entities_size; ++i) {
        dynamic_entities[i]->fromPrevState();
      }
      wantedStaticGoToDynamic(tick_number);
      for (int i = 0; i < dynamic_entities_size; ++i) {
        dynamic_entities[i]->savePrevState();
      }
      tryDoTickWithoutAnybodyBecomingDynamic(tick_number, main_robot_additional_jump_type, cur_goal_info);
    }

    for (int i = 0; i < dynamic_packs_size; ++i) {
      auto& pack = dynamic_packs[i];
      if (pack->state.alive) {
        continue;
      }
      pack->state.respawn_ticks--;
      if (pack->state.respawn_ticks == 0) {
        pack->state.alive = true;
      }
    }

    if (cur_goal_info.goal_to_me || cur_goal_info.goal_to_enemy) {
      goal_info = cur_goal_info;
      goal_info.goal_tick = tick_number;
    }

#ifdef DEBUG
    if (main_robot->id == viz_id && viz) {
      for (int i = 0; i < dynamic_entities_size; ++i) {
        auto& e = dynamic_entities[i];
        P::drawLine(e->state.position, e->prev_state.position, accurate ? 0x00FF00 : 0x0000FF);
      }
      for (int i = 0; i < static_entities_size; ++i) {
        auto& e = static_entities[i];
        P::drawLine(
            e->states[tick_number].position,
            e->states[tick_number + 1].position, accurate ? 0x00FFFF : 0xFF00FF);
      }
    }
#endif
    return main_robot_additional_jump_type;
  }

  bool collideEntitiesDynamic(const int number_of_tick, const int number_of_microticks, Entity* a, Entity* b, bool check_with_ball) {
    const Point& delta_position = b->state.position - a->state.position;
    const double distance_sq = delta_position.length_sq();
    const double sum_r = a->state.radius + b->state.radius;
    if (check_with_ball) {
      if ((3 + jr * a->action.max_jump_speed) * (3 + jr * a->action.max_jump_speed) > distance_sq) {
        a->collide_with_ball = true;
      }
    } else {
      if (a->is_teammate && !b->state.touch && (2 + jr * a->action.max_jump_speed) * (2 + jr * a->action.max_jump_speed) > distance_sq) {
        a->collide_with_entity_in_air = true;
      } else if (b->is_teammate && !a->state.touch && (2 + jr * b->action.max_jump_speed) * (2 + jr * b->action.max_jump_speed) > distance_sq) {
        b->collide_with_entity_in_air = true;
      }
    }
    if (sum_r * sum_r > distance_sq) {
      const double penetration = sum_r - sqrt(distance_sq);
      const double k_a = 1. / (a->mass * ((1 / a->mass) + (1 / b->mass)));
      const double k_b = 1. / (b->mass * ((1 / a->mass) + (1 / b->mass)));
      const Point& normal = delta_position.normalize();
      a->state.position -= normal * (penetration * k_a);
      b->state.position += normal * (penetration * k_b);
      const double delta_velocity = (b->state.velocity - a->state.velocity).dot(normal) - (b->radius_change_speed + a->radius_change_speed);
      if (check_with_ball) {
        //if (!accurate && main_robot->id == 4) {
        //  //H::t[18].call();
        //}
        entity_ball_collision_trigger = true;
      } else if (!a->state.touch && !b->state.touch) {
        entity_entity_collision_trigger = true;
      }
      if (delta_velocity < 0) {
        const Point& impulse = normal * ((1. + hit_e) * delta_velocity);
        a->state.velocity += impulse * k_a;
        b->state.velocity -= impulse * k_b;
        return true;
      }
    }
    return false;
  }

  /* inline bool collideEntitiesCheckDynamic(Entity* a_static, Entity* b_dynamic) {
     const double& sum_r = a_static->state_ptr->radius + b_dynamic->state.radius;
     const double& dx = a_static->state_ptr->position.x - b_dynamic->state.position.x;
     if (fabs(dx) > sum_r) {
       return false;
     }
     const double& dz = a_static->state_ptr->position.z - b_dynamic->state.position.z;
     if (fabs(dz) > sum_r) {
       return false;
     }
     const double& dy = a_static->state_ptr->position.y - b_dynamic->state.position.y;
     if (fabs(dy) > sum_r) {
       return false;
     }
     return
         sum_r
             * sum_r >
             dx * dx + dy * dy + dz * dz;
   }*/

  bool collideEntitiesCheckDynamic(Entity* a_static, Entity* b_dynamic) {
    return
        (a_static->state_ptr->radius + b_dynamic->state.radius)
            * (a_static->state_ptr->radius + b_dynamic->state.radius) >
            (b_dynamic->state.position - a_static->state_ptr->position).length_sq();
  }

  bool collideWithArenaDynamic(Entity* e, Point& result, int& collision_surface_id) {
    const Dan& dan = Dan::dan_to_arena(e->state.position, e->state.radius);
    const double distance = dan.distance;
    if (e->state.radius > distance) {
      const Point& normal = dan.normal.normalize();
      const double& penetration = e->state.radius - distance;
      e->state.position += normal * penetration;
      const double& velocity = e->state.velocity.dot(normal) - e->radius_change_speed;
      if (velocity < 0) {
        e->state.velocity -= normal * ((1. + e->arena_e) * velocity);
        result = normal;
        collision_surface_id = dan.collision_surface_id;
        return true;
      }
    }
    return false;
  }

  void moveDynamic(Entity* e, const double delta_time) {
    const double& length_sq = e->state.velocity.length_sq();
    if (length_sq > 10000.) {
      e->state.velocity *= (100. / sqrt(length_sq));
    }
    e->state.position += e->state.velocity * delta_time;
    e->state.position.y -= 15. * delta_time * delta_time;
    e->state.velocity.y -= 30. * delta_time;
  }

  void updateStatic(const double delta_time, const int number_of_tick, const int number_of_microticks) {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& robot = initial_static_robots[i];
      if (robot->state.touch) { //todo nitro max speed clamp
        const Point& target_velocity = robot->action.target_velocity - robot->state.touch_normal * robot->state.touch_normal.dot(robot->action.target_velocity);
        const Point& target_velocity_change = target_velocity - robot->state.velocity;
        double length = target_velocity_change.length_sq();
        if (length > 0) {
          const double& acceleration = C::rules.ROBOT_ACCELERATION * fmax(0., robot->state.touch_normal.y);
          length = sqrt(length);
          const double& delta = length - acceleration * delta_time;
          if (delta > 0) {
            const auto& robot_acceleration = target_velocity_change * (acceleration * delta_time / length);
            robot->state.velocity += robot_acceleration;
            const double& coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / 2. / number_of_microticks) : 0.;
            robot->state.position -= robot_acceleration * (coef * delta_time);
          } else {
            if (robot->state.touch_surface_id == 1) {
              acceleration_trigger = true;
            }
            robot->state.velocity += target_velocity_change;
          }
        }
      } else {
        if (robot->is_teammate && robot->action.use_nitro && robot->state.nitro > 0) {
          const auto& target_velocity_change = (robot->action.target_velocity - robot->state.velocity);
          const auto& tvc_length_sq = target_velocity_change.length_sq();
          if (tvc_length_sq > 0) {
            //const auto& max_nitro_change = robot->state.nitro * C::rules.NITRO_POINT_VELOCITY_CHANGE;
            const auto& ac_per_dt = C::rules.ROBOT_NITRO_ACCELERATION * delta_time;
            const auto& robot_acceleration = target_velocity_change * (ac_per_dt / sqrt(tvc_length_sq));
            robot->state.velocity += robot_acceleration;
            robot->state.nitro -= ac_per_dt / C::rules.NITRO_POINT_VELOCITY_CHANGE;
            const double coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / 2. / number_of_microticks) : 0.;
            robot->state.position -= robot_acceleration * (coef * delta_time);
          }
        }
      }

      moveStatic(robot, delta_time);

      robot->state.radius = C::rules.ROBOT_MIN_RADIUS + (C::rules.ROBOT_MAX_RADIUS - C::rules.ROBOT_MIN_RADIUS) * robot->action.jump_speed / C::rules.ROBOT_MAX_JUMP_SPEED;
      robot->radius_change_speed = robot->action.jump_speed;
    }

    moveStatic(ball, delta_time);

    for (int i = 0; i < initial_static_robots_size; i++) {
      for (int j = 0; j < i; j++) {
        collideEntitiesStatic(number_of_tick, initial_static_robots[i], initial_static_robots[j], false);
      }
    }

    Point collision_normal;
    int touch_surface_id;
    for (int i = 0; i < initial_static_robots_size; i++) {
      auto& robot = initial_static_robots[i];
      collideEntitiesStatic(number_of_tick, robot, ball, true);
      if (!collideWithArenaStatic(robot, collision_normal, touch_surface_id)) {
        if (robot->state.touch) {
          entity_arena_collision_trigger = true;
        }
        robot->state.touch = false;
      } else {
        if (!robot->state.touch || robot->state.touch_surface_id != touch_surface_id) {
          entity_arena_collision_trigger = true;
        }
        if (robot->is_teammate && touch_surface_id != 1) {
          robot->additional_jump = true;
        }
        robot->state.touch_surface_id = touch_surface_id;
        robot->state.touch = true;
        robot->state.touch_normal = collision_normal;
      }
    }
    if (!collideWithArenaStatic(ball, collision_normal, touch_surface_id)) {
      if (ball->state.touch) {
        if (ball->state.touch_surface_id != 1 || ball->state.velocity.y > C::ball_antiflap) {
          ball_arena_collision_trigger = true;
          ball->state.touch = false;
        }
      }
    } else {
      if (!ball->state.touch || ball->state.touch_surface_id != touch_surface_id) {
        ball_arena_collision_trigger = true;
      }
      ball->state.touch_surface_id = touch_surface_id;
      ball->state.touch = true;
    }

    for (int i = 0; i < initial_static_robots_size; i++) {
      auto& robot = initial_static_robots[i];
      if (robot->state.nitro == C::rules.MAX_NITRO_AMOUNT) {
        continue;
      }
      for (int j = 0; j < initial_static_packs_size; ++j) {
        auto& pack = initial_static_packs[j];
        if (!pack->state.alive) {
          continue;
        }
        const double& sum_r = robot->state.radius + pack->state.radius;
        if ((robot->state.position - pack->state.position).length_sq() <= sum_r * sum_r) {
          robot->state.nitro = C::rules.MAX_NITRO_AMOUNT;
          pack->state.alive = false;
          pack->state.respawn_ticks = C::rules.NITRO_PACK_RESPAWN_TICKS;
        }
      }
    }

  }


  // trajectory finding

  // 1. gall keeper:

  // 1.1 if has no nitro try safely swap with somebody with nitro

  // 1.2 if has no trajectory
  // 1.2.1 try find one vector2d trajectory without nitro
  //       this trajectory should be a:
  //       pass, a long ball flight, or any save from gall

  // 1.2.2 same as 1.2.1 with nitro vector3d

  // 1.2.3 same as 1.2.1 but stay N ticks, then one vector2d

  // 1.2.4 same as 1.2.2 but stay N ticks, then one vector3d

  // 1.2.5 try two vectors2d without nitro

  // 1.2.6 try two vectors3d without nitro


  double getSumScoreFighter(const int tick_number) {
    double score = 0;
    if (goal_info.goal_to_me) {
      score += tick_number == goal_info.goal_tick ? -1e3 : 0;
    } else if (goal_info.goal_to_enemy) {
      score += tick_number == goal_info.goal_tick ? 1e3 : 0;
    }
    if (!(goal_info.goal_to_me || goal_info.goal_to_enemy) || tick_number <= goal_info.goal_tick) {

      if (!main_robot->state.touch) {
        score -= 0.5 * C::TPT;
      }

      //if (main_robot->collide_with_ball) {
      //  score += 1;
      //}
      //if (main_robot->action.use_nitro) {
      //  //score -= 0.1 * C::TPT;
      //}
      score += 0.01 * main_robot->taken_nitro;

      /*for (int i = 0; i < static_robots_size; ++i) {
        auto& e = static_robots[i];
        if (!e->is_teammate && e->static_event_ptr->collide_with_ball) {
          score -= 10;
        }
      }
      for (int i = 0; i < dynamic_robots_size; ++i) {
        auto& e = dynamic_robots[i];
        if (!e->is_teammate && e->collide_with_ball) {
          score -= 10;
        }
      }*/
      /*if (tick_number < 100) {
        const int cell_x = std::clamp((int) ((ball->getState().position.x + 40. - 1.) / 2.), 0, 78);
        const int cell_y = std::clamp((int) ((ball->getState().position.y - 1.) / 2.), 0, 18);
        const int cell_z = std::clamp((int) ((ball->getState().position.z + 30. - 1.) / 2.), 0, 58);
        const int sum = H::danger_grid[cell_x][cell_y][cell_z][tick_number]
            + H::danger_grid[cell_x + 1][cell_y][cell_z][tick_number]
            + H::danger_grid[cell_x][cell_y + 1][cell_z][tick_number]
            + H::danger_grid[cell_x][cell_y][cell_z + 1][tick_number]
            + H::danger_grid[cell_x + 1][cell_y + 1][cell_z][tick_number]
            + H::danger_grid[cell_x + 1][cell_y][cell_z + 1][tick_number]
            + H::danger_grid[cell_x][cell_y + 1][cell_z + 1][tick_number]
            + H::danger_grid[cell_x + 1][cell_y + 1][cell_z + 1][tick_number];
        score -= 0 * 0.1 * sum;
      }*/
    }

    return score;
  }

  double getMinDistToGoalScoreFighter() {
    const double& d1 = (Point{
        -C::rules.arena.goal_width / 2 + 2,
        C::rules.arena.goal_height - 2,
        C::rules.arena.depth / 2 + 2} - ball->getState().position).length_sq();
    const double& d2 = (Point{
        0,
        C::rules.arena.goal_height - 2,
        C::rules.arena.depth / 2 + 2} - ball->getState().position).length_sq();
    const double& d3 = (Point{
        C::rules.arena.goal_width / 2 - 2,
        C::rules.arena.goal_height - 2,
        C::rules.arena.depth / 2 + 2} - ball->getState().position).length_sq();
    return sqrt(std::min(d1, std::min(d2, d3)));
  }

  double getMinDistToBallScoreFighter() {
    return (main_robot->state.position - ball->getState().position).length();
  }

  double getSumScoreEnemy(const int tick_number) {
    double score = 0;
    if (goal_info.goal_to_me) {
      score += tick_number == goal_info.goal_tick ? 1e3 : 0;
    } else if (goal_info.goal_to_enemy) {
      score += tick_number == goal_info.goal_tick ? -1e3 : 0;
    }
    if (!(goal_info.goal_to_me || goal_info.goal_to_enemy) || tick_number <= goal_info.goal_tick) {
      if (!main_robot->state.touch) {
        score -= 0.5 * C::TPT;
      }
      if (main_robot->collide_with_ball) {
        score += 0 * 20;
      }
    }
    return score;
  }

  double getMinDistToGoalScoreEnemy() {
    const double& d1 = (Point{
        -C::rules.arena.goal_width / 2 + 2,
        C::rules.arena.goal_height - 2,
        -C::rules.arena.depth / 2 - 2} - ball->getState().position).length_sq();
    const double& d2 = (Point{
        0,
        C::rules.arena.goal_height - 2,
        -C::rules.arena.depth / 2 - 2} - ball->getState().position).length_sq();
    const double& d3 = (Point{
        C::rules.arena.goal_width / 2 - 2,
        C::rules.arena.goal_height - 2,
        -C::rules.arena.depth / 2 - 2} - ball->getState().position).length_sq();
    return sqrt(std::min(d1, std::min(d2, d3)));
  }

  double getMinDistToBallScoreEnemy() {
    return (main_robot->state.position - ball->getState().position).length();
  }

  double getSumScoreDefender(const int tick_number) {
    double score = 0;
    if (goal_info.goal_to_me) {
      score += tick_number == goal_info.goal_tick ? -1e3 : 0;
    } else if (goal_info.goal_to_enemy) {
      score += tick_number == goal_info.goal_tick ? 1e3 : 0;
    }
    if (!(goal_info.goal_to_me || goal_info.goal_to_enemy) || tick_number <= goal_info.goal_tick) {
      if (!main_robot->state.touch) {
        score -= 0.5 * C::TPT;
      }
      score += 0.01 * main_robot->taken_nitro;

      //if (main_robot->collide_with_ball) {
      //  score += 1;
      //}
      //if (main_robot->action.use_nitro) {
      //score -= 0.1 * C::TPT;
      //}

      /*for (int i = 0; i < static_robots_size; ++i) {
        auto& e = static_robots[i];
        if (!e->is_teammate && e->static_event_ptr->collide_with_ball) {
          score -= 10;
        }
      }
      for (int i = 0; i < dynamic_robots_size; ++i) {
        auto& e = dynamic_robots[i];
        if (!e->is_teammate && e->collide_with_ball) {
          score -= 10;
        }
      }*/


      /*if (tick_number < 100) {
        const int cell_x = std::clamp((int) ((ball->getState().position.x + 40. - 1.) / 2.), 0, 78);
        const int cell_y = std::clamp((int) ((ball->getState().position.y - 1.) / 2.), 0, 18);
        const int cell_z = std::clamp((int) ((ball->getState().position.z + 30. - 1.) / 2.), 0, 58);
        const int sum = H::danger_grid[cell_x][cell_y][cell_z][tick_number]
            + H::danger_grid[cell_x + 1][cell_y][cell_z][tick_number]
            + H::danger_grid[cell_x][cell_y + 1][cell_z][tick_number]
            + H::danger_grid[cell_x][cell_y][cell_z + 1][tick_number]
            + H::danger_grid[cell_x + 1][cell_y + 1][cell_z][tick_number]
            + H::danger_grid[cell_x + 1][cell_y][cell_z + 1][tick_number]
            + H::danger_grid[cell_x][cell_y + 1][cell_z + 1][tick_number]
            + H::danger_grid[cell_x + 1][cell_y + 1][cell_z + 1][tick_number];
        score -= 0 * 0.1 * sum;
      }*/
    }
    score -= (0.0025 * C::TPT) * (main_robot->state.position - Point{
        0,
        1,
        -C::rules.arena.depth / 2}).length();

    return score;
  }

  double getMinDistToEnemyScore() {
    double min_dist = 1e9;
    for (int i = 0; i < static_robots_size; ++i) {
      auto& e = static_robots[i];
      if (!e->is_teammate) {
        min_dist = std::min(min_dist, (e->getState().position - ball->getState().position).length_sq());
      }
    }
    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& e = dynamic_robots[i];
      if (!e->is_teammate) {
        min_dist = std::min(min_dist, (e->getState().position - ball->getState().position).length_sq());
      }
    }
    return sqrt(min_dist);
  }

  double getMinDistFromGoalScoreDefender() {
    return ball->getState().position.z;
  }

  double getMinDistToBallScoreDefender() {
    return 0.1 * (main_robot->state.position - ball->getState().position).length();
  }

};

}
#endif
#endif //CODEBALL_SMARTSIMULATOR_H

---

*** H.h ***
---
#ifndef CODEBALL_HELPER_H
#define CODEBALL_HELPER_H

#ifdef LOCAL
#include <model/C.h>
#include <model/Plan.h>
#else
#include "model/C.h"
#include "model/Plan.h"
#endif


struct H {
  static model::Game game;

  static int cur_round_tick;
  static int tick;
  static model::Action actions[7];
  static int global_id;
  static int my_id;

  static Point2d prev_last_action[6];
  static Plan last_best_plan[6];
  static Plan best_plan[6];
  static Plan last_action_plan[6];
  static Plan last_action0_plan[6];
  static int player_score[2];
  static int waiting_ticks;
  static double time_limit;
  static double cur_tick_remaining_time;

  static double sum_iterations;
  static double iterations_k;

  static int danger_grid[60][20][100][C::MAX_SIMULATION_DEPTH];
  static DGState used_cells[1000007];
  static int used_cells_size;

  static Point prev_velocity[7];
  static Point prev_position[7];
  static std::map<int, int> best_plan_type;

  enum ROLE {FIGHTER, SEMI, DEFENDER};

  static ROLE role[6];

  static bool flag;

  static int tryInit(
      const model::Robot& _me,
      const model::Rules& _rules,
      const model::Game& _game) {
    global_id = _me.id;
    if (tick == _game.current_tick) {
      if (!flag) {
        flag = true;
        return 2;
      }
      return 3;
    }
    H::global_timer.start();
    game = _game;
    std::sort(game.robots.begin(), game.robots.end(), [](const auto& a, const auto& b) {
      return a.id < b.id;
    });
    C::rules = _rules;
    tick = game.current_tick;
    cur_round_tick++;
    if (cur_round_tick % C::TPT == 0) {
      actions[1] = actions[2] = actions[3] = model::Action();
    }
    if (tick == 0) { // init on tick 0
      for (auto& player : game.players) {
        if (player.me) {
          my_id = player.id;
          break;
        }
      }
      player_score[0] = player_score[1] = 0;
      waiting_ticks = 0;
      cur_round_tick = 0;
      C::rd.seed(229);
    }
    for (auto& player : game.players) {
      if (player_score[player.id - 1] != player.score) {
        player_score[player.id - 1] = player.score;
        waiting_ticks = 119;
        cur_round_tick = -119;
        std::cout << int(sum_iterations / iterations_k) << " ";
        sum_iterations = 0;
        iterations_k = 0;
      }
    }
    if (waiting_ticks > 0) {
      waiting_ticks--;
      return 3;
    }
    double time_per_tick = C::time_limit / 9000.;
    double ticks_remaining = (18000. - tick) / 2.;
    double half_ticks_remaining = ticks_remaining / 2.;
    double tick_end_balance = tick / 2 + half_ticks_remaining;
    double time_end_balance = C::time_limit - time_per_tick * (ticks_remaining - half_ticks_remaining);

    cur_tick_remaining_time = std::min(10., (time_end_balance - global_timer.getCumulative()) / half_ticks_remaining);
    //cur_tick_remaining_time = (C::time_limit - global_timer.getCumulative(true)) / ((18000 - (double)tick) / 2);

    flag = false;
    return 1;
  }

  static model::Action getCurrentAction() {
    return actions[global_id];
  }

  static int getRobotGlobalIdByLocal(int id) {
    if (my_id == 1) {
      return id + 1;
    } else if (id < 3) {
      return id + 4;
    } else {
      return id - 2;
    }
  }

  static int getRobotLocalIdByGlobal(int id) {
    if (my_id == 1) {
      return id - 1;
    } else if (id <= 3) {
      return id + 2;
    } else {
      return id - 4;
    }
  }

  static bool solve(double v0x, double v0z, double v1x, double v1z, double dvx, double dvz, double& ax, double& az) {
    const double eps = 1e-9;
    if (dvx * dvx + dvz * dvz < eps) {
      ax = v1x;
      az = v1z;
      return true;
    }
    double c = v0x * dvz - v0z * dvx;
    double D = 4 * dvz * dvz * c * c - 4 * (dvx * dvx + dvz * dvz) * (c * c - 900 * dvx * dvx);
    if (D >= 0) {
      double x1 = (2 * dvz * c + sqrt(D)) / (2 * (dvx * dvx + dvz * dvz));
      double x2 = (2 * dvz * c - sqrt(D)) / (2 * (dvx * dvx + dvz * dvz));
      double y1 = 900 - x1 * x1;
      double y2 = 900 - x2 * x2;

      if (y1 >= 0) {
        y1 = sqrt(y1);
        if (dvz * x1 - c < 0) {
          y1 = -y1;
        }
        if (fabs((x1 - v0x) * dvz - (y1 - v0z) * dvx) < eps) {
          ax = x1;
          az = y1;
          return true;
        }
      }

      if (y2 >= 0) {
        y2 = sqrt(y2);
        if (dvz * x2 - c > 0) {
          y2 = -y2;
        }
        if (fabs((x2 - v0x) * dvz - (y2 - v0z) * dvx) < eps) {
          ax = x2;
          az = y2;
          return true;
        }
      }

    }
    return false;
  }

  template<typename T>
  static int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  static bool solve2(Point2d v11, Point2d v12, Point2d v21, Point2d v22, Point& crossing) {

    Point2d cut1 = v12 - v11;
    Point2d cut2 = v22 - v21;

    double z1 = cut1.x * (v21 - v11).y - cut1.y * (v21 - v11).x;
    double z2 = cut1.x * (v22 - v11).y - cut1.y * (v22 - v11).x;

    if (sgn(z1) == sgn(z2) || (z1 == 0) || (z2 == 0))
      return false;

    z1 = cut2.x * (v11 - v21).y - cut2.y * (v11 - v21).x;
    z2 = cut2.x * (v12 - v21).y - cut2.y * (v12 - v21).x;

    if (sgn(z1) == sgn(z2) || (z1 == 0) || (z2 == 0))
      return false;

    crossing.x = v11.x + cut1.x * fabs(z1) / fabs(z2 - z1);
    crossing.z = v11.y + cut1.y * fabs(z1) / fabs(z2 - z1);

    return true;

  }

  static MyTimer t[100];
  static MyTimer c[100];
  static MyTimer global_timer;
  static MyTimer cur_tick_timer;
};

#ifndef LOCAL
namespace Frozen {

struct H {
  static model::Game game;

  static int tick;
  static model::Action actions[7];
  static int global_id;
  static int my_id;

  static Point2d prev_last_action[6];
  static Plan last_best_plan[6];
  static Plan best_plan[6];
  static Plan last_action_plan[6];
  static int player_score[2];
  static int waiting_ticks;
  static double time_limit;
  static double half_time;

  static double sum_asserts_failed;
  static double asserts_failed_k;

  static int danger_grid[80][20][60][100];
  static DGState used_cells[1000007];
  static int used_cells_size;

  static Point prev_velocity[7];
  static Point prev_position[7];

  static bool tryInit(
      const model::Robot& _me,
      const model::Rules& _rules,
      const model::Game& _game) {
    global_id = _me.id;
    if (tick == _game.current_tick) {
      return false;
    }
    H::global_timer.start();
    game = _game;
    std::sort(game.robots.begin(), game.robots.end(), [](const auto& a, const auto& b) {
      return a.id < b.id;
    });
    C::rules = _rules;
    tick = game.current_tick;
    actions[1] = actions[2] = actions[3] = actions[4] = model::Action();
    if (tick == 0) { // init on tick 0
      for (auto& player : game.players) {
        if (player.me) {
          my_id = player.id;
          break;
        }
      }
      player_score[0] = player_score[1] = 0;
      waiting_ticks = 0;
      C::rd.seed(229);
    }
    for (auto& player : game.players) {
      if (player_score[player.id - 1] != player.score) {
        player_score[player.id - 1] = player.score;
        waiting_ticks = 119;
        std::cout << int(sum_asserts_failed / asserts_failed_k) << " ?_?" << std::endl;
      }
    }
    if (waiting_ticks > 0) {
      waiting_ticks--;
      return false;
    }
    double time_per_tick = C::time_limit / 18000.;
    double ticks_remaining = (18000 - tick);
    double half_ticks_remaining = ticks_remaining / 50.;
    double tick_end_balance = tick + half_ticks_remaining;
    double time_end_balance = C::time_limit - time_per_tick * (ticks_remaining - half_ticks_remaining);

    half_time = (time_end_balance - global_timer.getCumulative()) / half_ticks_remaining / 2;
    //std::cout << "time: " << std::fixed << std::setprecision(3) << half_time * 2000 << std::endl;
    time_limit = global_timer.getCumulative() + half_time * 2;
    //std::cout << "time_limit: " << std::fixed << std::setprecision(3) << time_limit << std::endl;
    //std::cout << "global_timer: " << std::fixed << std::setprecision(3) <<  global_timer.getCumulative() << std::endl;
    half_time = global_timer.getCumulative() + half_time;
    return true;
  }

  static model::Action getCurrentAction() {
    return actions[global_id];
  }

  static int getRobotGlobalIdByLocal(int id) {
    if (my_id == 1) {
      return id + 1;
    } else if (id < 2) {
      return id + 3;
    } else {
      return id - 1;
    }
  }

  static int getRobotLocalIdByGlobal(int id) {
    if (my_id == 1) {
      return id - 1;
    } else if (id <= 2) {
      return id + 1;
    } else {
      return id - 3;
    }
  }

  static bool solve(double v0x, double v0z, double v1x, double v1z, double dvx, double dvz, double& ax, double& az) {
    const double eps = 1e-9;
    if (dvx * dvx + dvz * dvz < eps) {
      ax = v1x;
      az = v1z;
      return true;
    }
    double c = v0x * dvz - v0z * dvx;
    double D = 4 * dvz * dvz * c * c - 4 * (dvx * dvx + dvz * dvz) * (c * c - 900 * dvx * dvx);
    if (D >= 0) {
      double x1 = (2 * dvz * c + sqrt(D)) / (2 * (dvx * dvx + dvz * dvz));
      double x2 = (2 * dvz * c - sqrt(D)) / (2 * (dvx * dvx + dvz * dvz));
      double y1 = 900 - x1 * x1;
      double y2 = 900 - x2 * x2;

      if (y1 >= 0) {
        y1 = sqrt(y1);
        if (dvz * x1 - c < 0) {
          y1 = -y1;
        }
        if (fabs((x1 - v0x) * dvz - (y1 - v0z) * dvx) < eps) {
          ax = x1;
          az = y1;
          return true;
        }
      }

      if (y2 >= 0) {
        y2 = sqrt(y2);
        if (dvz * x2 - c > 0) {
          y2 = -y2;
        }
        if (fabs((x2 - v0x) * dvz - (y2 - v0z) * dvx) < eps) {
          ax = x2;
          az = y2;
          return true;
        }
      }

    }
    return false;
  }

  template<typename T>
  static int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  static bool solve2(Point2d v11, Point2d v12, Point2d v21, Point2d v22, Point& crossing) {

    Point2d cut1 = v12 - v11;
    Point2d cut2 = v22 - v21;

    double z1 = cut1.x * (v21 - v11).y - cut1.y * (v21 - v11).x;
    double z2 = cut1.x * (v22 - v11).y - cut1.y * (v22 - v11).x;

    if (sgn(z1) == sgn(z2) || (z1 == 0) || (z2 == 0)) // Отсекаем также и пограничные случаи
      return false;

    z1 = cut2.x * (v11 - v21).y - cut2.y * (v11 - v21).x;
    z2 = cut2.x * (v12 - v21).y - cut2.y * (v12 - v21).x;

    if (sgn(z1) == sgn(z2) || (z1 == 0) || (z2 == 0)) // Отсекаем также и пограничные случаи
      return false;

    crossing.x = v11.x + cut1.x * fabs(z1) / fabs(z2 - z1);
    crossing.z = v11.y + cut1.y * fabs(z1) / fabs(z2 - z1);

    return true;

  }

  static MyTimer t[100];
  static MyTimer c[100];
  static MyTimer global_timer;
};

}
#endif
#endif //CODEBALL_HELPER_H

---

*** MyStrategy.cpp ***
---
#ifdef LOCAL
#include <MyStrategy.h>
#include <model/C.h>
#include <model/P.h>
#include <H.h>
#include <SmartSimulator.h>
#else
#include "MyStrategy.h"
#include "SmartSimulator.h"
#include "model/C.h"
#include "model/P.h"
#include "H.h"
#endif

void clearBestPlans() {
  for (int id = 0; id < 3; id++) {
    H::best_plan[id].clearAndShift(C::MAX_SIMULATION_DEPTH);
  }
  for (int id = 3; id < 6; id++) {
    H::best_plan[id].clearAndShift(C::ENEMY_SIMULATION_DEPTH);
  }
}

void addCell(int x, int y, int z, int t) {
  if (x < 0 || y < 0 || z < 0 || t < 0) {
    return;
  }
  H::danger_grid[x][y][z][t]++;
  if (H::danger_grid[x][y][z][t] == 1) {
    H::used_cells[H::used_cells_size++] = {x, y, z, t};
  }
}

void addCell(int x, int y, int z, int t, int tpt) {
  for (int i = 0; i < tpt; ++i) {
    addCell(x, y, z, t * tpt + i);
  }
}

int enemiesPrediction() {

  for (int id = 0; id < 6; ++id) {
    for (auto& robot : H::game.robots) {
      if (robot.id == H::getRobotGlobalIdByLocal(id)) {
        static constexpr double jr = 0.0033333333333333333333333333333;
        double jump_speed = 0;
        if (!robot.touch) {
          jump_speed = (robot.radius - 1) / jr;
        }
        Entity e;
        e.fromRobot(robot);
        Entity ball;
        ball.fromBall(H::game.ball);
        bool is_dribler = (e.state.position - ball.state.position).length() < 3.05;
        H::last_action_plan[id] = Plan(71, C::MAX_SIMULATION_DEPTH, robot.velocity_x, robot.velocity_z, 0, 0, {0, 0, 0}, jump_speed, is_dribler);
        H::last_action0_plan[id] = Plan(710, C::MAX_SIMULATION_DEPTH, robot.velocity_x, robot.velocity_z, 0, 0, {0, 0, 0}, jump_speed, is_dribler);
      }
    }
  }

  //H::t[2].start();
  for (int id = 0; id < 6; ++id) {
    for (auto& robot : H::game.robots) {
      if (robot.id == H::getRobotGlobalIdByLocal(id)) {
        Point v0 = H::prev_velocity[id];
        Point v1 = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
        double dvx = (v1.x - v0.x);
        double dvy = (v1.y - v0.y);
        double dvz = (v1.z - v0.z);
        double ax, az;
        static constexpr double jr = 0.0033333333333333333333333333333;
        double jump_speed = 0;
        if (!robot.touch) {
          jump_speed = (robot.radius - 1) / jr;
        }
        Entity e;
        e.fromRobot(robot);
        Entity ball;
        ball.fromBall(H::game.ball);
        bool is_dribler = (e.state.position - ball.state.position).length() < 3.05;
        if (!robot.touch && !robot.is_teammate) {
          bool using_nitro = true;
          double eps = 1e-9;
          if (fabs(dvx) < eps && fabs(dvz) < eps && fabs(dvy - (-1./2)) < eps) {
            using_nitro = false;
          }
          if (using_nitro) {
            //P::logn(dvy);
            double min_error = 1e9;
            Point best;
            for (double x = -1; x < 1; x += 0.01) {
              for (double y = -1; y < 1; y += 0.01) {
                if (x * x + y * y > 1) {
                  continue;
                }
                for (double z : {sqrt(1 - (x * x + y * y)), -sqrt(1 - (x * x + y * y))}) {
                  //P::logn(x * x + y * y + z * z);
                  Point target_velocity{x, y, z};
                  target_velocity = target_velocity.normalize() * 100;
                  const auto& target_velocity_change = target_velocity - v0;
                  const auto& tvc_length_sq = target_velocity_change.length_sq();
                  if (tvc_length_sq > 0) {
                    const auto& ac_per_dt = C::rules.ROBOT_NITRO_ACCELERATION / 60.;
                    const auto& robot_acceleration = target_velocity_change * (ac_per_dt / sqrt(tvc_length_sq));
                    Point velocity = v0 + robot_acceleration;
                    velocity.y -= 1. / 2.;
                    double cur_error = (velocity - v1).length_sq();
                    if (cur_error < min_error) {
                      min_error = cur_error;
                      best = target_velocity;
                    }
                  }
                }
              }
            }
            if (min_error < 1e9) {
              //P::logn("me: ", min_error);
              //P::logn("acc: ", (v1 - v0).length());
              Point p0 = H::prev_position[id];
              Point p1 = {robot.x, robot.y, robot.z};
              //P::drawLine(p1, p1 + best, 0xFFF000);
              //P::logn(best.x - robot.velocity_x, " ", best.y - robot.velocity_y, " ",  best.z - robot.velocity_z);
              //P::logn(best.x, " ",  best.y, " ",  best.z);
              H::last_action_plan[id] = Plan(72, C::MAX_SIMULATION_DEPTH, 0, 0, 0, 0, best, jump_speed, is_dribler);
              H::last_action0_plan[id] = Plan(720, C::MAX_SIMULATION_DEPTH, 0, 0, 0, 0, best, jump_speed, is_dribler);
              /*Point target_velocity = best;
              target_velocity = target_velocity.normalize() * 100;
              const auto& target_velocity_change = target_velocity - v0;
              const auto& tvc_length_sq = target_velocity_change.length_sq();
              if (tvc_length_sq > 0) {
                const auto& ac_per_dt = C::rules.ROBOT_NITRO_ACCELERATION / 60.;
                const auto& robot_acceleration = target_velocity_change * (ac_per_dt / sqrt(tvc_length_sq));
                P::logn(robot_acceleration.x, " ", robot_acceleration.y," ",  robot_acceleration.z);
                Point velocity = v0 + robot_acceleration;
                velocity.y -= 1. / 2.;
                double cur_error = (velocity - v1).length_sq();
                if (cur_error < min_error) {
                  min_error = cur_error;
                  best = target_velocity;
                }
              }*/

            }
          }
        } else if (H::solve(v0.x, v0.z, v1.x, v1.z, dvx, dvz, ax, az)) {

          Point2d cur{ax, az};
          Point2d prev = H::prev_last_action[H::getRobotLocalIdByGlobal(robot.id)];

          Point p0 = H::prev_position[id];
          Point p1 = {robot.x, robot.y, robot.z};
          //P::drawLine(p1, {p1.x + dvx * 30, 1, p1.z + dvz * 30}, 0x0000FF);
          Point crossing;
          //P::drawLine({p0.x, 1, p0.z}, {p0.x + prev.x, 1, p0.z + prev.y}, 0xFF0000);
          //P::drawLine({p1.x, 1, p1.z}, {p1.x + cur.x, 1, p1.z + cur.y}, 0xFF0000);
          if (0 && H::solve2(
              {p0.x, p0.z},
              {p0.x + prev.x, p0.z + prev.y},
              {p1.x, p1.z},
              {p1.x + cur.x, p1.z + cur.y},
              crossing)) {
            //P::logn("kek");
            //P::drawLine(p0, crossing, 0xFF0000);
            //P::drawLine(p1, crossing, 0xFF0000);
            H::last_action_plan[id] = Plan(5, C::MAX_SIMULATION_DEPTH, ax, az, crossing.x, crossing.z);
            H::last_action0_plan[id] = Plan(50, C::MAX_SIMULATION_DEPTH, ax, az, crossing.x, crossing.z);
          } else {
            H::last_action_plan[id] = Plan(71, C::MAX_SIMULATION_DEPTH, ax, az, 0, 0, {0, 0, 0}, jump_speed, is_dribler);
            H::last_action0_plan[id] = Plan(710, C::MAX_SIMULATION_DEPTH, ax, az, 0, 0, {0, 0, 0}, jump_speed, is_dribler);
          }
          //if (!robot.is_teammate) {
          //  P::logn(robot.id, " ", sqrt(ax * ax + az * az));
          //  P::drawLine(p1, {p1.x + ax, p1.y, p1.z + az}, 0xFFFFFF);
          //}

          H::prev_last_action[H::getRobotLocalIdByGlobal(robot.id)] = {ax, az};

        }
      }
    }
  }
  //H::t[2].cur(true);
  for (int i = 0; i < H::used_cells_size; ++i) {
    const auto& cell = H::used_cells[i];
    H::danger_grid[cell.x][cell.y][cell.z][cell.t] = 0;

    // P::drawSphere({cell.x * 2 + 1 - 30, cell.y * 2 + 1, cell.z * 2 + 1 - 50}, 1, 0x00AA00);

  }
  H::used_cells_size = 0;

  int min_time_for_enemy_to_hit_the_ball = C::NEVER;

  //H::t[1].start();
  for (int enemy_id : {3, 4, 5}) {
    SmartSimulator simulator(true, C::TPT, C::ENEMY_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(enemy_id), 3, H::game.robots, H::game.ball, {});
    for (int iteration = 0; iteration < 100; iteration++) {
      Plan cur_plan(61, C::ENEMY_SIMULATION_DEPTH);
      if (iteration == 0) {
        cur_plan = H::best_plan[enemy_id];
      }
      cur_plan.score.start_fighter();
      simulator.initIteration(iteration, cur_plan);

      cur_plan.plans_config = 3;
      //double multiplier = 1.;
      bool main_fly_on_prefix = !(simulator.main_robot->state.touch && simulator.main_robot->state.touch_surface_id == 1);
      for (int sim_tick = 0; sim_tick < C::ENEMY_SIMULATION_DEPTH; sim_tick++) {
        simulator.tickDynamic(sim_tick);
        main_fly_on_prefix &= !(simulator.main_robot->state.touch && simulator.main_robot->state.touch_surface_id == 1);

        double x = simulator.main_robot->state.position.x + 30.;
        double y = simulator.main_robot->state.position.y;
        double z = simulator.main_robot->state.position.z + 50.;
        int cell_x = (int) (x / 2.);
        int cell_y = (int) (y / 2.);

        int cell_z = (int) (z / 2.);

        addCell(cell_x + 1, cell_y, cell_z, sim_tick, 1);
        addCell(cell_x, cell_y + 1, cell_z, sim_tick, 1);
        addCell(cell_x, cell_y, cell_z + 1, sim_tick, 1);
        addCell(cell_x - 1, cell_y, cell_z, sim_tick, 1);
        addCell(cell_x, cell_y - 1, cell_z, sim_tick, 1);
        addCell(cell_x, cell_y, cell_z - 1, sim_tick, 1);

        if (!main_fly_on_prefix && simulator.main_robot->collide_with_ball) {
          min_time_for_enemy_to_hit_the_ball = std::min(min_time_for_enemy_to_hit_the_ball, sim_tick);
        }

        /*
        cur_plan.score.sum_score += simulator.getSumScoreEnemy(sim_tick) * multiplier;
        cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreEnemy() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
        cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreEnemy() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
        if (sim_tick == enemy_depth - 1) {

          cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreEnemy();
        }*/

        //multiplier *= 0.999;
      }
      //H::best_plan[enemy_id] = std::max(H::best_plan[enemy_id], cur_plan);
    }
    /*if (enemy_id == 3) {
      Plan cur_plan = H::best_plan[enemy_id];

      SmartSimulator simulator_(
          C::MAX_SIMULATION_DEPTH,
          H::getRobotGlobalIdByLocal(enemy_id),
          cur_plan.plans_config,
          H::game.robots,
          H::game.ball,
          {},
          false,
          H::getRobotGlobalIdByLocal(enemy_id));
      simulator_.initIteration(250, cur_plan);

      for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
        simulator_.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(enemy_id), true);
      }
    }*/
  }
  //H::t[1].cur(true, true);
  //P::logn(H::t[1].avg());
  //P::logn("mtfethtb: ", min_time_for_enemy_to_hit_the_ball);
  return min_time_for_enemy_to_hit_the_ball;
}

void updateRoles() {

  int closest_to_goal;
  double closest_distance_to_goal = 1e9;
  for (auto& robot : H::game.robots) {
    if (robot.is_teammate) {
      Entity e;
      e.fromRobot(robot);
      double dist = (Point{0, 1, -42} - e.state.position).length();
      if (dist < closest_distance_to_goal) {
        closest_distance_to_goal = dist;
        closest_to_goal = robot.id;
      }
    }
  }

  int closest_to_ball;
  double closest_distance_to_ball = 1e9;
  Entity ball;
  ball.fromBall(H::game.ball);
  for (auto& robot : H::game.robots) {
    if (robot.is_teammate) {
      Entity e;
      e.fromRobot(robot);
      double dist = (Point{0,
          1,
          42} - e.state.position).length();
      if (robot.id != closest_to_goal && dist < closest_distance_to_ball) {
        closest_distance_to_ball = dist;
        closest_to_ball = robot.id;
      }
    }
  }

  int other;
  for (auto& robot : H::game.robots) {
    if (robot.is_teammate) {
      if (robot.id != closest_to_ball && robot.id != closest_to_goal) {
        other = robot.id;
      }
    }
  }
  H::role[H::getRobotLocalIdByGlobal(closest_to_goal)] = H::DEFENDER;
  //P::logn("def: ", closest_to_goal);
  H::role[H::getRobotLocalIdByGlobal(closest_to_ball)] = H::FIGHTER;
  //P::logn("fi: ", closest_to_ball);
  H::role[H::getRobotLocalIdByGlobal(other)] = H::SEMI;
  //P::logn("semi: ", other);
}

void doStrategy() {
#ifdef FROM_LOG
  for (auto& robot: H::game.robots) {
    Entity e;
    e.fromRobot(robot);
    P::drawEntities({e.state}, 0, e.is_teammate ? 0x00FF00 : 0xFF0000);
  }
  for (auto& nitro_pack : H::game.nitro_packs) {
    Entity e;
    e.fromPack(nitro_pack);
    if (e.state.alive) {
      P::drawEntities({e.state}, 0, 0x0000FF);
    }
  }
  Entity e;
  e.fromBall(H::game.ball);
  P::drawEntities({e.state}, 0, 0x333333);
#endif


  //todo saving packs collisions


#ifdef LOCAL
  H::cur_tick_timer.start();
#endif

  if (H::cur_round_tick % C::TPT == 0) {

    //P::logn("cur: ", H::cur_tick_remaining_time);
    //P::logn("sum: ", H::global_timer.getCumulative(true));

    //for (int i = 0; i < 100; ++i) {
    //  auto& t = H::t[i];
    //  t.clearCur();
    //}
    //for (int i = 0; i < 100; ++i) {
    //  auto& c = H::c[i];
    //  c.init_calls();
    //}

    //H::t[0].start();

    updateRoles();

    clearBestPlans();

    int min_time_for_enemy_to_hit_the_ball = enemiesPrediction();

    int iterations[3] = {200 * 2, 200 * 2, 200 * 2};
    //P::logn(H::cur_tick_remaining_time);
    double available_time[3] = {0, 0, 0};
    double available_time_prefix[3] = {H::global_timer.getCumulative() + H::cur_tick_remaining_time / 3, H::global_timer.getCumulative() + 2 * H::cur_tick_remaining_time / 3, H::global_timer.getCumulative() + H::cur_tick_remaining_time};

    bool ball_on_my_side = false;
    for (int id = 0; id < 3; id++) {
      int iteration = 0;
      SmartSimulator simulator_one(false, C::TPT, C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), 2, H::game.robots, H::game.ball, H::game.nitro_packs);
      SmartSimulator simulator_two(false, C::TPT, C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), 7, H::game.robots, H::game.ball, H::game.nitro_packs);

      bool need_minimax = false;
      if ((1 || simulator_one.static_goal_to_me
          || simulator_two.static_goal_to_me) &&
          (simulator_one.ball->state.position - simulator_two.ball->state.position).length() > 1e-9) {
        need_minimax = true;
      }

      if (id == 0) {
        for (int i = 0; i < C::MAX_SIMULATION_DEPTH; ++i) {
          if (simulator_one.ball->states[i].position.z < -0.01
              || simulator_two.ball->states[i].position.z < -0.01) {
            ball_on_my_side = true;
          }
        }
        if (!ball_on_my_side) {
          for (int i = 0; i < 3; ++i) {
            if (H::role[i] == H::DEFENDER) {
              available_time[i] = 0.1 * H::cur_tick_remaining_time;
              iterations[i] = 50 * 2;
            } else {
              iterations[i] = 275 * 2;
              available_time[i] = 0.45 * H::cur_tick_remaining_time;
            }
          }
          for (int i = 0; i < 3; ++i) {
            available_time_prefix[i] = i == 0 ? (H::global_timer.getCumulative() + available_time[i]) : (available_time[i] + available_time_prefix[i - 1]);
          }
        }
      }

      //for (;; iteration++) {
      //  if (iteration > iterations[id]) {
      //    break;
      //  }
      for (; H::global_timer.getCumulative(true) < available_time_prefix[id]; iteration++) {
        int plan_type;
        double rd = C::rand_double(0, 1);

        if (simulator_one.main_robot->state.touch
            && simulator_one.main_robot->state.touch_surface_id == 1) {
          if (rd < 1. / 7) {
            plan_type = 20;
          } else if (rd < 2. / 7) {
            plan_type = 21;
          } else if (rd < 3. / 7) {
            plan_type = 22;
          } else if (rd < 4. / 7) {
            plan_type = 23;
          } else if (rd < 5. / 7) {
            plan_type = 11;
          } else if (rd < 6. / 7) {
            plan_type = 12;
          } else {
            plan_type = 11;
          }
        } else {
          if (rd < 0.8) {
            plan_type = 31;
          } else {
            plan_type = 32;
          }
        }

        Plan cur_plan_one(plan_type, C::MAX_SIMULATION_DEPTH);
        if (iteration == 0) {
          cur_plan_one = H::best_plan[id];
        } else if (C::rand_double(0, 1) < 1. / 10.) { // todo check coefficient
          cur_plan_one = H::best_plan[id];
          cur_plan_one.mutate(cur_plan_one.configuration, C::MAX_SIMULATION_DEPTH);
        }

        if (H::role[id] == H::FIGHTER) {
          cur_plan_one.score.start_fighter();
        } else if (H::role[id] == H::SEMI) {
          cur_plan_one.score.start_fighter();
        } else if (H::role[id] == H::DEFENDER) {
          cur_plan_one.score.start_defender();
        }


        simulator_one.initIteration(iteration, cur_plan_one);

        Plan cur_plan_two = cur_plan_one;

        simulator_two.initIteration(iteration, cur_plan_two);

        cur_plan_one.plans_config = 2;

        cur_plan_two.plans_config = 7;
        if (!need_minimax) {
          cur_plan_two.score.sum_score = 1e18;
        }
        for (int minimax_id = need_minimax ? 0 : 1; minimax_id < 2; ++minimax_id) {
          auto& simulator = minimax_id == 0 ? simulator_two : simulator_one;
          auto& cur_plan = minimax_id == 0 ? cur_plan_two : cur_plan_one;

          double multiplier = 1.;
          double goal_multiplier = 1.;

          bool collide_with_smth = false;
          bool fly_on_prefix = (!simulator.main_robot->state.touch || simulator.main_robot->state.touch_surface_id != 1);

          for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {

            bool main_touch = (simulator.main_robot->state.touch && simulator.main_robot->state.touch_surface_id == 1) || simulator.main_robot->state.position.y < C::NITRO_TOUCH_EPSILON;

            int main_robot_additional_jump_type = simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(0), false);

            fly_on_prefix &= (!simulator.main_robot->state.touch || simulator.main_robot->state.touch_surface_id != 1);

            if (main_robot_additional_jump_type == 0 && simulator.main_robot->action.jump_speed > 0 && main_touch) {
              cur_plan.was_jumping = true;
            }

            if (main_robot_additional_jump_type > 0) { // 1 - with ball, 2 - with entity, 3 - additional
              if (fly_on_prefix &&
                  (main_robot_additional_jump_type == 1 || main_robot_additional_jump_type == 2)) {
                collide_with_smth = true;
              }
              if ((main_robot_additional_jump_type == 1 || main_robot_additional_jump_type == 2)
                  && cur_plan.was_jumping
                  && !cur_plan.was_on_ground_after_jumping
                  && !cur_plan.collide_with_entity_before_on_ground_after_jumping) {
                cur_plan.collide_with_entity_before_on_ground_after_jumping = true;
                if (H::role[id] == H::DEFENDER && main_robot_additional_jump_type == 1
                    && min_time_for_enemy_to_hit_the_ball < sim_tick
                    && cur_plan.time_jump <= min_time_for_enemy_to_hit_the_ball) {
                  cur_plan.score.minimal();
                  break;
                }
                if (sim_tick - cur_plan.time_jump > C::LONGEST_JUMP) {
                  cur_plan.score.minimal();
                  break;
                }
              }
              if (cur_plan.oncoming_jump == C::NEVER) {
                cur_plan.oncoming_jump = sim_tick;
                cur_plan.oncoming_jump_speed = main_robot_additional_jump_type == 3 ?
                    std::max(C::MIN_WALL_JUMP, cur_plan.max_jump_speed) : cur_plan.max_jump_speed;
              }
            }

            if (cur_plan.was_jumping && !cur_plan.was_on_ground_after_jumping && simulator.main_robot->state.touch) {
              cur_plan.was_on_ground_after_jumping = true;
              if (!cur_plan.collide_with_entity_before_on_ground_after_jumping) {
                cur_plan.score.minimal();
                break;
              }
            }

            if (H::role[id] == H::FIGHTER) {
              cur_plan.score.sum_score += simulator.getSumScoreFighter(sim_tick, goal_multiplier, ball_on_my_side) * multiplier;
              cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
              cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
              cur_plan.score.fighter_closest_enemy_ever = std::min(simulator.getMinDistToEnemyScore() * multiplier, cur_plan.score.fighter_closest_enemy_ever);
              if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
                cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreFighter();
              }
              if (sim_tick == C::ENEMY_LIVE_TICKS - 1) {
                cur_plan.score.fighter_closest_enemy_last = simulator.getMinDistToEnemyScore();
              }
            } else if (H::role[id] == H::DEFENDER) {
              cur_plan.score.sum_score += simulator.getSumScoreDefender(sim_tick, ball_on_my_side) * multiplier;
              cur_plan.score.defender_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreDefender() * multiplier, cur_plan.score.defender_min_dist_to_ball);
              cur_plan.score.defender_min_dist_from_goal = std::min(simulator.getMinDistFromGoalScoreDefender() * multiplier, cur_plan.score.defender_min_dist_from_goal);
              if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
                cur_plan.score.defender_last_dist_from_goal = simulator.getMinDistFromGoalScoreDefender();
              }
            } else if (H::role[id] == H::SEMI) {
              cur_plan.score.sum_score += simulator.getSumScoreFighter(sim_tick, goal_multiplier, ball_on_my_side) * multiplier;
              cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
              cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
              cur_plan.score.fighter_closest_enemy_ever = std::min(simulator.getMinDistToEnemyScore() * multiplier, cur_plan.score.fighter_closest_enemy_ever);
              if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
                cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreFighter();
              }
              if (sim_tick == C::ENEMY_LIVE_TICKS - 1) {
                cur_plan.score.fighter_closest_enemy_last = simulator.getMinDistToEnemyScore();
              }
            }

            multiplier *= 0.999;
            const double g_mult = 0.9;
            goal_multiplier *= g_mult * g_mult;
          }

          if (!collide_with_smth) {
            cur_plan.time_nitro_on = C::NEVER;
            cur_plan.time_nitro_off = C::NEVER;
          }

          if (!cur_plan.was_jumping) {
            cur_plan.time_jump = C::NEVER;
          } else if (cur_plan.was_jumping && !cur_plan.collide_with_entity_before_on_ground_after_jumping) {
            cur_plan.score.minimal();
          } else {
            if (cur_plan.oncoming_jump == C::NEVER) {
              cur_plan.oncoming_jump = cur_plan.time_jump;
              cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
            } else if (cur_plan.time_jump != C::NEVER) {
              if (cur_plan.oncoming_jump > cur_plan.time_jump) {
                cur_plan.oncoming_jump = cur_plan.time_jump;
                cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
              }
            }
          }
        }

        H::best_plan[id] = std::max(H::best_plan[id], std::min(cur_plan_one, cur_plan_two));
      }

      H::sum_iterations += iteration;
#ifdef DEBUG
      if (H::role[id] == H::DEFENDER) {
        P::logn("best plan id: ", H::best_plan[id].unique_id);
        P::logn("fighter score: ", H::best_plan[id].score.score());
        P::logn("sum_score: ", H::best_plan[id].score.sum_score);
        P::logn("fighter_min_dist_to_ball: ", -H::best_plan[id].score.fighter_min_dist_to_ball);
        P::logn("fighter_min_dist_to_goal: ", -H::best_plan[id].score.fighter_min_dist_to_goal);
        P::logn("fighter_last_dist_to_goal: ", -H::best_plan[id].score.fighter_last_dist_to_goal);
        P::logn("time_jump: ", H::best_plan[id].time_jump);
        P::logn("oncoming_jump: ", H::best_plan[id].oncoming_jump);
        P::logn("oncoming_jump_speed: ", H::best_plan[id].oncoming_jump_speed);
        P::logn("max_jump_speed: ", H::best_plan[id].max_jump_speed);
        P::logn("angle1: ", H::best_plan[id].angle1);
        P::logn("speed1: ", H::best_plan[id].speed1);
        P::logn("time_change: ", H::best_plan[id].time_change);
        P::logn("angle2: ", H::best_plan[id].angle2);
        P::logn("speed2: ", H::best_plan[id].speed2);
        P::logn("was_jumping: ", H::best_plan[id].was_jumping);
        P::logn("was_jumping: ", H::best_plan[id].was_jumping);
        P::logn("collide_with_entity_before_on_ground_after_jumping: ", H::best_plan[id].collide_with_entity_before_on_ground_after_jumping);
        P::logn("was_on_ground_after_jumping: ", H::best_plan[id].was_on_ground_after_jumping);

        Plan cur_plan = H::best_plan[id];

        SmartSimulator simulator(false, C::TPT, C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), cur_plan.plans_config, H::game.robots, H::game.ball, H::game.nitro_packs, false, H::getRobotGlobalIdByLocal(id));
        simulator.initIteration(iteration, cur_plan);

        SmartSimulator accurate_simulator(false, C::TPT, C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), cur_plan.plans_config, H::game.robots, H::game.ball, H::game.nitro_packs, true, H::getRobotGlobalIdByLocal(id));
        accurate_simulator.initIteration(iteration, cur_plan);

        for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
          simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(id), true);
          /*if (sim_tick < C::ENEMY_SIMULATION_DEPTH) {
            const int cell_x = std::clamp((int) ((simulator.ball->getState().position.x + 30. - 1.) / 2.), 0, 58);
            const int cell_y = std::clamp((int) ((simulator.ball->getState().position.y - 1.) / 2.), 0, 18);
            const int cell_z = std::clamp((int) ((simulator.ball->getState().position.z + 50. - 1.) / 2.), 0, 98);
            const int sum = H::danger_grid[cell_x][cell_y][cell_z][sim_tick]
                + H::danger_grid[cell_x + 1][cell_y][cell_z][sim_tick]
                + H::danger_grid[cell_x][cell_y + 1][cell_z][sim_tick]
                + H::danger_grid[cell_x][cell_y][cell_z + 1][sim_tick]
                + H::danger_grid[cell_x + 1][cell_y + 1][cell_z][sim_tick]
                + H::danger_grid[cell_x + 1][cell_y][cell_z + 1][sim_tick]
                + H::danger_grid[cell_x][cell_y + 1][cell_z + 1][sim_tick]
                + H::danger_grid[cell_x + 1][cell_y + 1][cell_z + 1][sim_tick];
            if (sum > 0) {
              P::drawEntities(simulator.ball->getState());
            }
          }*/
          accurate_simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(id), true);
        }
      }
#endif
    }
    H::iterations_k += 1;

#ifndef FROM_LOG
    for (auto& robot : H::game.robots) {
      if (robot.is_teammate) {
        H::best_plan_type[H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].configuration]++;
        Entity e;
        e.fromRobot(robot);
        Entity ball;
        ball.fromBall(H::game.ball);
        //P::logn(robot.id, ": ", (e.state.position - ball.state.position).length() - (e.state.radius + ball.state.radius));

        if (!robot.touch) {
          e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, true, e.state.position, e.state.velocity);
          e.nitroCheck();
          if (!e.action.use_nitro) {
            e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, false, e.state.position, e.state.velocity);
          }
        } else {
          e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, false, e.state.position, e.state.velocity);
        }
        H::actions[robot.id] = e.action.toAction();
      }
    }
    //for (auto& it : H::best_plan_type) {
    //  P::logn(it.first, " ", it.second);
    //}
#endif

    //H::t[0].cur(true);
    //for (int i = 0; i < 5; ++i) {
    //  auto& t = H::t[i];
    //  t.cur(false, true);
    //  P::logn("t", i, " avg: ", t.avg() * 1000, " cur: ", t.getCur() * 1000, " x", (int)(std::floor(t.getCur() / t.avg() * 100)), "%");
    //}

    //for (int i = 0; i < 5; ++i) {
    //  auto& c = H::c[i];
    //  c.capture();
    //  P::logn("c", i, " avg: ", c.avg_(), " cur: ", c.last_(), " x", (int)(std::floor((double)c.last_() / c.avg_() * 100)), "%");
    //}
  }

  for (auto& robot : H::game.robots) {
    int id = H::getRobotLocalIdByGlobal(robot.id);
    H::prev_velocity[id] = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
    H::prev_position[id] = {robot.x, robot.y, robot.z};
  }

#ifdef LOCAL
  H::cur_tick_timer.cur(true, true);
  std::cerr << H::cur_tick_timer.avg() << std::endl;
#endif
}

MyStrategy::MyStrategy() {}

#ifndef LOCAL
namespace Frozen {
void clearBestPlans() {
  for (int id = 0; id < 2; id++) {
    H::best_plan[id].clearAndShift(C::MAX_SIMULATION_DEPTH);
  }
  for (int id = 2; id < 4; id++) {
    H::best_plan[id].clearAndShift(C::ENEMY_SIMULATION_DEPTH);
  }
}

void enemiesPrediction() {

  for (int id = 0; id < 4; ++id) {
    for (auto& robot : H::game.robots) {
      if (robot.id == H::getRobotGlobalIdByLocal(id)) {
        H::last_action_plan[id] = Plan(4, C::MAX_SIMULATION_DEPTH, robot.velocity_x, robot.velocity_z);
      }
    }
  }

  for (int id = 0; id < 4; ++id) {
    for (auto& robot : H::game.robots) {
      if (robot.id == H::getRobotGlobalIdByLocal(id)) {
        Point v0 = H::prev_velocity[id];
        Point v1 = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
        double dvx = v1.x - v0.x;
        double dvz = v1.z - v0.z;
        double ax, az;
        if (H::solve(v0.x, v0.z, v1.x, v1.z, dvx, dvz, ax, az)) {

          Point2d cur{ax, az};
          Point2d prev = H::prev_last_action[H::getRobotLocalIdByGlobal(robot.id)];

          Point p0 = H::prev_position[id];
          Point p1 = {robot.x, robot.y, robot.z};
          //P::drawLine(p1, {p1.x + dvx * 30, 1, p1.z + dvz * 30}, 0x0000FF);
          Point crossing;
          //P::drawLine({p0.x, 1, p0.z}, {p0.x + prev.x, 1, p0.z + prev.y}, 0xFF0000);
          //P::drawLine({p1.x, 1, p1.z}, {p1.x + cur.x, 1, p1.z + cur.y}, 0xFF0000);
          if (0 && H::solve2(
              {p0.x, p0.z},
              {p0.x + prev.x, p0.z + prev.y},
              {p1.x, p1.z},
              {p1.x + cur.x, p1.z + cur.y},
              crossing)) {
            //P::logn("kek");
            //P::drawLine(p0, crossing, 0xFF0000);
            //P::drawLine(p1, crossing, 0xFF0000);
            H::last_action_plan[id] = Plan(5, C::MAX_SIMULATION_DEPTH, ax, az, crossing.x, crossing.z);
          } else {

            H::last_action_plan[id] = Plan(4, C::MAX_SIMULATION_DEPTH, ax, az);

          }
          //if (!robot.is_teammate) {
          //  P::logn(robot.id, " ", sqrt(ax * ax + az * az));
          //  P::drawLine(p1, {p1.x + ax, p1.y, p1.z + az}, 0xFFFFFF);
          //}

          H::prev_last_action[H::getRobotLocalIdByGlobal(robot.id)] = {ax, az};

        }
      }
    }
  }

  const int enemy_depth = 50;

  for (int enemy_id : {2, 3}) {
    continue;
    SmartSimulator simulator(enemy_depth, H::getRobotGlobalIdByLocal(enemy_id), 3, H::game.robots, H::game.ball, {});

    for (int iteration = 0; iteration < 100; iteration++) {
      Plan cur_plan(2, enemy_depth);
      if (iteration == 0) {
        cur_plan = H::best_plan[enemy_id];
      } else if (C::rand_double(0, 1) < 1. / 10.) {
        cur_plan = H::best_plan[enemy_id];
        cur_plan.mutate(2, enemy_depth);
      }
      cur_plan.score.start_fighter();
      simulator.initIteration(iteration, cur_plan);

      cur_plan.plans_config = 3;
      double multiplier = 1.;
      for (int sim_tick = 0; sim_tick < enemy_depth; sim_tick++) {
        /*int cell_x = std::clamp((int) ((simulator.main_robot->state.position.x + 40.) / 2.), 1, 78);
        int cell_y = std::clamp((int) ((simulator.main_robot->state.position.y + 1.) / 2.), 1, 18);
        int cell_z = std::clamp((int) ((simulator.main_robot->state.position.z + 30.) / 2.), 1, 58);
        H::danger_grid[cell_x + 1][cell_y][cell_z][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x + 1, cell_y, cell_z, sim_tick};
        H::danger_grid[cell_x][cell_y + 1][cell_z][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x, cell_y + 1, cell_z, sim_tick};
        H::danger_grid[cell_x][cell_y][cell_z + 1][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x, cell_y, cell_z + 1, sim_tick};

        H::danger_grid[cell_x - 1][cell_y][cell_z][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x - 1, cell_y, cell_z, sim_tick};
        H::danger_grid[cell_x][cell_y - 1][cell_z][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x, cell_y - 1, cell_z, sim_tick};
        H::danger_grid[cell_x][cell_y][cell_z - 1][sim_tick]++;
        H::used_cells[H::used_cells_size++] = {cell_x, cell_y, cell_z - 1, sim_tick};
        */
        simulator.tickDynamic(sim_tick);

        cur_plan.score.sum_score += simulator.getSumScoreEnemy(sim_tick) * multiplier;
        cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreEnemy() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
        cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreEnemy() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
        if (sim_tick == enemy_depth - 1) {
          cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreEnemy();
        }

        multiplier *= 0.999;
      }
      H::best_plan[enemy_id] = std::max(H::best_plan[enemy_id], cur_plan);
    }
    /*if (enemy_id == 3) {
      Plan cur_plan = H::best_plan[enemy_id];

      SmartSimulator simulator_(
          C::MAX_SIMULATION_DEPTH,
          H::getRobotGlobalIdByLocal(enemy_id),
          cur_plan.plans_config,
          H::game.robots,
          H::game.ball,
          {},
          false,
          H::getRobotGlobalIdByLocal(enemy_id));
      simulator_.initIteration(250, cur_plan);

      for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
        simulator_.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(enemy_id), true);
      }
    }*/
  }
}

void doStrategy() {
#ifdef FROM_LOG
  for (auto& robot: H::game.robots) {
    Entity e;
    e.fromRobot(robot);
    P::drawEntities({e.state}, 0, e.is_teammate ? 0x00FF00 : 0xFF0000);
  }
  for (auto& nitro_pack : H::game.nitro_packs) {
    Entity e;
    e.fromPack(nitro_pack);
    P::drawEntities({e.state}, 0, 0x0000FF);
  }
  Entity e;
  e.fromBall(H::game.ball);
  P::drawEntities({e.state}, 0, 0x333333);
#endif


  //todo saving packs collisions

  //for (int i = 0; i < 100; ++i) {
  //  auto& t = H::t[i];
  //  t.clearCur();
  //}
  //for (int i = 0; i < 100; ++i) {
  //  auto& c = H::c[i];
  //  c.init_calls();
  //}
  //H::t[0].start();

  if (H::tick % C::TPT == 0) {

    clearBestPlans();

    enemiesPrediction();

    int iterations[2] = {250 + 1, 250 + 1};
    for (int id = 1; id >= 0; id--) {
      int iteration = 0;
      SmartSimulator simulator_smart(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), 1, H::game.robots, H::game.ball, H::game.nitro_packs);
      SmartSimulator simulator_stupid(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), 2, H::game.robots, H::game.ball, H::game.nitro_packs);

      bool ball_on_my_side = false;
      if (id == 1) {
        for (int i = 0; i < C::MAX_SIMULATION_DEPTH; ++i) {
          if (simulator_smart.ball->states[i].position.z < -0.01 || simulator_stupid.ball->states[i].position.z < -0.01) {
            ball_on_my_side = true;
          }
        }
        if (!ball_on_my_side) {
          iterations[0] = 475 + 1;
          iterations[1] = 25 + 1;
        }
      }

      for (; H::global_timer.getCumulative(true) < H::time_limit; iteration++) {
        if (id == 1) {
          if (ball_on_my_side) {
            if (H::global_timer.getCumulative(true) > H::half_time) {
              break;
            }
          } else {
            if (H::global_timer.cur() > 0.002) {
              break;
            }
          }
        }
        /*for (;; iteration++) {
          if (iteration > iterations[id]) {
            break;
          }*/
        Plan cur_plan_smart(1, C::MAX_SIMULATION_DEPTH);
        if (iteration == 0) {
          cur_plan_smart = H::best_plan[id];
        } else if (C::rand_double(0, 1) < 1. / 10.) { // todo check coefficient
          cur_plan_smart = H::best_plan[id];
          cur_plan_smart.mutate(1, C::MAX_SIMULATION_DEPTH);
        }

        if (id == 0) {
          cur_plan_smart.score.start_fighter();
        } else {
          cur_plan_smart.score.start_defender();
        }

        simulator_smart.initIteration(iteration, cur_plan_smart);

        Plan cur_plan_stupid = cur_plan_smart;

        simulator_stupid.initIteration(iteration, cur_plan_stupid);

        cur_plan_smart.plans_config = 1;
        cur_plan_stupid.plans_config = 2;

        cur_plan_smart.score.sum_score += 1e18;
        for (int minimax = 1; minimax < 2; ++minimax) {
          auto& simulator = minimax == 0 ? simulator_smart : simulator_stupid;
          auto& cur_plan = minimax == 0 ? cur_plan_smart : cur_plan_stupid;
          double multiplier = 1.;
          for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {

            bool main_touch = simulator.main_robot->state.touch;

            int main_robot_additional_jump_type = simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(0), false);

            if (main_robot_additional_jump_type == 0 && simulator.main_robot->action.jump_speed > 0 && main_touch) {
              cur_plan.was_jumping = true;
            }

            if (main_robot_additional_jump_type > 0) { // 1 - with ball, 2 - with entity, 3 - additional
              if ((main_robot_additional_jump_type == 1 || main_robot_additional_jump_type == 2) && cur_plan.was_jumping && !cur_plan.was_on_ground_after_jumping) {
                cur_plan.collide_with_entity_before_on_ground_after_jumping = true;
              }
              if (cur_plan.oncoming_jump == C::NEVER) {
                cur_plan.oncoming_jump = sim_tick;
                cur_plan.oncoming_jump_speed = main_robot_additional_jump_type == 3 ?
                    std::max(C::MIN_WALL_JUMP, cur_plan.max_jump_speed) : cur_plan.max_jump_speed;
              }
            }

            if (cur_plan.was_jumping && !cur_plan.was_on_ground_after_jumping && simulator.main_robot->state.touch) {
              cur_plan.was_on_ground_after_jumping = true;
              if (!cur_plan.collide_with_entity_before_on_ground_after_jumping) {
                cur_plan.time_jump = C::NEVER;
              }
            }

            if (id == 0) {
              cur_plan.score.sum_score += simulator.getSumScoreFighter(sim_tick) * multiplier;
              cur_plan.score.fighter_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_ball);
              cur_plan.score.fighter_min_dist_to_goal = std::min(simulator.getMinDistToGoalScoreFighter() * multiplier, cur_plan.score.fighter_min_dist_to_goal);
              if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
                cur_plan.score.fighter_last_dist_to_goal = simulator.getMinDistToGoalScoreFighter();
              }
            } else if (id == 1) {
              cur_plan.score.sum_score += simulator.getSumScoreDefender(sim_tick) * multiplier;
              cur_plan.score.defender_min_dist_to_ball = std::min(simulator.getMinDistToBallScoreDefender() * multiplier, cur_plan.score.defender_min_dist_to_ball);
              cur_plan.score.defender_min_dist_from_goal = std::min(simulator.getMinDistFromGoalScoreDefender() * multiplier, cur_plan.score.defender_min_dist_from_goal);
              if (sim_tick == C::MAX_SIMULATION_DEPTH - 1) {
                cur_plan.score.defender_last_dist_from_goal = simulator.getMinDistFromGoalScoreDefender();
              }
            }
            multiplier *= 0.999;
          }

          if (cur_plan.was_jumping && !cur_plan.collide_with_entity_before_on_ground_after_jumping) {
            cur_plan.time_jump = C::NEVER;
          } else {

            if (cur_plan.oncoming_jump == C::NEVER) {
              cur_plan.oncoming_jump = cur_plan.time_jump;
              cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
            } else if (cur_plan.time_jump != C::NEVER) {
              if (cur_plan.oncoming_jump > cur_plan.time_jump) {
                cur_plan.oncoming_jump = cur_plan.time_jump;
                cur_plan.oncoming_jump_speed = cur_plan.max_jump_speed;
              }
            }
          }
        }
        H::best_plan[id] = std::max(H::best_plan[id], std::min(cur_plan_smart, cur_plan_stupid));
      }

      H::sum_asserts_failed += iteration;
#ifdef DEBUG
      if (id == 0) {
        P::logn("best plan id: ", H::best_plan[id].unique_id);
        P::logn("fighter score: ", H::best_plan[id].score.score());
        P::logn("sum_score: ", H::best_plan[id].score.sum_score);
        P::logn("fighter_min_dist_to_ball: ", -H::best_plan[id].score.fighter_min_dist_to_ball);
        P::logn("fighter_min_dist_to_goal: ", -H::best_plan[id].score.fighter_min_dist_to_goal);
        P::logn("fighter_last_dist_to_goal: ", -H::best_plan[id].score.fighter_last_dist_to_goal);
        P::logn("time_jump: ", H::best_plan[id].time_jump);
        P::logn("oncoming_jump: ", H::best_plan[id].oncoming_jump);
        P::logn("oncoming_jump_speed: ", H::best_plan[id].oncoming_jump_speed);
        P::logn("max_jump_speed: ", H::best_plan[id].max_jump_speed);
        P::logn("angle1: ", H::best_plan[id].angle1);
        P::logn("speed1: ", H::best_plan[id].speed1);
        P::logn("time_change: ", H::best_plan[id].time_change);
        P::logn("angle2: ", H::best_plan[id].angle2);
        P::logn("speed2: ", H::best_plan[id].speed2);
        P::logn("was_jumping: ", H::best_plan[id].was_jumping);
        P::logn("was_jumping: ", H::best_plan[id].was_jumping);
        P::logn("collide_with_entity_before_on_ground_after_jumping: ", H::best_plan[id].collide_with_entity_before_on_ground_after_jumping);
        P::logn("was_on_ground_after_jumping: ", H::best_plan[id].was_on_ground_after_jumping);


        Plan cur_plan = H::best_plan[id];

        SmartSimulator simulator(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), cur_plan.plans_config, H::game.robots, H::game.ball, H::game.nitro_packs, false, H::getRobotGlobalIdByLocal(id));
        simulator.initIteration(iteration, cur_plan);

        SmartSimulator accurate_simulator(C::MAX_SIMULATION_DEPTH, H::getRobotGlobalIdByLocal(id), cur_plan.plans_config, H::game.robots, H::game.ball, H::game.nitro_packs, true, H::getRobotGlobalIdByLocal(id));
        accurate_simulator.initIteration(iteration, cur_plan);

        for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; sim_tick++) {
          simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(id), true);
          accurate_simulator.tickDynamic(sim_tick, H::getRobotGlobalIdByLocal(id), true);
        }
      }
#endif
    }
    H::asserts_failed_k += 1;
  }

#ifndef FROM_LOG
  for (auto& robot : H::game.robots) {
    if (robot.is_teammate) {
      Entity e;
      e.fromRobot(robot);
      e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, true, e.state.position);
      e.nitroCheck();
      if (!e.action.use_nitro) {
        e.action = H::best_plan[H::getRobotLocalIdByGlobal(robot.id)].toMyAction(0, false, false, e.state.position);
      }
      H::actions[robot.id] = e.action.toAction();
    }
  }
#endif

  for (auto& robot : H::game.robots) {
    int id = H::getRobotLocalIdByGlobal(robot.id);
    H::prev_velocity[id] = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
    H::prev_position[id] = {robot.x, robot.y, robot.z};
  }

  //H::t[0].cur(true);
  //for (int i = 0; i < 7; ++i) {
  //  auto& t = H::t[i];
  //  t.cur(false, true);
  //  P::logn("t", i, " avg: ", t.avg() * 1000, " cur: ", t.getCur() * 1000, " x", (int)(std::floor(t.getCur() / t.avg() * 100)), "%");
  //}

  //for (int i = 0; i < 5; ++i) {
  //  auto& c = H::c[i];
  //  c.capture();
  //  P::logn("c", i, " avg: ", c.avg_(), " cur: ", c.last_(), " x", (int)(std::floor((double)c.last_() / c.avg_() * 100)), "%");
  //}

}
}
#endif
void MyStrategy::act(
    const model::Robot& me,
    const model::Rules& rules,
    const model::Game& game,
    model::Action& action) {
#ifndef LOCAL
  if (rules.team_size == 3) {
    int init = H::tryInit(me, rules, game);
  if (init == 1) {
    doStrategy();
    action = H::getCurrentAction();
  } else if (init == 2) {
    action = H::getCurrentAction();
  } else if (init == 3) {
    action = H::getCurrentAction();
    H::global_timer.cur(true, true);
  }
  } else {
      if (Frozen::H::tryInit(me, rules, game)) {
      Frozen::doStrategy();
      action = Frozen::H::getCurrentAction();
    } else {
      action = Frozen::H::getCurrentAction();
      Frozen::H::global_timer.cur(true, true);
    }
  }
#else
  int init = H::tryInit(me, rules, game);
  if (init == 1) {
    doStrategy();
    action = H::getCurrentAction();
  } else if (init == 2) {
    action = H::getCurrentAction();
  } else if (init == 3) {
    action = H::getCurrentAction();
    H::global_timer.cur(true, true);
  }
#endif
}

#ifdef LOCAL
#ifdef DRAWLR

#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

std::string MyStrategy::custom_rendering() {
  rapidjson::Document document;
  document.SetArray();
  rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
  for (auto line : P::lines_to_draw) {
    rapidjson::Value line_object;
    line_object.SetObject();
    rapidjson::Value line_data;
    line_data.SetObject();
    line_data.AddMember("x1", line.a.x, allocator);
    line_data.AddMember("y1", line.a.y, allocator);
    line_data.AddMember("z1", line.a.z, allocator);
    line_data.AddMember("x2", line.b.x, allocator);
    line_data.AddMember("y2", line.b.y, allocator);
    line_data.AddMember("z2", line.b.z, allocator);
    line_data.AddMember("width", 1.0, allocator);
    line_data.AddMember("r", line.getR(), allocator);
    line_data.AddMember("g", line.getG(), allocator);
    line_data.AddMember("b", line.getB(), allocator);
    line_data.AddMember("a", line.getA(), allocator);
    line_object.AddMember("Line", line_data, allocator);
    document.PushBack(line_object, allocator);
  }
  for (auto sphere : P::spheres_to_draw) {
    rapidjson::Value sphere_object;
    sphere_object.SetObject();
    rapidjson::Value sphere_data;
    sphere_data.SetObject();
    sphere_data.AddMember("x", sphere.center.x, allocator);
    sphere_data.AddMember("y", sphere.center.y, allocator);
    sphere_data.AddMember("z", sphere.center.z, allocator);
    sphere_data.AddMember("radius", sphere.radius, allocator);
    sphere_data.AddMember("r", sphere.getR(), allocator);
    sphere_data.AddMember("g", sphere.getG(), allocator);
    sphere_data.AddMember("b", sphere.getB(), allocator);
    sphere_data.AddMember("a", sphere.getA(), allocator);
    sphere_object.AddMember("Sphere", sphere_data, allocator);
    document.PushBack(sphere_object, allocator);
  }

  for (auto& log : P::logs) {
    rapidjson::Value log_object;
    log_object.SetObject();
    rapidjson::Value value;
    value.SetString(log.c_str(), allocator);
    log_object.AddMember("Text", value, allocator);
    document.PushBack(log_object, allocator);
  }

  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  document.Accept(writer);

  if (H::cur_round_tick % C::TPT == C::TPT - 1) {
    P::logs.clear();
    P::lines_to_draw.clear();
    P::spheres_to_draw.clear();
  }
  //if (H::tick == 3900) {
  return buf.GetString();
  //} else {
  //  return {};
  //}
}

#endif
#endif

---

*** model/Arena.h ***
---
#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_ARENA_H_
#define _MODEL_ARENA_H_

#include "../rapidjson/document.h"

namespace model {
    struct Arena {
        double width;
        double height;
        double depth;
        double bottom_radius;
        double top_radius;
        double corner_radius;
        double goal_top_radius;
        double goal_width;
        double goal_height;
        double goal_depth;
        double goal_side_radius;

        void read(const rapidjson::Value& json) {
            width = json["width"].GetDouble();
            height = json["height"].GetDouble();
            depth = json["depth"].GetDouble();
            bottom_radius = json["bottom_radius"].GetDouble();
            top_radius = json["top_radius"].GetDouble();
            corner_radius = json["corner_radius"].GetDouble();
            goal_top_radius = json["goal_top_radius"].GetDouble();
            goal_width = json["goal_width"].GetDouble();
            goal_height = json["goal_height"].GetDouble();
            goal_depth = json["goal_depth"].GetDouble();
            goal_side_radius = json["goal_side_radius"].GetDouble();
        }
    };
}

#endif
---

*** model/Point.h ***
---
#ifndef CODEBALL_POINT_H
#define CODEBALL_POINT_H

#include <math.h>
#include <iostream>


struct Point {
  double x, y, z;

  inline Point() {}

  inline Point(double x, double y, double z) : x(x), y(y), z(z) {}

  inline bool operator==(const Point& other) const {
    return
        x == other.x
            && y == other.y
            && z == other.z;
  }

  inline Point operator-(const Point& other) const {
    return {x - other.x, y - other.y, z - other.z};
  }
  inline Point operator+(const Point& other) const {
    return {x + other.x, y + other.y, z + other.z};
  }
  inline Point operator*(const double& value) const {
    return {x * value, y * value, z * value};
  }

  inline Point operator/(const double& value) const {
    return {x / value, y / value, z / value};
  }
  inline Point& operator-=(const Point& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }
  inline Point& operator+=(const Point& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
  inline Point& operator/=(const double value) {
    x /= value;
    y /= value;
    z /= value;
    return *this;
  }
  inline Point& operator*=(const double value) {
    x *= value;
    y *= value;
    z *= value;
    return *this;
  }
  inline double length() const {
    return sqrt(length_sq());
  }
  inline double length_sq() const {
    return x * x + y * y + z * z;
  }
  inline Point normalize() const {
    const double& norm = length();
    if (norm > 0) {
      return {x / norm, y / norm, z / norm};
    } else {
      return {x, y, z};
    }
  }
  inline double dot(const Point& other) const {
    return x * other.x + y * other.y + z * other.z;
  }
  inline Point clamp(const double& ub) const {
    if (length_sq() > ub * ub) {
      return normalize() * ub;
    }
    return *this;
  }

};

#ifndef LOCAL
namespace Frozen {

struct Point {
  double x, y, z;

  Point() {}

  Point(double x, double y, double z) : x(x), y(y), z(z) {}

  bool operator==(const Point& other) const {
    return
        x == other.x
            && y == other.y
            && z == other.z;
  }

  Point operator-(const Point& other) const {
    return {x - other.x, y - other.y, z - other.z};
  }
  Point operator+(const Point& other) const {
    return {x + other.x, y + other.y, z + other.z};
  }
  Point operator*(const double value) const {
    return {x * value, y * value, z * value};
  }

  Point operator/(const double value) const {
    return {x / value, y / value, z / value};
  }
  Point& operator-=(const Point& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }
  Point& operator+=(const Point& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
  Point& operator/=(const double value) {
    x /= value;
    y /= value;
    z /= value;
    return *this;
  }
  Point& operator*=(const double value) {
    x *= value;
    y *= value;
    z *= value;
    return *this;
  }
  double length() const {
    return sqrt(length_sq());
  }
  double length_sq() const {
    return x * x + y * y + z * z;
  }
  Point normalize() const {
    const double norm = length();
    if (norm > 0) {
      return {x / norm, y / norm, z / norm};
    } else {
      return {x, y, z};
    }
  }
  double dot(const Point& other) const {
    return x * other.x + y * other.y + z * other.z;
  }
  Point clamp(const double ub) const {
    if (length_sq() > ub * ub) {
      return normalize() * ub;
    }
    return *this;
  }

};

}
#endif

#endif //CODEBALL_POINT_H

---

*** model/Point2d.h ***
---
#ifndef CODEBALL_POINT2D_H
#define CODEBALL_POINT2D_H

#include <math.h>


struct Point2d {
  double x, y;

  Point2d() {}

  Point2d(double x, double y) : x(x), y(y) {}

  Point2d operator+(const Point2d& other) const {
    return {x + other.x, y + other.y};
  }
  Point2d operator-(const Point2d& other) const {
    return {x - other.x, y - other.y};
  }
  Point2d operator/(const double value) const {
    return {x / value, y / value};
  }
  Point2d operator*(const double value) const {
    return {x * value, y * value};
  }
  Point2d& operator-=(const Point2d& other) {
    x -= other.x;
    y -= other.y;
    return *this;
  }
  Point2d& operator+=(const Point2d& other) {
    x += other.x;
    y += other.y;
    return *this;
  }
  double length() const {
    return sqrt(length_sq());
  }
  double length_sq() const {
    return x * x + y * y;
  }
  Point2d normalize() const {
    double norm = length();
    return {x / norm, y / norm};
  }
  double dot(const Point2d& other) const {
    return x * other.x + y * other.y;
  }
  Point2d clamp(const double ub) const {
    if (length_sq() > ub * ub) {
      return normalize() * ub;
    }
    return *this;
  }

};
#ifndef LOCAL
namespace Frozen {

struct Point2d {
  double x, y;

  Point2d() {}

  Point2d(double x, double y) : x(x), y(y) {}

  Point2d operator+(const Point2d& other) const {
    return {x + other.x, y + other.y};
  }
  Point2d operator-(const Point2d& other) const {
    return {x - other.x, y - other.y};
  }
  Point2d operator/(const double value) const {
    return {x / value, y / value};
  }
  Point2d operator*(const double value) const {
    return {x * value, y * value};
  }
  Point2d& operator-=(const Point2d& other) {
    x -= other.x;
    y -= other.y;
    return *this;
  }
  Point2d& operator+=(const Point2d& other) {
    x += other.x;
    y += other.y;
    return *this;
  }
  double length() const {
    return sqrt(length_sq());
  }
  double length_sq() const {
    return x * x + y * y;
  }
  Point2d normalize() const {
    double norm = length();
    return {x / norm, y / norm};
  }
  double dot(const Point2d& other) const {
    return x * other.x + y * other.y;
  }
  Point2d clamp(const double ub) const {
    if (length_sq() > ub * ub) {
      return normalize() * ub;
    }
    return *this;
  }

};

}
#endif

#endif //CODEBALL_POINT2D_H

---

*** model/C.h ***
---
#ifndef CODEBALL_CONSTANTS_H
#define CODEBALL_CONSTANTS_H

#ifdef LOCAL
#include <model/Arena.h>
#include <model/Action.h>
#include <model/Robot.h>
#include <model/Rules.h>
#include <model/Game.h>
#include <model/Player.h>
#include <model/Ball.h>
#include <model/MyTimer.h>
#include <model/Point.h>
#include <model/Point2d.h>
#include <model/MyAction.h>
#else
#include "Arena.h"
#include "Action.h"
#include "Robot.h"
#include "Rules.h"
#include "Game.h"
#include "Player.h"
#include "Ball.h"
#include "MyTimer.h"
#include "Point.h"
#include "Point2d.h"
#include "MyAction.h"
#endif

#include <math.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <iomanip>
#include <random>
#include <string>
#include <set>

struct DGState {
  int x, y, z, t;
};

struct C {

  static int unique_plan_id;
  static model::Rules rules;
  static constexpr int TPT = 2;
  static constexpr int MAX_SIMULATION_DEPTH = 100 / TPT;
  static constexpr int ENEMY_SIMULATION_DEPTH = 100 / TPT;
  static constexpr double ball_antiflap = 0.1; // todo check
  static constexpr double MIN_WALL_JUMP = 15.;
  static constexpr int NEVER = 1000000000;
  static constexpr int ENEMY_LIVE_TICKS = 30 / TPT;
  static constexpr double NITRO_TOUCH_EPSILON = 1.01;
  static constexpr int LONGEST_JUMP = 50 / TPT;

#ifdef LOCAL
  static constexpr double time_limit = 320. * 1.5;
#else
  static constexpr double time_limit = 320.;
#endif

  static std::mt19937_64 rd;

  static double rand_double(double a, double b) {
    return a + (double) rd() / rd.max() * (b - a);
  }

  static int rand_int(int a, int b) {
    if (a > b) {
      std::swap(a, b);
    }
    return (int) (a + rd() % (b - a + 1));
  }

};



#ifndef LOCAL
namespace Frozen {

struct DGState {
  int x, y, z, t;
};

struct C {

  static int unique_plan_id;
  static model::Rules rules;
  static constexpr int TPT = 1;
  static constexpr int MAX_SIMULATION_DEPTH = 100 / TPT;
  static constexpr int ENEMY_SIMULATION_DEPTH = 50 / TPT;
  static constexpr int MICROTICKS_PER_TICK = 100 * TPT;
  static constexpr double ball_antiflap = 0.1; // todo check
  static constexpr double MIN_WALL_JUMP = 15.;
  static constexpr int NEVER = 1000000000;

#ifdef LOCAL
  static constexpr double time_limit = 330. * 1.5;
#else
  static constexpr double time_limit = 330.;
#endif

  static std::mt19937_64 rd;

  static double rand_double(double a, double b) {
    return a + (double) rd() / rd.max() * (b - a);
  }

  static int rand_int(int a, int b) {
    return (int) (a + rd() % (b - a + 1));
  }

};

}
#endif

#endif //CODEBALL_CONSTANTS_H

---

*** model/Rules.h ***
---
#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_RULES_H_
#define _MODEL_RULES_H_

#include "../rapidjson/document.h"
#include "Arena.h"

namespace model {
    struct Rules {
        int max_tick_count;
        Arena arena;
        int team_size;
        long long seed;
        double ROBOT_MIN_RADIUS;
        double ROBOT_MAX_RADIUS;
        double ROBOT_MAX_JUMP_SPEED;
        double ROBOT_ACCELERATION;
        double ROBOT_NITRO_ACCELERATION;
        double ROBOT_MAX_GROUND_SPEED;
        double ROBOT_ARENA_E;
        double ROBOT_RADIUS;
        double ROBOT_MASS;
        int TICKS_PER_SECOND;
        int MICROTICKS_PER_TICK;
        int RESET_TICKS;
        double BALL_ARENA_E;
        double BALL_RADIUS;
        double BALL_MASS;
        double MIN_HIT_E;
        double MAX_HIT_E;
        double MAX_ENTITY_SPEED;
        double MAX_NITRO_AMOUNT;
        double START_NITRO_AMOUNT;
        double NITRO_POINT_VELOCITY_CHANGE;
        double NITRO_PACK_X;
        double NITRO_PACK_Y;
        double NITRO_PACK_Z;
        double NITRO_PACK_RADIUS;
        double NITRO_PACK_AMOUNT;
        int NITRO_PACK_RESPAWN_TICKS;
        double GRAVITY;

        void read(const rapidjson::Value& json) {
            max_tick_count = json["max_tick_count"].GetInt();
            arena.read(json["arena"]);
            team_size = json["team_size"].GetInt();
            seed = json["seed"].GetInt64();
            ROBOT_MIN_RADIUS = json["ROBOT_MIN_RADIUS"].GetDouble();
            ROBOT_MAX_RADIUS = json["ROBOT_MAX_RADIUS"].GetDouble();
            ROBOT_MAX_JUMP_SPEED = json["ROBOT_MAX_JUMP_SPEED"].GetDouble();
            ROBOT_ACCELERATION = json["ROBOT_ACCELERATION"].GetDouble();
            ROBOT_NITRO_ACCELERATION = json["ROBOT_NITRO_ACCELERATION"].GetDouble();
            ROBOT_MAX_GROUND_SPEED = json["ROBOT_MAX_GROUND_SPEED"].GetDouble();
            ROBOT_ARENA_E = json["ROBOT_ARENA_E"].GetDouble();
            ROBOT_RADIUS = json["ROBOT_RADIUS"].GetDouble();
            ROBOT_MASS = json["ROBOT_MASS"].GetDouble();
            TICKS_PER_SECOND = json["TICKS_PER_SECOND"].GetInt();
            MICROTICKS_PER_TICK = json["MICROTICKS_PER_TICK"].GetInt();
            RESET_TICKS = json["RESET_TICKS"].GetInt();
            BALL_ARENA_E = json["BALL_ARENA_E"].GetDouble();
            BALL_RADIUS = json["BALL_RADIUS"].GetDouble();
            BALL_MASS = json["BALL_MASS"].GetDouble();
            MIN_HIT_E = json["MIN_HIT_E"].GetDouble();
            MAX_HIT_E = json["MAX_HIT_E"].GetDouble();
            MAX_ENTITY_SPEED = json["MAX_ENTITY_SPEED"].GetDouble();
            MAX_NITRO_AMOUNT = json["MAX_NITRO_AMOUNT"].GetDouble();
            START_NITRO_AMOUNT = json["START_NITRO_AMOUNT"].GetDouble();
            NITRO_POINT_VELOCITY_CHANGE = json["NITRO_POINT_VELOCITY_CHANGE"].GetDouble();
            NITRO_PACK_X = json["NITRO_PACK_X"].GetDouble();
            NITRO_PACK_Y = json["NITRO_PACK_Y"].GetDouble();
            NITRO_PACK_Z = json["NITRO_PACK_Z"].GetDouble();
            NITRO_PACK_RADIUS = json["NITRO_PACK_RADIUS"].GetDouble();
            NITRO_PACK_AMOUNT = json["NITRO_PACK_AMOUNT"].GetDouble();
            NITRO_PACK_RESPAWN_TICKS = json["NITRO_PACK_RESPAWN_TICKS"].GetInt();
            GRAVITY = json["GRAVITY"].GetDouble();
        }
    };
}

#endif
---

*** model/MyAction.h ***
---
#ifndef CODEBALL_MYACTION_H
#define CODEBALL_MYACTION_H

#ifdef LOCAL
#include <model/Point.h>
#include <model/Action.h>
#else
#include "Point.h"
#include "Action.h"
#endif


struct MyAction {
  Point target_velocity;
  double jump_speed;
  double max_jump_speed;
  bool use_nitro;
  model::Action toAction() {
        model::Action result;
        result.target_velocity_x = target_velocity.x;
        result.target_velocity_y = target_velocity.y;
        result.target_velocity_z = target_velocity.z;
        result.jump_speed = jump_speed;
        result.use_nitro = use_nitro;
        return result;
  };
};


#ifndef LOCAL
namespace Frozen {

struct MyAction {
  Point target_velocity = {0, 0, 0};
  double jump_speed = 0;
  double max_jump_speed = 15.;
  bool use_nitro = false;
  model::Action toAction() {
      model::Action result;
      result.target_velocity_x = target_velocity.x;
      result.target_velocity_y = target_velocity.y;
      result.target_velocity_z = target_velocity.z;
      result.jump_speed = jump_speed;
      result.use_nitro = use_nitro;
      return result;
  };
};

}
#endif

#endif //CODEBALL_MYACTION_H

---

*** model/Robot.h ***
---
#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_ROBOT_H_
#define _MODEL_ROBOT_H_

#include "../rapidjson/document.h"

namespace model {
struct Robot {
  int id;
  int player_id;
  bool is_teammate;
  double x;
  double y;
  double z;
  double velocity_x;
  double velocity_y;
  double velocity_z;
  double radius;
  double nitro_amount;
  bool touch;
  double touch_normal_x;
  double touch_normal_y;
  double touch_normal_z;

  void read(const rapidjson::Value& json) {
    id = json["id"].GetInt();
    player_id = json["player_id"].GetInt();
    is_teammate = json["is_teammate"].GetBool();
    x = json["x"].GetDouble();
    y = json["y"].GetDouble();
    z = json["z"].GetDouble();
    velocity_x = json["velocity_x"].GetDouble();
    velocity_y = json["velocity_y"].GetDouble();
    velocity_z = json["velocity_z"].GetDouble();
    radius = json["radius"].GetDouble();
    nitro_amount = json["nitro_amount"].GetDouble();
    touch = json["touch"].GetBool();
    if (touch) {
      touch_normal_x = json["touch_normal_x"].GetDouble();
      touch_normal_y = json["touch_normal_y"].GetDouble();
      touch_normal_z = json["touch_normal_z"].GetDouble();
    }
  }

  void read2(const rapidjson::Value& json, int my_id) {
    id = json["id"].GetInt();
    player_id = json["player_index"].GetInt() + 1;
    is_teammate = player_id == my_id;
    x = json["position"]["x"].GetDouble();
    y = json["position"]["y"].GetDouble();
    z = json["position"]["z"].GetDouble() * (my_id == 2 ? -1 : 1);
    velocity_x = json["velocity"]["x"].GetDouble();
    velocity_y = json["velocity"]["y"].GetDouble();
    velocity_z = json["velocity"]["z"].GetDouble() * (my_id == 2 ? -1 : 1);
    radius = json["radius"].GetDouble();
    nitro_amount = json["nitro"].GetDouble();
    auto& j_touch = json["last_touch"];
    touch = !j_touch.IsNull();
    if (touch) {
      touch_normal_x = j_touch["x"].GetDouble();
      touch_normal_y = j_touch["y"].GetDouble();
      touch_normal_z = j_touch["z"].GetDouble() * (my_id == 2 ? -1 : 1);
    }
  }

};
}

#endif
---

*** model/MyTimer.h ***
---
#ifndef CODEBALL_MYTIMER_H
#define CODEBALL_MYTIMER_H

//#define CHRONO

#ifdef CHRONO
#include <chrono>
#else
#ifdef LOCAL
#include <model/getCPUTime.h>
#else
#include "getCPUTime.h"
#endif
#endif

struct MyTimer {
#ifdef CHRONO
  std::chrono::time_point<std::chrono::high_resolution_clock> _start, _end;
#else
  double _start = 0, _end = 0;
#endif
  MyTimer() {
    start();
    end();
  }

  double cur_ = 0;
  double cumulative = 0;
  int k = 0;
  double max_ = 0;
  int64_t sum_calls = 0;
  int64_t captures = 0;
  int64_t last_calls = 0;

  void start() {
#ifdef CHRONO
    _start = std::chrono::high_resolution_clock::now();
#else
    _start = CPUTime::getCPUTime();
#endif
  }
  void end() {
#ifdef CHRONO
    _end = std::chrono::high_resolution_clock::now();
#else
    _end = CPUTime::getCPUTime();
#endif
  }

  double delta(bool accumulate = false) {
#ifdef CHRONO
    double res = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(_end - _start).count();
#else
    double res = _end - _start;
#endif
    if (accumulate) {
      cumulative += res;
      k++;
      return cumulative;
    }
    return res;
  }
  double cur(bool accumulate = false, bool counter = false, bool upd_max_ = false) {
#ifdef CHRONO
    double res = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>
        (std::chrono::high_resolution_clock::now() - _start).count();
    cumulative -= 640e-10;
#else
    double res = CPUTime::getCPUTime() - _start - 0.0000008715;
#endif
    if (counter) {
      k++;
    }
    if (upd_max_) {
      max_ = std::max(max_, res);
    }
    if (accumulate) {
      cumulative += res;
      cur_ += res;
    }
    return res;
  }

  void clear() {
    cumulative = 0;
    k = 0;
    max_ = 0;
    cur_ = 0;
  }

  void clearCur() {
    cur_ = 0;
  }

  double getCur() {
    return cur_;
  }

  double avg() {
    return cumulative / (k == 0 ? 1 : k);
  }

  double max() {
    return max_;
  }

  double getCumulative(bool with_cur = false) {
    if (!with_cur) {
      return cumulative;
    }
    return cumulative + cur();
  }

  void call() {
    sum_calls++;
    last_calls++;
  }

  void capture() {
    captures++;
  }

  void init_calls() {
    last_calls = 0;
  }

  int64_t avg_() {
    return captures == 0 ? 0 : sum_calls / captures;
  }

  int64_t last_() {
    return last_calls;
  }

};

#endif //CODEBALL_MYTIMER_H

---

*** model/Game.h ***
---
#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_GAME_H_
#define _MODEL_GAME_H_

#include <vector>
#include "../rapidjson/document.h"
#include "Player.h"
#include "Arena.h"
#include "Robot.h"
#include "NitroPack.h"
#include "Ball.h"

namespace model {
struct Game {
#ifdef FROM_LOG
  static int my_id;
#endif
  int current_tick;
  std::vector<Player> players;
  std::vector<Robot> robots;
  std::vector<NitroPack> nitro_packs;
  Ball ball;

  void read(const rapidjson::Value& json) {
    current_tick = json["current_tick"].GetInt();

    rapidjson::Value::ConstArray json_players = json["players"].GetArray();
    players.resize(json_players.Size());
    for (size_t i = 0; i < players.size(); i++) {
      players[i].read(json_players[i]);
    }

    rapidjson::Value::ConstArray json_robots = json["robots"].GetArray();
    robots.resize(json_robots.Size());
    for (size_t i = 0; i < robots.size(); i++) {
      robots[i].read(json_robots[i]);
    }

    rapidjson::Value::ConstArray json_nitro_packs = json["nitro_packs"].GetArray();
    nitro_packs.resize(json_nitro_packs.Size());
    for (size_t i = 0; i < nitro_packs.size(); i++) {
      nitro_packs[i].read(json_nitro_packs[i]);
    }

    ball.read(json["ball"]);
  }
#ifdef FROM_LOG
  void read2(const rapidjson::Value& json) {
    current_tick = json["current_tick"].GetInt();
    if (current_tick == 0) {
      rapidjson::Value::ConstArray json_names = json["names"].GetArray();
      for (int i = 0; i < json_names.Size(); i++) {
        if (std::string(json_names[i].GetString()) == "TonyK") {
          my_id = i + 1;
        }
      }
    }
    rapidjson::Value::ConstArray json_scores = json["scores"].GetArray();
    players.resize(2);
    for (int i = 0; i < json_scores.Size(); i++) {
      players[i] = Player{i + 1, i + 1 == my_id, false, json_scores[i].GetInt()};
    }
    rapidjson::Value::ConstArray json_robots = json["robots"].GetArray();
    robots.resize(json_robots.Size());
    for (size_t i = 0; i < robots.size(); i++) {
      robots[i].read2(json_robots[i], my_id);
    }

    rapidjson::Value::ConstArray json_nitro_packs = json["nitro_packs"].GetArray();
    nitro_packs.resize(json_nitro_packs.Size());
    for (size_t i = 0; i < nitro_packs.size(); i++) {
      nitro_packs[i].read2(json_nitro_packs[i], my_id);
    }

    ball.read2(json["ball"], my_id);


  }
#endif
};
}

#endif
---

*** model/Player.h ***
---
#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_PLAYER_H_
#define _MODEL_PLAYER_H_

#include "../rapidjson/document.h"

namespace model {
    struct Player {
        int id;
        bool me;
        bool strategy_crashed;
        int score;

        void read(const rapidjson::Value& json) {
            id = json["id"].GetInt();
            me = json["me"].GetBool();
            strategy_crashed = json["strategy_crashed"].GetBool();
            score = json["score"].GetInt();
        }
    };
}

#endif
---

*** model/Plan.h ***
---
#ifndef CODEBALL_PLAN_H
#define CODEBALL_PLAN_H

struct Plan {
  double angle1;
  double sangle1;
  double cangle1;
  double y1;
  double cos_lat1;
  double angle2;
  double sangle2;
  double cangle2;
  double y2;
  double cos_lat2;
  double max_jump_speed;
  double max_speed;
  int time_nitro_on = C::NEVER;
  int time_nitro_off = C::NEVER;
  int time_change = C::NEVER;
  int time_jump = C::NEVER;
  int plans_config;
  int unique_id;
  int parent_id;
  double speed1 = 1., speed2 = 1.;

  int oncoming_jump;

  double oncoming_jump_speed;

  bool was_jumping;
  bool was_on_ground_after_jumping;
  bool collide_with_entity_before_on_ground_after_jumping;

  Point crossing;
  Point crossing2;

  int configuration;

  Point nitro_velocity1, nitro_velocity2;
  Point velocity1, velocity2;

  struct Score {
    double sum_score;
    double fighter_min_dist_to_ball;
    double fighter_min_dist_to_goal;
    double fighter_last_dist_to_goal;
    double defender_min_dist_to_ball;
    double defender_min_dist_from_goal;
    double defender_last_dist_from_goal;
    double fighter_closest_enemy_ever;
    double fighter_closest_enemy_last;

    bool operator<(const Score& other) const {
      return score() < other.score();
    }

    double score() const {
      return
          sum_score
              - fighter_min_dist_to_ball
              - fighter_min_dist_to_goal
              - fighter_last_dist_to_goal
              - defender_min_dist_to_ball
              + defender_min_dist_from_goal
              + defender_last_dist_from_goal
              + fighter_closest_enemy_ever
              + fighter_closest_enemy_last;
    }

    void minimal() {
      sum_score = -1e18;
      fighter_min_dist_to_ball = 1e9;
      fighter_min_dist_to_goal = 1e9;
      fighter_last_dist_to_goal = 1e9;
      defender_min_dist_to_ball = 1e9;
      defender_min_dist_from_goal = 1e9;
      defender_last_dist_from_goal = 1e9;
      fighter_closest_enemy_ever = 1e9;
      fighter_closest_enemy_last = 1e9;
    }

    void start_fighter() {
      sum_score = 0;
      fighter_min_dist_to_ball = 1e9;
      fighter_min_dist_to_goal = 1e9;
      fighter_last_dist_to_goal = 1e9;
      defender_min_dist_to_ball = 0;
      defender_min_dist_from_goal = 0;
      defender_last_dist_from_goal = 0;
      fighter_closest_enemy_ever = 1e9;
      fighter_closest_enemy_last = 1e9;
    }

    void start_defender() {
      sum_score = 0;
      fighter_min_dist_to_ball = 0;
      fighter_min_dist_to_goal = 0;
      fighter_last_dist_to_goal = 0;
      defender_min_dist_to_ball = 1e9;
      defender_min_dist_from_goal = 1e9;
      defender_last_dist_from_goal = 0;
      fighter_closest_enemy_ever = 0;
      fighter_closest_enemy_last = 0;
    }

  } score;

  Plan() {}

  bool nitro_as_velocity = false, nitro_up = false;

  void rand_angle1() {
    angle1 = C::rand_double(0, 2 * M_PI);
    cangle1 = cos(angle1);
    sangle1 = sin(angle1);
  }

  void rand_angle2() {
    angle2 = C::rand_double(0, 2 * M_PI);
    cangle2 = cos(angle2);
    sangle2 = sin(angle2);
  }

  void rand_y1() {
    y1 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
    cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));
  }

  void rand_y2() {
    y2 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
    cos_lat2 = cos(asin(y2 / C::rules.MAX_ENTITY_SPEED));
  }

  void rand_time_change(int simulation_depth) {
    time_change = C::rand_int(0, simulation_depth);
  }

  void speed1_1_or_0() {
    speed1 = 1;
    if (C::rand_double(0, 1) < 0.01) {
      speed1 = 0;
    }
  }

  void speed2_1_or_0() {
    speed2 = 1;
    if (C::rand_double(0, 1) < 0.01) {
      speed2 = 0;
    }
  }

  void rand_speed1() {
    speed1 = C::rand_double(0, 1);
  }

  void rand_time_jump(int simulation_depth, int start_at = 0) {
    time_jump = C::rand_int(start_at, simulation_depth);
  }

  Plan(int configuration,
       const int simulation_depth,
       const double initial_vx = 0,
       const double initial_vz = 0,
       const double crossing_x = 0,
       const double crossing_z = 0,
       const Point& nitro_acceleration = {0, 0, 0},
       const double jump_speed = 0,
       const bool is_dribler = false) : configuration(configuration) {
    unique_id = C::unique_plan_id++;
    parent_id = unique_id;

    was_jumping = false;
    was_on_ground_after_jumping = false;
    collide_with_entity_before_on_ground_after_jumping = false;
    oncoming_jump = C::NEVER;
    nitro_as_velocity = false;
    nitro_up = false;

    if (configuration == 31) { // in jump nitro and hit power
      rand_angle1();
      rand_y1();
      max_jump_speed = C::rand_double(0, 15);
      time_nitro_on = 0;
      time_nitro_off = simulation_depth;
    } else if (configuration == 32) { // in jump only hit power without nitro
      max_jump_speed = C::rand_double(0, 15);
    } else if (configuration == 20) { // me 2 vec no jump
      rand_angle1();
      rand_angle2();
      rand_time_change(simulation_depth);
      speed1_1_or_0();
      speed2_1_or_0();
      max_jump_speed = 0;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
    } else if (configuration == 21) { // me 2 vec
      rand_angle1();
      rand_angle2();
      rand_time_change(simulation_depth);
      rand_time_jump(simulation_depth, time_change + 1);
      speed1_1_or_0();
      speed2_1_or_0();
      max_jump_speed = 15;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
    } else if (configuration == 22) { // me 2 vec nitro up
      rand_angle1();
      rand_angle2();
      rand_time_change(simulation_depth);
      rand_time_jump(simulation_depth, time_change + 1);
      speed1_1_or_0();
      speed2_1_or_0();
      max_jump_speed = 15;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
      time_nitro_on = 0;
      time_nitro_off = simulation_depth;
      nitro_up = true;
    } else if (configuration == 23) { // me 2 vec nitro vel
      rand_angle1();
      rand_angle2();
      rand_time_change(simulation_depth);
      rand_time_jump(simulation_depth, time_change + 1);
      speed1_1_or_0();
      speed2_1_or_0();
      max_jump_speed = 15;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
      time_nitro_on = 0;
      time_nitro_off = simulation_depth;
      nitro_as_velocity = true;
    } else if (configuration == 11) {
      rand_angle1();
      rand_time_jump(simulation_depth);
      speed1_1_or_0();
      speed2_1_or_0();
      max_jump_speed = 15;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
    } else if (configuration == 12) {
      rand_angle1();
      rand_time_jump(simulation_depth);
      speed1_1_or_0();
      speed2_1_or_0();
      max_jump_speed = 15;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
      time_nitro_on = 0;
      time_nitro_off = simulation_depth;
      nitro_as_velocity = true;
    } else if (configuration == 13) {
      rand_angle1();
      rand_time_jump(simulation_depth);
      max_jump_speed = 15;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
      speed1_1_or_0();
      speed2_1_or_0();
      time_nitro_on = 0;
      time_nitro_off = simulation_depth;
      nitro_up = true;
    } else if (configuration == 61) { // smart enemy
      rand_angle1();
      rand_time_jump(simulation_depth);
      //rand_speed1();
      max_jump_speed = 15;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
    } else if (configuration == 71) { // last action
      if (!is_dribler) {

        angle1 = atan2(initial_vz, initial_vx);
        cangle1 = cos(angle1);
        sangle1 = sin(angle1);

        max_speed = Point2d{initial_vx, initial_vz}.length();
        max_jump_speed = (jump_speed == 0) ? 15 : jump_speed;
      } else {
        angle1 = 0;
        cangle1 = cos(angle1);
        sangle1 = sin(angle1);
        max_speed = 0;
        max_jump_speed = 0;
      }
    } else if (configuration == 710) { // last action 0
      if (!is_dribler) {
        angle1 = atan2(initial_vz, initial_vx);
        cangle1 = cos(angle1);
        sangle1 = sin(angle1);

        max_speed = Point2d{initial_vx, initial_vz}.length();
        max_jump_speed = 0;
      } else {

        angle1 = 0;
        cangle1 = cos(angle1);
        sangle1 = sin(angle1);
        max_speed = 0;
        max_jump_speed = 0;
      }
    } else if (configuration == 72) { // last action nitro
      angle1 = atan2(nitro_acceleration.z, nitro_acceleration.x);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      y1 = nitro_acceleration.y;
      cos_lat1 = cos(asin(y1 / 100.));

      max_speed = 100.;
      max_jump_speed = (jump_speed == 0) ? (is_dribler ? 0 : 15) : jump_speed;

      time_nitro_on = 0;
      time_nitro_off = simulation_depth;
    } else if (configuration == 720) { // last action nitro 0
      angle1 = atan2(nitro_acceleration.z, nitro_acceleration.x);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      y1 = nitro_acceleration.y;
      cos_lat1 = cos(asin(y1 / 100.));

      max_speed = 100.;
      max_jump_speed = (jump_speed == 0) ? (is_dribler ? 15 : 0) : 0;

      time_nitro_on = 0;
      time_nitro_off = simulation_depth;
    }

    calcVelocities();

    score.minimal();
  }

  inline void calcVelocities() {
    velocity1.x = speed1 * max_speed * cangle1;
    velocity1.y = 0;
    velocity1.z = speed1 * max_speed * sangle1;

    velocity2.x = speed2 * max_speed * cangle2;
    velocity2.y = 0;
    velocity2.z = speed2 * max_speed * sangle2;

    nitro_velocity1.x = speed1 * 100 * cos_lat1 * cangle1;
    nitro_velocity1.y = y1;
    nitro_velocity1.z = speed1 * 100 * cos_lat1 * sangle1;

    nitro_velocity2.x = speed2 * 100 * cos_lat2 * cangle2;
    nitro_velocity2.y = y2;
    nitro_velocity2.z = speed2 * 100 * cos_lat2 * sangle2;
  }

  static constexpr double angle_mutation = M_PI / 100;
  static constexpr double speed_mutation = 0.05;
  static constexpr double z_mutation = 1;
  static constexpr double crossing_mutation = 1;
  static constexpr double jump_speed_mutation = 0.1;

  static constexpr int nitro_mutation = 1;
  static constexpr int jump_mutation = 1;
  static constexpr int time_mutation = 1;

  void mutate_angle1() {
    angle1 += C::rand_double(-angle_mutation, angle_mutation);
    if (angle1 > 2 * M_PI) {
      angle1 -= 2 * M_PI;
    }
    if (angle1 < 0) {
      angle1 += 2 * M_PI;
    }
    cangle1 = cos(angle1);
    sangle1 = sin(angle1);
  }

  void mutate_angle2() {
    angle2 += C::rand_double(-angle_mutation, angle_mutation);
    if (angle2 > 2 * M_PI) {
      angle2 -= 2 * M_PI;
    }
    if (angle2 < 0) {
      angle2 += 2 * M_PI;
    }
    cangle2 = cos(angle2);
    sangle2 = sin(angle2);
  }

  void mutate_y1() {
    y1 += C::rand_double(-z_mutation, z_mutation);
    if (y1 > C::rules.MAX_ENTITY_SPEED) {
      y1 = C::rules.MAX_ENTITY_SPEED;
    } else if (y1 < -C::rules.MAX_ENTITY_SPEED) {
      y1 = -C::rules.MAX_ENTITY_SPEED;
    }
    cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));
  }

  void mutate_y2() {
    y2 += C::rand_double(-z_mutation, z_mutation);
    if (y2 > C::rules.MAX_ENTITY_SPEED) {
      y2 = C::rules.MAX_ENTITY_SPEED;
    } else if (y2 < -C::rules.MAX_ENTITY_SPEED) {
      y2 = -C::rules.MAX_ENTITY_SPEED;
    }
    cos_lat2 = cos(asin(y2 / C::rules.MAX_ENTITY_SPEED));
  }

  void mutate_time_change(int simulation_depth) {

    if (time_change != C::NEVER) {
      time_change += C::rand_int(-time_mutation, time_mutation);
      if (time_change < 0) {
        time_change = 0;
      }
      if (time_change > simulation_depth) {
        time_change = simulation_depth;
      }
    }
  }

  void mutate_time_jump(int simulation_depth) {
    if (time_jump != C::NEVER) {
      time_jump += C::rand_int(-time_mutation, time_mutation);
      if (time_jump < 0) {
        time_jump = 0;
      }
      if (time_jump > simulation_depth) {
        time_jump = simulation_depth;
      }
    }
  }

  void mutate_jump_speed() {
    max_jump_speed += C::rand_double(-jump_speed_mutation, jump_speed_mutation);
    if (max_jump_speed > 15) {
      max_jump_speed = 15;
    }
    if (max_jump_speed < 0) {
      max_jump_speed = 0;
    }
  }

  void mutate_speed1() {
    speed1 += C::rand_double(-speed_mutation, speed_mutation);
    if (speed1 > 1) {
      speed1 = 1;
    }
    if (speed1 < 0) {
      speed1 = 0;
    }
  }
  void mutate_speed2() {
    speed2 += C::rand_double(-speed_mutation, speed_mutation);
    if (speed2 > 1) {
      speed2 = 1;
    }
    if (speed2 < 0) {
      speed2 = 0;
    }
  }

  void mutate(int configuration, const int simulation_depth) {
    unique_id = C::unique_plan_id++;

    was_jumping = false;
    was_on_ground_after_jumping = false;
    collide_with_entity_before_on_ground_after_jumping = false;
    oncoming_jump = C::NEVER;
    if (configuration == 31) {
      mutate_angle1();
      mutate_y1();
      mutate_jump_speed();
    } else if (configuration == 32) {
      mutate_jump_speed();
    } else if (configuration == 20) {
      mutate_angle1();
      mutate_angle2();
      mutate_speed1();
      mutate_speed2();
      mutate_time_change(simulation_depth);
    } else if (configuration == 21) {
      mutate_angle1();
      mutate_angle2();
      mutate_speed1();
      mutate_speed2();
      mutate_time_change(simulation_depth);
      mutate_time_jump(simulation_depth);
      mutate_jump_speed();
    } else if (configuration == 22) {
      mutate_angle1();
      mutate_angle2();
      mutate_speed1();
      mutate_speed2();
      mutate_time_change(simulation_depth);
      mutate_time_jump(simulation_depth);
      mutate_jump_speed();
    } else if (configuration == 23) {
      mutate_angle1();
      mutate_angle2();
      mutate_speed1();
      mutate_speed2();
      mutate_time_change(simulation_depth);
      mutate_time_jump(simulation_depth);
      mutate_jump_speed();
    } else if (configuration == 11) {
      mutate_angle1();
      mutate_speed1();
      mutate_time_jump(simulation_depth);
      mutate_jump_speed();
    } else if (configuration == 12) {
      mutate_angle1();
      mutate_speed1();
      mutate_time_jump(simulation_depth);
      mutate_jump_speed();
    } else if (configuration == 13) {
      mutate_angle1();
      mutate_speed1();
      mutate_time_jump(simulation_depth);
      mutate_jump_speed();
    }

    calcVelocities();
    score.minimal();
  }

  void clearAndShift(const int simulation_depth) {
    score.minimal();

    was_jumping = false;
    was_on_ground_after_jumping = false;
    collide_with_entity_before_on_ground_after_jumping = false;
    oncoming_jump = C::NEVER;

    if (time_jump != C::NEVER) {
      time_jump--;
      if (time_jump < 0) {
        time_jump = C::NEVER;
      }
    }
    if (time_change != C::NEVER) {
      time_change--;
      if (time_change < 0) {
        time_change = C::NEVER;
        std::swap(angle1, angle2);
        std::swap(sangle1, sangle2);
        std::swap(cangle1, cangle2);
        std::swap(y1, y2);
        std::swap(cos_lat1, cos_lat2);
        std::swap(speed1, speed2);
        std::swap(crossing, crossing2);
        std::swap(velocity1, velocity2);
        std::swap(nitro_velocity1, nitro_velocity2);
      }
    }
    if (time_nitro_on != C::NEVER && time_nitro_off != C::NEVER) {
      time_nitro_on--;
      time_nitro_off--;
      if (time_nitro_on < 0 && time_nitro_off < 0) {
        time_nitro_on = C::NEVER;
        time_nitro_off = C::NEVER;
      } else if (time_nitro_on < 0) {
        time_nitro_on = 0;
      } else {
        time_nitro_off = 0;
      }
    }
  }

  inline MyAction toMyAction(const int& simulation_tick, const bool& simulation, const bool& can_use_nitro, const Point& position, const Point& velocity) {
    const double& jump_speed = simulation ? (simulation_tick == time_jump ? max_jump_speed : 0) : (simulation_tick == oncoming_jump ? oncoming_jump_speed : 0);
    const bool& now_use_nitro = can_use_nitro && simulation_tick >= time_nitro_on && simulation_tick < time_nitro_off;
    if (now_use_nitro) {
      if (nitro_up) {
        const double& x = velocity.x;
        const double& z = velocity.z;
        const double& y = sqrt(10000 - x * x - z * z);
        return MyAction{{x, y, z},
            jump_speed,
            max_jump_speed,
            now_use_nitro};
      } else if (nitro_as_velocity) {
        return MyAction{velocity.normalize() * 100,
            jump_speed,
            max_jump_speed,
            now_use_nitro};
      } else {
        if (simulation_tick < time_change) {
          return MyAction{nitro_velocity1,
              jump_speed,
              max_jump_speed,
              now_use_nitro};

        } else {
          return MyAction{nitro_velocity2,
              jump_speed,
              max_jump_speed,
              now_use_nitro};
        }
      }
    } else {
      if (simulation_tick < time_change) {
        return MyAction{velocity1,
            jump_speed,
            max_jump_speed,
            now_use_nitro};
      } else {
        return MyAction{velocity2,
            jump_speed,
            max_jump_speed,
            now_use_nitro};
      }
    }
  }
  bool operator<(const Plan& other) const {
    return score < other.score;
  }
};

#ifndef LOCAL
namespace Frozen {

struct Plan {
  double angle1;
  double sangle1;
  double cangle1;
  double y1;
  double cos_lat1;
  double angle2;
  double sangle2;
  double cangle2;
  double y2;
  double cos_lat2;
  double max_jump_speed;
  double max_speed;
  int time_nitro_on;
  int time_nitro_off;
  int time_change;
  int time_jump;
  int plans_config;
  int unique_id;
  int parent_id;
  double speed1, speed2;

  int oncoming_jump;

  double oncoming_jump_speed;

  bool was_jumping;
  bool was_on_ground_after_jumping;
  bool collide_with_entity_before_on_ground_after_jumping;

  Point crossing;
  Point crossing2;

  int configuration;

  struct Score {
    double sum_score;
    double fighter_min_dist_to_ball;
    double fighter_min_dist_to_goal;
    double fighter_last_dist_to_goal;
    double defender_min_dist_to_ball;
    double defender_min_dist_from_goal;
    double defender_last_dist_from_goal;

    bool operator<(const Score& other) const {
      return score() < other.score();
    }

    double score() const {
      return
          sum_score
              - fighter_min_dist_to_ball
              - fighter_min_dist_to_goal
              - fighter_last_dist_to_goal
              - defender_min_dist_to_ball
              + defender_min_dist_from_goal
              + defender_last_dist_from_goal;
    }

    void minimal() {
      sum_score = -1e18;
      fighter_min_dist_to_ball = 1e9;
      fighter_min_dist_to_goal = 1e9;
      fighter_last_dist_to_goal = 1e9;
      defender_min_dist_to_ball = 1e9;
      defender_min_dist_from_goal = 1e9;
      defender_last_dist_from_goal = 1e9;
    }

    void start_fighter() {
      sum_score = 0;
      fighter_min_dist_to_ball = 1e9;
      fighter_min_dist_to_goal = 1e9;
      fighter_last_dist_to_goal = 1e9;
      defender_min_dist_to_ball = 0;
      defender_min_dist_from_goal = 0;
      defender_last_dist_from_goal = 0;
    }

    void start_defender() {
      sum_score = 0;
      fighter_min_dist_to_ball = 0;
      fighter_min_dist_to_goal = 0;
      fighter_last_dist_to_goal = 0;
      defender_min_dist_to_ball = 1e9;
      defender_min_dist_from_goal = 1e9;
      defender_last_dist_from_goal = 0;
    }

  } score;
  Plan() : Plan(3, 0) {}

  Plan(int configuration,
       const int simulation_depth,
       const double initial_vx = 0,
       const double initial_vz = 0,
       const double crossing_x = 0,
       const double crossing_z = 0) : configuration(configuration) {
    unique_id = C::unique_plan_id++;
    parent_id = unique_id;

    was_jumping = false;
    was_on_ground_after_jumping = false;
    collide_with_entity_before_on_ground_after_jumping = false;
    oncoming_jump = C::NEVER;

    if (configuration == 1) {
      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      y1 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
      cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));
      angle2 = C::rand_double(0, 2 * M_PI);
      cangle2 = cos(angle2);
      sangle2 = sin(angle2);
      y2 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
      cos_lat2 = cos(asin(y2 / C::rules.MAX_ENTITY_SPEED));

      time_change = C::rand_int(0, simulation_depth);
      time_jump = C::rand_int(0, simulation_depth);

      speed1 = speed2 = 1;
      if (C::rand_double(0, 1) < 0.01) {
        speed1 = 0;
      }
      if (C::rand_double(0, 1) < 0.01) {
        speed2 = 0;
      }

      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;

      max_jump_speed = C::rand_int(0, 15);

      time_nitro_on = C::rand_int(0, simulation_depth);
      time_nitro_off = C::rand_int(0, simulation_depth);

    } else if (configuration == 2) { // smart enemy
      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      time_change = C::NEVER;
      time_jump = C::rand_int(0, simulation_depth);
      speed1 = 1.;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
      max_jump_speed = 15;
      time_nitro_on = C::NEVER;
      time_nitro_off = C::NEVER;
    } else if (configuration == 3) { // sptupid enemy
      angle1 = 0;
      cangle1 = 0;
      sangle1 = 0;
      time_change = C::NEVER;
      time_jump = C::NEVER;
      speed1 = 0;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
      max_jump_speed = 0; // todo keep in mind
      time_nitro_on = C::NEVER;
      time_nitro_off = C::NEVER;
    } else if (configuration == 4) { // last action
      angle1 = atan2(initial_vz, initial_vx);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      time_change = C::NEVER;
      time_jump = C::NEVER;
      speed1 = 1;
      max_speed = Point2d{initial_vx, initial_vz}.length();
      max_jump_speed = 15;  // todo keep in mind
      // todo last action nitro
      time_nitro_on = C::NEVER;
      time_nitro_off = C::NEVER;
    } else if (configuration == 5) {
      time_change = C::NEVER;
      time_jump = C::NEVER;
      speed1 = 1;
      max_speed = Point2d{initial_vx, initial_vz}.length();
      crossing = Point{crossing_x, 1, crossing_z};
      max_jump_speed = 15;  // todo keep in mind
      // todo last action nitro
      time_nitro_on = C::NEVER;
      time_nitro_off = C::NEVER;
    } else if (configuration == 6) {

      crossing = {C::rand_double(-30, 30), C::rand_double(0, 20), C::rand_double(-50, 50)};

      time_change = C::NEVER;
      time_jump = C::rand_int(0, simulation_depth);

      speed1 = 1;
      if (C::rand_double(0, 1) < 0.01) {
        speed1 = 0;
      }

      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;

      max_jump_speed = C::rand_int(0, 15);
      time_nitro_on = C::rand_int(0, simulation_depth);
      time_nitro_off = C::rand_int(0, simulation_depth);
    } else if (configuration == 7) {

      crossing = {C::rand_double(-30, 30), C::rand_double(0, 20), C::rand_double(-50, 50)};

      crossing2 = {C::rand_double(-30, 30), C::rand_double(0, 20), C::rand_double(-50, 50)};

      time_change = C::rand_int(0, simulation_depth);
      time_jump = C::rand_int(0, simulation_depth);

      speed1 = C::rand_double(0, 1);
      speed2 = C::rand_double(0, 1);
      //if (C::rand_double(0, 1) < 0.01) {
      //  speed1 = 0;
      //}
      //if (C::rand_double(0, 1) < 0.01) {
      //  speed2 = 0;
      //}

      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;

      max_jump_speed = C::rand_int(0, 15);
      time_nitro_on = C::rand_int(0, simulation_depth);
      time_nitro_off = C::rand_int(0, simulation_depth);
    }

    score.minimal();
  }

  static constexpr double angle_mutation = M_PI / 100;
  static constexpr double speed_mutation = 0.05;
  static constexpr double z_mutation = 1;
  static constexpr double crossing_mutation = 1;

  static constexpr int nitro_mutation = 1;
  static constexpr int jump_mutation = 1;
  static constexpr int time_mutation = 1;

  void mutate(int configuration, const int simulation_depth) {
    unique_id = C::unique_plan_id++;

    was_jumping = false;
    was_on_ground_after_jumping = false;
    collide_with_entity_before_on_ground_after_jumping = false;
    oncoming_jump = C::NEVER;

    if (configuration == 1) {
      angle1 += C::rand_double(-angle_mutation, angle_mutation);
      if (angle1 > 2 * M_PI) {
        angle1 -= 2 * M_PI;
      }
      if (angle1 < 0) {
        angle1 += 2 * M_PI;
      }
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      angle2 += C::rand_double(-angle_mutation, angle_mutation);
      if (angle2 > 2 * M_PI) {
        angle2 -= 2 * M_PI;
      }
      if (angle2 < 0) {
        angle2 += 2 * M_PI;
      }
      cangle2 = cos(angle2);
      sangle2 = sin(angle2);

      y1 += C::rand_double(-z_mutation, z_mutation);
      if (y1 > C::rules.MAX_ENTITY_SPEED) {
        y1 = C::rules.MAX_ENTITY_SPEED;
      } else if (y1 < -C::rules.MAX_ENTITY_SPEED) {
        y1 = -C::rules.MAX_ENTITY_SPEED;
      }
      cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));

      y2 += C::rand_double(-z_mutation, z_mutation);
      if (y2 > C::rules.MAX_ENTITY_SPEED) {
        y2 = C::rules.MAX_ENTITY_SPEED;
      } else if (y2 < -C::rules.MAX_ENTITY_SPEED) {
        y2 = -C::rules.MAX_ENTITY_SPEED;
      }
      cos_lat2 = cos(asin(y2 / C::rules.MAX_ENTITY_SPEED));

      if (time_change != C::NEVER) {
        time_change += C::rand_int(-time_mutation, time_mutation);
        if (time_change < 0) {
          time_change = 0;
        }
        if (time_change > simulation_depth) {
          time_change = simulation_depth;
        }
      }
      if (time_jump != C::NEVER) {
        time_jump += C::rand_int(-time_mutation, time_mutation);
        if (time_jump < 0) {
          time_jump = 0;
        }
        if (time_jump > simulation_depth) {
          time_jump = simulation_depth;
        }
      }

      /*speed1 += C::rand_double(-speed_mutation, speed_mutation); // todo change speed mutation
      if (speed1 > 1) {
        speed1 = 1;
      }
      if (speed1 < 0) {
        speed1 = 0;
      }

      speed2 += C::rand_double(-speed_mutation, speed_mutation);
      if (speed2 > 1) {
        speed2 = 1;
      }
      if (speed2 < 0) {
        speed2 = 0;
      }*/

      max_jump_speed += C::rand_int(-jump_mutation, jump_mutation);
      if (max_jump_speed < 0) {
        max_jump_speed = 0;
      }
      if (max_jump_speed > 15) {
        max_jump_speed = 15;
      }

      if (time_nitro_on != C::NEVER) {
        time_nitro_on += C::rand_int(-nitro_mutation, nitro_mutation);
        if (time_nitro_on < 0) {
          time_nitro_on = 0;
        }
        if (time_nitro_on > simulation_depth) {
          time_nitro_on = simulation_depth;
        }
      }
      if (time_nitro_off != C::NEVER) {
        time_nitro_off += C::rand_int(-nitro_mutation, nitro_mutation);
        if (time_nitro_off < 0) {
          time_nitro_off = 0;
        }
        if (time_nitro_off > simulation_depth) {
          time_nitro_off = simulation_depth;
        }
      }
    } else if (configuration == 2) { // smart enemy
      angle1 += C::rand_double(-angle_mutation, angle_mutation);
      if (angle1 > 2 * M_PI) {
        angle1 -= 2 * M_PI;
      }
      if (angle1 < 0) {
        angle1 += 2 * M_PI;
      }
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);

      if (time_jump != C::NEVER) {
        time_jump += C::rand_int(-time_mutation, time_mutation);
        if (time_jump < 0) {
          time_jump = 0;
        }
        if (time_jump > simulation_depth) {
          time_jump = simulation_depth;
        }
      }

    } else if (configuration == 6) {
      crossing.x += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing.x > 30) {
        crossing.x = 30;
      } else if (crossing.x < -30) {
        crossing.x = -30;
      }

      crossing.y += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing.y > 20) {
        crossing.y = 20;
      } else if (crossing.y < 0) {
        crossing.y = 0;
      }

      crossing.z += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing.z > 50) {
        crossing.z = 50;
      } else if (crossing.z < -50) {
        crossing.z = -50;
      }

      if (time_jump != C::NEVER) {
        time_jump += C::rand_int(-time_mutation, time_mutation);
        if (time_jump < 0) {
          time_jump = 0;
        }
        if (time_jump > simulation_depth) {
          time_jump = simulation_depth;
        }
      }

      max_jump_speed += C::rand_int(-jump_mutation, jump_mutation);
      if (max_jump_speed < 0) {
        max_jump_speed = 0;
      }
      if (max_jump_speed > 15) {
        max_jump_speed = 15;
      }

    } else if (configuration == 7) {

      crossing.x += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing.x > 30) {
        crossing.x = 30;
      } else if (crossing.x < -30) {
        crossing.x = -30;
      }

      crossing.y += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing.y > 20) {
        crossing.y = 20;
      } else if (crossing.y < 0) {
        crossing.y = 0;
      }

      crossing.z += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing.z > 50) {
        crossing.z = 50;
      } else if (crossing.z < -50) {
        crossing.z = -50;
      }

      crossing2.x += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing2.x > 30) {
        crossing2.x = 30;
      } else if (crossing2.x < -30) {
        crossing2.x = -30;
      }

      crossing2.y += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing2.y > 20) {
        crossing2.y = 20;
      } else if (crossing2.y < 0) {
        crossing2.y = 0;
      }

      crossing2.z += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing2.z > 50) {
        crossing2.z = 50;
      } else if (crossing2.z < -50) {
        crossing2.z = -50;
      }

      if (time_change != C::NEVER) {
        time_change += C::rand_int(-time_mutation, time_mutation);
        if (time_change < 0) {
          time_change = 0;
        }
        if (time_change > simulation_depth) {
          time_change = simulation_depth;
        }
      }

      if (time_jump != C::NEVER) {
        time_jump += C::rand_int(-time_mutation, time_mutation);
        if (time_jump < 0) {
          time_jump = 0;
        }
        if (time_jump > simulation_depth) {
          time_jump = simulation_depth;
        }
      }

      speed1 += C::rand_double(-speed_mutation, speed_mutation); // todo change speed mutation
      if (speed1 > 1) {
        speed1 = 1;
      }
      if (speed1 < 0) {
        speed1 = 0;
      }

      speed2 += C::rand_double(-speed_mutation, speed_mutation);
      if (speed2 > 1) {
        speed2 = 1;
      }
      if (speed2 < 0) {
        speed2 = 0;
      }

      max_jump_speed += C::rand_int(-jump_mutation, jump_mutation);
      if (max_jump_speed < 0) {
        max_jump_speed = 0;
      }
      if (max_jump_speed > 15) {
        max_jump_speed = 15;
      }

      if (time_nitro_on != C::NEVER) {

        time_nitro_on += C::rand_int(-nitro_mutation, nitro_mutation);
        if (time_nitro_on < 0) {
          time_nitro_on = 0;
        }
        if (time_nitro_on > simulation_depth) {
          time_nitro_on = simulation_depth;
        }
      }

      if (time_nitro_off != C::NEVER) {

        time_nitro_off += C::rand_int(-nitro_mutation, nitro_mutation);
        if (time_nitro_off < 0) {
          time_nitro_off = 0;
        }
        if (time_nitro_off > simulation_depth) {
          time_nitro_off = simulation_depth;
        }
      }

    }

    score.minimal();
  }

  void clearAndShift(const int simulation_depth) {
    score.minimal();

    was_jumping = false;
    was_on_ground_after_jumping = false;
    collide_with_entity_before_on_ground_after_jumping = false;
    oncoming_jump = C::NEVER;

    if (time_jump != C::NEVER) {
      time_jump--;
      if (time_jump < 0) {
        time_jump = C::NEVER;
      }
    }
    if (time_change != C::NEVER) {
      time_change--;
      if (time_change < 0) {
        time_change = C::NEVER;
        std::swap(angle1, angle2);
        std::swap(sangle1, sangle2);
        std::swap(cangle1, cangle2);
        std::swap(y1, y2);
        std::swap(cos_lat1, cos_lat2);
        std::swap(speed1, speed2);
        std::swap(crossing, crossing2);
      }
    }
    if (time_nitro_on != C::NEVER && time_nitro_off != C::NEVER) {
      time_nitro_on--;
      time_nitro_off--;
      if (time_nitro_on < 0 && time_nitro_off < 0) {
        time_nitro_on = C::NEVER;
        time_nitro_off = C::NEVER;
      } else if (time_nitro_on < 0) {
        time_nitro_on = 0;
      } else {
        time_nitro_off = 0;
      }
    }
  }

  MyAction toMyAction(int simulation_tick, bool simulation, bool can_use_nitro, const Point& position) {
    double jump_speed;
    if (simulation) {
      jump_speed = simulation_tick == time_jump ? max_jump_speed : 0;
    } else {
      jump_speed = simulation_tick == oncoming_jump ? oncoming_jump_speed : 0;
    }
    bool now_use_nitro = (simulation_tick >= time_nitro_on && simulation_tick < time_nitro_off);
    Point velocity;
    if (now_use_nitro && can_use_nitro) {
      if (simulation_tick < time_change) {
        if (configuration != 7) {
          velocity.x = speed1 * C::rules.MAX_ENTITY_SPEED * cos_lat1 * cangle1;
          velocity.y = y1;
          velocity.z = speed1 * C::rules.MAX_ENTITY_SPEED * cos_lat1 * sangle1;
        } else {
          velocity = (crossing - position).normalize() * (speed1 * C::rules.MAX_ENTITY_SPEED);
        }
      } else {
        if (configuration != 7) {
          velocity.x = speed2 * C::rules.MAX_ENTITY_SPEED * cos_lat2 * cangle2;
          velocity.y = y2;
          velocity.z = speed2 * C::rules.MAX_ENTITY_SPEED * cos_lat2 * sangle2;
        } else {
          velocity = (crossing2 - position).normalize() * (speed2 * C::rules.MAX_ENTITY_SPEED);
        }
      }
    } else {
      if (simulation_tick < time_change) {
        if (configuration != 5 && configuration != 6 && configuration != 7) {
          velocity.x = speed1 * max_speed * cangle1;
          velocity.y = 0;
          velocity.z = speed1 * max_speed * sangle1;
        } else {
          Point p = (crossing - position);
          p.y = 0;
          velocity = p.normalize() * (speed1 * max_speed);
        }
      } else {
        if (configuration != 7) {
          velocity.x = speed2 * max_speed * cangle2;
          velocity.y = 0;
          velocity.z = speed2 * max_speed * sangle2;
        } else {
          Point p = (crossing2 - position);
          p.y = 0;
          velocity = p.normalize() * (speed2 * max_speed);
        }
      }
    }
    return MyAction{velocity,
        jump_speed,
        max_jump_speed,
        (now_use_nitro && can_use_nitro)};
  }
  bool operator<(const Plan& other) const {
    return score < other.score;
  }
};

}
#endif
#endif //CODEBALL_PLAN_H

---

*** model/Action.h ***
---
#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_ACTION_H_
#define _MODEL_ACTION_H_

#include <string>
#include "../rapidjson/document.h"

namespace model {
    struct Action {
        double target_velocity_x;
        double target_velocity_y;
        double target_velocity_z;
        double jump_speed;
        bool use_nitro;

        Action() {
            this->target_velocity_x = 0.0;
            this->target_velocity_y = 0.0;
            this->target_velocity_z = 0.0;
            this->jump_speed = 0.0;
            this->use_nitro = false;
        }

        rapidjson::Value to_json(rapidjson::Document::AllocatorType& allocator) const {
            rapidjson::Value json;
            json.SetObject();
            json.AddMember("target_velocity_x", target_velocity_x, allocator);
            json.AddMember("target_velocity_y", target_velocity_y, allocator);
            json.AddMember("target_velocity_z", target_velocity_z, allocator);
            json.AddMember("jump_speed", jump_speed, allocator);
            json.AddMember("use_nitro", use_nitro, allocator);
            return json;
        }
    };
}

#endif

---

*** model/Game.cpp ***
---
#ifdef FROM_LOG
#include <model/Game.h>
int model::Game::my_id;
#endif

---

*** model/Dan.h ***
---
#ifndef CODEBALL_DAN_H
#define CODEBALL_DAN_H

#ifdef LOCAL
#include <H.h>
#else
#include "../H.h"
#endif

struct Dan {
  double distance;
  Point normal;
  int collision_surface_id;
  inline bool operator<(const Dan& other) const {
    return distance < other.distance;
  }

  inline static Dan dan_to_plane(
      const Point& point,
      const Point& point_on_plane,
      const Point& plane_normal,
      const int& collision_surface_id) {
    return {(point - point_on_plane).dot(plane_normal), plane_normal, collision_surface_id};
  }

  inline static Dan dan_to_sphere_inner(
      const double& radius,
      const Point& point,
      const Point& sphere_center,
      const double& sphere_radius,
      const int& collision_surface_id) {
    const double length_sq = (point - sphere_center).length_sq();
    if ((sphere_radius - radius) * (sphere_radius - radius) > length_sq) {
      return {1e9, {0, 0, 0}, 0};
    }
    return {sphere_radius - sqrt(length_sq), sphere_center - point, collision_surface_id};
  }

  inline static Dan dan_to_sphere_outer(
      const double& radius,
      const Point& point,
      const Point& sphere_center,
      const double& sphere_radius,
      const int& collision_surface_id) {
    const double length_sq = (point - sphere_center).length_sq();
    if (length_sq > (radius + sphere_radius) * (radius + sphere_radius)) {
      return {1e9, {0, 0, 0}, 0};
    }
    return {sqrt(length_sq) - sphere_radius, point - sphere_center, collision_surface_id};
  }

  inline static Dan dan_to_arena(Point& point, const double& radius) {
    const bool& negate_x = point.x < 0;
    const bool& negate_z = point.z < 0;
    if (negate_x) {
      point.x = -point.x;
    }
    if (negate_z) {
      point.z = -point.z;
    }
    Dan result = dan_to_arena_quarter(point, radius);
    if (negate_x) {
      result.normal.x = -result.normal.x;
      point.x = -point.x;
    }
    if (negate_z) {
      result.normal.z = -result.normal.z;
      point.z = -point.z;
    }
    return result;
  }

  inline static double my_clamp(const double& x, const double& a, const double& b) {
    if (x < a) {
      return a;
    }
    if (x > b) {
      return b;
    }
    return x;
  }

  inline static Dan dan_to_arena_quarter(const Point& point, const double& radius) {
    //H::t[11].start();
    Dan dan = Dan({1e9, {0, 0, 0}});
    //H::t[11].cur(true);

    // Ground
    // 2.59172e-07 30
    // 2.417e-08 2
    // 1.8985e-07 50
    //H::t[12].start(); // 12
    if (point.y < radius) {
      dan.distance = point.y;
      dan.normal.y = 1;
      dan.collision_surface_id = 1;
      if (point.x < 24 && point.z < 34) {
        //H::t[12].cur(true);
        return dan;
      }
    }


    //H::t[12].cur(true);

    // Side x
    // 28 30
    // 1.59766e-06 20
    // 2.05669e-06 50
    //H::t[17].start(); // 14
    if (point.x > 28) {
      dan = std::min(dan, dan_to_plane(point, {30., 0, 0}, {-1, 0, 0}, 2));
    }
    //H::t[17].cur(true);

    // Goal back corners
    // 1.48532e-05 30
    // 4.90724e-07 20
    // 47 50
    //H::t[13].start(); // 19
    if (point.z > 47) {
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  my_clamp(
                                                      point.x,
                                                      -12.,
                                                      12.
                                                  ),
                                                  my_clamp(
                                                      point.y,
                                                      3.,
                                                      7.
                                                  ),
                                                  47.},
                                              3., 3));
    }
    //H::t[13].cur(true);


    // Bottom corners 1 part
    // 27 30
    // 2.02544e-07 3
    // 3.46701e-06 50
    //H::t[14].start(); // 26
    if (point.y < 3 && point.x > 27) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  27.,
                                                  3.,
                                                  point.z
                                              },
                                              3., 4));
    }
    //H::t[14].cur(true);

    // Corner
    // 17 30
    // 1.59766e-06 20
    // 27 50
    //H::t[15].start(); // 20
    if (point.z > 27 && point.x > 17) {
      dan = std::min(dan, dan_to_sphere_inner(
          radius,
          point,
          {
              17.,
              point.y,
              27.
          },
          13., 5));
    }
    //H::t[15].cur(true);

    // Bottom corners 2 part
    // 12 30
    // 4.90724e-07 3
    // 41 50
    //H::t[16].start(); // 30
    if (point.y < 3 && point.x > 12 && point.z > 41) {
      // Side x (goal)
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  12.,
                                                  3.,
                                                  point.z
                                              },
                                              3., 6));
    }
    //H::t[16].cur(true);


    // Bottom corners 3 part
    // 17 30
    // 1.59766e-06 3
    // 27.0001 50
    //H::t[32].start(); // 31
    if (point.y < 3 && point.x > 17 && point.z > 27) {
      // Corner
      const Point2d& corner_o{17., 27.};
      const Point2d& n = Point2d{point.x, point.z} - corner_o;
      if (n.length_sq() > 100.) {
        const Point2d& o_ = corner_o + n.normalize() * 10.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 3., o_.y},
                                                3., 7));
      }

    }
    //H::t[32].cur(true);




    // Bottom corners 3 part

    // 12.0009 16
    // 9.86179e-07 2.99999
    // 37.0001 41
    //H::t[18].start(); // 29
    if (point.y < 3 && point.x > 12 && point.x < 16 && point.z > 37 && point.z < 41) {
      // Goal outer corner
      const Point2d& o{16., 41.};
      const Point2d& v = Point2d{point.x, point.z} - o;
      if (v.x < 0 && v.y < 0
          && v.length_sq() < 16.) {
        const Point2d& o_ = o + v.normalize() * 4.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 3., o_.y},
                                                3., 8));
      }
    }
    //H::t[18].cur(true);


    // Side x & ceiling (goal) 1 part
    // 13 30
    // 4.90724e-07 20
    // 41 50
    //H::t[19].start(); // 17
    if (point.z > 41 && point.x > 13) {
      // x
      dan = std::min(dan, dan_to_plane(
          point,
          {15., 0, 0},
          {-1, 0, 0}, 9));
    }
    //H::t[19].cur(true);


    // Bottom corners 4 part
    // 16 30
    // 1.77708e-06 3
    // 37 50
    //H::t[20].start(); // 27
    if (point.y < 3 && point.x > 16 && point.z > 37) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  3.,
                                                  37.
                                              },
                                              3., 10));
    }
    //H::t[20].cur(true);


    // Goal inside top corners 1 part
    // 12.0001 30
    // 7 20
    // 41 50
    //H::t[21].start(); // 24
    if (point.z > 41 && point.y > 7 && point.x > 12) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  12.,
                                                  7.,
                                                  point.z
                                              },
                                              3., 11));

    }
    //H::t[21].cur(true);



    // Side z
    // 1.08854e-05 30
    // 3.3164e-05 20
    // 38 50
    //H::t[22].start(); // 16
    if (point.z > 38) {
      const Point2d& v = Point2d{point.x, point.y} - Point2d{12., 7.};
      if (point.x >= 16.
          || point.y >= 11.
          || (v.x > 0
              && v.y > 0
              && v.length_sq() >= 16.)) {
        dan = std::min(dan, dan_to_plane(point, {0, 0, 40.}, {0, 0, -1}, 12));
      }
    }
    //H::t[22].cur(true);


    // Goal outer corner 1 part
    // 13.0007 16
    // 0.000182224 19.9999
    // 38.0002 41
    //H::t[23].start(); // 21
    if (point.x > 13 && point.x < 16 && point.z > 38 && point.z < 41) {
      // Side x
      dan = std::min(dan, dan_to_sphere_outer(radius,
                                              point,
                                              {
                                                  16.,
                                                  point.y,
                                                  41.
                                              },
                                              1., 13));

    }
    //H::t[23].cur(true);


    // Side x & ceiling (goal) 2 part
    // 1.08854e-05 30
    // 8 20
    // 41 50
    //H::t[24].start(); // 18
    if (point.z > 41 && point.y > 8) {
      // y
      dan = std::min(dan, dan_to_plane(point, {0, 10., 0}, {0, -1, 0}, 14));
    }
    //H::t[24].cur(true);


    // Ceiling corners 1 part
    // 23 30
    // 13 20
    // 7.43039e-05 49.9999
    //H::t[25].start(); // 32
    if (point.y > 13 && point.x > 23) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  23.,
                                                  13.,
                                                  point.z
                                              },
                                              7., 12));
    }
    //H::t[25].cur(true);


    // Ceiling
    // 4.36196e-05 30
    // 18 20
    // 3.01327e-06 50
    //H::t[26].start(); // 13
    if (point.y > 18) {
      dan = std::min(dan, dan_to_plane(point, {0, 20., 0}, {0, -1, 0}, 15));
    }
    //H::t[26].cur(true);




    // Ceiling corners 2 part
    // 1.08854e-05 30
    // 13 20
    // 33 50
    //H::t[27].start(); // 33
    if (point.y > 13 && point.z > 33) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  13.,
                                                  33.
                                              },
                                              7., 16));

    }
    //H::t[27].cur(true);



    // Ceiling corners 3 part
    // 17 30
    // 13 20
    // 27 50
    //H::t[33].start(); // 34
    if (point.y > 13 && point.x > 17 && point.z > 27) {
      // Corner
      const Point2d& corner_o{17., 27.};
      const Point2d& dv = Point2d{point.x, point.z} - corner_o;
      if (dv.length_sq() > 36.) {
        const Point2d& o_ = corner_o + dv.normalize() * 6.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 13., o_.y},
                                                7., 17));
      }

    }
    //H::t[33].cur(true);



    // Goal outer corner 2 part
    // 5.9563e-05 29.9999
    // 8.00095 11
    // 38.0001 41
    //H::t[28].start(); // 22
    if (point.z > 38 && point.y > 8 && point.y < 11 && point.z < 41) {
      // Ceiling
      dan = std::min(dan, dan_to_sphere_outer(radius,
                                              point,
                                              {
                                                  point.x,
                                                  11.,
                                                  41.
                                              },
                                              1., 18));
    }
    //H::t[28].cur(true);



    // Goal outer corner 3 part
    // 12 18.9966
    // 7.00001 13.9835
    // 38.0001 41
    //H::t[34].start(); // 23
    if (point.x > 12 && point.x < 19 && point.y > 7 && point.y < 14 && point.z > 38 && point.z < 41) {
      // Top corner
      const Point2d& o = Point2d{12., 7.};
      const Point2d& v = Point2d{point.x, point.y} - o;
      if (v.x > 0 && v.y > 0) {
        const Point2d& o_ = o + v.normalize() * 4.;
        dan = std::min(dan, dan_to_sphere_outer(radius,
                                                point,
                                                {o_.x, o_.y, 41.},
                                                1., 19));
      }
    }
    //H::t[34].cur(true);


    // Bottom corners 5 part
    // 5.89812e-05 30
    // 9.81447e-07 2.99997
    // 47 50
    //H::t[29].start(); // 28
    if (point.y < 3 && point.z > 47) {
      // Side z (goal)
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  3.,
                                                  47.
                                              },
                                              3., 20));
    }
    //H::t[29].cur(true);


    // Side z (goal)
    // 1.48532e-05 29.9999
    // 9.81447e-07 20
    // 48 50
    //H::t[30].start(); //15
    if (point.z > 48) {
      dan = std::min(dan, dan_to_plane(
          point,
          {0, 0, 50.},
          {0, 0, -1}, 21));
    }
    //H::t[30].cur(true);


    // Goal inside top corners 2 part
    // 1.48532e-05 29.9999
    // 7.00007 20
    // 47 50
    //H::t[31].start(); //25
    if (point.z > 47 && point.y > 7) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  7.,
                                                  47.
                                              },
                                              3., 22));
    }
    //H::t[31].cur(true);
    return dan;
  }

};

#ifndef LOCAL
namespace Frozen {

struct Dan {
  double distance;
  Point normal;
  int collision_surface_id;
  bool operator<(const Dan& other) const {
    return distance < other.distance;
  }

  static Dan dan_to_plane(
      const Point& point,
      const Point& point_on_plane,
      const Point& plane_normal,
      const int collision_surface_id) {
    return {(point - point_on_plane).dot(plane_normal), plane_normal, collision_surface_id};
  }

  static Dan dan_to_sphere_inner(
      const double radius,
      const Point& point,
      const Point& sphere_center,
      const double sphere_radius,
      const int collision_surface_id) {
    const double length_sq = (point - sphere_center).length_sq();
    if ((sphere_radius - radius) * (sphere_radius - radius) > length_sq) {
      return {1e9, {0, 0, 0}, 0};
    }
    return {sphere_radius - sqrt(length_sq), sphere_center - point, collision_surface_id};
  }

  static Dan dan_to_sphere_outer(
      const double radius,
      const Point& point,
      const Point& sphere_center,
      const double sphere_radius,
      const int collision_surface_id) {
    const double length_sq = (point - sphere_center).length_sq();
    if (length_sq > (radius + sphere_radius) * (radius + sphere_radius)) {
      return {1e9, {0, 0, 0}, 0};
    }
    return {sqrt(length_sq) - sphere_radius, point - sphere_center, collision_surface_id};
  }

  static Dan dan_to_arena(Point& point, const double radius) {
    const bool& negate_x = point.x < 0;
    const bool& negate_z = point.z < 0;
    if (negate_x) {
      point.x = -point.x;
    }
    if (negate_z) {
      point.z = -point.z;
    }
    Dan result = dan_to_arena_quarter(point, radius);
    if (negate_x) {
      result.normal.x = -result.normal.x;
      point.x = -point.x;
    }
    if (negate_z) {
      result.normal.z = -result.normal.z;
      point.z = -point.z;
    }
    return result;
  }

  inline static double my_clamp(const double x, const double a, const double b) {
    if (x < a) {
      return a;
    }
    if (x > b) {
      return b;
    }
    return x;
  }

  static Dan dan_to_arena_quarter(const Point& point, const double radius) {
    //H::t[11].start();
    Dan dan = Dan({1e9, {0, 0, 0}});
    //H::t[11].cur(true);

    // Ground
    // 2.59172e-07 30
    // 2.417e-08 2
    // 1.8985e-07 50
    //H::t[12].start(); // 12
    if (point.y < radius) {
      dan.distance = point.y;
      dan.normal.y = 1;
      dan.collision_surface_id = 1;
      if (point.x < 24 && point.z < 34) {
        //H::t[12].cur(true);
        return dan;
      }
    }


    //H::t[12].cur(true);

    // Side x
    // 28 30
    // 1.59766e-06 20
    // 2.05669e-06 50
    //H::t[17].start(); // 14
    if (point.x > 28) {
      dan = std::min(dan, dan_to_plane(point, {30., 0, 0}, {-1, 0, 0}, 2));
    }
    //H::t[17].cur(true);

    // Goal back corners
    // 1.48532e-05 30
    // 4.90724e-07 20
    // 47 50
    //H::t[13].start(); // 19
    if (point.z > 47) {
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  my_clamp(
                                                      point.x,
                                                      -12.,
                                                      12.
                                                  ),
                                                  my_clamp(
                                                      point.y,
                                                      3.,
                                                      7.
                                                  ),
                                                  47.},
                                              3., 3));
    }
    //H::t[13].cur(true);


    // Bottom corners 1 part
    // 27 30
    // 2.02544e-07 3
    // 3.46701e-06 50
    //H::t[14].start(); // 26
    if (point.y < 3 && point.x > 27) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  27.,
                                                  3.,
                                                  point.z
                                              },
                                              3., 4));
    }
    //H::t[14].cur(true);

    // Corner
    // 17 30
    // 1.59766e-06 20
    // 27 50
    //H::t[15].start(); // 20
    if (point.z > 27 && point.x > 17) {
      dan = std::min(dan, dan_to_sphere_inner(
          radius,
          point,
          {
              17.,
              point.y,
              27.
          },
          13., 5));
    }
    //H::t[15].cur(true);

    // Bottom corners 2 part
    // 12 30
    // 4.90724e-07 3
    // 41 50
    //H::t[16].start(); // 30
    if (point.y < 3 && point.x > 12 && point.z > 41) {
      // Side x (goal)
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  12.,
                                                  3.,
                                                  point.z
                                              },
                                              3., 6));
    }
    //H::t[16].cur(true);


    // Bottom corners 3 part
    // 17 30
    // 1.59766e-06 3
    // 27.0001 50
    //H::t[32].start(); // 31
    if (point.y < 3 && point.x > 17 && point.z > 27) {
      // Corner
      const Point2d& corner_o{17., 27.};
      const Point2d& n = Point2d{point.x, point.z} - corner_o;
      if (n.length_sq() > 100.) {
        const Point2d& o_ = corner_o + n.normalize() * 10.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 3., o_.y},
                                                3., 7));
      }

    }
    //H::t[32].cur(true);




    // Bottom corners 3 part

    // 12.0009 16
    // 9.86179e-07 2.99999
    // 37.0001 41
    //H::t[18].start(); // 29
    if (point.y < 3 && point.x > 12 && point.x < 16 && point.z > 37 && point.z < 41) {
      // Goal outer corner
      const Point2d& o{16., 41.};
      const Point2d& v = Point2d{point.x, point.z} - o;
      if (v.x < 0 && v.y < 0
          && v.length_sq() < 16.) {
        const Point2d& o_ = o + v.normalize() * 4.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 3., o_.y},
                                                3., 8));
      }
    }
    //H::t[18].cur(true);


    // Side x & ceiling (goal) 1 part
    // 13 30
    // 4.90724e-07 20
    // 41 50
    //H::t[19].start(); // 17
    if (point.z > 41 && point.x > 13) {
      // x
      dan = std::min(dan, dan_to_plane(
          point,
          {15., 0, 0},
          {-1, 0, 0}, 9));
    }
    //H::t[19].cur(true);


    // Bottom corners 4 part
    // 16 30
    // 1.77708e-06 3
    // 37 50
    //H::t[20].start(); // 27
    if (point.y < 3 && point.x > 16 && point.z > 37) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  3.,
                                                  37.
                                              },
                                              3., 10));
    }
    //H::t[20].cur(true);


    // Goal inside top corners 1 part
    // 12.0001 30
    // 7 20
    // 41 50
    //H::t[21].start(); // 24
    if (point.z > 41 && point.y > 7 && point.x > 12) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  12.,
                                                  7.,
                                                  point.z
                                              },
                                              3., 11));

    }
    //H::t[21].cur(true);



    // Side z
    // 1.08854e-05 30
    // 3.3164e-05 20
    // 38 50
    //H::t[22].start(); // 16
    if (point.z > 38) {
      const Point2d& v = Point2d{point.x, point.y} - Point2d{12., 7.};
      if (point.x >= 16.
          || point.y >= 11.
          || (v.x > 0
              && v.y > 0
              && v.length_sq() >= 16.)) {
        dan = std::min(dan, dan_to_plane(point, {0, 0, 40.}, {0, 0, -1}, 12));
      }
    }
    //H::t[22].cur(true);


    // Goal outer corner 1 part
    // 13.0007 16
    // 0.000182224 19.9999
    // 38.0002 41
    //H::t[23].start(); // 21
    if (point.x > 13 && point.x < 16 && point.z > 38 && point.z < 41) {
      // Side x
      dan = std::min(dan, dan_to_sphere_outer(radius,
                                              point,
                                              {
                                                  16.,
                                                  point.y,
                                                  41.
                                              },
                                              1., 13));

    }
    //H::t[23].cur(true);


    // Side x & ceiling (goal) 2 part
    // 1.08854e-05 30
    // 8 20
    // 41 50
    //H::t[24].start(); // 18
    if (point.z > 41 && point.y > 8) {
      // y
      dan = std::min(dan, dan_to_plane(point, {0, 10., 0}, {0, -1, 0}, 14));
    }
    //H::t[24].cur(true);


    // Ceiling corners 1 part
    // 23 30
    // 13 20
    // 7.43039e-05 49.9999
    //H::t[25].start(); // 32
    if (point.y > 13 && point.x > 23) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  23.,
                                                  13.,
                                                  point.z
                                              },
                                              7., 12));
    }
    //H::t[25].cur(true);


    // Ceiling
    // 4.36196e-05 30
    // 18 20
    // 3.01327e-06 50
    //H::t[26].start(); // 13
    if (point.y > 18) {
      dan = std::min(dan, dan_to_plane(point, {0, 20., 0}, {0, -1, 0}, 15));
    }
    //H::t[26].cur(true);




    // Ceiling corners 2 part
    // 1.08854e-05 30
    // 13 20
    // 33 50
    //H::t[27].start(); // 33
    if (point.y > 13 && point.z > 33) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  13.,
                                                  33.
                                              },
                                              7., 16));

    }
    //H::t[27].cur(true);



    // Ceiling corners 3 part
    // 17 30
    // 13 20
    // 27 50
    //H::t[33].start(); // 34
    if (point.y > 13 && point.x > 17 && point.z > 27) {
      // Corner
      const Point2d& corner_o{17., 27.};
      const Point2d& dv = Point2d{point.x, point.z} - corner_o;
      if (dv.length_sq() > 36.) {
        const Point2d& o_ = corner_o + dv.normalize() * 6.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 13., o_.y},
                                                7., 17));
      }

    }
    //H::t[33].cur(true);



    // Goal outer corner 2 part
    // 5.9563e-05 29.9999
    // 8.00095 11
    // 38.0001 41
    //H::t[28].start(); // 22
    if (point.z > 38 && point.y > 8 && point.y < 11 && point.z < 41) {
      // Ceiling
      dan = std::min(dan, dan_to_sphere_outer(radius,
                                              point,
                                              {
                                                  point.x,
                                                  11.,
                                                  41.
                                              },
                                              1., 18));
    }
    //H::t[28].cur(true);



    // Goal outer corner 3 part
    // 12 18.9966
    // 7.00001 13.9835
    // 38.0001 41
    //H::t[34].start(); // 23
    if (point.x > 12 && point.x < 19 && point.y > 7 && point.y < 14 && point.z > 38 && point.z < 41) {
      // Top corner
      const Point2d& o = Point2d{12., 7.};
      const Point2d& v = Point2d{point.x, point.y} - o;
      if (v.x > 0 && v.y > 0) {
        const Point2d& o_ = o + v.normalize() * 4.;
        dan = std::min(dan, dan_to_sphere_outer(radius,
                                                point,
                                                {o_.x, o_.y, 41.},
                                                1., 19));
      }
    }
    //H::t[34].cur(true);


    // Bottom corners 5 part
    // 5.89812e-05 30
    // 9.81447e-07 2.99997
    // 47 50
    //H::t[29].start(); // 28
    if (point.y < 3 && point.z > 47) {
      // Side z (goal)
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  3.,
                                                  47.
                                              },
                                              3., 20));
    }
    //H::t[29].cur(true);


    // Side z (goal)
    // 1.48532e-05 29.9999
    // 9.81447e-07 20
    // 48 50
    //H::t[30].start(); //15
    if (point.z > 48) {
      dan = std::min(dan, dan_to_plane(
          point,
          {0, 0, 50.},
          {0, 0, -1}, 21));
    }
    //H::t[30].cur(true);


    // Goal inside top corners 2 part
    // 1.48532e-05 29.9999
    // 7.00007 20
    // 47 50
    //H::t[31].start(); //25
    if (point.z > 47 && point.y > 7) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  7.,
                                                  47.
                                              },
                                              3., 22));
    }
    //H::t[31].cur(true);
    return dan;
  }

};

}
#endif

#endif //CODEBALL_DAN_H

---

*** model/NitroPack.h ***
---
#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_NITRO_PACK_H_
#define _MODEL_NITRO_PACK_H_

#include "../rapidjson/document.h"

namespace model {
    struct NitroPack {
        int id;
        double x;
        double y;
        double z;
        double radius;
        bool alive;
        int respawn_ticks;

        void read(const rapidjson::Value& json) {
            id = json["id"].GetInt();
            x = json["x"].GetDouble();
            y = json["y"].GetDouble();
            z = json["z"].GetDouble();
            radius = json["radius"].GetDouble();
            alive = json["respawn_ticks"].IsNull();
            if (!alive) {
                respawn_ticks = json["respawn_ticks"].GetInt();
            }
        }

      void read2(const rapidjson::Value& json, const int my_id) {
        id = 228;
        x = json["position"]["x"].GetDouble();
        y = json["position"]["y"].GetDouble();
        z = json["position"]["z"].GetDouble() * (my_id == 2 ? -1 : 1);
        radius = json["radius"].GetDouble();
        alive = json["respawn_ticks"].IsNull();
        if (!alive) {
          respawn_ticks = json["respawn_ticks"].GetInt();
        }
      }

    };
}

#endif
---

*** model/Entity.h ***
---
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

  inline bool operator!=(const EntityState& other) const {
    return !(*this == other);
  }
  inline bool operator==(const EntityState& other) const {
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
  inline void clear() {
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

  inline EntityState& getState() { // use outside simulator, if we dont know static or dynamic
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
  bool accelerate_trigger_on_cur_tick;
  bool accelerate_trigger_on_prev_tick = false;

  bool is_robot, is_ball, is_pack;

  inline bool operator<(const Entity& other) const {
    return id < other.id;
  }

  Entity() {}

  inline void addCollision(const Collision& collision) {
    if (collisions_size == max_collisions) {
      return;
    }
    collisions[collisions_size++] = collision;
  }

  inline void fromPack(const model::NitroPack& _pack) {
    is_pack = true;
    is_ball = is_robot = false;
    state.respawn_ticks = _pack.respawn_ticks;
    state.alive = _pack.alive;
    id = _pack.id;
    state.radius = _pack.radius;
    state.position = {_pack.x, _pack.y, _pack.z};
    collisions_size = 0;
  }

  inline void fromBall(const model::Ball& ball) {
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

  inline void fromRobot(const model::Robot& robot) {
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

  inline void saveState(const int& tick_number) {
    states[tick_number] = state;
  }

  inline void savePrevState() {
    prev_state = state;
  }

  inline void fromPrevState() {
    state = prev_state;
  }

  inline void savePrevMicroState() {
    prev_micro_state = state;
  }

  inline void fromPrevMicroState() {
    state = prev_micro_state;
  }

  inline void fromStateStatic(const int& tick_number) {
    state_ptr = states + tick_number;
    static_event_ptr = static_events + tick_number;
  }

  inline void fromState(const int& tick_number) {
    state = states[tick_number];
  }

  inline void wantToBecomeDynamic(const int& tick_number) {
    want_to_become_dynamic = true;
    want_to_become_dynamic_on_tick = tick_number;
    for (int i = 0; i < collisions_size; ++i) {
      if (collisions[i].tick >= tick_number && (!collisions[i].e->want_to_become_dynamic
          || collisions[i].e->want_to_become_dynamic_on_tick > tick_number)) {
        collisions[i].e->wantToBecomeDynamic(collisions[i].tick);
      }
    }
  }

  inline void nitroCheck() {
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

#ifndef LOCAL
namespace Frozen {

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

  EntityState prev_state;
  EntityState prev_micro_state;
  EntityState states[101];
  StaticEvent static_events[101];
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

    arena_e = C::rules.ROBOT_ARENA_E;
    mass = C::rules.ROBOT_MASS;
    radius_change_speed = 0;

    id = robot.id;
    is_teammate = robot.is_teammate;

    collisions_size = 0;

    action = {{0, 0, 0}, 0, 0, false};

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

}
#endif

#endif //CODEBALL_ENTITY_H

---

*** model/P.cpp ***
---
#ifdef LOCAL
#include <model/P.h>
#else
#include "P.h"
#endif


std::vector<P::Line> P::lines_to_draw;
std::vector<P::Sphere> P::spheres_to_draw;
std::vector<std::string>  P::logs;


#ifndef LOCAL
namespace Frozen {

std::vector<P::Line> P::lines_to_draw;
std::vector<P::Sphere> P::spheres_to_draw;
std::vector<std::string>  P::logs;

}
#endif
---

*** model/Ball.h ***
---
#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_BALL_H_
#define _MODEL_BALL_H_

#include "../rapidjson/document.h"

namespace model {
struct Ball {
  double x;
  double y;
  double z;
  double velocity_x;
  double velocity_y;
  double velocity_z;
  double radius;

  void read(const rapidjson::Value& json) {
    x = json["x"].GetDouble();
    y = json["y"].GetDouble();
    z = json["z"].GetDouble();
    velocity_x = json["velocity_x"].GetDouble();
    velocity_y = json["velocity_y"].GetDouble();
    velocity_z = json["velocity_z"].GetDouble();
    radius = json["radius"].GetDouble();
  }

  void read2(const rapidjson::Value& json, int my_id) {
    x = json["position"]["x"].GetDouble();
    y = json["position"]["y"].GetDouble();
    z = json["position"]["z"].GetDouble() * (my_id == 2 ? -1 : 1);
    velocity_x = json["velocity"]["x"].GetDouble();
    velocity_y = json["velocity"]["y"].GetDouble();
    velocity_z = json["velocity"]["z"].GetDouble() * (my_id == 2 ? -1 : 1);
    radius = json["radius"].GetDouble();
  }

};
}

#endif
---

*** model/P.h ***
---
#ifndef CODEBALL_PAINTER_H
#define CODEBALL_PAINTER_H

#ifdef LOCAL
#include <model/Entity.h>
#else
#include "Entity.h"
#endif

struct P {
  struct Line {
    Point a, b;
    uint32_t color;
    double getR() {
      return color / 256 / 256 / 255.;
    }
    double getG() {
      return color / 256 % 256 / 255.;
    }
    double getB() {
      return color % 256 / 255.;
    }
    double getA() {
      return 0.5;
    }
  };

  struct Sphere {
    Point center;
    double radius;
    uint32_t color;

    double getR() {
      return color / 256 / 256 / 255.;
    }
    double getG() {
      return color / 256 % 256 / 255.;
    }
    double getB() {
      return color % 256 / 255.;
    }
    double getA() {
      return 1;
    }
  };

  static std::vector<Line> lines_to_draw;
  static std::vector<Sphere> spheres_to_draw;
  static std::vector<std::string> logs;

  static void drawLine(const Point& p1, const Point& p2, const uint32_t color = 0) {
#ifdef LOCAL
#ifdef DRAWLR
    lines_to_draw.push_back(Line{p1, p2, color});
#else
    RewindClient::instance().line3d(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, color);
#endif
#endif
  }

  static void drawEntities(std::vector<EntityState> entities, const EntityState& e, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
    entities.push_back(e);
    drawEntities(entities, delta_time, color);
#endif
  }

  static void drawEntities(const EntityState& e, std::vector<EntityState>& entities, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
    entities.push_back(e);
    drawEntities(entities, delta_time, color);
#endif
  }

  static void drawEntities(const EntityState& e, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
    drawEntities(std::vector<EntityState>{e}, delta_time, color);
#endif
  }

  static void drawSphere(const Point& position, const double radius,
                         const uint32_t color = 0) {
#ifdef LOCAL
#ifdef DRAWLR
    spheres_to_draw.push_back(Sphere{position, radius, color});
#endif
#endif
  }

  static void drawEntities(const std::vector<EntityState>& entities, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
#ifdef DRAWLR
    for (auto& e : entities) {
      spheres_to_draw.push_back(Sphere{e.position, e.radius, color});
      //Point next_pos = e.position + e.velocity * delta_time;
      //lines_to_draw.push_back(Line{e.position, next_pos, color});
    }
#else
    auto& draw = RewindClient::instance();
      draw.circle3d(
          e.position.x,
          e.position.y,
          e.position.z,
          e.radius, color);
      Point next_pos = e.position + e.velocity * delta_time;
      draw.line3d(
          e.position.x,
          e.position.y,
          e.position.z,
          next_pos.x,
          next_pos.y,
          next_pos.z, color);
    }
#endif
#endif
  }

  static void drawArena() {
#ifdef LOCAL
#ifndef DRAWLR
    auto& draw = RewindClient::instance();
    //walls
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    //floor
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2);
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2);
    //ceiling
    draw.line3d(
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    draw.line3d(
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    //goal1
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    //goal2
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
#endif
#endif
  }

  static void endFrame() {
#ifdef LOCAL
#ifndef DRAWLR
    RewindClient::instance().end_frame();
#endif
#endif
  }

  template<typename... Args>
  static void logn(Args... args) {
#ifdef LOCAL
#ifdef DRAWLR
    logs.push_back(to_string(args...));
#endif
#endif
  }

  template<typename... Args>
  static void log(Args... args) {
#ifdef LOCAL
#ifdef DRAWLR
    if (logs.size() == 0) {
      logn(args...);
    } else {
      logs.back() += to_string(args...);
    }
#endif
#endif
  }

  template<typename T, typename... Args>
  static inline std::string to_string(T fmt, Args... args) {
    return to_string(fmt) + to_string(args...);
  }
  static inline std::string to_string(std::string fmt) {
    return fmt;
  }
  static inline std::string to_string(double fmt) {
    std::stringstream _s;
    _s << std::fixed << std::setprecision(9) << fmt;
    return _s.str();
  }
  static inline std::string to_string(const char*& fmt) {
    return std::string(fmt);
  }
  template<typename T>
  static inline std::string to_string(T fmt) {
    return std::to_string(fmt);
  }

};

#ifndef LOCAL
namespace Frozen {

struct P {
  struct Line {
    Point a, b;
    uint32_t color;
    double getR() {
      return color / 256 / 256 / 255.;
    }
    double getG() {
      return color / 256 % 256 / 255.;
    }
    double getB() {
      return color % 256 / 255.;
    }
    double getA() {
      return 0.5;
    }
  };

  struct Sphere {
    Point center;
    double radius;
    uint32_t color;

    double getR() {
      return color / 256 / 256 / 255.;
    }
    double getG() {
      return color / 256 % 256 / 255.;
    }
    double getB() {
      return color % 256 / 255.;
    }
    double getA() {
      return 1;
    }
  };

  static std::vector<Line> lines_to_draw;
  static std::vector<Sphere> spheres_to_draw;
  static std::vector<std::string> logs;

  static void drawLine(const Point& p1, const Point& p2, const uint32_t color = 0) {
#ifdef LOCAL
#ifdef DRAWLR
    lines_to_draw.push_back(Line{p1, p2, color});
#else
    RewindClient::instance().line3d(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, color);
#endif
#endif
  }

  static void drawEntities(std::vector<EntityState> entities, const EntityState& e, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
    entities.push_back(e);
    drawEntities(entities, delta_time, color);
#endif
  }

  static void drawEntities(const EntityState& e, std::vector<EntityState>& entities, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
    entities.push_back(e);
    drawEntities(entities, delta_time, color);
#endif
  }

  static void drawEntities(const EntityState& e, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
    drawEntities(std::vector<EntityState>{e}, delta_time, color);
#endif
  }

  static void drawSphere(const Point& position, const double radius,
                           const uint32_t color = 0) {
#ifdef LOCAL
#ifdef DRAWLR
    spheres_to_draw.push_back(Sphere{position, radius, color});
#endif
#endif
  }

  static void drawEntities(const std::vector<EntityState>& entities, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
#ifdef DRAWLR
    for (auto& e : entities) {
      spheres_to_draw.push_back(Sphere{e.position, e.radius, color});
      //Point next_pos = e.position + e.velocity * delta_time;
      //lines_to_draw.push_back(Line{e.position, next_pos, color});
    }
#else
    auto& draw = RewindClient::instance();
      draw.circle3d(
          e.position.x,
          e.position.y,
          e.position.z,
          e.radius, color);
      Point next_pos = e.position + e.velocity * delta_time;
      draw.line3d(
          e.position.x,
          e.position.y,
          e.position.z,
          next_pos.x,
          next_pos.y,
          next_pos.z, color);
    }
#endif
#endif
  }

  static void drawArena() {
#ifdef LOCAL
#ifndef DRAWLR
    auto& draw = RewindClient::instance();
    //walls
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    //floor
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2);
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2);
    //ceiling
    draw.line3d(
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    draw.line3d(
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    //goal1
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    //goal2
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
#endif
#endif
  }

  static void endFrame() {
#ifdef LOCAL
#ifndef DRAWLR
    RewindClient::instance().end_frame();
#endif
#endif
  }

  template<typename... Args>
  static void logn(Args... args) {
#ifdef LOCAL
#ifdef DRAWLR
    logs.push_back(to_string(args...));
#endif
#endif
  }

  template<typename... Args>
  static void log(Args... args) {
#ifdef LOCAL
#ifdef DRAWLR
    if (logs.size() == 0) {
      logn(args...);
    } else {
      logs.back() += to_string(args...);
    }
#endif
#endif
  }

  template<typename T, typename... Args>
  static inline std::string to_string(T fmt, Args... args) {
    return to_string(fmt) + to_string(args...);
  }
  static inline std::string to_string(std::string fmt) {
    return fmt;
  }
  static inline std::string to_string(double fmt) {
    std::stringstream _s;
    _s << std::fixed << std::setprecision(9) << fmt;
    return _s.str();
  }
  static inline std::string to_string(const char*& fmt) {
    return std::string(fmt);
  }
  template<typename T>
  static inline std::string to_string(T fmt) {
    return std::to_string(fmt);
  }

};

}
#endif

#endif //CODEBALL_PAINTER_H

---

*** model/C.cpp ***
---
#ifdef LOCAL
#include <model/C.h>
#else
#include "C.h"
#endif

model::Rules C::rules;
std::mt19937_64 C::rd;
int C::unique_plan_id = 1;

#ifndef LOCAL
namespace Frozen {

model::Rules C::rules;
std::mt19937_64 C::rd;
int C::unique_plan_id = 1;

}
#endif
---

*** model/getCPUTime.h ***
---
#ifndef CODEBALL_GETCPUTIME_H
#define CODEBALL_GETCPUTIME_H

struct CPUTime {
  static double getCPUTime() {
    clock_t cl = clock();
    return (double) cl / (double) CLOCKS_PER_SEC;
  }
};

#endif //CODEBALL_GETCPUTIME_H

---

*** H.cpp ***
---
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
---
