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
      static_entities[i]->fromStateStatic(tick_number + 1);
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


  double getSumScoreFighter(const int tick_number, const double goal_multiplier, const bool was_ball_hit) {
    double score = 0;
    if (goal_info.goal_to_me) {
      score += tick_number == goal_info.goal_tick ? -1e9 : 0;
    } else if (goal_info.goal_to_enemy) {
      //const double& height = ball->getState().position.y;
      //const double& height_score = 1e3 + 1e3 * ((height - 2) / 6.);
      //score += tick_number == goal_info.goal_tick ? height_score : 0;
      score += (tick_number == goal_info.goal_tick && was_ball_hit) ?  1e9 * goal_multiplier : 0;
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
      if (main_robot->action.use_nitro) {
        score -= 0.01 * C::TPT;
      }

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

  double getSumScoreDefender(const int tick_number) {
    double score = 0;
    if (goal_info.goal_to_me) {
      score += tick_number == goal_info.goal_tick ? -1e9 : 0;
    } else if (goal_info.goal_to_enemy) {
      score += tick_number == goal_info.goal_tick ? 1e3 : 0;
    }
    if (!(goal_info.goal_to_me || goal_info.goal_to_enemy) || tick_number <= goal_info.goal_tick) {
      if (!main_robot->state.touch) {
        score -= 0.5 * C::TPT;
      }
      score += 0.01 * main_robot->taken_nitro;

      if (main_robot->action.use_nitro) {
        score -= 0.01 * C::TPT;
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
