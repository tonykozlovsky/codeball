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

  Entity initial_static_entities[7];
  int initial_static_entities_size = 0;

  Entity initial_dynamic_entities[7];
  int initial_dynamic_entities_size = 0;

  Entity* initial_static_robots[7];
  int initial_static_robots_size = 0;

  Entity* initial_dynamic_robots[7];
  int initial_dynamic_robots_size = 0;

  Entity* static_entities[7];
  int static_entities_size = 0;

  Entity* dynamic_entities[7];
  int dynamic_entities_size = 0;

  Entity* static_robots[7];
  int static_robots_size = 0;

  Entity* dynamic_robots[7];
  int dynamic_robots_size = 0;

  Entity* main_robot;
  Entity* ball;

  bool any_triggers_fired;
  std::vector<std::string> trigger_fired_causes;

  bool goal_to_me, goal_to_enemy;
  int goal_tick;

  // maybe we can have 4x-5x performance boost, and more when 3x3
  SmartSimulator(const int main_robot_id, const std::vector<model::Robot>& _robots, const model::Ball& _ball) {

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

    for (int i = 0; i < C::MAX_SIMULATION_DEPTH + 1; ++i) {
      for (int j = 0; j < initial_static_robots_size; ++j) {
        auto& robot = initial_static_robots[j];
        if (robot->is_teammate) {
          robot->action = H::best_plan[robot->id % 2].toMyAction(i);
          if (robot->action.jump_speed > 0 && !robot->state.touch) {
            robot->action.jump_speed = 0;
          }
        }
      }
      tickWithJumpsStatic(i, true);
    }
    /*
    if (main_robot_id == 4) {
      for (int i = 0; i < initial_static_entities_size; ++i) {
        auto& e = initial_static_entities[i];
        for (int j = 1; j < 100; ++j) {
          P::drawLine(e.states[j - 1].position, e.states[j].position, 0xAA0000);
        }
      }
    }*/


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

  void clearCollideWithBallInAirStatic() {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      initial_static_robots[i]->collide_with_ball_in_air = false;
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

  void tickMicroticksStatic(const int number_of_tick, const int number_of_microticks) {
    any_triggers_fired = false;
    trigger_fired_causes.clear();
    if (number_of_microticks == 0) {
      return;
    }
    updateStatic((double) number_of_microticks / C::rules.TICKS_PER_SECOND / C::rules.MICROTICKS_PER_TICK, number_of_tick, number_of_microticks);

    /*if (is_fair) {
      std::cout << "fair: ";
    } else {
      //H::t[44].cur(false, true);
      std::cout << "not fair: ";
    }
    std::cout << number_of_tick << " " << number_of_microticks << "\n";
    for (auto& cause : trigger_fired_causes) {
      std::cout << cause << std::endl;
    }
    std::cout << std::endl;*/
  }

  void tickWithJumpsStatic(const int tick_number, bool with_jumps) {
    for (int i = 0; i < initial_static_entities_size; ++i) { // save state
      auto& e = initial_static_entities[i];
      e.saveState(tick_number);
    }
    if (with_jumps) {
      clearCollideWithBallInAirStatic();
    }
    tickStatic(tick_number);
    if (with_jumps) {
      bool needs_rollback = false;
      for (int i = 0; i < initial_static_robots_size; ++i) {
        auto& e = initial_static_robots[i];
        if (e->collide_with_ball_in_air) {
          needs_rollback = true;
          e->action.jump_speed = C::rules.ROBOT_MAX_JUMP_SPEED;
        }
      }
      if (needs_rollback) {
        //P::logn("needs rollback on tick: ", tick_number);
        for (int i = 0; i < initial_static_entities_size; ++i) { // from state
          auto& e = initial_static_entities[i];
          e.fromState(tick_number);
        }
        tickStatic(tick_number);
      }
    }
  }

  void tickStatic(const int tick_number) {
    int remaining_microticks = 100;
    if (somebodyJumpThisTickStatic()) {
      tickMicroticksStatic(tick_number, 1);
      tickMicroticksStatic(tick_number, 1);
      remaining_microticks = 98;
    } else if (tick_number == 0) {
      tickMicroticksStatic(tick_number, 1);
      remaining_microticks = 99;
    }
    bool need_more_iterations = true;
    int max_iterations = 0;
    while (need_more_iterations) {
      max_iterations--;
      need_more_iterations = false;

      for (int i = 0; i < initial_static_entities_size; ++i) {
        auto& e = initial_static_entities[i];
        e.savePrevMicroState();
      }

      tickMicroticksStatic(tick_number, remaining_microticks);

      if (any_triggers_fired && remaining_microticks > 1 && max_iterations >= 0) {
        int l = 0;
        int r = remaining_microticks;
        while (r - l > 1) {
          int mid = (r + l) / 2;
          for (int i = 0; i < initial_static_entities_size; ++i) {
            auto& e = initial_static_entities[i];
            e.fromPrevMicroState();
          }
          tickMicroticksStatic(tick_number, mid);
          if (any_triggers_fired) {
            r = mid;
          } else {
            l = mid;
          }
        }
        for (int i = 0; i < initial_static_entities_size; ++i) {
          auto& e = initial_static_entities[i];
          e.fromPrevMicroState();
        }
        if (l > 0) {
          tickMicroticksStatic(tick_number, l);
          remaining_microticks -= l;
        }
        tickMicroticksStatic(tick_number, 1);
        remaining_microticks--;
        need_more_iterations = true;
      }
    }
  }

  bool collideEntitiesStatic(Entity* a, Entity* b, bool check_with_ball) {
    const Point& delta_position = b->state.position - a->state.position;
    const double distance_sq = delta_position.length_sq();
    const double sum_r = a->state.radius + b->state.radius;
    if (check_with_ball && (3.05) * (3.05) > distance_sq) {
      a->collide_with_ball_in_air = true;
    }
    if (sum_r * sum_r > distance_sq) {
      const double penetration = sum_r - sqrt(distance_sq);
      any_triggers_fired = true;
      //trigger_fired_causes.push_back("collide_entities " + std::to_string(a->id) + " " + std::to_string(b->id) + " " + std::to_string(penetration));
      const double k_a = 1. / (a->mass * ((1 / a->mass) + (1 / b->mass)));
      const double k_b = 1. / (b->mass * ((1 / a->mass) + (1 / b->mass)));
      const Point& normal = delta_position.normalize();
      a->state.position -= normal * (penetration * k_a);
      b->state.position += normal * (penetration * k_b);
      const double delta_velocity = (b->state.velocity - a->state.velocity).dot(normal) - (b->radius_change_speed + a->radius_change_speed);
      if (delta_velocity < 0) {
        const Point& impulse = normal * ((1. + (C::rules.MIN_HIT_E + C::rules.MAX_HIT_E) / 2.) * delta_velocity);
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

  bool updateDynamic(const double delta_time, const int number_of_tick, const int number_of_microticks) {

    //H::t[7].cumulative += 1e-3;
    bool has_collision_with_static = false;

    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& robot = dynamic_robots[i];
      if (robot->state.touch) {
        const Point& target_velocity = robot->action.target_velocity - robot->state.touch_normal * robot->state.touch_normal.dot(robot->action.target_velocity);
        const Point& target_velocity_change = target_velocity - robot->state.velocity;
        double length = target_velocity_change.length_sq();
        if (length > 0) {
          const double acceleration = C::rules.ROBOT_ACCELERATION * fmax(0., robot->state.touch_normal.y);
          length = sqrt(length);
          const double delta = length - acceleration * delta_time;
          if (delta > 0) {
            const auto& robot_acceleration = target_velocity_change * (acceleration * delta_time / length);
            robot->state.velocity += robot_acceleration;
            const double coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / 2. / number_of_microticks) : 0.;
            robot->state.position -= robot_acceleration * (coef * delta_time);
          } else {
            if (robot->state.touch_surface_id == 1) {
              //H::t[14].cumulative += 1e-3;
              any_triggers_fired = true;
            }
            robot->state.velocity += target_velocity_change;
          }
        }
      }

      moveDynamic(robot, delta_time);

      robot->state.radius = C::rules.ROBOT_MIN_RADIUS + (C::rules.ROBOT_MAX_RADIUS - C::rules.ROBOT_MIN_RADIUS) * robot->action.jump_speed / C::rules.ROBOT_MAX_JUMP_SPEED;
      robot->radius_change_speed = robot->action.jump_speed;
    }

    moveDynamic(ball, delta_time);

    for (int i = 0; i < static_robots_size; i++) {
      for (int j = 0; j < dynamic_robots_size; j++) {
        if (collideEntitiesCheckDynamic(static_robots[i], dynamic_robots[j])) {
          static_robots[i]->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
          break;
        }
      }
    }

    for (int i = 0; i < dynamic_robots_size; i++) {
      for (int j = 0; j < i; j++) {
        collideEntitiesDynamic(dynamic_robots[i], dynamic_robots[j], false);
      }
    }

    Point collision_normal;
    int touch_surface_id;

    for (int i = 0; i < dynamic_robots_size; i++) {
      auto& robot = dynamic_robots[i];
      if (ball->is_dynamic) {
        collideEntitiesDynamic(robot, ball, true);
      } else {
        if (collideEntitiesCheckDynamic(robot, ball)) {
          ball->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
        }
      }
      if (!collideWithArenaDynamic(robot, collision_normal, touch_surface_id)) {
        if (robot->state.touch) {
          any_triggers_fired = true;
          //H::t[16].cumulative += 1e-3;
        }
        robot->state.touch = false;
      } else {
        if (!robot->state.touch || robot->state.touch_surface_id != touch_surface_id) {
          //if (robot->is_teammate && touch_surface_id != 1) {
          //  robot->collide_with_ball_in_air = true;
          //}
          //H::t[17].cumulative += 1e-3;
          any_triggers_fired = true;
        }
        robot->state.touch_surface_id = touch_surface_id;
        robot->state.touch = true;
        robot->state.touch_normal = collision_normal;
      }
    }
    if (ball->is_dynamic) {
      for (int i = 0; i < static_robots_size; i++) {
        if (collideEntitiesCheckDynamic(static_robots[i], ball)) {
          static_robots[i]->wantToBecomeDynamic(number_of_tick);
          has_collision_with_static = true;
        }
      }
      if (has_collision_with_static) {
        return true;
      }

      if (!collideWithArenaDynamic(ball, collision_normal, touch_surface_id)) {
        if (ball->state.touch) {
          if (ball->state.touch_surface_id != 1 || ball->state.velocity.y > C::ball_antiflap) {
            //H::t[18].cumulative += 1e-3;
            any_triggers_fired = true;
            ball->state.touch = false;
          }
        }
      } else {
        if (!ball->state.touch || ball->state.touch_surface_id != touch_surface_id) {
          //H::t[19].cumulative += 1e-3;
          any_triggers_fired = true;
        }
        ball->state.touch_surface_id = touch_surface_id;
        ball->state.touch = true;
      }
    }

    return has_collision_with_static;
  }

  int cur_iteration;

  void initIteration(const int iteration) {
    goal_to_me = false;
    goal_to_enemy = false;
    cur_iteration = iteration;
    //H::t[4].cumulative += 1e-3;
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
    //H::t[3].cumulative += 1e-3;
    static_robots_size = 0;
    int new_static_entities_size = 0;
    for (int i = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      if (e->want_to_become_dynamic && e->want_to_become_dynamic_on_tick == tick_number) {
        e->fromState(tick_number);
        e->is_dynamic = true;
        dynamic_entities[dynamic_entities_size++] = e;
        if (e != ball) {
          dynamic_robots[dynamic_robots_size++] = e;
        }
      } else {
        static_entities[new_static_entities_size++] = e;
        if (e != ball) {
          static_robots[static_robots_size++] = e;
        }
      }
    }
    static_entities_size = new_static_entities_size;
  }

  void clearCollideWithBallInAirDynamic() {
    for (int i = 0; i < dynamic_robots_size; ++i) {
      dynamic_robots[i]->collide_with_ball_in_air = false;
    }
  }

  bool tryTickWithJumpsDynamic(const int tick_number, bool with_jumps, bool& is_main_robot_collide_with_ball_in_air) {
    //H::t[5].cumulative += 1e-3;
    if (with_jumps) {
      clearCollideWithBallInAirDynamic();
      is_main_robot_collide_with_ball_in_air = false;
    }
    bool sbd_become_dynamic = tickDihaDynamic(tick_number); // todo check need return here
    if (with_jumps) {
      bool needs_rollback = false;
      for (int i = 0; i < dynamic_robots_size; ++i) {
        auto& e = dynamic_robots[i];
        if (e->collide_with_ball_in_air) {
          needs_rollback = true;
          e->action.jump_speed = C::rules.ROBOT_MAX_JUMP_SPEED;
          if (e == main_robot) {
            is_main_robot_collide_with_ball_in_air = true;
          }
        }
      }
      if (needs_rollback) {
        for (int i = 0; i < dynamic_entities_size; ++i) {
          auto& e = dynamic_entities[i];
          e->fromPrevState();
        }
        sbd_become_dynamic = tickDihaDynamic(tick_number);
      }
    }
    return sbd_become_dynamic;
  }

  bool tryDoTickWithoutAnybodyBecomingDynamic(const int tick_number, bool& is_main_robot_collide_with_ball_in_air) {
    return tryTickWithJumpsDynamic(tick_number, true, is_main_robot_collide_with_ball_in_air);
  }

  bool somebodyJumpThisTickDynamic() {
    for (int i = 0; i < dynamic_robots_size; ++i) {
      if (dynamic_robots[i]->action.jump_speed > 0) {
        return true;
      }
    }
    return false;
  }

  bool tickMicroticksDynamic(const int number_of_tick, const int number_of_microticks) {
    any_triggers_fired = false;
    // trigger_fired_causes.clear();
    if (number_of_microticks == 0) {
      return false;
    }
    return updateDynamic((double) number_of_microticks / C::rules.TICKS_PER_SECOND / C::rules.MICROTICKS_PER_TICK, number_of_tick, number_of_microticks);
    /*if (is_fair) {
      std::cout << "fair: ";
    } else {
      //H::t[44].cur(false, true);
      std::cout << "not fair: ";
    }
    std::cout << number_of_tick << " " << number_of_microticks << "\n";
    for (auto& cause : trigger_fired_causes) {
      std::cout << cause << std::endl;
    }
    std::cout << std::endl;*/
  }

  bool tickDihaDynamic(const int tick_number) {
    //H::t[6].cumulative += 1e-3;
    bool sbd_wants_to_become_dynamic = false;
    int remaining_microticks = 100;
    if (somebodyJumpThisTickDynamic()) {
      //H::t[8].cumulative += 1e-3;
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1);
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1);
      remaining_microticks = 98;
    } else if (tick_number == 0) {
      //H::t[9].cumulative += 1e-3;
      sbd_wants_to_become_dynamic |= tickMicroticksDynamic(tick_number, 1); //todo do single initialisation (low prior)
      remaining_microticks = 99;
    }
    if (sbd_wants_to_become_dynamic) {
      return true;
    }
    bool need_more_iterations = true;
    int iteration = 0;
    int max_iterations = 1;
    while (need_more_iterations) {
      iteration++;
      need_more_iterations = false;

      for (int i = 0; i < dynamic_entities_size; ++i) {
        auto& e = dynamic_entities[i];
        e->savePrevMicroState();
      }

      sbd_wants_to_become_dynamic = tickMicroticksDynamic(tick_number, remaining_microticks);
      //H::t[10].cumulative += 1e-3;
      if (iteration == 1 && sbd_wants_to_become_dynamic) {
        return true;
      }

      if (any_triggers_fired && remaining_microticks > 1 && iteration <= max_iterations) {
        int l = 0;
        int r = remaining_microticks;
        while (r - l > 1) {
          int mid = (r + l) / 2;
          for (int i = 0; i < dynamic_entities_size; ++i) {
            auto& e = dynamic_entities[i];
            e->fromPrevMicroState();
          }
          //H::t[11].cumulative += 1e-3;
          tickMicroticksDynamic(tick_number, mid);
          if (any_triggers_fired) {
            r = mid;
          } else {
            l = mid;
          }
        }
        for (int i = 0; i < dynamic_entities_size; ++i) {
          auto& e = dynamic_entities[i];
          e->fromPrevMicroState();
        }
        if (l > 0) {
          //H::t[12].cumulative += 1e-3;
          tickMicroticksDynamic(tick_number, l);
          remaining_microticks -= l;
        }
        //H::t[13].cumulative += 1e-3;
        tickMicroticksDynamic(tick_number, 1);
        remaining_microticks--;
        need_more_iterations = true;
      }
    }
    return false;
  }

  bool tickDynamic(const int tick_number, bool viz) {
    if (goal_to_me || goal_to_enemy) {
      return false;
    }
    bool is_main_robot_collide_with_ball_in_air = false;
    //H::t[1].cumulative += 1e-3;
    wantedStaticGoToDynamic(tick_number);
    for (int i = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      e->fromState(tick_number + 1);
    }
    for (int i = 0; i < dynamic_entities_size; ++i) {
      auto& e = dynamic_entities[i];
      e->savePrevState();
    }
    if (tryDoTickWithoutAnybodyBecomingDynamic(tick_number, is_main_robot_collide_with_ball_in_air)) {
      //H::t[2].cumulative += 1e-3;
      for (int i = 0; i < dynamic_entities_size; ++i) {
        auto& e = dynamic_entities[i];
        e->fromPrevState();
      }
      wantedStaticGoToDynamic(tick_number);
      for (int i = 0; i < dynamic_entities_size; ++i) {
        auto& e = dynamic_entities[i];
        e->savePrevState();
      }
      tryDoTickWithoutAnybodyBecomingDynamic(tick_number, is_main_robot_collide_with_ball_in_air);
    }
    if (!goal_to_me && !goal_to_enemy) {
      if (ball->state.position.z > C::rules.arena.depth / 2 + 2) {
        goal_to_enemy = true;
        goal_tick = tick_number;
      } else if (ball->state.position.z < -C::rules.arena.depth / 2 - 2) {
        goal_to_me = true;
        goal_tick = tick_number;
      }
    }
    /*
    if (main_robot->id == 4 && viz) {
      for (int i = 0; i < dynamic_entities_size; ++i) {
        auto& e = dynamic_entities[i];
        P::drawLine(e->state.position, e->prev_state.position, 0xFFFFFF);
      }
    }*/
    return is_main_robot_collide_with_ball_in_air;
  }

  bool collideEntitiesDynamic(Entity* a, Entity* b, bool check_with_ball) {
    const Point& delta_position = b->state.position - a->state.position;
    const double distance_sq = delta_position.length_sq();
    const double sum_r = a->state.radius + b->state.radius;
    if (check_with_ball && (3.05) * (3.05) > distance_sq) {
      /*
      if (cur_iteration == 400 && a->id == 4) {
        P::drawEntities({a->state, b->state});
      }*/
      a->collide_with_ball_in_air = true;
    }
    if (sum_r * sum_r > distance_sq) {
      const double penetration = sum_r - sqrt(distance_sq);
      //H::t[15].cumulative += 1e-3;
      any_triggers_fired = true;
      const double k_a = 1. / (a->mass * ((1 / a->mass) + (1 / b->mass)));
      const double k_b = 1. / (b->mass * ((1 / a->mass) + (1 / b->mass)));
      const Point& normal = delta_position.normalize();
      a->state.position -= normal * (penetration * k_a);
      b->state.position += normal * (penetration * k_b);
      const double delta_velocity = (b->state.velocity - a->state.velocity).dot(normal) - (b->radius_change_speed + a->radius_change_speed);
      if (delta_velocity < 0) {
        const Point& impulse = normal * ((1. + (C::rules.MIN_HIT_E + C::rules.MAX_HIT_E) / 2.) * delta_velocity);
        a->state.velocity += impulse * k_a;
        b->state.velocity -= impulse * k_b;
        return true;
      }
    }
    return false;
  }

  bool collideEntitiesCheckDynamic(Entity* a, Entity* b) {
    return (a->state.radius + b->state.radius) * (a->state.radius + b->state.radius) > (b->state.position - a->state.position).length_sq();
  }

  bool collideWithArenaDynamic(Entity* e, Point& result, int& collision_surface_id) {
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

  void moveDynamic(Entity* e, const double delta_time) {
    e->state.velocity = e->state.velocity.clamp(C::rules.MAX_ENTITY_SPEED);
    e->state.position += e->state.velocity * delta_time;
    e->state.position.y -= C::rules.GRAVITY * delta_time * delta_time / 2;
    e->state.velocity.y -= C::rules.GRAVITY * delta_time;
  }

  void updateStatic(const double delta_time, const int number_of_tick, const int number_of_microticks) {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& robot = initial_static_robots[i];
      if (robot->state.touch) {
        const Point& target_velocity = robot->action.target_velocity - robot->state.touch_normal * robot->state.touch_normal.dot(robot->action.target_velocity);
        const Point& target_velocity_change = target_velocity - robot->state.velocity;
        double length = target_velocity_change.length_sq();
        if (length > 0) {
          const double acceleration = C::rules.ROBOT_ACCELERATION * fmax(0., robot->state.touch_normal.y);
          length = sqrt(length);
          const double delta = length - acceleration * delta_time;
          if (delta > 0) {
            //if (!is_fair) {
            // P::log("y", robot->id, "|", number_of_tick * 100 + number_of_microticks, " ");
            //}
            const auto& robot_acceleration = target_velocity_change * (acceleration * delta_time / length);
            robot->state.velocity += robot_acceleration;
            const double coef = number_of_microticks > 1 ? (1 - (number_of_microticks + 1) / 2. / number_of_microticks) : 0.;
            robot->state.position -= robot_acceleration * (coef * delta_time);
          } else {
            if (robot->state.touch_surface_id == 1) {
              any_triggers_fired = true;
              //trigger_fired_causes.push_back("reached max speed " + std::to_string(robot->id) + std::to_string(length));
            }
            robot->state.velocity += target_velocity_change;
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
        collideEntitiesStatic(initial_static_robots[i], initial_static_robots[j], false);
      }
    }

    Point collision_normal;
    int touch_surface_id;
    for (int i = 0; i < initial_static_robots_size; i++) {
      auto& robot = initial_static_robots[i];
      collideEntitiesStatic(robot, ball, true);
      if (!collideWithArenaStatic(robot, collision_normal, touch_surface_id)) {
        if (robot->state.touch) {
          any_triggers_fired = true;
          //trigger_fired_causes.push_back("robot->state.touch become false " + std::to_string(robot->id));
        }
        robot->state.touch = false;
      } else {
        if (!robot->state.touch || robot->state.touch_surface_id != touch_surface_id) {
          //if (!robot->state.touch) {
          //trigger_fired_causes.push_back("robot->state.touch become true " + std::to_string(robot->id));
          //} else {
          //trigger_fired_causes.push_back("robot touch_surface_id " + std::to_string(robot->id) + " " + std::to_string(robot->state.touch_surface_id) + " " + std::to_string(touch_surface_id));
          //}
          //if (robot->is_teammate && touch_surface_id != 1) {
          //  robot->collide_with_ball_in_air = true;
          //}
          any_triggers_fired = true;
        }
        robot->state.touch_surface_id = touch_surface_id;
        robot->state.touch = true;
        robot->state.touch_normal = collision_normal;
      }
    }
    if (!collideWithArenaStatic(ball, collision_normal, touch_surface_id)) {
      if (ball->state.touch) {
        if (ball->state.touch_surface_id != 1 || ball->state.velocity.y > C::ball_antiflap) {
          //trigger_fired_causes.push_back("ball->state.touch become false");
          any_triggers_fired = true;
          ball->state.touch = false;
        }
      }
    } else {
      if (!ball->state.touch || ball->state.touch_surface_id != touch_surface_id) {
        //if (!ball->state.touch) {
        //  trigger_fired_causes.push_back("ball->state.touch become true");
        //} else {
        //  trigger_fired_causes.push_back("ball touch_surface_id " + std::to_string(ball->state.touch_surface_id) + " " + std::to_string(touch_surface_id));
        //}
        any_triggers_fired = true;
      }
      ball->state.touch_surface_id = touch_surface_id;
      ball->state.touch = true;
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


  double getScoreFighter(const int tick_number, bool viz) {
    if (goal_to_me) {
      return tick_number == goal_tick ? -1e9 : 0;
    } else if (goal_to_enemy) {
      return tick_number == goal_tick ? 1e9 : 0;
    }
    if (!ball->is_dynamic) {
      ball->fromState(tick_number);
    }
    double score = 0;
    score += -0.01 * (main_robot->state.position - ball->state.position).length();
    if (!main_robot->state.touch) {
      score -= 20;
    }
    double closest_enemy_dist = 1e9;
    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& robot = dynamic_robots[i];
      if (!robot->is_teammate) {
        closest_enemy_dist = std::min(closest_enemy_dist, (ball->state.position - robot->state.position).length());
      }
    }
    for (int i = 0; i < static_robots_size; ++i) {
      auto& robot = static_robots[i];
      if (!robot->is_teammate) {
        robot->fromState(tick_number);
        closest_enemy_dist = std::min(closest_enemy_dist, (ball->state.position - robot->state.position).length());
      }
    }
    score += closest_enemy_dist;
    double dist = (ball->state.position - Point{0, 0, C::rules.arena.depth / 2 + C::rules.arena.goal_depth}).length();
    score += -dist * dist;
    return score;
  }

  double getScoreFighter1() {
    double d1 = (Point{-C::rules.arena.goal_width / 2 + 2, C::rules.arena.goal_height - 2, C::rules.arena.depth / 2 + 2} - ball->state.position).length();
    double d2 = (Point{0, C::rules.arena.goal_height - 2, C::rules.arena.depth / 2 + 2} - ball->state.position).length();
    double d3 = (Point{C::rules.arena.goal_width / 2 - 2, C::rules.arena.goal_height - 2, C::rules.arena.depth / 2 + 2} - ball->state.position).length();
    return C::rules.arena.depth + C::rules.arena.width + C::rules.arena.height - std::min(d1, std::min(d2, d3));
  }

  double getScoreDefender(const int tick_number) {
    if (goal_to_me) {
      return tick_number == goal_tick ? -1e9 : 0;
    } else if (goal_to_enemy) {
      return tick_number == goal_tick ? 1e9 : 0;
    }
    if (!ball->is_dynamic) {
      ball->fromState(tick_number);
    }
    double score = 0;
    double x = 0;
    score += -0.1 * (main_robot->state.position - Point{x, 0, -C::rules.arena.depth / 2 - 2}).length();
    if (!main_robot->state.touch) {
      score -= 0.1;
    }
    double dist = (ball->state.position - Point{0, 0, -C::rules.arena.depth / 2 - C::rules.arena.goal_depth}).length();
    dist = std::min(dist, C::rules.arena.depth / 2 + C::rules.arena.goal_depth);
    score += dist;
    return score;
  }

};

#endif //CODEBALL_SMARTSIMULATOR_H
