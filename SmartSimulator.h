#ifndef CODEBALL_SMARTSIMULATOR_H
#define CODEBALL_SMARTSIMULATOR_H

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
      tickWithJumpsStatic(i, false);
    }

    for (int i = 0; i < initial_static_entities_size; ++i) {
      auto& e = initial_static_entities[i];
       for (int j = 1; j < 100; ++j) {
         P::drawLine(e.states[j - 1].position, e.states[j].position, 0xAA0000);
       }
    }

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

  void tickMicroticksStatic(int number_of_microticks) {
    any_triggers_fired = false;
    if (number_of_microticks == 0) {
      return;
    }
    updateStatic((double) number_of_microticks / C::rules.TICKS_PER_SECOND / C::rules.MICROTICKS_PER_TICK, number_of_microticks);
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
      for (int i = 0; i < initial_static_entities_size; ++i) {
        auto& e = initial_static_entities[i];
        if (e.collide_with_ball_in_air) {
          needs_rollback = true;
          e.action.jump_speed = C::rules.ROBOT_MAX_JUMP_SPEED;
        }
      }
      if (needs_rollback) {
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
    /*if (somebodyJumpThisTickStatic()) {
      tickMicroticksStatic(1);
      tickMicroticksStatic(1);
      remaining_microticks = 98;
    }*/
    /*for (int i = 0; i < initial_static_entities_size; ++i) { // save micro
      auto& e = initial_static_entities[i];
      e.savePrevMicroState();
    }*/
    tickMicroticksStatic(remaining_microticks);
    return;
    if (any_triggers_fired) {
      int l = 0;
      int r = remaining_microticks;
      while (r - l > 1) {
        int mid = (r + l) / 2;
        for (int i = 0; i < initial_static_entities_size; ++i) { // from micro
          auto& e = initial_static_entities[i];
          e.fromPrevMicroState();
        }
        tickMicroticksStatic(mid);
        if (any_triggers_fired) {
          r = mid;
        } else {
          l = mid;
        }
      }
      for (int i = 0; i < initial_static_entities_size; ++i) { // from micro
        auto& e = initial_static_entities[i];
        e.fromPrevMicroState();
      }
      tickMicroticksStatic(l);
      tickMicroticksStatic(1);
      tickMicroticksStatic(remaining_microticks - l - 1);
    }
  }

  bool collide_entities_static(Entity* a, Entity* b, bool check_with_ball) {
    const Point& delta_position = b->state.position - a->state.position;
    const double distance_sq = delta_position.length_sq();
    const double sum_r = a->state.radius + b->state.radius;
    if (check_with_ball && (3.05) * (3.05) > distance_sq) {
      a->collide_with_ball_in_air = true;
    }
    if (sum_r * sum_r > distance_sq) {
      any_triggers_fired = true;
      const double penetration = sum_r - sqrt(distance_sq);
      const double k_a = 1. / (a->mass * ((1 / a->mass) + (1 / b->mass)));
      const double k_b = 1. / (b->mass * ((1 / a->mass) + (1 / b->mass)));
      const Point& normal = delta_position.normalize();
      a->state.position -= normal * (penetration * k_a);
      b->state.position += normal * (penetration * k_b);
      const double delta_velocity = (b->state.velocity - a->state.velocity).dot(normal) - (b->radius_change_speed + a->radius_change_speed);
      if (delta_velocity < 0) {
        const Point& impulse = normal * ((1. + (C::rules.MAX_HIT_E + C::rules.MIN_HIT_E) / 2.) * delta_velocity);
        a->state.velocity += impulse * k_a;
        b->state.velocity -= impulse * k_b;
        return true;
      }
    }
    return false;
  }

  bool collide_with_arena_static(Entity* e, Point& result, int& collision_surface_id) {
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

  void move_static(Entity* e, const double delta_time) {
    e->state.velocity = e->state.velocity.clamp(C::rules.MAX_ENTITY_SPEED);
    e->state.position += e->state.velocity * delta_time;
    e->state.position.y -= C::rules.GRAVITY * delta_time * delta_time / 2;
    e->state.velocity.y -= C::rules.GRAVITY * delta_time;
  }

  void updateStatic(const double delta_time, const int number_of_microticks) {
    for (int i = 0; i < initial_static_robots_size; ++i) {
      auto& robot = initial_static_robots[i];
      if (robot->state.touch) {
        const Point& target_velocity = robot->action.target_velocity - robot->state.touch_normal * robot->state.touch_normal.dot(robot->action.target_velocity);
        const Point& target_velocity_change = target_velocity - robot->state.velocity;
        double length = target_velocity_change.length_sq();
        if (length > 0) {
          const double acceleration = C::rules.ROBOT_ACCELERATION * fmax(0., robot->state.touch_normal.y);
          length = sqrt(length);
          if (acceleration * delta_time < length) {
            const auto& robot_acceleration = target_velocity_change * (acceleration * delta_time / length);
            robot->state.velocity += robot_acceleration;
            const double coef = (1 - (number_of_microticks + 1) / 2. / number_of_microticks);
            robot->state.position -= robot_acceleration * (coef * delta_time);
          } else {
            robot->state.velocity += target_velocity_change;
          }
        }
      }

      move_static(robot, delta_time);

      robot->state.radius = C::rules.ROBOT_MIN_RADIUS + (C::rules.ROBOT_MAX_RADIUS - C::rules.ROBOT_MIN_RADIUS) * robot->action.jump_speed / C::rules.ROBOT_MAX_JUMP_SPEED;
      robot->radius_change_speed = robot->action.jump_speed;
    }

    move_static(ball, delta_time);

    for (int i = 0; i < initial_static_robots_size; i++) {
      for (int j = 0; j < i; j++) {
        collide_entities_static(initial_static_robots[i], initial_static_robots[j], false);
      }
    }

    Point collision_normal;
    int touch_surface_id;
    for (int i = 0; i < initial_static_robots_size; i++) {
      auto& robot = initial_static_robots[i];
      collide_entities_static(robot, ball, true);
      if (!collide_with_arena_static(robot, collision_normal, touch_surface_id)) {
        if (robot->state.touch) {
          any_triggers_fired = true;
        }
        robot->state.touch = false;
      } else {
        if (!robot->state.touch || robot->state.touch_surface_id != touch_surface_id) {
          any_triggers_fired = true;
        }
        robot->state.touch_surface_id = touch_surface_id;
        robot->state.touch = true;
        robot->state.touch_normal = collision_normal;
      }
    }
    if (!collide_with_arena_static(ball, collision_normal, touch_surface_id)) {
      if (ball->state.touch) {
        any_triggers_fired = true;
      }
      ball->state.touch = false;
    } else {
      if (!ball->state.touch || ball->state.touch_surface_id != touch_surface_id) {
        any_triggers_fired = true;
      }
      ball->state.touch_surface_id = touch_surface_id;
      ball->state.touch = true;
    }
  }

  void initIteration() {
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

  void tick_dynamic(const int tick_number) {
    wantedStaticGoToDynamic(tick_number);
    for (int i = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      e->fromState(tick_number + 1);
    }
    for (int i = 0; i < dynamic_entities_size; ++i) {
      auto& e = dynamic_entities[i];
      e->savePrevState();
    }
    bool flag = update_dynamic(1. / C::rules.TICKS_PER_SECOND, tick_number);
    if (flag) {
      for (int i = 0; i < dynamic_entities_size; ++i) {
        auto& e = dynamic_entities[i];
        e->fromPrevState();
      }
      wantedStaticGoToDynamic(tick_number);
      for (int i = 0; i < dynamic_entities_size; ++i) {
        auto& e = dynamic_entities[i];
        e->savePrevState();
      }
      update_dynamic(1. / C::rules.TICKS_PER_SECOND, tick_number);
    }
  }

  bool collide_entities_dynamic(Entity* a, Entity* b) {
    const Point& delta_position = b->state.position - a->state.position;
    const double distance_sq = delta_position.length_sq();
    const double sum_r = a->state.radius + b->state.radius;
    if (sum_r * sum_r > distance_sq) {
      const double penetration = sum_r - sqrt(distance_sq);
      const double k_a = 1. / (a->mass * ((1 / a->mass) + (1 / b->mass)));
      const double k_b = 1. / (b->mass * ((1 / a->mass) + (1 / b->mass)));
      const Point& normal = delta_position.normalize();
      a->state.position -= normal * (penetration * k_a);
      b->state.position += normal * (penetration * k_b);
      const double delta_velocity = (b->state.velocity - a->state.velocity).dot(normal)
          - (b->radius_change_speed + a->radius_change_speed);
      if (delta_velocity < 0) {
        const Point& impulse = normal * ((1. + (C::rules.MAX_HIT_E + C::rules.MIN_HIT_E) / 2.) * delta_velocity);
        a->state.velocity += impulse * k_a;
        b->state.velocity -= impulse * k_b;
        return true;
      }
    }
    return false;
  }

  bool collide_entities_check(Entity* a, Entity* b) {
    const double distance_sq = (b->state.position - a->state.position).length_sq();
    return (a->state.radius + b->state.radius) * (a->state.radius + b->state.radius) > distance_sq;
  }

  bool collide_with_arena_dynamic(Entity* e, Point& result) {
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
        return true;
      }
    }
    return false;
  }

  void move_dynamic(Entity* e, const double delta_time) {
    e->state.velocity = e->state.velocity.clamp(C::rules.MAX_ENTITY_SPEED);
    e->state.position += e->state.velocity * delta_time;
    e->state.position.y -= C::rules.GRAVITY * delta_time * delta_time / 2;
    e->state.velocity.y -= C::rules.GRAVITY * delta_time;
  }

  bool update_dynamic(const double delta_time, const int tick_number) {

    bool has_collision_with_static = false;

    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& robot = dynamic_robots[i];
      if (robot->state.touch) {
        //const Point& target_velocity = robot->action.target_velocity - robot->state.touch_normal * robot->state.touch_normal.dot(robot->action.target_velocity);
        const Point& target_velocity_change = robot->action.target_velocity - robot->state.velocity;
        double length = target_velocity_change.length_sq();
        if (length > 0) {
          const double acceleration = C::rules.ROBOT_ACCELERATION * fmax(0., robot->state.touch_normal.y);
          length = sqrt(length);
          if (acceleration * delta_time < length) {
            robot->state.velocity += target_velocity_change * (acceleration * delta_time / length);
          } else {
            robot->state.velocity += target_velocity_change;
          }
        }
      }

      move_dynamic(robot, delta_time);

      robot->state.radius = C::rules.ROBOT_MIN_RADIUS + (C::rules.ROBOT_MAX_RADIUS - C::rules.ROBOT_MIN_RADIUS) * robot->action.jump_speed / C::rules.ROBOT_MAX_JUMP_SPEED;
      robot->radius_change_speed = robot->action.jump_speed;
    }

    if (ball->is_dynamic) {
      move_dynamic(ball, delta_time);
    }

    for (int i = 0; i < dynamic_robots_size; i++) {
      for (int j = 0; j < i; j++) {
        collide_entities_dynamic(dynamic_robots[i], dynamic_robots[j]);
      }
    }

    for (int i = 0; i < static_robots_size; i++) {
      for (int j = 0; j < dynamic_robots_size; j++) {
        if (collide_entities_check(static_robots[i], dynamic_robots[j])) {
          static_robots[i]->wantToBecomeDynamic(tick_number);
          has_collision_with_static = true;
        }
      }
    }

    Point collision_normal;
    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& robot = dynamic_robots[i];
      if (ball->is_dynamic) {
        collide_entities_dynamic(robot, ball);
      } else {
        if (collide_entities_check(robot, ball)) {
          ball->wantToBecomeDynamic(tick_number);
          has_collision_with_static = true;
        }
      }

      if (has_collision_with_static) {
        continue;
      }

      if (!collide_with_arena_dynamic(robot, collision_normal)) {
        robot->state.touch = false;
      } else {
        robot->state.touch = true;
        robot->state.touch_normal = collision_normal;
      }
    }

    if (ball->is_dynamic) {
      for (int i = 0; i < static_robots_size; ++i) {
        auto& robot = static_robots[i];
        if (collide_entities_check(robot, ball)) {
          robot->wantToBecomeDynamic(tick_number);
          has_collision_with_static = true;
        }
      }
    }
    if (has_collision_with_static) {
      return true;
    }

    if (ball->is_dynamic) {
      collide_with_arena_dynamic(ball, collision_normal);
    }

    return false;
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






};

#endif //CODEBALL_SMARTSIMULATOR_H
