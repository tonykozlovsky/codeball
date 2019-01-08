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

  // maybe we can have 4x-5x performance boost, and more when 3x3
  SmartSimulator(const int main_robot_id, const std::vector<model::Robot>& _robots, const model::Ball& _ball) {

    initial_static_entities[initial_dynamic_entities_size].fromBall(_ball);
    ball = &initial_static_entities[initial_dynamic_entities_size++];
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
      tick_static(i);
    }

    // for (auto e : initial_static_entities) {
    //   for (int i = 1; i < e->states.size(); ++i) {
    //     P::drawLine(e->states[i - 1].position, e->states[i].position, 0xAA0000);
    //   }
    // }

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

  void tick_static(const int tick_number) {
    for (int i = 0; i < initial_static_entities_size; ++i) {
      auto& e = initial_static_entities[i];
      e.saveState(tick_number);
    }
    update_static(1. / C::rules.TICKS_PER_SECOND);
  }

  bool collide_entities_static(Entity* a, Entity* b) {
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

  bool collide_with_arena_static(Entity* e, Point& result) {
    const Dan& dan = Dan::dan_to_arena(e->state.position, e->state.radius);
    const double distance = dan.distance;
    const Point& normal = dan.normal;
    if (e->state.radius > distance) {
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

  void move_static(Entity* e, const double delta_time) {
    e->state.velocity = e->state.velocity.clamp(C::rules.MAX_ENTITY_SPEED);
    e->state.position += e->state.velocity * delta_time;
    e->state.position.y -= C::rules.GRAVITY * delta_time * delta_time / 2;
    e->state.velocity.y -= C::rules.GRAVITY * delta_time;
  }

  void update_static(const double delta_time) {
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
            robot->state.velocity += target_velocity_change * (acceleration * delta_time / length);
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
        collide_entities_static(initial_static_robots[i], initial_static_robots[j]);
      }
    }

    Point collision_normal;
    for (int i = 0; i < initial_static_robots_size; i++) {
      auto& robot = initial_static_robots[i];
      collide_entities_static(robot, ball);
      if (!collide_with_arena_static(robot, collision_normal)) {
        robot->state.touch = false;
      } else {
        robot->state.touch = true;
        robot->state.touch_normal = collision_normal;
      }
    }

    collide_with_arena_static(ball, collision_normal);

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
    for (int i = 0, j = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      if (e->want_to_become_dynamic && e->want_to_become_dynamic_on_tick == tick_number) {
        e->fromState(tick_number);
        e->is_dynamic = true;
        dynamic_entities[dynamic_entities_size++] = e;
        if (e != ball) {
          dynamic_robots[dynamic_entities_size++] = e;
        }
      } else {
        static_entities[j++] = e;
        if (e != ball) {
          static_robots[static_robots_size++] = e;
        }
      }
    }
  }

  void tick_dynamic(const int tick_number) {
    // wantedStaticGoToDynamic(tick_number);
    //H::t[0].start();
    for (int i = 0; i < static_entities_size; ++i) {
      auto& e = static_entities[i];
      e->fromState(tick_number + 1);
    }
    //H::t[0].cur(true);
    /* for (auto& e : dynamic_entities) {
      e->savePrevState();
    }*/
    if (update_dynamic(1. / C::rules.TICKS_PER_SECOND, tick_number)) {
      // P::logn("tick: ", tick_number);
      // P::logn("d: ", dynamic_entities.size());
      // P::logn("s: ", static_entities.size());
      /*for (auto& e : dynamic_entities) {
        e->fromPrevState();
      }
      wantedStaticGoToDynamic(tick_number);
      // P::logn("d: ", dynamic_entities.size());
      // P::logn("s: ", static_entities.size());
      for (auto& e : dynamic_entities) {
        e->savePrevState();
      }
      update_dynamic(1. / C::rules.TICKS_PER_SECOND, tick_number);*/
      // P::logn("d: ", dynamic_entities.size());
      // P::logn("s: ", static_entities.size());
    }

    // for (auto& e : dynamic_entities) {
    //   P::drawLine(e->prev_state.position, e->state.position);
    // }
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
    const Point& delta_position = b->state.position - a->state.position;
    const double distance_sq = delta_position.length_sq();
    return (a->state.radius + b->state.radius) * (a->state.radius + b->state.radius) > distance_sq;
  }

  bool collide_with_arena_dynamic(Entity* e, Point& result) {
    const Dan& dan = Dan::dan_to_arena(e->state.position, e->state.radius);
    const double distance = dan.distance;
    const Point& normal = dan.normal;
    if (e->state.radius > distance) {
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

    //H::t[7].start();
    for (int i = 0; i < dynamic_robots_size; ++i) {
      auto& robot = dynamic_robots[i];
      if (robot->state.touch) {
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
    //H::t[7].cur(true);

    //H::t[1].start();
    if (ball->is_dynamic) {
      move_dynamic(ball, delta_time);
    }
    //H::t[1].cur(true);
    //H::t[2].start();
    for (int i = 0; i < dynamic_robots_size; i++) {
      for (int j = 0; j < i; j++) {
        collide_entities_dynamic(dynamic_robots[i], dynamic_robots[j]);
      }
    }

    //H::t[2].cur(true);
    //H::t[3].start();
    for (int i = 0; i < static_robots_size; i++) {
      for (int j = 0; j < dynamic_robots_size; j++) {
        if (collide_entities_check(static_robots[i], dynamic_robots[j])) {
          static_robots[i]->wantToBecomeDynamic(tick_number);
          has_collision_with_static = true;
        }
      }
    }
    //H::t[3].cur(true);

    //H::t[4].start();
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
    //H::t[4].cur(true);

    //H::t[5].start();
    if (ball->is_dynamic) {
      for (int i = 0; i < static_robots_size; ++i) {
        auto& robot = static_robots[i];
        if (collide_entities_check(robot, ball)) {
          robot->wantToBecomeDynamic(tick_number);
          has_collision_with_static = true;
        }
      }
    }
    //H::t[5].cur(true);
    if (has_collision_with_static) {
      return true;
    }

    //H::t[6].start();
    if (ball->is_dynamic) {
      collide_with_arena_dynamic(ball, collision_normal);
    }
    //H::t[6].cur(true);

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
