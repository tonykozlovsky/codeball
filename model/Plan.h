#ifndef CODEBALL_PLAN_H
#define CODEBALL_PLAN_H

struct Plan {
  double angle1;
  double sangle1;
  double cangle1;
  double z1;
  double cos_lat1;
  double angle2;
  double sangle2;
  double cangle2;
  double z2;
  double cos_lat2;
  double max_jump_speed;
  double max_speed;
  int time_nitro_on;
  int time_nitro_off;
  bool use_nitro;
  int time_change;
  int time_jump;
  int plans_config;
  int unique_id;
  int parent_id;
  struct Score {
    double sum_score;
    double fighter_min_dist_to_ball;
    double fighter_min_dist_to_goal;
    double fighter_last_dist_to_goal;
    double defender_min_dist_to_ball;
    double defender_min_dist_from_goal;
    double defender_last_dist_from_goal;

    bool operator < (const Score& other) const {
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
  double speed1, speed2;

  int oncoming_jump;

  double oncoming_jump_speed;

  bool was_jumping;
  bool was_on_ground_after_jumping;
  bool collide_with_entity_before_on_ground_after_jumping;

  Point2d crossing;
  Point2d crossing2;

  int configuration;

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
    oncoming_jump  = -1;

    if (configuration == 1) {
      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      z1 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
      cos_lat1 = cos(asin(z1 / C::rules.MAX_ENTITY_SPEED));
      angle2 = C::rand_double(0, 2 * M_PI);
      cangle2 = cos(angle2);
      sangle2 = sin(angle2);
      z2 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
      cos_lat2 = cos(asin(z2 / C::rules.MAX_ENTITY_SPEED));

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
      use_nitro = time_nitro_off > time_nitro_on;
    } else if (configuration == 2) { // smart enemy
      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      time_change = simulation_depth;
      time_jump = C::rand_int(0, simulation_depth);
      speed1 = 1.;

      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;

      max_jump_speed = 15;
      use_nitro = false;
    } else if (configuration == 3) { // sptupid enemy
      angle1 = 0;
      cangle1 = 0;
      sangle1 = 0;
      time_change = simulation_depth;
      time_jump = simulation_depth;
      speed1 = 0;

      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;

      max_jump_speed = 0; // todo keep in mind
      use_nitro = false;
    } else if (configuration == 4) { // last action
      angle1 = atan2(initial_vz, initial_vx);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      time_change = simulation_depth;
      time_jump = simulation_depth;
      speed1 = 1;

      max_speed = Point2d{initial_vx, initial_vz}.length();

      max_jump_speed = 15;  // todo keep in mind
      use_nitro = false; // todo last action nitro
    } else if (configuration == 5) {
      time_change = simulation_depth;
      time_jump = simulation_depth;
      speed1 = 1;
      max_speed = Point2d{initial_vx, initial_vz}.length();
      crossing = Point2d{crossing_x, crossing_z};
      max_jump_speed = 15;  // todo keep in mind
      use_nitro = false; // todo last action nitro
    } else if (configuration == 6) {

      crossing = {C::rand_double(-30, 30), C::rand_double(-50, 50)};

      time_change = simulation_depth;
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
      use_nitro = false;
    } else if (configuration == 7) {

      crossing = {C::rand_double(-30, 30), C::rand_double(-50, 50)};

      crossing2 = {C::rand_double(-30, 30), C::rand_double(-50, 50)};

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
      use_nitro = time_nitro_off > time_nitro_on;
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
    oncoming_jump  = -1;

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

      z1 += C::rand_double(-z_mutation, z_mutation);
      if (z1 > C::rules.MAX_ENTITY_SPEED) {
        z1 = C::rules.MAX_ENTITY_SPEED;
      } else if (z1 < -C::rules.MAX_ENTITY_SPEED) {
        z1 = -C::rules.MAX_ENTITY_SPEED;
      }
      cos_lat1 = cos(asin(z1 / C::rules.MAX_ENTITY_SPEED));

      z2 += C::rand_double(-z_mutation, z_mutation);
      if (z2 > C::rules.MAX_ENTITY_SPEED) {
        z2 = C::rules.MAX_ENTITY_SPEED;
      } else if (z2 < -C::rules.MAX_ENTITY_SPEED) {
        z2 = -C::rules.MAX_ENTITY_SPEED;
      }
      cos_lat2 = cos(asin(z2 / C::rules.MAX_ENTITY_SPEED));

      time_change += C::rand_int(-time_mutation, time_mutation);
      if (time_change < 0) {
        time_change = 0;
      }
      if (time_change > simulation_depth) {
        time_change = simulation_depth;
      }
      time_jump += C::rand_int(-time_mutation, time_mutation);
      if (time_jump < 0) {
        time_jump = 0;
      }
      if (time_jump > simulation_depth) {
        time_jump = simulation_depth;
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

      time_nitro_on += C::rand_int(-nitro_mutation, nitro_mutation);
      if (time_nitro_on < 0) {
        time_nitro_on = 0;
      }
      if (time_nitro_on > simulation_depth) {
        time_nitro_on = simulation_depth;
      }
      time_nitro_off += C::rand_int(-nitro_mutation, nitro_mutation);
      if (time_nitro_off < 0) {
        time_nitro_off = 0;
      }
      if (time_nitro_off > simulation_depth) {
        time_nitro_off = simulation_depth;
      }
      use_nitro = time_nitro_off > time_nitro_on;

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

      time_jump += C::rand_int(-time_mutation, time_mutation);
      if (time_jump < 0) {
        time_jump = 0;
      }
      if (time_jump > simulation_depth) {
        time_jump = simulation_depth;
      }

    } else if (configuration == 6) {
      crossing.x += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing.x > 30) {
        crossing.x = 30;
      } else if (crossing.x < -30) {
        crossing.x = -30;
      }
      crossing.y += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing.y > 50) {
        crossing.y = 50;
      } else if (crossing.y < -50) {
        crossing.y = -50;
      }

      time_jump += C::rand_int(-time_mutation, time_mutation);
      if (time_jump < 0) {
        time_jump = 0;
      }
      if (time_jump > simulation_depth) {
        time_jump = simulation_depth;
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
      if (crossing.y > 50) {
        crossing.y = 50;
      } else if (crossing.y < -50) {
        crossing.y = -50;
      }

      crossing2.x += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing2.x > 30) {
        crossing2.x = 30;
      } else if (crossing2.x < -30) {
        crossing2.x = -30;
      }
      crossing2.y += C::rand_double(-crossing_mutation, crossing_mutation);
      if (crossing2.y > 50) {
        crossing2.y = 50;
      } else if (crossing2.y < -50) {
        crossing2.y = -50;
      }


      time_change += C::rand_int(-time_mutation, time_mutation);
      if (time_change < 0) {
        time_change = 0;
      }
      if (time_change > simulation_depth) {
        time_change = simulation_depth;
      }
      time_jump += C::rand_int(-time_mutation, time_mutation);
      if (time_jump < 0) {
        time_jump = 0;
      }
      if (time_jump > simulation_depth) {
        time_jump = simulation_depth;
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

      time_nitro_on += C::rand_int(-nitro_mutation, nitro_mutation);
      if (time_nitro_on < 0) {
        time_nitro_on = 0;
      }
      if (time_nitro_on > simulation_depth) {
        time_nitro_on = simulation_depth;
      }
      time_nitro_off += C::rand_int(-nitro_mutation, nitro_mutation);
      if (time_nitro_off < 0) {
        time_nitro_off = 0;
      }
      if (time_nitro_off > simulation_depth) {
        time_nitro_off = simulation_depth;
      }
      use_nitro = time_nitro_off > time_nitro_on;

    }

    score.minimal();
  }

  MyAction toMyAction(int simulation_tick, bool simulation, bool can_use_nitro, const Point& position) {
    double jump_speed;
    if (simulation) {
      jump_speed = simulation_tick == time_jump ? max_jump_speed : 0;
    } else {
      jump_speed = simulation_tick == oncoming_jump ? oncoming_jump_speed : 0;
    }
    bool now_use_nitro = (use_nitro && simulation_tick >= time_nitro_on && simulation_tick < time_nitro_off);
    Point velocity;
    if (now_use_nitro && can_use_nitro) {
      if (simulation_tick < time_change) {
        velocity.x = speed1 * C::rules.MAX_ENTITY_SPEED * cos_lat1 * cangle1;
        velocity.y = z1;
        velocity.z = speed1 * C::rules.MAX_ENTITY_SPEED * cos_lat1 * sangle1;
      } else {
        velocity.x = speed2 * C::rules.MAX_ENTITY_SPEED * cos_lat2 * cangle2;
        velocity.y = z2;
        velocity.z = speed2 * C::rules.MAX_ENTITY_SPEED * cos_lat2 * sangle2;
      }
    } else {
      if (simulation_tick < time_change) {
        if (configuration != 5 && configuration != 6 && configuration != 7) {
          velocity.x = speed1 * max_speed * cangle1;
          velocity.y = 0;
          velocity.z = speed1 * max_speed * sangle1;
        } else {
          double dx = crossing.x - position.x;
          double dz = crossing.y - position.z;
          velocity = Point{dx, 0, dz}.normalize() * speed1 * max_speed;
        }
      } else {
        if (configuration != 7) {
          velocity.x = speed2 * max_speed * cangle2;
          velocity.y = 0;
          velocity.z = speed2 * max_speed * sangle2;
        } else {
          double dx = crossing2.x - position.x;
          double dz = crossing2.y - position.z;
          velocity = Point{dx, 0, dz}.normalize() * speed2 * max_speed;
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

#endif //CODEBALL_PLAN_H
