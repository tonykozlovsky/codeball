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

  bool nitro_as_velocity, nitro_up;

  Plan(int configuration,
       const int simulation_depth,
       const double initial_vx = 0,
       const double initial_vz = 0,
       const double crossing_x = 0,
       const double crossing_z = 0,
       const Point& nitro_acceleration = {0, 0, 0}) : configuration(configuration) {
    unique_id = C::unique_plan_id++;
    parent_id = unique_id;

    was_jumping = false;
    was_on_ground_after_jumping = false;
    collide_with_entity_before_on_ground_after_jumping = false;
    oncoming_jump = C::NEVER;
    nitro_as_velocity = false;
    nitro_up = false;

    if (configuration == 1) { // me 2 vec
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

      max_jump_speed = C::rand_int(0, 1) * 15;

      time_nitro_on = C::rand_int(0, simulation_depth);
      time_nitro_off = C::rand_int(0, simulation_depth);

    } else if (configuration == 11) {
      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      y1 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
      cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));

      time_change = C::NEVER;
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

    } else if (configuration == 12) {
      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      y1 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
      cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));

      time_change = C::NEVER;
      time_jump = C::rand_int(0, simulation_depth);

      speed1 = 1;

      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;

      max_jump_speed = C::rand_int(0, 15);

      time_nitro_on = 0;
      time_nitro_off = C::NEVER;
      nitro_as_velocity = true;
    } else if (configuration == 13) {
      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      y1 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
      cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));

      time_change = C::NEVER;
      time_jump = C::rand_int(0, simulation_depth);

      speed1 = 1;

      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;

      max_jump_speed = C::rand_int(0, 15);

      time_nitro_on = 0;
      time_nitro_off = C::NEVER;
      nitro_up = true;
    } else

    if (configuration == 2) { // smart enemy

      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      y1 = C::rand_double(-C::rules.MAX_ENTITY_SPEED, C::rules.MAX_ENTITY_SPEED);
      cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));

      time_change = C::NEVER;
      time_jump = C::rand_int(0, simulation_depth);
      //speed1 = C::rand_double(0, 1);
      speed1 = 1.;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
      max_jump_speed = 15;
      //time_nitro_on = C::rand_int(0, C::ENEMY_LIVE_TICKS);
      //time_nitro_off = C::rand_int(0, C::ENEMY_LIVE_TICKS);
      time_nitro_on = C::NEVER;
      time_nitro_off = C::NEVER;
    } else
      /*if (configuration == 3) { // dont need
      angle1 = 0;
      cangle1 = 0;
      sangle1 = 0;
      time_change = C::NEVER;
      time_jump = C::NEVER;
      speed1 = 0;
      max_speed = C::rules.ROBOT_MAX_GROUND_SPEED;
      max_jump_speed = 0; // todo keep in mind !
      time_nitro_on = C::NEVER;
      time_nitro_off = C::NEVER;
    } else */
      if (configuration == 4) { // last action
      angle1 = atan2(initial_vz, initial_vx);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      time_change = C::NEVER;
      time_jump = C::NEVER;
      speed1 = 1;
      max_speed = Point2d{initial_vx, initial_vz}.length();
      max_jump_speed = 15;  // todo keep in mind !
      // todo last action nitro !
      time_nitro_on = C::NEVER;
      time_nitro_off = C::NEVER;
    } else if (configuration == 26) {
        angle1 = atan2(nitro_acceleration.z, nitro_acceleration.x);
        cangle1 = cos(angle1);
        sangle1 = sin(angle1);
        y1 = nitro_acceleration.y;
        cos_lat1 = cos(asin(y1 / 100.));
        time_change = C::NEVER;
        time_jump = C::NEVER;
        speed1 = 1;
        max_speed = 100.;
        max_jump_speed = 15;  // todo keep in mind !
        // todo last action nitro !
        time_nitro_on = 0;
        time_nitro_off = simulation_depth;
      }
      /*else
      if (configuration == 5) {
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
    }*/

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
    } else if (configuration == 11) {
      angle1 += C::rand_double(-angle_mutation, angle_mutation);
      if (angle1 > 2 * M_PI) {
        angle1 -= 2 * M_PI;
      }
      if (angle1 < 0) {
        angle1 += 2 * M_PI;
      }
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);

      y1 += C::rand_double(-z_mutation, z_mutation);
      if (y1 > C::rules.MAX_ENTITY_SPEED) {
        y1 = C::rules.MAX_ENTITY_SPEED;
      } else if (y1 < -C::rules.MAX_ENTITY_SPEED) {
        y1 = -C::rules.MAX_ENTITY_SPEED;
      }
      cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));


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
    } else

      if (configuration == 2) { // smart enemy
      angle1 += C::rand_double(-angle_mutation, angle_mutation);
      if (angle1 > 2 * M_PI) {
        angle1 -= 2 * M_PI;
      }
      if (angle1 < 0) {
        angle1 += 2 * M_PI;
      }
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);

      /*y1 += C::rand_double(-z_mutation, z_mutation);
      if (y1 > C::rules.MAX_ENTITY_SPEED) {
        y1 = C::rules.MAX_ENTITY_SPEED;
      } else if (y1 < -C::rules.MAX_ENTITY_SPEED) {
        y1 = -C::rules.MAX_ENTITY_SPEED;
      }
      cos_lat1 = cos(asin(y1 / C::rules.MAX_ENTITY_SPEED));
      */
      if (time_jump != C::NEVER) {
        time_jump += C::rand_int(-time_mutation, time_mutation);
        if (time_jump < 0) {
          time_jump = 0;
        }
        if (time_jump > simulation_depth) {
          time_jump = simulation_depth;
        }
      }

      /*speed1 += C::rand_double(-speed_mutation, speed_mutation); // todo check maybe need
      if (speed1 > 1) {
        speed1 = 1;
      }
      if (speed1 < 0) {
        speed1 = 0;
      }*/

      /*max_jump_speed += C::rand_int(-jump_mutation, jump_mutation);
      if (max_jump_speed < 0) {
        max_jump_speed = 0;
      }
      if (max_jump_speed > 15) {
        max_jump_speed = 15;
      }*/

      /*if (time_nitro_on != C::NEVER) {
        time_nitro_on += C::rand_int(-nitro_mutation, nitro_mutation);
        if (time_nitro_on < 0) {
          time_nitro_on = 0;
        }
        if (time_nitro_on > C::ENEMY_LIVE_TICKS) {
          time_nitro_on = C::ENEMY_LIVE_TICKS;
        }
      }
      if (time_nitro_off != C::NEVER) {
        time_nitro_off += C::rand_int(-nitro_mutation, nitro_mutation);
        if (time_nitro_off < 0) {
          time_nitro_off = 0;
        }
        if (time_nitro_off > C::ENEMY_LIVE_TICKS) {
          time_nitro_off = C::ENEMY_LIVE_TICKS;
        }
      }*/

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
    const double& jump_speed = simulation ?  (simulation_tick == time_jump ? max_jump_speed : 0) : (simulation_tick == oncoming_jump ? oncoming_jump_speed : 0);
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
