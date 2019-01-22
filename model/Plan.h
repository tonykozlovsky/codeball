#ifndef CODEBALL_PLAN_H
#define CODEBALL_PLAN_H

struct Plan {
  double angle1;
  double sangle1;
  double cangle1;
  double angle2;
  double sangle2;
  double cangle2;
  double max_jump_speed;
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
  bool was_in_air_after_jumping;
  bool was_on_ground_after_in_air_after_jumping;
  bool collide_with_ball_before_on_ground_after_jumping;

  Plan() : Plan(3, 0) {}

  Plan(int configuration, const int simulation_depth, const double initial_vx = 0, const double initial_vz = 0) {
    unique_id = C::unique_plan_id++;
    parent_id = unique_id;

    was_jumping = false;
    was_in_air_after_jumping = false;
    was_on_ground_after_in_air_after_jumping = false;
    collide_with_ball_before_on_ground_after_jumping = false;
    oncoming_jump  = -1;

    if (configuration == 1) {
      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      angle2 = C::rand_double(0, 2 * M_PI);
      cangle2 = cos(angle2);
      sangle2 = sin(angle2);
      time_change = C::rand_int(0, simulation_depth);
      time_jump = C::rand_int(0, simulation_depth);

      speed1 = C::rules.ROBOT_MAX_GROUND_SPEED;//C::rand_double(0, C::rules.ROBOT_MAX_GROUND_SPEED);
      speed2 = C::rules.ROBOT_MAX_GROUND_SPEED;//C::rand_double(0, C::rules.ROBOT_MAX_GROUND_SPEED);
      if (C::rand_double(0, 1) < 0.01) {
        speed1 = 0;
      }
      if (C::rand_double(0, 1) < 0.01) {
        speed2 = 0;
      }
      max_jump_speed = C::rand_int(0, 15);
    } else if (configuration == 2) { // smart enemy
      angle1 = C::rand_double(0, 2 * M_PI);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      time_change = simulation_depth;
      time_jump = C::rand_int(0, simulation_depth);
      speed1 = C::rules.ROBOT_MAX_GROUND_SPEED;
      max_jump_speed = 15;
    } else if (configuration == 3) { // sptupid enemy
      angle1 = 0;
      cangle1 = 0;
      sangle1 = 0;
      time_change = simulation_depth;
      time_jump = simulation_depth;
      speed1 = 0;
      max_jump_speed = 0; // todo keep in mind
    } else if (configuration == 4) { // last action
      angle1 = atan2(initial_vz, initial_vx);
      cangle1 = cos(angle1);
      sangle1 = sin(angle1);
      time_change = simulation_depth;
      time_jump = simulation_depth;
      speed1 = Point2d{initial_vx, initial_vz}.length();
      max_jump_speed = 15;  // todo keep in mind
    }

    score.minimal();
  }

  static constexpr double angle_mutation = M_PI / 100;
  static constexpr double speed_mutation = 1;
  static constexpr int jump_mutation = 1;
  static constexpr int time_mutation = 1;

  void mutate(int configuration, const int simulation_depth) {
    unique_id = C::unique_plan_id++;

    was_jumping = false;
    was_in_air_after_jumping = false;
    was_on_ground_after_in_air_after_jumping = false;
    collide_with_ball_before_on_ground_after_jumping = false;
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

      /*speed1 += C::rand_double(-speed_mutation, speed_mutation);
      if (speed1 > C::rules.ROBOT_MAX_GROUND_SPEED) {
        speed1 = C::rules.ROBOT_MAX_GROUND_SPEED;
      }
      if (speed1 < 0) {
        speed1 = 0;
      }

      speed2 += C::rand_double(-speed_mutation, speed_mutation);
      if (speed2 > C::rules.ROBOT_MAX_GROUND_SPEED) {
        speed2 = C::rules.ROBOT_MAX_GROUND_SPEED;
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

    }

    score.minimal();
  }

  MyAction toMyAction(int simulation_tick, bool simulation) {
    double jump_speed;
    if (simulation) {
      jump_speed = simulation_tick == time_jump ? max_jump_speed : 0;
    } else {
      jump_speed = simulation_tick == oncoming_jump ? oncoming_jump_speed : 0;
    }
    if (simulation_tick < time_change) {
      return MyAction{{
          speed1 * cangle1,
          0,
          speed1 * sangle1},
          jump_speed,
          max_jump_speed,
          false};
    } else {
      return MyAction{{
          speed2 * cangle2,
                0,
          speed2 * sangle2},
          jump_speed,
          max_jump_speed, false};
    }
  }
  bool operator<(const Plan& other) const {
    return score < other.score;
  }
};

#endif //CODEBALL_PLAN_H
