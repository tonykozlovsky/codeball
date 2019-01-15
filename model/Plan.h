#ifndef CODEBALL_PLAN_H
#define CODEBALL_PLAN_H

struct Plan {
  double angle1;
  double sangle1;
  double cangle1;
  double angle2;
  double sangle2;
  double cangle2;
  int time_change;
  int time_jump;
  struct Score {
    double sum_score;
    double fighter_min_dist_to_ball;
    double fighter_min_dist_to_goal;
    double fighter_last_dist_to_goal;
    double fighter_closest_enemy_dist;
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
      + fighter_closest_enemy_dist
      + defender_min_dist_from_goal
      + defender_last_dist_from_goal;
    }

    void minimal() {
      sum_score = -1e18;
      fighter_min_dist_to_ball = 1e9;
      fighter_min_dist_to_goal = 1e9;
      fighter_last_dist_to_goal = 1e9;
      fighter_closest_enemy_dist = 1e9;
      defender_min_dist_to_ball = 1e9;
      defender_min_dist_from_goal = 1e9;
      defender_last_dist_from_goal = 1e9;
    }

    void start_fighter() {
      sum_score = 0;
      fighter_min_dist_to_ball = 1e9;
      fighter_min_dist_to_goal = 1e9;
      fighter_last_dist_to_goal = 1e9;
      fighter_closest_enemy_dist = 1e9;
      defender_min_dist_to_ball = 0;
      defender_min_dist_from_goal = 0;
      defender_last_dist_from_goal = 0;
    }

    void start_defender() {
      sum_score = 0;
      fighter_min_dist_to_ball = 0;
      fighter_min_dist_to_goal = 0;
      fighter_last_dist_to_goal = 0;
      fighter_closest_enemy_dist = 0;
      defender_min_dist_to_ball = 1e9;
      defender_min_dist_from_goal = 1e9;
      defender_last_dist_from_goal = 0;
    }

  } score;
  double speed1, speed2;

  int oncoming_jump;

  bool was_jumping;
  bool was_in_air_after_jumping;
  bool was_on_ground_after_in_air_after_jumping;
  bool collide_with_ball_before_on_ground_after_jumping;
  Plan() {
    was_jumping = false;
    was_in_air_after_jumping = false;
    was_on_ground_after_in_air_after_jumping = false;
    collide_with_ball_before_on_ground_after_jumping = false;
    oncoming_jump  = -1;

    angle1 = C::rand_double(0, 2 * M_PI);
    cangle1 = cos(angle1);
    sangle1 = sin(angle1);
    angle2 = C::rand_double(0, 2 * M_PI);
    cangle2 = cos(angle2);
    sangle2 = sin(angle2);
    time_change = C::rand_int(0, C::MAX_SIMULATION_DEPTH);
    time_jump = C::rand_int(0, C::MAX_SIMULATION_DEPTH);

    speed1 = speed2 = C::rand_double(0, C::rules.ROBOT_MAX_GROUND_SPEED);

    //if (C::rand_double(0, 1) < 0.01) {
    //  speed1 = 0;
    //}
    //if (C::rand_double(0, 1) < 0.01) {
    //  speed2 = 0;
    //}

    score.minimal();
  }

  static constexpr double angle_mutation = M_PI / 100;
  static constexpr double speed_mutation = 2;
  static constexpr int time_mutation = 1;

  void mutate() {

    was_jumping = false;
    was_in_air_after_jumping = false;
    was_on_ground_after_in_air_after_jumping = false;
    collide_with_ball_before_on_ground_after_jumping = false;
    oncoming_jump  = -1;

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
    if (time_change > C::MAX_SIMULATION_DEPTH) {
      time_change = C::MAX_SIMULATION_DEPTH;
    }
    time_jump = time_jump + C::rand_int(-time_mutation, time_mutation);
    if (time_jump < 0) {
      time_jump = 0;
    }
    if (time_jump > C::MAX_SIMULATION_DEPTH) {
      time_jump = C::MAX_SIMULATION_DEPTH;
    }

    speed1 += C::rand_double(-speed_mutation, speed_mutation);
    if (speed1 < 0) {
      speed1 = 0;
    }
    if (speed1 > C::rules.ROBOT_MAX_GROUND_SPEED) {
      speed1 = C::rules.ROBOT_MAX_GROUND_SPEED;
    }

    speed2 += C::rand_double(-speed_mutation, speed_mutation);
    if (speed2 < 0) {
      speed2 = 0;
    }
    if (speed2 > C::rules.ROBOT_MAX_GROUND_SPEED) {
      speed2 = C::rules.ROBOT_MAX_GROUND_SPEED;
    }

    score.minimal();
  }

  MyAction toMyAction(int simulation_tick, bool simulation) {
    double jump_speed;
    if (simulation) {
      jump_speed = simulation_tick == time_jump ? C::rules.ROBOT_MAX_JUMP_SPEED : 0;
    } else {
      jump_speed = simulation_tick == oncoming_jump ? C::rules.ROBOT_MAX_JUMP_SPEED : 0;
    }
    if (simulation_tick < time_change) {
      return MyAction{{
          speed1 * cangle1,
          0,
          speed1 * sangle1},
          jump_speed, false};
    } else {
      return MyAction{{
          speed2 * cangle2,
                0,
          speed2 * sangle2},
                jump_speed, false};
    }
  }
  bool operator<(const Plan& other) const {
    return score < other.score;
  }
};

#endif //CODEBALL_PLAN_H
