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
  double score;
  bool collide_with_ball;
  double speed1, speed2;
  std::vector<Point> robot_trace;
  std::vector<Point> ball_trace;
  int additional_jump;
  Plan() {
    additional_jump  = -1;
    collide_with_ball = false;
    angle1 = C::rand_double(0, 2 * M_PI);
    cangle1 = cos(angle1);
    sangle1 = sin(angle1);
    angle2 = C::rand_double(0, 2 * M_PI);
    cangle2 = cos(angle2);
    sangle2 = sin(angle2);
    time_change = C::rand_int(0, C::MAX_SIMULATION_DEPTH);
    time_jump = C::rand_int(0, C::MAX_SIMULATION_DEPTH);

    speed1 = speed2 = C::rules.ROBOT_MAX_GROUND_SPEED;

    if (C::rand_double(0, 1) < 0.01) {
      speed1 = 0;
    }
    if (C::rand_double(0, 1) < 0.01) {
      speed2 = 0;
    }

    score = -1e18;
  }

  static constexpr double angle_mutation = M_PI / 50;
  static constexpr int time_mutation = 1;

  void mutate() {
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
    time_change = time_change + C::rand_int(-time_mutation, time_mutation);
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
  }

  MyAction toMyAction(int simulation_tick, bool check_jump = false) {
    double jump_speed = ((!check_jump || collide_with_ball) && (simulation_tick == time_jump || simulation_tick == additional_jump)) ? C::rules.ROBOT_MAX_JUMP_SPEED : 0;
    if (simulation_tick < time_change) {
      return MyAction{{
          speed1 * cangle1,
          0,
          speed1 * sangle1},
          jump_speed};
    } else {
      return MyAction{{
          speed2 * cangle2,
                0,
          speed2 * sangle2},
                jump_speed};
    }
  }
  bool operator<(const Plan& other) const {
    return score < other.score;
  }
};

#endif //CODEBALL_PLAN_H
