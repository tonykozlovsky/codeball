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
  double speed1, speed2;
  std::vector<Point> robot_trace;
  std::vector<Point> ball_trace;

  int additional_jump;
  bool was_jumping;
  bool was_in_air_after_jumping;
  bool was_on_ground_after_in_air_after_jumping;
  bool collide_with_ball_before_on_ground_after_jumping;
  Plan() {
    was_jumping = false;
    was_in_air_after_jumping = false;
    was_on_ground_after_in_air_after_jumping = false;
    collide_with_ball_before_on_ground_after_jumping = false;
    additional_jump  = -1;

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

    was_jumping = false;
    was_in_air_after_jumping = false;
    was_on_ground_after_in_air_after_jumping = false;
    collide_with_ball_before_on_ground_after_jumping = false;
    additional_jump  = -1;

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

  double solve2(double k, double a, double b, double c, double v) {
    return b * v / sqrt((a * k + c) * (a * k + c) + b * b * (k * k + 1));
  }

  Point solve(const Point& touch_normal, double cs, double sn, double v) {
    Point res;
    if (fabs(sn) > fabs(cs)) {
      const double k = cs / sn;
      res.z = solve2(k, touch_normal.x, touch_normal.y, touch_normal.z, v);
      if (sn < 0) {
        res.z = -res.z;
      }
      res.x = k * res.z;
      const double yq = v * v - res.z * res.z * (k * k + 1);
      res.y = yq > 0 ? sqrt(yq) : 0.;
      if ((touch_normal.x * res.x + touch_normal.z * res.z) * touch_normal.y > 0) {
        res.y = -res.y;
      }
      if (res.dot(touch_normal) > 1e-6) {
        std::cout << res.dot(touch_normal) << std::endl;
        std::cout << sn << " " << cs << " " << k << std::endl;
        std::cout << touch_normal.x << " " << touch_normal.y << " " << touch_normal.z << " " << res.x << " " << res.y << " " << res.z << std::endl;
      }
      return res;
    } else {
      const double k = sn / cs;
      res.x = solve2(k, touch_normal.z, touch_normal.y, touch_normal.x, v);
      if (cs < 0) {
        res.x = -res.x;
      }
      res.z = k * res.x;
      const double yq = v * v - res.x * res.x * (k * k + 1);
      res.y = yq > 0 ? sqrt(yq) : 0.;
      if ((touch_normal.x * res.x + touch_normal.z * res.z) * touch_normal.y > 0) {
        res.y = -res.y;
      }
      if (res.dot(touch_normal) > 1e-6) {
        std::cout << res.dot(touch_normal) << std::endl;
        std::cout << sn << " " << cs << " " << k << std::endl;
        std::cout << touch_normal.x << " " << touch_normal.y << " " << touch_normal.z << " " << res.x << " " << res.y << " " << res.z << std::endl;
      }
      return res;
    }
  }

  MyAction toMyAction(int simulation_tick, Point touch_normal, const bool robot_touch) {
    if (!robot_touch) {
      touch_normal = {0, 1, 0};
    }
    double jump_speed = ((simulation_tick == time_jump || simulation_tick == additional_jump) ? C::rules.ROBOT_MAX_JUMP_SPEED : 0);
    if (simulation_tick < time_change) {
      return MyAction{solve(touch_normal, cangle1, sangle1, speed1), jump_speed};
    } else {
      return MyAction{solve(touch_normal, cangle2, sangle2, speed2), jump_speed};
    }
  }
  bool operator<(const Plan& other) const {
    return score < other.score;
  }
};

#endif //CODEBALL_PLAN_H
