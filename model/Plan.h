#ifndef CODEBALL_PLAN_H
#define CODEBALL_PLAN_H

struct Plan {
  double angle1[2];
  double angle2[2];
  int time_change[2];
  int time_jump[2];
  double score;
  bool collide_with_ball[2];
  Plan() {
    collide_with_ball[0] = collide_with_ball[1] = false;
    angle1[0] = Constants::rand_double(0, 2 * M_PI);
    angle1[1] = Constants::rand_double(0, 2 * M_PI);
    angle2[0] = Constants::rand_double(0, 2 * M_PI);
    angle2[1] = Constants::rand_double(0, 2 * M_PI);
    time_change[0] = Constants::rand_int(0, Constants::MAX_SIMULATION_DEPTH);
    time_change[1] = Constants::rand_int(0, Constants::MAX_SIMULATION_DEPTH);
    time_jump[0] = Constants::rand_int(0, Constants::MAX_SIMULATION_DEPTH);
    time_jump[1] = Constants::rand_int(0, Constants::MAX_SIMULATION_DEPTH);
    score = -1e18;
  }
  MyAction toMyAction(int simulation_tick, int id, bool check_jump = false) {
    double jump_speed = ((!check_jump || collide_with_ball[id]) && simulation_tick == time_jump[id]) ? Constants::rules.ROBOT_MAX_JUMP_SPEED : 0;
    if (simulation_tick < time_change[id]) {
      return MyAction{{
          Constants::rules.MAX_ENTITY_SPEED * cos(angle1[id]),
          0,
          Constants::rules.MAX_ENTITY_SPEED * sin(angle1[id])},
          jump_speed};
    } else {
      return MyAction{{
                Constants::rules.MAX_ENTITY_SPEED * cos(angle2[id]),
                0,
                Constants::rules.MAX_ENTITY_SPEED * sin(angle2[id])},
                jump_speed};
    }
  }
  bool operator<(const Plan& other) const {
    return score < other.score;
  }
};

#endif //CODEBALL_PLAN_H
