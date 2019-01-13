#ifndef CODEBALL_MYACTION_H
#define CODEBALL_MYACTION_H

#ifdef LOCAL
#include <model/Point.h>
#include <model/Action.h>
#else
#include "Point.h"
#include "Action.h"
#endif

struct MyAction {
  Point target_velocity = {0, 0, 0};
  double jump_speed = 0;
  bool use_nitro = false;
  model::Action toAction() {
    model::Action result;
    result.target_velocity_x = target_velocity.x;
    result.target_velocity_y = target_velocity.y;
    result.target_velocity_z = target_velocity.z;
    result.jump_speed = jump_speed;
    result.use_nitro = use_nitro;
    return result;
  };
};

#endif //CODEBALL_MYACTION_H
