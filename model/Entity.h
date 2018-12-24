#ifndef CODEBALL_ENTITY_H
#define CODEBALL_ENTITY_H


#ifdef LOCAL
#include <model/C.h>
#else
#include "Constants.h"
#endif


struct Entity {
  Point position;
  Point velocity;
  double radius;

  bool touch;
  Point touch_normal;

  MyAction action;

  double arena_e;
  double mass;

  double radius_change_speed;

  std::vector<Point> trace;

  int global_id;
  bool is_teammate;

  Entity() {}

  Entity(const std::string& type) {
    if (type == "ball") {
      radius = C::rules.BALL_RADIUS;

      arena_e = C::rules.BALL_ARENA_E;
      mass = C::rules.BALL_MASS;
      radius_change_speed = 0;
    } else if (type == "robot") {

    }
  }

  Entity(const model::Ball& ball) {
    position = {ball.x, ball.y, ball.z};
    velocity = {ball.velocity_x, ball.velocity_y, ball.velocity_z};
    radius = ball.radius;

    arena_e = C::rules.BALL_ARENA_E;
    mass = C::rules.BALL_MASS;
    radius_change_speed = 0;
  }

  Entity(const model::Robot& robot) {
    position = {robot.x, robot.y, robot.z};
    velocity = {robot.velocity_x, robot.velocity_y, robot.velocity_z};
    radius = robot.radius;
    touch = robot.touch;
    touch_normal =
        {robot.touch_normal_x,
            robot.touch_normal_y,
            robot.touch_normal_z};

    arena_e = C::rules.ROBOT_ARENA_E;
    mass = C::rules.ROBOT_MASS;

    radius_change_speed = 0;

    global_id = robot.id;
    is_teammate = robot.is_teammate;
  }

};

#endif //CODEBALL_ENTITY_H
