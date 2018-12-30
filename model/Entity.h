#ifndef CODEBALL_ENTITY_H
#define CODEBALL_ENTITY_H


#ifdef LOCAL
#include <model/C.h>
#else
#include "C.h"
#endif


struct Entity {
  Point position;
  Point prev_position;
  Point velocity;
  Point prev_velocity;
  double radius;
  double prev_radius;

  bool touch;
  bool prev_touch;
  Point touch_normal;
  Point prev_touch_normal;

  MyAction action;

  double arena_e;
  double mass;

  double radius_change_speed;
  double prev_radius_change_speed;

  std::vector<Point> trace;

  int global_id;
  bool is_teammate;

  bool collide_with_ball_in_air;

  bool operator <(const Entity& other) const {
    return global_id < other.global_id;
  }

  Entity() {}

  Entity(const std::string& type) {
    if (type == "ball") {
      radius = C::rules.BALL_RADIUS;

      arena_e = C::rules.BALL_ARENA_E;
      mass = C::rules.BALL_MASS;
      radius_change_speed = 0;
    } else if (type == "robot") {

    } else if (type == "test_points") {
      radius = C::rules.BALL_RADIUS;
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

    if (!is_teammate) {
      action = {velocity.normalize() * C::rules.ROBOT_MAX_GROUND_SPEED, 0.};
    }

  }

  void save() {
    prev_position = position;
    prev_velocity = velocity;
    prev_radius = radius;
    prev_radius_change_speed = radius_change_speed;
    prev_touch = touch;
    prev_touch_normal = touch_normal;
  }

  void roll_back() {
    position = prev_position;
    velocity = prev_velocity;
    radius = prev_radius;
    radius_change_speed = prev_radius_change_speed;
    touch = prev_touch;
    touch_normal = prev_touch_normal;
  }

};

#endif //CODEBALL_ENTITY_H
