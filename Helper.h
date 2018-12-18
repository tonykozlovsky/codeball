#ifndef CODEBALL_HELPER_H
#define CODEBALL_HELPER_H

#include <model/Arena.h>
#include <model/Action.h>
#include <model/Robot.h>
#include <model/Rules.h>
#include <model/Game.h>
#include <model/Player.h>
#include <model/Ball.h>
#include <MyTimer.h>

#include <map>

struct Helper {
  static model::Arena arena;
  static model::Game game;

  static int tick;
  static std::map<int, model::Action> actions;
  static int current_id;

  static bool tryInit(
      const model::Robot& _me,
      const model::Rules& _rules,
      const model::Game& _game) {
    game = _game;
    arena = _rules.arena;
    current_id = _me.id;
    if (tick == game.current_tick) {
      return false;
    }
    tick = game.current_tick;
    return true;
  }

  static model::Action getCurrentAction() {
    return actions[current_id];
  }

  static MyTimer t[100];

  static constexpr double ROBOT_MIN_RADIUS = 1;
  static constexpr double ROBOT_MAX_RADIUS = 1.05;
  static constexpr double ROBOT_MAX_JUMP_SPEED = 15;
  static constexpr double ROBOT_ACCELERATION = 100;
  static constexpr double ROBOT_NITRO_ACCELERATION = 30;
  static constexpr double ROBOT_MAX_GROUND_SPEED = 30;
  static constexpr double ROBOT_ARENA_E = 0;
  static constexpr double ROBOT_RADIUS = 1;
  static constexpr double ROBOT_MASS = 2;
  static constexpr double TICKS_PER_SECOND = 60;
  static constexpr double MICROTICKS_PER_TICK = 1;
  static constexpr double RESET_TICKS = 2 * TICKS_PER_SECOND;
  static constexpr double BALL_ARENA_E = 0.7;
  static constexpr double BALL_RADIUS = 2;
  static constexpr double BALL_MASS = 1;
  static constexpr double MIN_HIT_E = 0.4;
  static constexpr double MAX_HIT_E = 0.5;
  static constexpr double AVG_HIT_E = (MIN_HIT_E + MAX_HIT_E) * 0.5;
  static constexpr double MAX_ENTITY_SPEED = 100;
  static constexpr double MAX_NITRO_AMOUNT = 100;
  static constexpr double START_NITRO_AMOUNT = 50;
  static constexpr double NITRO_POINT_VELOCITY_CHANGE = 0.6;
  static constexpr double NITRO_PACK_X = 20;
  static constexpr double NITRO_PACK_Y = 1;
  static constexpr double NITRO_PACK_Z = 30;
  static constexpr double NITRO_PACK_RADIUS = 0.5;
  static constexpr double NITRO_PACK_AMOUNT = 100;
  static constexpr double NITRO_RESPAWN_TICKS = 10 * TICKS_PER_SECOND;
  static constexpr double GRAVITY = 30;
};

#endif //CODEBALL_HELPER_H
