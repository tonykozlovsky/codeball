#ifndef CODEBALL_HELPER_H
#define CODEBALL_HELPER_H

#ifdef LOCAL
#include <model/Constants.h>
#include <model/Plan.h>
#else
#include "model/Constants.h"
#include "model/Plan.h"
#endif

struct Helper {
  static model::Game game;

  static int tick;
  static model::Action actions[3];
  static int global_id;
  static int id;

  static Plan best_plan[2];

  static bool tryInit(
      const model::Robot& _me,
      const model::Rules& _rules,
      const model::Game& _game) {
    game = _game;
    Constants::rules = _rules;
    global_id = _me.id;
    id = global_id % _rules.team_size;
    if (tick == game.current_tick) {
      return false;
    }
    tick = game.current_tick;
    actions[0] = actions[1] = actions[2] = model::Action();
    return true;
  }

  static model::Action getCurrentAction() {
    return actions[id];
  }

  static MyTimer t[100];
};

#endif //CODEBALL_HELPER_H
