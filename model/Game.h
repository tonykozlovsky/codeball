#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_GAME_H_
#define _MODEL_GAME_H_

#include <vector>
#include "../rapidjson/document.h"
#include "Player.h"
#include "Arena.h"
#include "Robot.h"
#include "NitroPack.h"
#include "Ball.h"

namespace model {
struct Game {
#ifdef FROM_LOG
  static int my_id;
#endif
  int current_tick;
  std::vector<Player> players;
  std::vector<Robot> robots;
  std::vector<NitroPack> nitro_packs;
  Ball ball;

  void read(const rapidjson::Value& json) {
    current_tick = json["current_tick"].GetInt();

    rapidjson::Value::ConstArray json_players = json["players"].GetArray();
    players.resize(json_players.Size());
    for (size_t i = 0; i < players.size(); i++) {
      players[i].read(json_players[i]);
    }

    rapidjson::Value::ConstArray json_robots = json["robots"].GetArray();
    robots.resize(json_robots.Size());
    for (size_t i = 0; i < robots.size(); i++) {
      robots[i].read(json_robots[i]);
    }

    rapidjson::Value::ConstArray json_nitro_packs = json["nitro_packs"].GetArray();
    nitro_packs.resize(json_nitro_packs.Size());
    for (size_t i = 0; i < nitro_packs.size(); i++) {
      nitro_packs[i].read(json_nitro_packs[i]);
    }

    ball.read(json["ball"]);
  }

  void read2(const rapidjson::Value& json) {
    current_tick = json["current_tick"].GetInt();
    if (current_tick == 0) {
      rapidjson::Value::ConstArray json_names = json["names"].GetArray();
      for (int i = 0; i < json_names.Size(); i++) {
        if (std::string(json_names[i].GetString()) == "TonyK") {
          my_id = i;
        }
      }
    }
    rapidjson::Value::ConstArray json_scores = json["scores"].GetArray();
    players.resize(2);
    for (int i = 0; i < json_scores.Size(); i++) {
      players[i] = Player{i, i == my_id, false, json_scores[i].GetInt()};
    }
    rapidjson::Value::ConstArray json_robots = json["robots"].GetArray();
    robots.resize(json_robots.Size());
    for (size_t i = 0; i < robots.size(); i++) {
      robots[i].read2(json_robots[i], my_id);
    }

    ball.read2(json["ball"], my_id);
    //todo nitro from log
  }
};
}

#endif