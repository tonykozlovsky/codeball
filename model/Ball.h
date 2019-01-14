#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_BALL_H_
#define _MODEL_BALL_H_

#include "../rapidjson/document.h"

namespace model {
struct Ball {
  double x;
  double y;
  double z;
  double velocity_x;
  double velocity_y;
  double velocity_z;
  double radius;

  void read(const rapidjson::Value& json) {
    x = json["x"].GetDouble();
    y = json["y"].GetDouble();
    z = json["z"].GetDouble();
    velocity_x = json["velocity_x"].GetDouble();
    velocity_y = json["velocity_y"].GetDouble();
    velocity_z = json["velocity_z"].GetDouble();
    radius = json["radius"].GetDouble();
  }

  void read2(const rapidjson::Value& json, int my_id) {
    x = json["position"]["x"].GetDouble();
    y = json["position"]["y"].GetDouble();
    z = json["position"]["z"].GetDouble() * (my_id == 1 ? -1 : 1);
    velocity_x = json["velocity"]["x"].GetDouble();
    velocity_y = json["velocity"]["y"].GetDouble();
    velocity_z = json["velocity"]["z"].GetDouble() * (my_id == 1 ? -1 : 1);
    radius = json["radius"].GetDouble();
  }

};
}

#endif