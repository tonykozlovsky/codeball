#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MODEL_NITRO_PACK_H_
#define _MODEL_NITRO_PACK_H_

#include "../rapidjson/document.h"

namespace model {
    struct NitroPack {
        int id;
        double x;
        double y;
        double z;
        double radius;
        bool alive;
        int respawn_ticks;

        void read(const rapidjson::Value& json) {
            id = json["id"].GetInt();
            x = json["x"].GetDouble();
            y = json["y"].GetDouble();
            z = json["z"].GetDouble();
            radius = json["radius"].GetDouble();
            alive = json["respawn_ticks"].IsNull();
            if (!alive) {
                respawn_ticks = json["respawn_ticks"].GetInt();
            }
        }

      void read2(const rapidjson::Value& json, const int my_id) {
        id = 228;
        x = json["position"]["x"].GetDouble();
        y = json["position"]["y"].GetDouble();
        z = json["position"]["z"].GetDouble() * (my_id == 2 ? -1 : 1);
        radius = json["radius"].GetDouble();
        alive = json["respawn_ticks"].IsNull();
        if (!alive) {
          respawn_ticks = json["respawn_ticks"].GetInt();
        }
      }

    };
}

#endif