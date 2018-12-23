#ifdef LOCAL
#include <MyStrategy.h>
#include <Simulator.h>
#include <model/Constants.h>
#include <model/Painter.h>
#include <Helper.h>
#else
#include "MyStrategy.h"
#include "Simulator.h"
#include "model/Constants.h"
#include "model/Painter.h"
#include "Helper.h"
#endif

MyStrategy::MyStrategy() {}

void doStrategy() {
  Helper::t[0].start();

  for (int id = 0; id < 2; id++) {
    Helper::best_plan[id].score = -1e18;
    Helper::best_plan[id].time_jump--;
    Helper::best_plan[id].time_change--;
    Helper::best_plan[id].collide_with_ball = false;
  }

  for (int id = 1; id >= 0; id--) {
    int iteration = 0;
    for (; Helper::t[0].cur() < 0.017; iteration++) {
      if (id == 1) {
        if (Helper::game.ball.z < 0) {
          if (Helper::t[0].cur() > 0.0085) {
            break;
          }
        } else {
          if (Helper::t[0].cur() > 0.002) {
            break;
          }
        }
      }
      Plan cur_plan;
      if (iteration == 0) {
        cur_plan = Helper::best_plan[id];
      }
      Simulator simulator(Helper::game.robots, Helper::game.ball);
      double score = 0;
      double multiplier = 1.;
      for (int sim_tick = 0; sim_tick < Constants::MAX_SIMULATION_DEPTH; sim_tick++) {
        for (auto& robot : simulator.robots) {
          if (robot.is_teammate) {
            if (robot.global_id % 2 == id) {
              robot.action = cur_plan.toMyAction(sim_tick);
            } else {
              robot.action = Helper::best_plan[robot.global_id % 2].toMyAction(sim_tick);
            }
          } else {
            robot.action = {robot.velocity.normalize() * Constants::rules.ROBOT_MAX_GROUND_SPEED, 0.};
          };
        }
        simulator.tick();
        if (id == 0) {
          score += simulator.getScoreFighter() * multiplier;
        } else {
          score += simulator.getScoreDefender() * multiplier;
        }
        multiplier *= 0.9;
      }
      /*if (iteration == 0) {
        for (auto& robot : simulator.robots) {
          if (robot.global_id % 2 == id) {
            for (int i = 1; i < robot.trace.size(); i++) {
              Painter::drawLine(robot.trace[i - 1], robot.trace[i]);
            }
          }
        }
        if (id == 0) {
          for (int i = 1; i < simulator.ball.trace.size(); i++) {
            Painter::drawLine(simulator.ball.trace[i - 1], simulator.ball.trace[i]);
          }
        }
      }*/

      cur_plan.score = score;
      for (auto& robot : simulator.robots) {
        if (robot.global_id % 2 == id) {
          cur_plan.collide_with_ball = simulator.collide_with_ball[robot.global_id];
        }
      }
      Helper::best_plan[id] = std::max(Helper::best_plan[id], cur_plan);
    }
    //std::cout << Helper::best_plan[id].score << std::endl;
    //std::cout << iteration << std::endl;
  }
  //std::cout << std::endl;

  Helper::actions[0] = Helper::best_plan[0].toMyAction(0, true).toAction();
  Helper::actions[1] = Helper::best_plan[1].toMyAction(0, true).toAction();
  //Painter::drawArena();
  //Painter::drawEntities(simulator.robots, simulator.ball, 1. / Constants::rules.TICKS_PER_SECOND, 0xFF0000);
  //Painter::endFrame();
}

void MyStrategy::act(
    const model::Robot& me,
    const model::Rules& rules,
    const model::Game& game,
    model::Action& action) {
  if (Helper::tryInit(me, rules, game)) {
    doStrategy();
  }
  action = Helper::getCurrentAction();
}

#ifdef LOCAL
#ifdef DRAWLR

#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

std::string MyStrategy::custom_rendering() {
  rapidjson::Document document;
  document.SetArray();
  rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
  for (auto line : Painter::lines_to_draw) {
    rapidjson::Value line_object;
    line_object.SetObject();
    rapidjson::Value line_data;
    line_data.SetObject();
    line_data.AddMember("x1", line.a.x, allocator);
    line_data.AddMember("y1", line.a.y, allocator);
    line_data.AddMember("z1", line.a.z, allocator);
    line_data.AddMember("x2", line.b.x, allocator);
    line_data.AddMember("y2", line.b.y, allocator);
    line_data.AddMember("z2", line.b.z, allocator);
    line_data.AddMember("width", 1.0, allocator);
    line_data.AddMember("r", line.getR(), allocator);
    line_data.AddMember("g", line.getG(), allocator);
    line_data.AddMember("b", line.getB(), allocator);
    line_data.AddMember("a", line.getA(), allocator);
    line_object.AddMember("Line", line_data, allocator);
    document.PushBack(line_object, allocator);
  }
  for (auto sphere : Painter::spheres_to_draw) {
    rapidjson::Value sphere_object;
    sphere_object.SetObject();
    rapidjson::Value sphere_data;
    sphere_data.SetObject();
    sphere_data.AddMember("x", sphere.center.x, allocator);
    sphere_data.AddMember("y", sphere.center.y, allocator);
    sphere_data.AddMember("z", sphere.center.z, allocator);
    sphere_data.AddMember("radius", sphere.radius, allocator);
    sphere_data.AddMember("r", sphere.getR(), allocator);
    sphere_data.AddMember("g", sphere.getG(), allocator);
    sphere_data.AddMember("b", sphere.getB(), allocator);
    sphere_data.AddMember("a", sphere.getA(), allocator);
    sphere_object.AddMember("Sphere", sphere_data, allocator);
    document.PushBack(sphere_object, allocator);
  }
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  document.Accept(writer);

  Painter::lines_to_draw.clear();
  Painter::spheres_to_draw.clear();
  return buf.GetString();
}
#endif
#endif
