#ifdef LOCAL
#include <MyStrategy.h>
#include <Simulator.h>
#include <SmartSimulator.h>
#include <model/C.h>
#include <model/P.h>
#include <H.h>
#else
#include "MyStrategy.h"
#include "Simulator.h"
#include "SmartSimulator.h"
#include "model/C.h"
#include "model/P.h"
#include "H.h"
#endif

MyStrategy::MyStrategy() {}

void doStrategy() {
  std::cout << H::tick << " ===============================================================================" << std::endl;
  P::logn(H::tick);
  H::t[0].start();
  SmartSimulator simulator(3, H::game.robots, H::game.ball);
  P::logn(" ");
  SmartSimulator simulator2(3, H::game.robots, H::game.ball, true);
  for (int i = 0; i < 50; ++i) {
    EntityState s1;
    for (int j = 0; j < simulator.initial_static_entities_size; j++) {
      if (simulator.initial_static_entities[j].id == 2) {
        s1 = simulator.initial_static_entities[j].states[i];
      }
    }
    EntityState s2;
    for (int j = 0; j < simulator2.initial_static_entities_size; j++) {
      if (simulator2.initial_static_entities[j].id == 2) {
        s2 = simulator2.initial_static_entities[j].states[i];
      }
    }
    P::logn(" ");
    P::log("pos: ", (s1.position - s2.position).length(), " vel: ", (s1.velocity - s2.velocity).length(), " ", s1.touch, " ", s2.touch);
    P::log("ball pos: ", (simulator.ball->states[i].position - simulator2.ball->states[i].position).length(),
        " ball vel: ", (simulator.ball->states[i].velocity - simulator2.ball->states[i].velocity).length());
  }
  // double clocest_dist = 1e9;
  // MyAction best_action;
  // for (int iteration = 0; iteration < 400; iteration++) {
  //   simulator.initIteration();
  //   double angle = C::rand_double(0, 2 * M_PI);
  //   double vx = cos(angle) * C::rules.ROBOT_MAX_GROUND_SPEED;
  //   double vy = sin(angle) * C::rules.ROBOT_MAX_GROUND_SPEED;
  //   for (int sim_tick = 0; sim_tick < C::MAX_SIMULATION_DEPTH; ++sim_tick) {
  //     simulator.main_robot->action = MyAction{{vx, 0, vy}, 0};
  //     simulator.tick_dynamic(sim_tick);
      /*if (clocest_dist > (simulator.main_robot->state.position - simulator.ball->state.position).length()) {
        clocest_dist = (simulator.main_robot->state.position - simulator.ball->state.position).length();
        best_action = MyAction{{vx, 0, vy}, 0};
      }*/
  //   }
 //  }
  H::t[0].cur(true);
  // H::actions[1] = best_action.toAction();
  for (int i = 0; i < 35; ++i) {
    H::t[i].cur(false, true);
    // P::logn("t" + std::to_string(i) + ": ", H::t[i].avg() * 1000);
  }
  std::cout.flush();
}

void MyStrategy::act(
    const model::Robot& me,
    const model::Rules& rules,
    const model::Game& game,
    model::Action& action) {
  if (H::tryInit(me, rules, game)) {
    doStrategy();
    action = H::getCurrentAction();
  } else {
    action = H::getCurrentAction();
    H::global_timer.cur(true, true);
  }
}

#ifdef LOCAL
#ifdef DRAWLR

#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

std::string MyStrategy::custom_rendering() {
  rapidjson::Document document;
  document.SetArray();
  rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
  for (auto line : P::lines_to_draw) {
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
  for (auto sphere : P::spheres_to_draw) {
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

  for (auto& log : P::logs) {
    rapidjson::Value log_object;
    log_object.SetObject();
    rapidjson::Value value;
    value.SetString(log.c_str(), allocator);
    log_object.AddMember("Text", value, allocator);
    document.PushBack(log_object, allocator);
  }

  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  document.Accept(writer);

  P::logs.clear();
  P::lines_to_draw.clear();
  P::spheres_to_draw.clear();
  return buf.GetString();
}
#endif
#endif
