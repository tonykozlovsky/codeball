#ifdef LOCAL
#include <MyStrategy.h>
#include <Simulator.h>
#include <model/C.h>
#include <model/P.h>
#include <H.h>
#else
#include "MyStrategy.h"
#include "Simulator.h"
#include "model/C.h"
#include "model/P.h"
#include "H.h"
#endif

MyStrategy::MyStrategy() {}

double func(const Simulator& simulator) {
  double score = 1e9;
  // double multiplier = 1.;
  for (auto& robot : simulator.robots) {
    if (robot.global_id == 1) {
      bool flag = false;
      for (int i = 1; i < 200; i++) {
        double delta = (robot.trace[i].to2d() - simulator.ball.trace[i].to2d()).length() - (robot.trace[i - 1].to2d() - simulator.ball.trace[i - 1].to2d()).length();
        if (delta > 0 && flag) {
          break;
        }
        if (delta < 0) {
          flag = true;
        }
        score = std::min(score, (robot.trace[i].to2d() - simulator.ball.trace[i].to2d()).length());
      }
    }
  }
  return score;
}

double simulate(double angle, bool draw = false) {
  Simulator simulator(H::game.robots, H::game.ball);
  for (auto& robot : simulator.robots) {
    if (robot.global_id == 1) {
      robot.action.target_velocity = Point{cos(angle), 0, sin(angle)} * C::rules.ROBOT_MAX_GROUND_SPEED;
    }
  }
  for (int i = 0; i < 200; i++) {
    simulator.tick();
  }
  if (draw) {
    for (auto& robot : simulator.robots) {
      if (robot.global_id == 1) {
        for (int i = 1; i < robot.trace.size(); i++) {
          P::drawLine(robot.trace[i], robot.trace[i - 1], 0x00FF00);
        }
      } else {
        for (int i = 1; i < robot.trace.size(); i++) {
          P::drawLine(robot.trace[i], robot.trace[i - 1], 0x0000FF);
        }
      }
    }
    for (int i = 1; i < simulator.ball.trace.size(); i++) {
      P::drawLine(simulator.ball.trace[i], simulator.ball.trace[i - 1], 0xFF0000);
    }

  }
  return func(simulator);
}

double gradient(double angle) {
  return simulate(angle + 0.01) - simulate(angle);
}

void doStrategy1() {
  double angle[5] = {0, M_PI / 2, M_PI, M_PI * 3 / 2, 2 * M_PI};
  double g[4] = {gradient(0), gradient(M_PI / 2), gradient(M_PI), gradient(M_PI * 3 / 2)};

  for (int i = 0; i < 4; ++i) {
    P::logn(g[i]);
    //P::logn(simulate(angle[i]));
  }

  double l = 0, r = 0;
  for (int i = 0; i < 4; i++) {
    int j = (i + 1) % 4;
    if (g[i] < 0 && g[j] > 0) {
      l = angle[i];
      r = angle[i + 1];
    }
  }

  //for (double angle = 0; angle < 2 * M_PI; angle += 2 * M_PI / 10) {
    //P::logn(simulate(angle, true));
  //}
  //P::logn("");

  //P::logn("0: ", simulate(0));
  //P::logn("PI: ", simulate(M_PI));

  for (int it = 0; it < 10; ++it) {
    double m1 = l + (r - l) / 3.;
    double m2 = r - (r - l) / 3.;
    double v1 = simulate(m1);
    double v2 = simulate(m2);
    //P::logn(m1, " ", m2, " ", v1," " , v2);
    if (v1 > v2) {
      l = m1;
    } else {
      r = m2;
    }
  }

  P::logn(simulate(l, true));
  double jump = 0;
  if (C::rand_double(0, 1) < 0.01) {
    jump = C::rules.ROBOT_MAX_JUMP_SPEED;
  }
  H::actions[1] = MyAction{Point{cos(l), 0, sin(l)} * C::rules.ROBOT_MAX_GROUND_SPEED, jump}.toAction();
}

void doStrategy() {

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
