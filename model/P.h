#ifndef CODEBALL_PAINTER_H
#define CODEBALL_PAINTER_H

#ifdef LOCAL
#include <RewindClient/RewindClient.h>
#include <model/Entity.h>
#else
#include "Entity.h"
#endif

struct P {
  struct Line {
    Point a, b;
    uint32_t color;
    double getR() {
      return color / 256 / 256 / 255.;
    }
    double getG() {
      return color / 256 % 256 / 255.;
    }
    double getB() {
      return color % 256 / 255.;
    }
    double getA() {
      return 0.5;
    }
  };

  struct Sphere {
    Point center;
    double radius;
    uint32_t color;

    double getR() {
      return color / 256 / 256 / 255.;
    }
    double getG() {
      return color / 256 % 256 / 255.;
    }
    double getB() {
      return color % 256 / 255.;
    }
    double getA() {
      return 0.5;
    }
  };

  static std::vector<Line> lines_to_draw;
  static std::vector<Sphere> spheres_to_draw;
  static std::vector<std::string> logs;

  static void drawLine(const Point& p1, const Point& p2, const uint32_t color = 0) {
#ifdef LOCAL
#ifdef DRAWLR
    lines_to_draw.push_back(Line{p1, p2, color});
#else
    RewindClient::instance().line3d(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, color);
#endif
#endif
  }

  static void drawEntities(std::vector<Entity> entities, const Entity& e, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
    entities.push_back(e);
    drawEntities(entities, delta_time, color);
#endif
  }

  static void drawEntities(const Entity& e, std::vector<Entity>& entities, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
    entities.push_back(e);
    drawEntities(entities, delta_time, color);
#endif
  }

  static void drawEntities(const Entity& e, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
    drawEntities(std::vector<Entity>{e}, delta_time, color);
#endif
  }

  static void drawEntities(const std::vector<Entity>& entities, const double delta_time = 0,
                           const uint32_t color = 0) {
#ifdef LOCAL
#ifdef DRAWLR
    for (auto& e : entities) {
      spheres_to_draw.push_back(Sphere{e.position, e.radius * 10. / 9., color});
      Point next_pos = e.position + e.velocity * delta_time;
      lines_to_draw.push_back(Line{e.position, next_pos, color});
    }
#else
    auto& draw = RewindClient::instance();
      draw.circle3d(
          e.position.x,
          e.position.y,
          e.position.z,
          e.radius, color);
      Point next_pos = e.position + e.velocity * delta_time;
      draw.line3d(
          e.position.x,
          e.position.y,
          e.position.z,
          next_pos.x,
          next_pos.y,
          next_pos.z, color);
    }
#endif
#endif
  }

  static void drawArena() {
#ifdef LOCAL
#ifndef DRAWLR
    auto& draw = RewindClient::instance();
    //walls
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    //floor
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2);
    draw.line3d(
        -Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, 0, -Constants::rules.arena.depth / 2);
    //ceiling
    draw.line3d(
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    draw.line3d(
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, Constants::rules.arena.height, Constants::rules.arena.depth / 2,
        Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    draw.line3d(
        Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.width / 2, Constants::rules.arena.height, -Constants::rules.arena.depth / 2);
    //goal1
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        -Constants::rules.arena.depth / 2 - Constants::rules.arena.goal_depth);
    //goal2
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        -Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        -Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
    draw.line3d(
        Constants::rules.arena.goal_width / 2,
        0,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth,
        Constants::rules.arena.goal_width / 2,
        Constants::rules.arena.goal_height,
        Constants::rules.arena.depth / 2 + Constants::rules.arena.goal_depth);
#endif
#endif
  }

  static void endFrame() {
#ifdef LOCAL
#ifndef DRAWLR
    RewindClient::instance().end_frame();
#endif
#endif
  }

  template<typename... Args>
  static void logn(Args... args) {
#ifdef LOCAL
#ifdef DRAWLR
    logs.push_back(to_string(args...));
#endif
#endif
  }

  template<typename... Args>
  static void log(Args... args) {
#ifdef LOCAL
#ifdef DRAWLR
    if (logs.size() == 0) {
      logn(args...);
    } else {
      logs.back() += " " + to_string(args...);
    }
#endif
#endif
  }

  template<typename T, typename... Args>
  static inline std::string to_string(T fmt, Args... args) {
    return to_string(fmt) + to_string(args...);
  }
  static inline std::string to_string(std::string fmt) {
    return fmt;
  }
  static inline std::string to_string(double fmt) {
    std::stringstream _s;
    _s << std::fixed << std::setprecision(6) << fmt;
    return _s.str();
  }
  static inline std::string to_string(const char*& fmt) {
    return std::string(fmt);
  }
  template<typename T>
  static inline std::string to_string(T fmt) {
    return std::to_string(fmt);
  }

};

#endif //CODEBALL_PAINTER_H
