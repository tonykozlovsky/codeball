#ifndef CODEBALL_SIMULATOR_H
#define CODEBALL_SIMULATOR_H

#include <math.h>
#include <algorithm>

#include <Helper.h>

struct Simulator {

  struct Point {
    double x, y, z;
    Point operator-(const Point& other) const {
      return {x - other.x, y - other.y, z - other.z};
    }
    Point operator*(const double value) const {
      return {x * value, y * value, z * value};
    }
    Point& operator-=(const Point& other) {
      x -= other.x;
      y -= other.y;
      z -= other.z;
      return *this;
    }
    Point& operator+=(const Point& other) {
      x += other.x;
      y += other.y;
      z += other.z;
      return *this;
    }
    double length() const {
      return sqrt(length_sq());
    }
    double length_sq() const {
      return x * x + y * y + z * z;
    }
    Point normalize() const {
      double norm = length();
      return {x / norm, y / norm, z / norm};
    }
    double dot(const Point& other) const {
      return x * other.x + y * other.y + z * other.z;
    }
    Point clamp(const double ub) const {
      if (length_sq() > ub * ub) {
        return normalize() * ub;
      }
      return *this;
    }

  };

  struct Point2d {
    double x, y;
    Point2d operator+(const Point2d& other) const {
      return {x + other.x, y + other.y};
    }
    Point2d operator-(const Point2d& other) const {
      return {x - other.x, y - other.y};
    }
    Point2d operator/(const double value) const {
      return {x / value, y / value};
    }
    Point2d operator*(const double value) const {
      return {x * value, y * value};
    }
    Point2d& operator-=(const Point2d& other) {
      x -= other.x;
      y -= other.y;
      return *this;
    }
    Point2d& operator+=(const Point2d& other) {
      x += other.x;
      y += other.y;
      return *this;
    }
    double length() const {
      return sqrt(length_sq());
    }
    double length_sq() const {
      return x * x + y * y;
    }
    Point2d normalize() const {
      double norm = length();
      return {x / norm, y / norm};
    }
    double dot(const Point2d& other) const {
      return x * other.x + y * other.y;
    }
    Point2d clamp(const double ub) const {
      if (length_sq() > ub * ub) {
        return normalize() * ub;
      }
      return *this;
    }

  };

  struct Action {
    Point target_velocity;
    double jump_speed;
  };

  struct Entity {
    Point position;
    Point velocity;
    double radius;

    bool touch;
    Point touch_normal;

    Action action;

    double arena_e;
    double mass;

    double radius_change_speed;

    std::vector<Point> trace;

    Entity() {}

    Entity(const model::Ball& ball) {
      position = {ball.x, ball.y, ball.z};
      velocity = {ball.velocity_x, ball.velocity_y, ball.velocity_z};
      radius = ball.radius;

      arena_e = Helper::BALL_ARENA_E;
      mass = Helper::BALL_MASS;
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

      arena_e = Helper::ROBOT_ARENA_E;
      mass = Helper::ROBOT_MASS;

      radius_change_speed = 0;
    }

  };

  struct Dan {
    double distance;
    Point normal;
    bool operator<(const Dan& other) const {
      return distance < other.distance;
    }
  };

  std::vector<Entity> robots;
  Entity ball;

  Simulator(){}

  Simulator(const std::vector<model::Robot>& _robots, const model::Ball& _ball) {
    for (auto& robot : _robots) {
      robots.push_back(Entity(robot));
    }
    ball = Entity(_ball);
    update_trace();
  }

  double length(const Point& p) {
    return p.length();
  }

  double length(const Point2d& p) {
    return p.length();
  }

  Point normalize(const Point& p) {
    return p.normalize();
  }

  Point2d normalize(const Point2d& p) {
    return p.normalize();
  }

  double dot(const Point& a, const Point& b) {
    return a.dot(b);
  }

  Point clamp(const Point& v, const double ub) {
    return v.clamp(ub);
  }

  void collide_entities(Entity& a, Entity& b) {
    Point delta_position = b.position - a.position;
    double distance = length(delta_position);
    double penetration = a.radius + b.radius - distance;
    if (penetration > 0) {
      double k_a = (1 / a.mass) / ((1 / a.mass) + (1 / b.mass));
      double k_b = (1 / b.mass) / ((1 / a.mass) + (1 / b.mass));
      Point normal = normalize(delta_position);
      a.position -= normal * penetration * k_a;
      b.position += normal * penetration * k_b;
      double delta_velocity = dot(b.velocity - a.velocity, normal)
          + b.radius_change_speed - a.radius_change_speed;
      if (delta_velocity < 0) {
        Point impulse = normal * (1 + Helper::AVG_HIT_E) * delta_velocity;
        a.velocity += impulse * k_a;
        b.velocity -= impulse * k_b;
      }
    }
  }

  bool collide_with_arena(Entity& e, Point& result) {
    Dan dan = dan_to_arena(e.position);
    double distance = dan.distance;
    Point normal = dan.normal;
    double penetration = e.radius - distance;
    if (penetration > 0) {
      e.position += normal * penetration;
      double velocity = dot(e.velocity, normal) - e.radius_change_speed;
      if (velocity < 0) {
        e.velocity -= normal * (1 + e.arena_e) * velocity;
        result = normal;
        return true;
      }
    }
    return false;
  }

  void move(Entity& e, const double delta_time) {
    e.velocity = clamp(e.velocity, Helper::MAX_ENTITY_SPEED);
    e.position += e.velocity * delta_time;
    e.position.y -= Helper::GRAVITY * delta_time * delta_time / 2;
    e.velocity.y -= Helper::GRAVITY * delta_time;
  }

  void update(const double delta_time) {
    for (auto& robot : robots) {
      if (robot.touch) {
        Point target_velocity = clamp(
            robot.action.target_velocity,
            Helper::ROBOT_MAX_GROUND_SPEED);
        target_velocity -= robot.touch_normal * robot.touch_normal.dot(target_velocity);
        Point target_velocity_change = target_velocity - robot.velocity;
        if (length(target_velocity_change) > 0) {
          double acceleration = Helper::ROBOT_ACCELERATION * fmax(0., robot.touch_normal.y);
          robot.velocity += clamp(
              normalize(target_velocity_change) * acceleration * delta_time,
              length(target_velocity_change));
        }
      }
      move(robot, delta_time);
      robot.radius = Helper::ROBOT_MIN_RADIUS
          + (Helper::ROBOT_MAX_RADIUS - Helper::ROBOT_MIN_RADIUS)
              * robot.action.jump_speed / Helper::ROBOT_MAX_JUMP_SPEED;
      robot.radius_change_speed = robot.action.jump_speed;
    }
    move(ball, delta_time);
    for (int i = 0; i < robots.size(); i++) {
      for (int j = 0; j < i; j++) {
        collide_entities(robots[i], robots[j]);
      }
    }
    Point collision_normal;
    for (auto& robot : robots) {
      collide_entities(robot, ball);
      if (!collide_with_arena(robot, collision_normal)) {
        robot.touch = false;
      } else {
        robot.touch = true;
        robot.touch_normal = collision_normal;
      }
    }
    collide_with_arena(ball, collision_normal);
    if (abs(ball.position.z) > Helper::arena.depth / 2 + ball.radius) {
      goal_scored();
    }
  }

  void update_trace() {
    return;
    for (auto& robot : robots) {
      robot.trace.push_back(robot.position);
    }
    ball.trace.push_back(ball.position);
  }

  void tick() {
    double delta_time = 1. / Helper::TICKS_PER_SECOND;
    for (int i = 0; i < Helper::MICROTICKS_PER_TICK; i++) {
      update(delta_time / Helper::MICROTICKS_PER_TICK);
    }
    update_trace();
  }

  void goal_scored() {
    ;
  }

  Dan dan_to_plane(
      const Point& point,
      const Point& point_on_plane,
      const Point& plane_normal) {
    return {dot(point - point_on_plane, plane_normal), plane_normal};
  }

  Dan dan_to_sphere_inner(
      const Point& point,
      const Point& sphere_center,
      double sphere_radius) {
    return {sphere_radius - length(point - sphere_center),
        normalize(sphere_center - point)};
  }

  Dan dan_to_sphere_outer(
      const Point& point,
      const Point& sphere_center,
      double sphere_radius) {
    return {length(point - sphere_center) - sphere_radius,
        normalize(point - sphere_center)};
  }

  Dan dan_to_arena_quarter(const Point& point) {
    auto dan = dan_to_plane(point, {0, 0, 0}, {0, 1, 0});
    // Ceiling
    dan = std::min(dan, dan_to_plane(point, {0, Helper::arena.height, 0}, {0, -1, 0}));
    // Side x
    dan = std::min(dan, dan_to_plane(point, {Helper::arena.width / 2, 0, 0}, {-1, 0, 0}));
    // Side z (goal)
    dan = std::min(dan, dan_to_plane(
        point,
        {0, 0, (Helper::arena.depth / 2) + Helper::arena.goal_depth},
        {0, 0, -1}));
    // Side z
    Point2d v = Point2d{point.x, point.y} - Point2d{
        Helper::arena.goal_width / 2 - Helper::arena.goal_top_radius,
        Helper::arena.goal_height - Helper::arena.goal_top_radius};
    if (point.x >= Helper::arena.goal_width / 2 + Helper::arena.goal_side_radius ||
        point.y >= Helper::arena.goal_height + Helper::arena.goal_side_radius ||
        (v.x > 0 & v.y > 0
            & length(v) >= Helper::arena.goal_top_radius + Helper::arena.goal_side_radius)) {
      dan = std::min(dan, dan_to_plane(point, {0, 0, Helper::arena.depth / 2}, {0, 0, -1}));
    }

    // Side x & ceiling (goal)
    if (point.z >= (Helper::arena.depth / 2) + Helper::arena.goal_side_radius) {
      // x
      dan = std::min(dan, dan_to_plane(
          point,
          {Helper::arena.goal_width / 2, 0, 0},
          {-1, 0, 0}));
      // y
      dan = std::min(dan, dan_to_plane(point, {0, Helper::arena.goal_height, 0}, {0, -1, 0}));
    }

    // Goal back corners
    assert(Helper::arena.bottom_radius == Helper::arena.goal_top_radius);
    if (point.z
        > (Helper::arena.depth / 2) + Helper::arena.goal_depth - Helper::arena.bottom_radius) {
      dan = std::min(dan, dan_to_sphere_inner(
          point, {
              std::clamp(
                  point.x,
                  Helper::arena.bottom_radius - (Helper::arena.goal_width / 2),
                  (Helper::arena.goal_width / 2) - Helper::arena.bottom_radius
              ),
              std::clamp(
                  point.y,
                  Helper::arena.bottom_radius,
                  Helper::arena.goal_height - Helper::arena.goal_top_radius
              ),
              (Helper::arena.depth / 2) + Helper::arena.goal_depth - Helper::arena.bottom_radius},
          Helper::arena.bottom_radius));
    }

    // Corner
    if (point.x > (Helper::arena.width / 2) - Helper::arena.corner_radius
        && point.z > (Helper::arena.depth / 2) - Helper::arena.corner_radius) {
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              (Helper::arena.width / 2) - Helper::arena.corner_radius,
              point.y,
              (Helper::arena.depth / 2) - Helper::arena.corner_radius
          },
          Helper::arena.corner_radius));
    }

    // Goal outer corner
    if (point.z < (Helper::arena.depth / 2) + Helper::arena.goal_side_radius) {
      // Side x
      if (point.x < (Helper::arena.goal_width / 2) + Helper::arena.goal_side_radius) {
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            {
                (Helper::arena.goal_width / 2) + Helper::arena.goal_side_radius,
                point.y,
                (Helper::arena.depth / 2) + Helper::arena.goal_side_radius
            },
            Helper::arena.goal_side_radius));
      }
      // Ceiling
      if (point.y < Helper::arena.goal_height + Helper::arena.goal_side_radius) {
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            {
                point.x,
                Helper::arena.goal_height + Helper::arena.goal_side_radius,
                (Helper::arena.depth / 2) + Helper::arena.goal_side_radius
            },
            Helper::arena.goal_side_radius));
      }
      // Top corner
      Point2d o{
          (Helper::arena.goal_width / 2) - Helper::arena.goal_top_radius,
          Helper::arena.goal_height - Helper::arena.goal_top_radius
      };
      auto v = Point2d{point.x, point.y} - o;
      if (v.x > 0 && v.y > 0) {
        o = o + normalize(v) * (Helper::arena.goal_top_radius + Helper::arena.goal_side_radius);
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            {o.x, o.y, (Helper::arena.depth / 2) + Helper::arena.goal_side_radius},
            Helper::arena.goal_side_radius));
      }
    }

    // Goal inside top corners
    if (point.z > (Helper::arena.depth / 2) + Helper::arena.goal_side_radius
        && point.y > Helper::arena.goal_height - Helper::arena.goal_top_radius) {
      // Side x
      if (point.x > (Helper::arena.goal_width / 2) - Helper::arena.goal_top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {
                (Helper::arena.goal_width / 2) - Helper::arena.goal_top_radius,
                Helper::arena.goal_height - Helper::arena.goal_top_radius,
                point.z
            },
            Helper::arena.goal_top_radius));
      }
      // Side z
      if (point.z
          > (Helper::arena.depth / 2) + Helper::arena.goal_depth - Helper::arena.goal_top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {
                point.x,
                Helper::arena.goal_height - Helper::arena.goal_top_radius,
                (Helper::arena.depth / 2) + Helper::arena.goal_depth - Helper::arena.goal_top_radius
            },
            Helper::arena.goal_top_radius));
      }
    }

    // Bottom corners
    if (point.y < Helper::arena.bottom_radius) {
      // Side x
      if (point.x > (Helper::arena.width / 2) - Helper::arena.bottom_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {
                (Helper::arena.width / 2) - Helper::arena.bottom_radius,
                Helper::arena.bottom_radius,
                point.z
            },
            Helper::arena.bottom_radius));
      }
      // Side z
      if (point.z > (Helper::arena.depth / 2) - Helper::arena.bottom_radius
          && point.x >= (Helper::arena.goal_width / 2) + Helper::arena.goal_side_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {
                point.x,
                Helper::arena.bottom_radius,
                (Helper::arena.depth / 2) - Helper::arena.bottom_radius
            },
            Helper::arena.bottom_radius));
      }
      // Side z (goal)
      if (point.z
          > (Helper::arena.depth / 2) + Helper::arena.goal_depth - Helper::arena.bottom_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {
                point.x,
                Helper::arena.bottom_radius,
                (Helper::arena.depth / 2) + Helper::arena.goal_depth - Helper::arena.bottom_radius
            },
            Helper::arena.bottom_radius));
      }
      // Goal outer corner
      Point2d o{
          (Helper::arena.goal_width / 2) + Helper::arena.goal_side_radius,
          (Helper::arena.depth / 2) + Helper::arena.goal_side_radius
      };
      auto v = Point2d{point.x, point.z} - o;
      if (v.x < 0 && v.y < 0
          && length(v) < Helper::arena.goal_side_radius + Helper::arena.bottom_radius) {
        o = o + normalize(v) * (Helper::arena.goal_side_radius + Helper::arena.bottom_radius);
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {o.x, Helper::arena.bottom_radius, o.y},
            Helper::arena.bottom_radius));
      }
      // Side x (goal)
      if (point.z >= (Helper::arena.depth / 2) + Helper::arena.goal_side_radius
          && point.x > (Helper::arena.goal_width / 2) - Helper::arena.bottom_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {
                (Helper::arena.goal_width / 2) - Helper::arena.bottom_radius,
                Helper::arena.bottom_radius,
                point.z
            },
            Helper::arena.bottom_radius));
      }
      // Corner
      if (point.x > (Helper::arena.width / 2) - Helper::arena.corner_radius
          && point.z > (Helper::arena.depth / 2) - Helper::arena.corner_radius) {
        Point2d corner_o{
            (Helper::arena.width / 2) - Helper::arena.corner_radius,
            (Helper::arena.depth / 2) - Helper::arena.corner_radius
        };
        auto
            n = Point2d{point.x, point.z} - corner_o;
        double dist = length(n);
        if (dist > Helper::arena.corner_radius - Helper::arena.bottom_radius) {
          n = n / dist;
          auto o2 = corner_o + n * (Helper::arena.corner_radius - Helper::arena.bottom_radius);
          dan = std::min(dan, dan_to_sphere_inner(
              point,
              {o2.x, Helper::arena.bottom_radius, o2.y},
              Helper::arena.bottom_radius));
        }
      }
      // Ceiling corners
      if (point.y > Helper::arena.height - Helper::arena.top_radius) {
        // Side x
        if (point.x > (Helper::arena.width / 2) - Helper::arena.top_radius) {
          dan = std::min(dan, dan_to_sphere_inner(
              point,
              {
                  (Helper::arena.width / 2) - Helper::arena.top_radius,
                  Helper::arena.height - Helper::arena.top_radius,
                  point.z,
              },
              Helper::arena.top_radius));
        }
        // Side z
        if (point.z > (Helper::arena.depth / 2) - Helper::arena.top_radius) {
          dan = std::min(dan, dan_to_sphere_inner(
              point,
              {
                  point.x,
                  Helper::arena.height - Helper::arena.top_radius,
                  (Helper::arena.depth / 2) - Helper::arena.top_radius,
              },
              Helper::arena.top_radius));
        }
        // Corner
        if (point.x > (Helper::arena.width / 2) - Helper::arena.corner_radius
            && point.z > (Helper::arena.depth / 2) - Helper::arena.corner_radius) {
          Point2d corner_o{
              (Helper::arena.width / 2) - Helper::arena.corner_radius,
              (Helper::arena.depth / 2) - Helper::arena.corner_radius};
          auto dv = Point2d{point.x, point.z} - corner_o;
          if (length(dv) > Helper::arena.corner_radius - Helper::arena.top_radius) {
            auto n = normalize(dv);
            auto o2 = corner_o + n * (Helper::arena.corner_radius - Helper::arena.top_radius);
            dan = std::min(dan, dan_to_sphere_inner(
                point,
                {o2.x, Helper::arena.height - Helper::arena.top_radius, o2.y},
                Helper::arena.top_radius));
          }
        }
      }
    }
    return dan;
  }

  Dan dan_to_arena(Point point) {
    bool negate_x = point.x < 0;
    bool negate_z = point.z < 0;
    if (negate_x) {
      point.x = -point.x;
    }
    if (negate_z) {
      point.z = -point.z;
    }
    auto result = dan_to_arena_quarter(point);
    if (negate_x) {
      result.normal.x = -result.normal.x;
    }
    if (negate_z) {
      result.normal.z = -result.normal.z;
    }
    return result;
  }


};

#endif //CODEBALL_SIMULATOR_H
