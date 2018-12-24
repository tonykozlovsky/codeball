#ifndef CODEBALL_SIMULATOR_H
#define CODEBALL_SIMULATOR_H

#ifdef LOCAL
#include <model/Entity.h>
#include <model/Painter.h>
#else
#include "model/Entity.h"
#include "model/Painter.h"
#endif

struct Simulator {

  struct Dan {
    double distance;
    Point normal;
    bool operator<(const Dan& other) const {
      return distance < other.distance;
    }
  };

  std::vector<Entity> robots;
  Entity ball;
  bool my_goal = false;
  bool enemy_goal = false;
  bool collide_with_ball[10];

  double getScoreFighter() {
    if (my_goal) {
      return 1e9;
    }
    if (enemy_goal) {
      return -1e9;
    }
    return -(ball.position - Point{0, 0,
        Constants::rules.arena.depth / 2
        + Constants::rules.arena.goal_depth}).length();
  }

  double getScoreDefender() {
      if (enemy_goal) {
        return -1e9;
      }
      double score = 0;
      for (auto& robot : robots) {
        if (robot.is_teammate && robot.global_id % 2 == 1) {
          score += -(robot.position - Point{
              0, 0, -Constants::rules.arena.depth / 2}).length();
        }
      }
      score += -2 * std::min(ball.position.z, 0.) * std::min(ball.position.z, 0.);
      return score;
    }

  Simulator() {}

  Simulator(const std::vector<model::Robot>& _robots, const model::Ball& _ball) {
    for (auto& robot : _robots) {
      robots.push_back(Entity(robot));
      collide_with_ball[robot.id] = false;
    }
    ball = Entity(_ball);
    update_trace();
  }

  void print_velocity(const Point& p) {
    std::cerr << "velocity: " << std::fixed << std::setprecision(30) << p.length() << std::endl;
    std::cerr << "x: " << std::fixed << std::setprecision(30) << p.x << std::endl;
    std::cerr << "y: " << std::fixed << std::setprecision(30) << p.y << std::endl;
    std::cerr << "z: " << std::fixed << std::setprecision(30) << p.z << std::endl;
  }

  void print_error(const Point& p1, const Point& p2) {
    std::cerr << "d: " << std::fixed << std::setprecision(30) << (p1 - p2).length() << std::endl;
    std::cerr << "x: " << std::fixed << std::setprecision(30) << fabs(p1.x - p2.x) << std::endl;
    std::cerr << "y: " << std::fixed << std::setprecision(30) << fabs(p1.y - p2.y) << std::endl;
    std::cerr << "z: " << std::fixed << std::setprecision(30) << fabs(p1.z - p2.z) << std::endl;
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

  bool collide_entities(Entity& a, Entity& b) {
    Point delta_position = b.position - a.position;
    double distance = length(delta_position);
    double penetration = a.radius + b.radius - distance;
    if (penetration > 0) {
      double k_a = (1. / a.mass) / ((1. / a.mass) + (1. / b.mass));
      double k_b = (1. / b.mass) / ((1. / a.mass) + (1. / b.mass));
      Point normal = normalize(delta_position);
      a.position -= normal * penetration * k_a;
      b.position += normal * penetration * k_b;
      double delta_velocity = dot(b.velocity - a.velocity, normal)
          + b.radius_change_speed - a.radius_change_speed;
      if (delta_velocity < 0) {
        Point impulse = normal * (1. + (Constants::rules.MAX_HIT_E + Constants::rules.MIN_HIT_E) / 2.) * delta_velocity;
        a.velocity += impulse * k_a;
        b.velocity -= impulse * k_b;
        return true;
      }
    }
    return false;
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
    e.velocity = clamp(e.velocity, Constants::rules.MAX_ENTITY_SPEED);
    e.position += e.velocity * delta_time;
    e.position.y -= Constants::rules.GRAVITY * delta_time * delta_time / 2;
    e.velocity.y -= Constants::rules.GRAVITY * delta_time;
  }

  void update(const double delta_time) {
    for (auto& robot : robots) {
      if (robot.touch) {
        Point target_velocity = clamp(
            robot.action.target_velocity,
            Constants::rules.ROBOT_MAX_GROUND_SPEED);
        target_velocity -= robot.touch_normal * robot.touch_normal.dot(target_velocity);
        Point target_velocity_change = target_velocity - robot.velocity;
        if (length(target_velocity_change) > 0) {
          double acceleration = Constants::rules.ROBOT_ACCELERATION * fmax(0., robot.touch_normal.y);
          robot.velocity += clamp(
              normalize(target_velocity_change) * acceleration * delta_time,
              length(target_velocity_change));
        }
      }
      move(robot, delta_time);
      robot.radius = Constants::rules.ROBOT_MIN_RADIUS
          + (Constants::rules.ROBOT_MAX_RADIUS - Constants::rules.ROBOT_MIN_RADIUS)
              * robot.action.jump_speed / Constants::rules.ROBOT_MAX_JUMP_SPEED;
      robot.radius_change_speed = robot.action.jump_speed;
    }
    move(ball, delta_time);
    Point collision_normal;
    for (int i = 0; i < robots.size(); i++) {
      for (int j = 0; j < i; j++) {
        collide_entities(robots[i], robots[j]);
      }
    }
    for (auto& robot : robots) {
      if (collide_entities(robot, ball) && !robot.touch) {
        collide_with_ball[robot.global_id] = true;
      }
      if (!collide_with_arena(robot, collision_normal)) {
        robot.touch = false;
      } else {
        robot.touch = true;
        robot.touch_normal = collision_normal;
      }
    }

    collide_with_arena(ball, collision_normal);

    if (!my_goal && !enemy_goal) {
      if (ball.position.z > Constants::rules.arena.depth / 2 + ball.radius) {
        my_goal = true;
      } else if (-ball.position.z > Constants::rules.arena.depth / 2 + ball.radius) {
        enemy_goal = true;
      }
    }
  }

  void update_trace() {
    for (auto& robot : robots) {
      robot.trace.push_back(robot.position);
    }
    ball.trace.push_back(ball.position);
  }

  void tick() {
    double delta_time = 1. / Constants::rules.TICKS_PER_SECOND;
    //for (int i = 0; i < Constants::rules.MICROTICKS_PER_TICK; i++) {
      //update(delta_time / Constants::rules.MICROTICKS_PER_TICK);
    //}
    update(delta_time);
    update_trace();
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

  Dan dan_to_arena_quarter(Point point) {
    // Ground
    Dan dan = dan_to_plane(point, Point(0, 0, 0), Point(0, 1, 0));

    // Ceiling
    dan = std::min(dan, dan_to_plane(point, Point(0, Constants::rules.arena.height, 0), Point(0, -1, 0)));

    // Side x
    dan = std::min(dan, dan_to_plane(point, Point(Constants::rules.arena.width / 2, 0, 0), Point(-1, 0, 0)));

    // Side z (goal)
    dan = std::min(dan, dan_to_plane(
        point,
        Point(0, 0, (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_depth),
        Point(0, 0, -1)));

    // Side z
    Point2d v = Point2d(point.x, point.y) - Point2d(
        (Constants::rules.arena.goal_width / 2) - Constants::rules.arena.goal_top_radius,
        Constants::rules.arena.goal_height - Constants::rules.arena.goal_top_radius);
    if (point.x >= (Constants::rules.arena.goal_width / 2) + Constants::rules.arena.goal_side_radius
        || point.y >= Constants::rules.arena.goal_height + Constants::rules.arena.goal_side_radius
        || (
            v.x > 0
                && v.y > 0
                && length(v) >= Constants::rules.arena.goal_top_radius + Constants::rules.arena.goal_side_radius)) {
      dan = std::min(dan, dan_to_plane(point, Point(0, 0, Constants::rules.arena.depth / 2), Point(0, 0, -1)));
    }
    // Side x & ceiling (goal)
    if (point.z >= (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_side_radius) {
      // x
      dan = std::min(dan, dan_to_plane(
          point,
          Point(Constants::rules.arena.goal_width / 2, 0, 0),
          Point(-1, 0, 0)));
      // y
      dan = std::min(dan, dan_to_plane(point, Point(0, Constants::rules.arena.goal_height, 0), Point(0, -1, 0)));
    }

    // Goal back corners
    if (point.z > (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_depth - Constants::rules.arena.bottom_radius) {
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          Point(
              std::clamp(
                  point.x,
                  Constants::rules.arena.bottom_radius - (Constants::rules.arena.goal_width / 2),
                  (Constants::rules.arena.goal_width / 2) - Constants::rules.arena.bottom_radius
              ),
              std::clamp(
                  point.y,
                  Constants::rules.arena.bottom_radius,
                  Constants::rules.arena.goal_height - Constants::rules.arena.goal_top_radius
              ),
              (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_depth - Constants::rules.arena.bottom_radius),
          Constants::rules.arena.bottom_radius));
    }
    // Corner
    if (point.x > (Constants::rules.arena.width / 2) - Constants::rules.arena.corner_radius
        && point.z > (Constants::rules.arena.depth / 2) - Constants::rules.arena.corner_radius) {
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          Point(
              (Constants::rules.arena.width / 2) - Constants::rules.arena.corner_radius,
              point.y,
              (Constants::rules.arena.depth / 2) - Constants::rules.arena.corner_radius
          ),
          Constants::rules.arena.corner_radius));
    }
    // Goal outer corner
    if (point.z < (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_side_radius) {
      // Side x
      if (point.x < (Constants::rules.arena.goal_width / 2) + Constants::rules.arena.goal_side_radius) {
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            Point(
                (Constants::rules.arena.goal_width / 2) + Constants::rules.arena.goal_side_radius,
                point.y,
                (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_side_radius
            ),
            Constants::rules.arena.goal_side_radius));
      }
      // Ceiling
      if (point.y < Constants::rules.arena.goal_height + Constants::rules.arena.goal_side_radius) {
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            Point(
                point.x,
                Constants::rules.arena.goal_height + Constants::rules.arena.goal_side_radius,
                (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_side_radius
            ),
            Constants::rules.arena.goal_side_radius));
      }
      // Top corner
      Point2d o = Point2d(
          (Constants::rules.arena.goal_width / 2) - Constants::rules.arena.goal_top_radius,
          Constants::rules.arena.goal_height - Constants::rules.arena.goal_top_radius
      );
      Point2d v = Point2d(point.x, point.y) - o;
      if (v.x > 0 && v.y > 0) {
        o = o + normalize(v) * (Constants::rules.arena.goal_top_radius + Constants::rules.arena.goal_side_radius);
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            Point(o.x, o.y, (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_side_radius),
            Constants::rules.arena.goal_side_radius));
      }
    }
    // Goal inside top corners
    if (point.z > (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_side_radius
        && point.y > Constants::rules.arena.goal_height - Constants::rules.arena.goal_top_radius) {
      // Side x
      if (point.x > (Constants::rules.arena.goal_width / 2) - Constants::rules.arena.goal_top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                (Constants::rules.arena.goal_width / 2) - Constants::rules.arena.goal_top_radius,
                Constants::rules.arena.goal_height - Constants::rules.arena.goal_top_radius,
                point.z
            ),
            Constants::rules.arena.goal_top_radius));
      }
      // Side z
      if (point.z > (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_depth - Constants::rules.arena.goal_top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                point.x,
                Constants::rules.arena.goal_height - Constants::rules.arena.goal_top_radius,
                (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_depth - Constants::rules.arena.goal_top_radius
            ),
            Constants::rules.arena.goal_top_radius));
      }
    }

    // Bottom corners
    if (point.y < Constants::rules.arena.bottom_radius) {
      // Side x
      if (point.x > (Constants::rules.arena.width / 2) - Constants::rules.arena.bottom_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                (Constants::rules.arena.width / 2) - Constants::rules.arena.bottom_radius,
                Constants::rules.arena.bottom_radius,
                point.z
            ),
            Constants::rules.arena.bottom_radius));
      }
      // Side z
      if (point.z > (Constants::rules.arena.depth / 2) - Constants::rules.arena.bottom_radius
          && point.x >= (Constants::rules.arena.goal_width / 2) + Constants::rules.arena.goal_side_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                point.x,
                Constants::rules.arena.bottom_radius,
                (Constants::rules.arena.depth / 2) - Constants::rules.arena.bottom_radius
            ),
            Constants::rules.arena.bottom_radius));
      }
      // Side z (goal)
      if (point.z > (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_depth - Constants::rules.arena.bottom_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                point.x,
                Constants::rules.arena.bottom_radius,
                (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_depth - Constants::rules.arena.bottom_radius
            ),
            Constants::rules.arena.bottom_radius));
      }
      // Goal outer corner
      Point2d o = Point2d(
          (Constants::rules.arena.goal_width / 2) + Constants::rules.arena.goal_side_radius,
          (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_side_radius
      );
      Point2d v = Point2d(point.x, point.z) - o;
      if (v.x < 0 && v.y < 0
          && length(v) < Constants::rules.arena.goal_side_radius + Constants::rules.arena.bottom_radius) {
        o = o + normalize(v) * (Constants::rules.arena.goal_side_radius + Constants::rules.arena.bottom_radius);
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(o.x, Constants::rules.arena.bottom_radius, o.y),
            Constants::rules.arena.bottom_radius));
      }
      // Side x (goal)
      if (point.z >= (Constants::rules.arena.depth / 2) + Constants::rules.arena.goal_side_radius
          && point.x > (Constants::rules.arena.goal_width / 2) - Constants::rules.arena.bottom_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                (Constants::rules.arena.goal_width / 2) - Constants::rules.arena.bottom_radius,
                Constants::rules.arena.bottom_radius,
                point.z
            ),
            Constants::rules.arena.bottom_radius));
      }
      // Corner
      if (point.x > (Constants::rules.arena.width / 2) - Constants::rules.arena.corner_radius
          && point.z > (Constants::rules.arena.depth / 2) - Constants::rules.arena.corner_radius) {
        Point2d corner_o = Point2d(
            (Constants::rules.arena.width / 2) - Constants::rules.arena.corner_radius,
            (Constants::rules.arena.depth / 2) - Constants::rules.arena.corner_radius
        );
        Point2d n = Point2d(point.x, point.z) - corner_o;
        double dist = n.length();
        if (dist > Constants::rules.arena.corner_radius - Constants::rules.arena.bottom_radius) {
          n = n / dist;
          Point2d o2 = corner_o + n * (Constants::rules.arena.corner_radius - Constants::rules.arena.bottom_radius);
          dan = std::min(dan, dan_to_sphere_inner(
              point,
              Point(o2.x, Constants::rules.arena.bottom_radius, o2.y),
              Constants::rules.arena.bottom_radius));
        }
      }
    }

    // Ceiling corners
    if (point.y > Constants::rules.arena.height - Constants::rules.arena.top_radius) {
      // Side x
      if (point.x > (Constants::rules.arena.width / 2) - Constants::rules.arena.top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                (Constants::rules.arena.width / 2) - Constants::rules.arena.top_radius,
                Constants::rules.arena.height - Constants::rules.arena.top_radius,
                point.z
            ),
            Constants::rules.arena.top_radius));
      }
      // Side z
      if (point.z > (Constants::rules.arena.depth / 2) - Constants::rules.arena.top_radius) {
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            Point(
                point.x,
                Constants::rules.arena.height - Constants::rules.arena.top_radius,
                (Constants::rules.arena.depth / 2) - Constants::rules.arena.top_radius
            ),
            Constants::rules.arena.top_radius));
      }

      // Corner
      if (point.x > (Constants::rules.arena.width / 2) - Constants::rules.arena.corner_radius
          && point.z > (Constants::rules.arena.depth / 2) - Constants::rules.arena.corner_radius) {
        Point2d corner_o = Point2d(
            (Constants::rules.arena.width / 2) - Constants::rules.arena.corner_radius,
            (Constants::rules.arena.depth / 2) - Constants::rules.arena.corner_radius
        );
        Point2d dv = Point2d(point.x, point.z) - corner_o;
        if (length(dv) > Constants::rules.arena.corner_radius - Constants::rules.arena.top_radius) {
          Point2d n = normalize(dv);
          Point2d o2 = corner_o + n * (Constants::rules.arena.corner_radius - Constants::rules.arena.top_radius);
          dan = std::min(dan, dan_to_sphere_inner(
              point,
              Point(o2.x, Constants::rules.arena.height - Constants::rules.arena.top_radius, o2.y),
              Constants::rules.arena.top_radius));
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

  void test() {
    { // test 1
      ball = Entity("ball");
      ball.position = {-3.094469, 14.393752, -37.612641};
      ball.velocity = {1.741709, 5.221515, -24.657713};
      tick();
      std::cerr << "########## Test 1:" << std::endl;
      std::cerr << "Position:" << std::endl;
      print_error(ball.position, Point{-3.065441, 14.362125, -37.640170});
      std::cerr << "Velocity:" << std::endl;
      print_error(ball.velocity, Point{1.741709, -7.465922, 16.160170});
    }
    { // test 2
      ball = Entity("ball");
      ball.position = {-23.019338622046326748, 17.999962430128810809, -19.14657426491466552};
      ball.velocity = {14.682921642666871165, 0.058461879134667917024, -1.4818202142470027205};
      tick();
      std::cerr << "########## Test 2:" << std::endl;
      std::cerr << "Position:" << std::endl;
      print_error(ball.position,
                  Point{-22.774622648706465355, 17.996454426285627193, -19.171271268485416073});
      std::cerr << "Velocity:" << std::endl;
      print_error(ball.velocity,
                  Point{14.682959226403681896, -0.46121522321528829469, -1.4818202142470027205});
    }
    { // test 3
      ball = Entity("ball");
      ball.position = Point{16.329517921537998859, 16.195591242457055614, -36.845542433926816273};
      ball.velocity = Point{-25.283203469330487678, 7.6680203103518476127, 6.3722070924858815744};

      tick();
      std::cerr << "########## Test 3:" << std::endl;
      std::cerr << "Position:" << std::endl;
      print_error(ball.position,
                  Point{15.908131197049071304, 16.319208932268558954, -36.739319891201859036});
      std::cerr << "Velocity:" << std::endl;
      print_error(ball.velocity,
                  Point{-25.283203469330487678, 7.1669386313658574039, 6.3734987090127770415});

    }
    { // test 4
      ball = Entity("ball");
      ball.position = Point{-27.995519339371629286, 2.9054418436248079516, 6.1702947673222912073};
      ball.velocity = Point{-0.57403220611490801684, 6.0725454135943017775, -12.125100730674212457};
      tick();
      std::cerr << "########## Test 4:" << std::endl;
      std::cerr << "Position:" << std::endl;
      print_error(ball.position,
                  Point{-27.999998915097581431, 3.0028025199069756646, 5.9682097551443806793});
      std::cerr << "Velocity:" << std::endl;
      print_error(ball.velocity,
                  Point{0.0021698048401537694749, 5.600275987957815893, -12.125100730674212457});

    }

  }

};

#endif //CODEBALL_SIMULATOR_H
