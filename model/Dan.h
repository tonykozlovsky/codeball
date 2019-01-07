#ifndef CODEBALL_DAN_H
#define CODEBALL_DAN_H

#ifdef LOCAL
#include <model/C.h>
#else
#include "C.h"
#endif

struct Dan {
  double distance;
  Point normal;
  bool operator<(const Dan& other) const {
    return distance < other.distance;
  }

  static Dan dan_to_plane(
      const Point& point,
      const Point& point_on_plane,
      const Point& plane_normal) {
    return {(point - point_on_plane).dot(plane_normal), plane_normal};
  }

  static Dan dan_to_sphere_inner(
      const Point& point,
      const Point& sphere_center,
      double sphere_radius) {
    return {sphere_radius - (point - sphere_center).length(),
        (sphere_center - point).normalize()};
  }

  static Dan dan_to_sphere_outer(
      const Point& point,
      const Point& sphere_center,
      double sphere_radius) {
    return {(point - sphere_center).length() - sphere_radius,
        (point - sphere_center).normalize()};
  }

  static Dan dan_to_arena(Point& point, const double radius) {
    const bool negate_x = point.x < 0;
    const bool negate_z = point.z < 0;
    if (negate_x) {
      point.x = -point.x;
    }
    if (negate_z) {
      point.z = -point.z;
    }
    Dan result = dan_to_arena_quarter(point, radius);
    if (negate_x) {
      result.normal.x = -result.normal.x;
      point.x = -point.x;
    }
    if (negate_z) {
      result.normal.z = -result.normal.z;
      point.z = -point.z;
    }
    return result;
  }

  static Dan dan_to_arena_quarter(const Point& point, const double radius) {
    Dan dan = Dan({1e9, {0, 0, 0}});

    // Ground
    // 2.59172e-07 30
    // 2.417e-08 2
    // 1.8985e-07 50
    //H::t[12].start(); // 12
    if (point.y < radius) {
      dan = {point.y, {0, 1, 0}};
      if (point.x < 24 && point.z < 34) {
        //H::t[12].cur(true);
        return dan;
      }
      //std::min(dan, dan_to_plane(point, {0, 0, 0}, {0, 1, 0})); // TODO simplify
    }
    ////if (radius > dan.distance) {
    //  //return dan;
    //}
    //H::t[12].cur(true);




    // Goal back corners
    // 1.48532e-05 30
    // 4.90724e-07 20
    // 47 50
    //H::t[13].start(); // 19
    if (point.z > 47) {
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              std::clamp(
                  point.x,
                  C::rules.arena.bottom_radius - (C::rules.arena.goal_width / 2),
                  (C::rules.arena.goal_width / 2) - C::rules.arena.bottom_radius
              ),
              std::clamp(
                  point.y,
                  C::rules.arena.bottom_radius,
                  C::rules.arena.goal_height - C::rules.arena.goal_top_radius
              ),
              (C::rules.arena.depth / 2) + C::rules.arena.goal_depth - C::rules.arena.bottom_radius},
          C::rules.arena.bottom_radius));
      //if (radius > dan.distance) {
      //H::t[13].cur(true);
      //return dan;
      //}
    }
    //H::t[13].cur(true);


    // Bottom corners 1 part
    // 27 30
    // 2.02544e-07 3
    // 3.46701e-06 50
    //H::t[14].start(); // 26
    if (point.y < 3 && point.x > 27) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              (C::rules.arena.width / 2) - C::rules.arena.bottom_radius,
              C::rules.arena.bottom_radius,
              point.z
          },
          C::rules.arena.bottom_radius));
      //if (radius > dan.distance) {
      //H::t[14].cur(true);
      //return dan;
      //}
    }
    //H::t[14].cur(true);


    // Corner
    // 17 30
    // 1.59766e-06 20
    // 27 50
    //H::t[15].start(); // 20
    if (!(point.z < 27 || point.x < 17)) {
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              (C::rules.arena.width / 2) - C::rules.arena.corner_radius,
              point.y,
              (C::rules.arena.depth / 2) - C::rules.arena.corner_radius
          },
          C::rules.arena.corner_radius));
      //if (radius > dan.distance) {
      //H::t[15].cur(true);
      //return dan;
      //}
    }
    //H::t[15].cur(true);



    // Bottom corners 2 part
    // 12 30
    // 4.90724e-07 3
    // 41 50
    //H::t[16].start(); // 30
    if (point.y < 3 && point.x > 12 && point.z > 41) {
      // Side x (goal)
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              (C::rules.arena.goal_width / 2) - C::rules.arena.bottom_radius,
              C::rules.arena.bottom_radius,
              point.z
          },
          C::rules.arena.bottom_radius));
      //if (radius > dan.distance) {
      //H::t[16].cur(true);
      //return dan;

      //}
    }
    //H::t[16].cur(true);


    // Bottom corners 3 part
    // 17 30
    // 1.59766e-06 3
    // 27.0001 50
    //H::t[32].start(); // 31
    if (point.y < 3 && point.x > 17 && point.z > 27) {
      // Corner
      Point2d corner_o = Point2d(
          (C::rules.arena.width / 2) - C::rules.arena.corner_radius,
          (C::rules.arena.depth / 2) - C::rules.arena.corner_radius
      );
      Point2d n = Point2d(point.x, point.z) - corner_o;
      double dist = n.length();
      if (dist > C::rules.arena.corner_radius - C::rules.arena.bottom_radius) {
        n = n / dist;
        Point2d o2 = corner_o + n * (C::rules.arena.corner_radius - C::rules.arena.bottom_radius);
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {o2.x, C::rules.arena.bottom_radius, o2.y},
            C::rules.arena.bottom_radius));
        //if (radius > dan.distance) {
        //H::t[32].cur(true);
        //return dan;
        //}
      }

    }
    //H::t[32].cur(true);


    // Side x
    // 28 30
    // 1.59766e-06 20
    // 2.05669e-06 50
    //H::t[17].start(); // 14
    if (point.x > 28) {
      dan = std::min(dan, dan_to_plane(point, {C::rules.arena.width / 2, 0, 0}, {-1, 0, 0}));
      //if (radius > dan.distance) {
      //H::t[17].cur(true);
      //return dan;
      //}
    }
    //H::t[17].cur(true);


    // Bottom corners 3 part

    // 12.0009 16
    // 9.86179e-07 2.99999
    // 37.0001 41
    //H::t[18].start(); // 29
    if (point.y < 3 && point.x > 12 && point.x < 16 && point.z > 37 && point.z < 41) {
      // Goal outer corner
      Point2d o = Point2d(
          (C::rules.arena.goal_width / 2) + C::rules.arena.goal_side_radius,
          (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius
      );
      Point2d v = Point2d(point.x, point.z) - o;
      if (v.x < 0 && v.y < 0
          && v.length() < C::rules.arena.goal_side_radius + C::rules.arena.bottom_radius) {
        o = o + v.normalize() * (C::rules.arena.goal_side_radius + C::rules.arena.bottom_radius);
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {o.x, C::rules.arena.bottom_radius, o.y},
            C::rules.arena.bottom_radius));
        //if (radius > dan.distance) {
        //H::t[18].cur(true);
        //return dan;
        //}
      }
    }
    //H::t[18].cur(true);


    // Side x & ceiling (goal) 1 part
    // 13 30
    // 4.90724e-07 20
    // 41 50
    //H::t[19].start(); // 17
    if (point.z >= 41 && point.x > 13) {
      // x
      dan = std::min(dan, dan_to_plane(
          point,
          {C::rules.arena.goal_width / 2, 0, 0},
          {-1, 0, 0}));
      //if (radius > dan.distance) {
      //H::t[19].cur(true);
      //return dan;
      //}
    }
    //H::t[19].cur(true);


    // Bottom corners 4 part
    // 16 30
    // 1.77708e-06 3
    // 37 50
    //H::t[20].start(); // 27
    if (point.y < 3 && point.x > 16 && point.z > 37) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              point.x,
              C::rules.arena.bottom_radius,
              (C::rules.arena.depth / 2) - C::rules.arena.bottom_radius
          },
          C::rules.arena.bottom_radius));
      //if (radius > dan.distance) {
      //H::t[20].cur(true);
      //return dan;
      //}
    }
    //H::t[20].cur(true);


    // Goal inside top corners 1 part
    // 12.0001 30
    // 7 20
    // 41 50
    //H::t[21].start(); // 24
    if (point.z > 41 && point.y > 3 && point.x > 12) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              (C::rules.arena.goal_width / 2) - C::rules.arena.goal_top_radius,
              C::rules.arena.goal_height - C::rules.arena.goal_top_radius,
              point.z
          },
          C::rules.arena.goal_top_radius));
      //if (radius > dan.distance) {
      //H::t[21].cur(true);
      //return dan;
      //}

    }
    //H::t[21].cur(true);



    // Side z
    // 1.08854e-05 30
    // 3.3164e-05 20
    // 38 50
    //H::t[22].start(); // 16
    if (point.z > 38) {
      Point2d v = Point2d(point.x, point.y) - Point2d(
          (C::rules.arena.goal_width / 2) - C::rules.arena.goal_top_radius,
          C::rules.arena.goal_height - C::rules.arena.goal_top_radius);
      if (point.x >= (C::rules.arena.goal_width / 2) + C::rules.arena.goal_side_radius
          || point.y >= C::rules.arena.goal_height + C::rules.arena.goal_side_radius
          || (v.x > 0
              && v.y > 0
              && v.length_sq() >= (C::rules.arena.goal_top_radius + C::rules.arena.goal_side_radius) * (C::rules.arena.goal_top_radius + C::rules.arena.goal_side_radius))) {
        dan = std::min(dan, dan_to_plane(point, {0, 0, C::rules.arena.depth / 2}, {0, 0, -1}));
        //if (radius > dan.distance) {
        //H::t[22].cur(true);
        //return dan;
        //}
      }
    }
    //H::t[22].cur(true);


    // Goal outer corner 1 part
    // 13.0007 16
    // 0.000182224 19.9999
    // 38.0002 41
    //H::t[23].start(); // 21
    if (!(point.z < 38 || point.x < 13 || point.x > 16 || point.z > 41)) {
      // Side x
      dan = std::min(dan, dan_to_sphere_outer(
          point,
          {
              (C::rules.arena.goal_width / 2) + C::rules.arena.goal_side_radius,
              point.y,
              (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius
          },
          C::rules.arena.goal_side_radius));
      //if (radius > dan.distance) {
      //H::t[23].cur(true);
      //return dan;
      //}

    }
    //H::t[23].cur(true);


    // Side x & ceiling (goal) 2 part
    // 1.08854e-05 30
    // 8 20
    // 41 50
    //H::t[24].start(); // 18
    if (point.z >= 41 && point.y > 8) {
      // y
      dan = std::min(dan, dan_to_plane(point, {0, C::rules.arena.goal_height, 0}, {0, -1, 0}));
      //if (radius > dan.distance) {
      //H::t[24].cur(true);
      //return dan;
      //}
    }
    //H::t[24].cur(true);


    // Ceiling corners 1 part
    // 23 30
    // 13 20
    // 7.43039e-05 49.9999
    //H::t[25].start(); // 32
    if (point.y > 13 && point.x > 23) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              (C::rules.arena.width / 2) - C::rules.arena.top_radius,
              C::rules.arena.height - C::rules.arena.top_radius,
              point.z
          },
          C::rules.arena.top_radius));
      //if (radius > dan.distance) {
      //H::t[25].cur(true);
      //return dan;

      //}
    }
    //H::t[25].cur(true);


    // Ceiling
    // 4.36196e-05 30
    // 18 20
    // 3.01327e-06 50
    //H::t[26].start(); // 13
    if (point.y > 18) {
      dan = std::min(dan, dan_to_plane(point, {0, C::rules.arena.height, 0}, {0, -1, 0}));
      //if (radius > dan.distance) {
      //H::t[26].cur(true);
      //return dan;
      //}
    }
    //H::t[26].cur(true);




    // Ceiling corners 2 part
    // 1.08854e-05 30
    // 13 20
    // 33 50
    //H::t[27].start(); // 33
    if (point.y > 13 && point.z > 33) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              point.x,
              C::rules.arena.height - C::rules.arena.top_radius,
              (C::rules.arena.depth / 2) - C::rules.arena.top_radius
          },
          C::rules.arena.top_radius));
      //if (radius > dan.distance) {
      //H::t[27].cur(true);
      //return dan;
      //}

    }
    //H::t[27].cur(true);



    // Ceiling corners 3 part
    // 17 30
    // 13 20
    // 27 50
    //H::t[33].start(); // 34
    if (point.y > 13 && point.x > 17 && point.z > 27) {
      // Corner
      Point2d corner_o = Point2d(
          (C::rules.arena.width / 2) - C::rules.arena.corner_radius,
          (C::rules.arena.depth / 2) - C::rules.arena.corner_radius
      );
      Point2d dv = Point2d(point.x, point.z) - corner_o;
      if (dv.length() > C::rules.arena.corner_radius - C::rules.arena.top_radius) {
        Point2d n = dv.normalize();
        Point2d o2 = corner_o + n * (C::rules.arena.corner_radius - C::rules.arena.top_radius);
        dan = std::min(dan, dan_to_sphere_inner(
            point,
            {o2.x, C::rules.arena.height - C::rules.arena.top_radius, o2.y},
            C::rules.arena.top_radius));
        //if (radius > dan.distance) {
        //H::t[33].cur(true);
        //return dan;
        //}
      }

    }
    //H::t[33].cur(true);



    // Goal outer corner 2 part
    // 5.9563e-05 29.9999
    // 8.00095 11
    // 38.0001 41
    //H::t[28].start(); // 22
    if (!(point.z < 38 || point.y < 8 || point.y > 11 || point.z > 41)) {
      // Ceiling
      dan = std::min(dan, dan_to_sphere_outer(
          point,
          {
              point.x,
              C::rules.arena.goal_height + C::rules.arena.goal_side_radius,
              (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius
          },
          C::rules.arena.goal_side_radius));
      //if (radius > dan.distance) {
      //H::t[28].cur(true);
      //return dan;
      //}

    }
    //H::t[28].cur(true);



    // Goal outer corner 3 part
    // 12 18.9966
    // 7.00001 13.9835
    // 38.0001 41
    //H::t[34].start(); // 23
    if (point.x > 12 && point.x < 19 && point.y > 7 && point.y < 14 && point.z > 38 && point.z < 41) {
      // Top corner
      Point2d o = Point2d(
          (C::rules.arena.goal_width / 2) - C::rules.arena.goal_top_radius,
          C::rules.arena.goal_height - C::rules.arena.goal_top_radius
      );
      Point2d v = Point2d(point.x, point.y) - o;
      if (v.x > 0 && v.y > 0) {
        o = o + v.normalize() * (C::rules.arena.goal_top_radius + C::rules.arena.goal_side_radius);
        dan = std::min(dan, dan_to_sphere_outer(
            point,
            {o.x, o.y, (C::rules.arena.depth / 2) + C::rules.arena.goal_side_radius},
            C::rules.arena.goal_side_radius));
        //if (radius > dan.distance) {
        //H::t[34].cur(true);
        //return dan;
        // }
      }
    }
    //H::t[34].cur(true);


    // Bottom corners 5 part
    // 5.89812e-05 30
    // 9.81447e-07 2.99997
    // 47 50
    //H::t[29].start(); // 28
    if (point.y < 3 && point.z > 47) {
      // Side z (goal)
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              point.x,
              C::rules.arena.bottom_radius,
              (C::rules.arena.depth / 2) + C::rules.arena.goal_depth - C::rules.arena.bottom_radius
          },
          C::rules.arena.bottom_radius));
      //if (radius > dan.distance) {
      //H::t[29].cur(true);
      //return dan;
      //}

    }
    //H::t[29].cur(true);


    // Side z (goal)
    // 1.48532e-05 29.9999
    // 9.81447e-07 20
    // 48 50
    //H::t[30].start(); //15
    if (point.z > 48) {
      dan = std::min(dan, dan_to_plane(
          point,
          {0, 0, (C::rules.arena.depth / 2) + C::rules.arena.goal_depth},
          {0, 0, -1}));
      //if (radius > dan.distance) {
      //H::t[30].cur(true);
      //return dan;
      //}
    }
    //H::t[30].cur(true);


    // Goal inside top corners 2 part
    // 1.48532e-05 29.9999
    // 7.00007 20
    // 47 50
    //H::t[31].start(); //25
    if (point.z > 47 && point.y > 7) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(
          point,
          {
              point.x,
              C::rules.arena.goal_height - C::rules.arena.goal_top_radius,
              (C::rules.arena.depth / 2) + C::rules.arena.goal_depth - C::rules.arena.goal_top_radius
          },
          C::rules.arena.goal_top_radius));
      //if (radius > dan.distance) {
      //H::t[31].cur(true);
      //return dan;
      //}

    }
    //H::t[31].cur(true);
    return dan;
  }

};

#endif //CODEBALL_DAN_H
