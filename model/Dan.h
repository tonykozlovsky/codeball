#ifndef CODEBALL_DAN_H
#define CODEBALL_DAN_H

#ifdef LOCAL
#include <H.h>
#else
#include "../H.h"
#endif


struct Dan {
  double distance;
  Point normal;
  int collision_surface_id;
  bool operator<(const Dan& other) const {
    return distance < other.distance;
  }

  static Dan dan_to_plane(
      const Point& point,
      const Point& point_on_plane,
      const Point& plane_normal,
      const int collision_surface_id) {
    return {(point - point_on_plane).dot(plane_normal), plane_normal, collision_surface_id};
  }

  static Dan dan_to_sphere_inner(
      const double radius,
      const Point& point,
      const Point& sphere_center,
      const double sphere_radius,
      const int collision_surface_id) {
    const double length_sq = (point - sphere_center).length_sq();
    if ((sphere_radius - radius) * (sphere_radius - radius) > length_sq) {
      return {1e9, {0, 0, 0}, 0};
    }
    return {sphere_radius - sqrt(length_sq), sphere_center - point, collision_surface_id};
  }

  static Dan dan_to_sphere_outer(
      const double radius,
      const Point& point,
      const Point& sphere_center,
      const double sphere_radius,
      const int collision_surface_id) {
    const double length_sq = (point - sphere_center).length_sq();
    if (length_sq > (radius + sphere_radius) * (radius + sphere_radius)) {
      return {1e9, {0, 0, 0}, 0};
    }
    return {sqrt(length_sq) - sphere_radius, point - sphere_center, collision_surface_id};
  }

  static Dan dan_to_arena(Point& point, const double radius) {
    const bool& negate_x = point.x < 0;
    const bool& negate_z = point.z < 0;
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

  inline static double my_clamp(const double x, const double a, const double b) {
    if (x < a) {
      return a;
    }
    if (x > b) {
      return b;
    }
    return x;
  }

  static Dan dan_to_arena_quarter(const Point& point, const double radius) {
    //H::t[11].start();
    Dan dan = Dan({1e9, {0, 0, 0}});
    //H::t[11].cur(true);

    // Ground
    // 2.59172e-07 30
    // 2.417e-08 2
    // 1.8985e-07 50
    //H::t[12].start(); // 12
    if (point.y < radius) {
      dan.distance = point.y;
      dan.normal.y = 1;
      dan.collision_surface_id = 1;
      if (point.x < 24 && point.z < 34) {
        //H::t[12].cur(true);
        return dan;
      }
    }


    //H::t[12].cur(true);

    // Side x
    // 28 30
    // 1.59766e-06 20
    // 2.05669e-06 50
    //H::t[17].start(); // 14
    if (point.x > 28) {
      dan = std::min(dan, dan_to_plane(point, {30., 0, 0}, {-1, 0, 0}, 2));
    }
    //H::t[17].cur(true);

    // Goal back corners
    // 1.48532e-05 30
    // 4.90724e-07 20
    // 47 50
    //H::t[13].start(); // 19
    if (point.z > 47) {
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  my_clamp(
                                                      point.x,
                                                      -12.,
                                                      12.
                                                  ),
                                                  my_clamp(
                                                      point.y,
                                                      3.,
                                                      7.
                                                  ),
                                                  47.},
                                              3., 3));
    }
    //H::t[13].cur(true);


    // Bottom corners 1 part
    // 27 30
    // 2.02544e-07 3
    // 3.46701e-06 50
    //H::t[14].start(); // 26
    if (point.y < 3 && point.x > 27) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  27.,
                                                  3.,
                                                  point.z
                                              },
                                              3., 4));
    }
    //H::t[14].cur(true);

    // Corner
    // 17 30
    // 1.59766e-06 20
    // 27 50
    //H::t[15].start(); // 20
    if (point.z > 27 && point.x > 17) {
      dan = std::min(dan, dan_to_sphere_inner(
          radius,
          point,
          {
              17.,
              point.y,
              27.
          },
          13., 5));
    }
    //H::t[15].cur(true);

    // Bottom corners 2 part
    // 12 30
    // 4.90724e-07 3
    // 41 50
    //H::t[16].start(); // 30
    if (point.y < 3 && point.x > 12 && point.z > 41) {
      // Side x (goal)
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  12.,
                                                  3.,
                                                  point.z
                                              },
                                              3., 6));
    }
    //H::t[16].cur(true);


    // Bottom corners 3 part
    // 17 30
    // 1.59766e-06 3
    // 27.0001 50
    //H::t[32].start(); // 31
    if (point.y < 3 && point.x > 17 && point.z > 27) {
      // Corner
      const Point2d& corner_o{17., 27.};
      const Point2d& n = Point2d{point.x, point.z} - corner_o;
      if (n.length_sq() > 100.) {
        const Point2d& o_ = corner_o + n.normalize() * 10.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 3., o_.y},
                                                3., 7));
      }

    }
    //H::t[32].cur(true);




    // Bottom corners 3 part

    // 12.0009 16
    // 9.86179e-07 2.99999
    // 37.0001 41
    //H::t[18].start(); // 29
    if (point.y < 3 && point.x > 12 && point.x < 16 && point.z > 37 && point.z < 41) {
      // Goal outer corner
      const Point2d& o{16., 41.};
      const Point2d& v = Point2d{point.x, point.z} - o;
      if (v.x < 0 && v.y < 0
          && v.length_sq() < 16.) {
        const Point2d& o_ = o + v.normalize() * 4.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 3., o_.y},
                                                3., 8));
      }
    }
    //H::t[18].cur(true);


    // Side x & ceiling (goal) 1 part
    // 13 30
    // 4.90724e-07 20
    // 41 50
    //H::t[19].start(); // 17
    if (point.z > 41 && point.x > 13) {
      // x
      dan = std::min(dan, dan_to_plane(
          point,
          {15., 0, 0},
          {-1, 0, 0}, 9));
    }
    //H::t[19].cur(true);


    // Bottom corners 4 part
    // 16 30
    // 1.77708e-06 3
    // 37 50
    //H::t[20].start(); // 27
    if (point.y < 3 && point.x > 16 && point.z > 37) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  3.,
                                                  37.
                                              },
                                              3., 10));
    }
    //H::t[20].cur(true);


    // Goal inside top corners 1 part
    // 12.0001 30
    // 7 20
    // 41 50
    //H::t[21].start(); // 24
    if (point.z > 41 && point.y > 7 && point.x > 12) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  12.,
                                                  7.,
                                                  point.z
                                              },
                                              3., 11));

    }
    //H::t[21].cur(true);



    // Side z
    // 1.08854e-05 30
    // 3.3164e-05 20
    // 38 50
    //H::t[22].start(); // 16
    if (point.z > 38) {
      const Point2d& v = Point2d{point.x, point.y} - Point2d{12., 7.};
      if (point.x >= 16.
          || point.y >= 11.
          || (v.x > 0
              && v.y > 0
              && v.length_sq() >= 16.)) {
        dan = std::min(dan, dan_to_plane(point, {0, 0, 40.}, {0, 0, -1}, 12));
      }
    }
    //H::t[22].cur(true);


    // Goal outer corner 1 part
    // 13.0007 16
    // 0.000182224 19.9999
    // 38.0002 41
    //H::t[23].start(); // 21
    if (point.x > 13 && point.x < 16 && point.z > 38 && point.z < 41) {
      // Side x
      dan = std::min(dan, dan_to_sphere_outer(radius,
                                              point,
                                              {
                                                  16.,
                                                  point.y,
                                                  41.
                                              },
                                              1., 13));

    }
    //H::t[23].cur(true);


    // Side x & ceiling (goal) 2 part
    // 1.08854e-05 30
    // 8 20
    // 41 50
    //H::t[24].start(); // 18
    if (point.z > 41 && point.y > 8) {
      // y
      dan = std::min(dan, dan_to_plane(point, {0, 10., 0}, {0, -1, 0}, 14));
    }
    //H::t[24].cur(true);


    // Ceiling corners 1 part
    // 23 30
    // 13 20
    // 7.43039e-05 49.9999
    //H::t[25].start(); // 32
    if (point.y > 13 && point.x > 23) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  23.,
                                                  13.,
                                                  point.z
                                              },
                                              7., 12));
    }
    //H::t[25].cur(true);


    // Ceiling
    // 4.36196e-05 30
    // 18 20
    // 3.01327e-06 50
    //H::t[26].start(); // 13
    if (point.y > 18) {
      dan = std::min(dan, dan_to_plane(point, {0, 20., 0}, {0, -1, 0}, 15));
    }
    //H::t[26].cur(true);




    // Ceiling corners 2 part
    // 1.08854e-05 30
    // 13 20
    // 33 50
    //H::t[27].start(); // 33
    if (point.y > 13 && point.z > 33) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  13.,
                                                  33.
                                              },
                                              7., 16));

    }
    //H::t[27].cur(true);



    // Ceiling corners 3 part
    // 17 30
    // 13 20
    // 27 50
    //H::t[33].start(); // 34
    if (point.y > 13 && point.x > 17 && point.z > 27) {
      // Corner
      const Point2d& corner_o{17., 27.};
      const Point2d& dv = Point2d{point.x, point.z} - corner_o;
      if (dv.length_sq() > 36.) {
        const Point2d& o_ = corner_o + dv.normalize() * 6.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 13., o_.y},
                                                7., 17));
      }

    }
    //H::t[33].cur(true);



    // Goal outer corner 2 part
    // 5.9563e-05 29.9999
    // 8.00095 11
    // 38.0001 41
    //H::t[28].start(); // 22
    if (point.z > 38 && point.y > 8 && point.y < 11 && point.z < 41) {
      // Ceiling
      dan = std::min(dan, dan_to_sphere_outer(radius,
                                              point,
                                              {
                                                  point.x,
                                                  11.,
                                                  41.
                                              },
                                              1., 18));
    }
    //H::t[28].cur(true);



    // Goal outer corner 3 part
    // 12 18.9966
    // 7.00001 13.9835
    // 38.0001 41
    //H::t[34].start(); // 23
    if (point.x > 12 && point.x < 19 && point.y > 7 && point.y < 14 && point.z > 38 && point.z < 41) {
      // Top corner
      const Point2d& o = Point2d{12., 7.};
      const Point2d& v = Point2d{point.x, point.y} - o;
      if (v.x > 0 && v.y > 0) {
        const Point2d& o_ = o + v.normalize() * 4.;
        dan = std::min(dan, dan_to_sphere_outer(radius,
                                                point,
                                                {o_.x, o_.y, 41.},
                                                1., 19));
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
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  3.,
                                                  47.
                                              },
                                              3., 20));
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
          {0, 0, 50.},
          {0, 0, -1}, 21));
    }
    //H::t[30].cur(true);


    // Goal inside top corners 2 part
    // 1.48532e-05 29.9999
    // 7.00007 20
    // 47 50
    //H::t[31].start(); //25
    if (point.z > 47 && point.y > 7) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  7.,
                                                  47.
                                              },
                                              3., 22));
    }
    //H::t[31].cur(true);
    return dan;
  }

};

#ifndef LOCAL
namespace Frozen {

struct Dan {
  double distance;
  Point normal;
  int collision_surface_id;
  bool operator<(const Dan& other) const {
    return distance < other.distance;
  }

  static Dan dan_to_plane(
      const Point& point,
      const Point& point_on_plane,
      const Point& plane_normal,
      const int collision_surface_id) {
    return {(point - point_on_plane).dot(plane_normal), plane_normal, collision_surface_id};
  }

  static Dan dan_to_sphere_inner(
      const double radius,
      const Point& point,
      const Point& sphere_center,
      const double sphere_radius,
      const int collision_surface_id) {
    const double length_sq = (point - sphere_center).length_sq();
    if ((sphere_radius - radius) * (sphere_radius - radius) > length_sq) {
      return {1e9, {0, 0, 0}, 0};
    }
    return {sphere_radius - sqrt(length_sq), sphere_center - point, collision_surface_id};
  }

  static Dan dan_to_sphere_outer(
      const double radius,
      const Point& point,
      const Point& sphere_center,
      const double sphere_radius,
      const int collision_surface_id) {
    const double length_sq = (point - sphere_center).length_sq();
    if (length_sq > (radius + sphere_radius) * (radius + sphere_radius)) {
      return {1e9, {0, 0, 0}, 0};
    }
    return {sqrt(length_sq) - sphere_radius, point - sphere_center, collision_surface_id};
  }

  static Dan dan_to_arena(Point& point, const double radius) {
    const bool& negate_x = point.x < 0;
    const bool& negate_z = point.z < 0;
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

  inline static double my_clamp(const double x, const double a, const double b) {
    if (x < a) {
      return a;
    }
    if (x > b) {
      return b;
    }
    return x;
  }

  static Dan dan_to_arena_quarter(const Point& point, const double radius) {
    //H::t[11].start();
    Dan dan = Dan({1e9, {0, 0, 0}});
    //H::t[11].cur(true);

    // Ground
    // 2.59172e-07 30
    // 2.417e-08 2
    // 1.8985e-07 50
    //H::t[12].start(); // 12
    if (point.y < radius) {
      dan.distance = point.y;
      dan.normal.y = 1;
      dan.collision_surface_id = 1;
      if (point.x < 24 && point.z < 34) {
        //H::t[12].cur(true);
        return dan;
      }
    }


    //H::t[12].cur(true);

    // Side x
    // 28 30
    // 1.59766e-06 20
    // 2.05669e-06 50
    //H::t[17].start(); // 14
    if (point.x > 28) {
      dan = std::min(dan, dan_to_plane(point, {30., 0, 0}, {-1, 0, 0}, 2));
    }
    //H::t[17].cur(true);

    // Goal back corners
    // 1.48532e-05 30
    // 4.90724e-07 20
    // 47 50
    //H::t[13].start(); // 19
    if (point.z > 47) {
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  my_clamp(
                                                      point.x,
                                                      -12.,
                                                      12.
                                                  ),
                                                  my_clamp(
                                                      point.y,
                                                      3.,
                                                      7.
                                                  ),
                                                  47.},
                                              3., 3));
    }
    //H::t[13].cur(true);


    // Bottom corners 1 part
    // 27 30
    // 2.02544e-07 3
    // 3.46701e-06 50
    //H::t[14].start(); // 26
    if (point.y < 3 && point.x > 27) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  27.,
                                                  3.,
                                                  point.z
                                              },
                                              3., 4));
    }
    //H::t[14].cur(true);

    // Corner
    // 17 30
    // 1.59766e-06 20
    // 27 50
    //H::t[15].start(); // 20
    if (point.z > 27 && point.x > 17) {
      dan = std::min(dan, dan_to_sphere_inner(
          radius,
          point,
          {
              17.,
              point.y,
              27.
          },
          13., 5));
    }
    //H::t[15].cur(true);

    // Bottom corners 2 part
    // 12 30
    // 4.90724e-07 3
    // 41 50
    //H::t[16].start(); // 30
    if (point.y < 3 && point.x > 12 && point.z > 41) {
      // Side x (goal)
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  12.,
                                                  3.,
                                                  point.z
                                              },
                                              3., 6));
    }
    //H::t[16].cur(true);


    // Bottom corners 3 part
    // 17 30
    // 1.59766e-06 3
    // 27.0001 50
    //H::t[32].start(); // 31
    if (point.y < 3 && point.x > 17 && point.z > 27) {
      // Corner
      const Point2d& corner_o{17., 27.};
      const Point2d& n = Point2d{point.x, point.z} - corner_o;
      if (n.length_sq() > 100.) {
        const Point2d& o_ = corner_o + n.normalize() * 10.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 3., o_.y},
                                                3., 7));
      }

    }
    //H::t[32].cur(true);




    // Bottom corners 3 part

    // 12.0009 16
    // 9.86179e-07 2.99999
    // 37.0001 41
    //H::t[18].start(); // 29
    if (point.y < 3 && point.x > 12 && point.x < 16 && point.z > 37 && point.z < 41) {
      // Goal outer corner
      const Point2d& o{16., 41.};
      const Point2d& v = Point2d{point.x, point.z} - o;
      if (v.x < 0 && v.y < 0
          && v.length_sq() < 16.) {
        const Point2d& o_ = o + v.normalize() * 4.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 3., o_.y},
                                                3., 8));
      }
    }
    //H::t[18].cur(true);


    // Side x & ceiling (goal) 1 part
    // 13 30
    // 4.90724e-07 20
    // 41 50
    //H::t[19].start(); // 17
    if (point.z > 41 && point.x > 13) {
      // x
      dan = std::min(dan, dan_to_plane(
          point,
          {15., 0, 0},
          {-1, 0, 0}, 9));
    }
    //H::t[19].cur(true);


    // Bottom corners 4 part
    // 16 30
    // 1.77708e-06 3
    // 37 50
    //H::t[20].start(); // 27
    if (point.y < 3 && point.x > 16 && point.z > 37) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  3.,
                                                  37.
                                              },
                                              3., 10));
    }
    //H::t[20].cur(true);


    // Goal inside top corners 1 part
    // 12.0001 30
    // 7 20
    // 41 50
    //H::t[21].start(); // 24
    if (point.z > 41 && point.y > 7 && point.x > 12) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  12.,
                                                  7.,
                                                  point.z
                                              },
                                              3., 11));

    }
    //H::t[21].cur(true);



    // Side z
    // 1.08854e-05 30
    // 3.3164e-05 20
    // 38 50
    //H::t[22].start(); // 16
    if (point.z > 38) {
      const Point2d& v = Point2d{point.x, point.y} - Point2d{12., 7.};
      if (point.x >= 16.
          || point.y >= 11.
          || (v.x > 0
              && v.y > 0
              && v.length_sq() >= 16.)) {
        dan = std::min(dan, dan_to_plane(point, {0, 0, 40.}, {0, 0, -1}, 12));
      }
    }
    //H::t[22].cur(true);


    // Goal outer corner 1 part
    // 13.0007 16
    // 0.000182224 19.9999
    // 38.0002 41
    //H::t[23].start(); // 21
    if (point.x > 13 && point.x < 16 && point.z > 38 && point.z < 41) {
      // Side x
      dan = std::min(dan, dan_to_sphere_outer(radius,
                                              point,
                                              {
                                                  16.,
                                                  point.y,
                                                  41.
                                              },
                                              1., 13));

    }
    //H::t[23].cur(true);


    // Side x & ceiling (goal) 2 part
    // 1.08854e-05 30
    // 8 20
    // 41 50
    //H::t[24].start(); // 18
    if (point.z > 41 && point.y > 8) {
      // y
      dan = std::min(dan, dan_to_plane(point, {0, 10., 0}, {0, -1, 0}, 14));
    }
    //H::t[24].cur(true);


    // Ceiling corners 1 part
    // 23 30
    // 13 20
    // 7.43039e-05 49.9999
    //H::t[25].start(); // 32
    if (point.y > 13 && point.x > 23) {
      // Side x
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  23.,
                                                  13.,
                                                  point.z
                                              },
                                              7., 12));
    }
    //H::t[25].cur(true);


    // Ceiling
    // 4.36196e-05 30
    // 18 20
    // 3.01327e-06 50
    //H::t[26].start(); // 13
    if (point.y > 18) {
      dan = std::min(dan, dan_to_plane(point, {0, 20., 0}, {0, -1, 0}, 15));
    }
    //H::t[26].cur(true);




    // Ceiling corners 2 part
    // 1.08854e-05 30
    // 13 20
    // 33 50
    //H::t[27].start(); // 33
    if (point.y > 13 && point.z > 33) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  13.,
                                                  33.
                                              },
                                              7., 16));

    }
    //H::t[27].cur(true);



    // Ceiling corners 3 part
    // 17 30
    // 13 20
    // 27 50
    //H::t[33].start(); // 34
    if (point.y > 13 && point.x > 17 && point.z > 27) {
      // Corner
      const Point2d& corner_o{17., 27.};
      const Point2d& dv = Point2d{point.x, point.z} - corner_o;
      if (dv.length_sq() > 36.) {
        const Point2d& o_ = corner_o + dv.normalize() * 6.;
        dan = std::min(dan, dan_to_sphere_inner(radius,
                                                point,
                                                {o_.x, 13., o_.y},
                                                7., 17));
      }

    }
    //H::t[33].cur(true);



    // Goal outer corner 2 part
    // 5.9563e-05 29.9999
    // 8.00095 11
    // 38.0001 41
    //H::t[28].start(); // 22
    if (point.z > 38 && point.y > 8 && point.y < 11 && point.z < 41) {
      // Ceiling
      dan = std::min(dan, dan_to_sphere_outer(radius,
                                              point,
                                              {
                                                  point.x,
                                                  11.,
                                                  41.
                                              },
                                              1., 18));
    }
    //H::t[28].cur(true);



    // Goal outer corner 3 part
    // 12 18.9966
    // 7.00001 13.9835
    // 38.0001 41
    //H::t[34].start(); // 23
    if (point.x > 12 && point.x < 19 && point.y > 7 && point.y < 14 && point.z > 38 && point.z < 41) {
      // Top corner
      const Point2d& o = Point2d{12., 7.};
      const Point2d& v = Point2d{point.x, point.y} - o;
      if (v.x > 0 && v.y > 0) {
        const Point2d& o_ = o + v.normalize() * 4.;
        dan = std::min(dan, dan_to_sphere_outer(radius,
                                                point,
                                                {o_.x, o_.y, 41.},
                                                1., 19));
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
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  3.,
                                                  47.
                                              },
                                              3., 20));
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
          {0, 0, 50.},
          {0, 0, -1}, 21));
    }
    //H::t[30].cur(true);


    // Goal inside top corners 2 part
    // 1.48532e-05 29.9999
    // 7.00007 20
    // 47 50
    //H::t[31].start(); //25
    if (point.z > 47 && point.y > 7) {
      // Side z
      dan = std::min(dan, dan_to_sphere_inner(radius,
                                              point,
                                              {
                                                  point.x,
                                                  7.,
                                                  47.
                                              },
                                              3., 22));
    }
    //H::t[31].cur(true);
    return dan;
  }

};

}
#endif

#endif //CODEBALL_DAN_H
