#ifndef CODEBALL_POINT2D_H
#define CODEBALL_POINT2D_H

#include <math.h>


struct Point2d {
  double x, y;

  Point2d() {}

  Point2d(double x, double y) : x(x), y(y) {}

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
#ifndef LOCAL
namespace Frozen {

struct Point2d {
  double x, y;

  Point2d() {}

  Point2d(double x, double y) : x(x), y(y) {}

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

}
#endif

#endif //CODEBALL_POINT2D_H
