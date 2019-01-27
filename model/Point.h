#ifndef CODEBALL_POINT_H
#define CODEBALL_POINT_H

#include <math.h>
#include <iostream>


struct Point {
  double x, y, z;

  inline Point() {}

  inline Point(double x, double y, double z) : x(x), y(y), z(z) {}

  inline bool operator==(const Point& other) const {
    return
        x == other.x
            && y == other.y
            && z == other.z;
  }

  inline Point operator-(const Point& other) const {
    return {x - other.x, y - other.y, z - other.z};
  }
  inline Point operator+(const Point& other) const {
    return {x + other.x, y + other.y, z + other.z};
  }
  inline Point operator*(const double& value) const {
    return {x * value, y * value, z * value};
  }

  inline Point operator/(const double& value) const {
    return {x / value, y / value, z / value};
  }
  inline Point& operator-=(const Point& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }
  inline Point& operator+=(const Point& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
  inline Point& operator/=(const double value) {
    x /= value;
    y /= value;
    z /= value;
    return *this;
  }
  inline Point& operator*=(const double value) {
    x *= value;
    y *= value;
    z *= value;
    return *this;
  }
  inline double length() const {
    return sqrt(length_sq());
  }
  inline double length_sq() const {
    return x * x + y * y + z * z;
  }
  inline Point normalize() const {
    const double& norm = length();
    if (norm > 0) {
      return {x / norm, y / norm, z / norm};
    } else {
      return {x, y, z};
    }
  }
  inline double dot(const Point& other) const {
    return x * other.x + y * other.y + z * other.z;
  }
  inline Point clamp(const double& ub) const {
    if (length_sq() > ub * ub) {
      return normalize() * ub;
    }
    return *this;
  }

};

#ifndef LOCAL
namespace Frozen {

struct Point {
  double x, y, z;

  Point() {}

  Point(double x, double y, double z) : x(x), y(y), z(z) {}

  bool operator==(const Point& other) const {
    return
        x == other.x
            && y == other.y
            && z == other.z;
  }

  Point operator-(const Point& other) const {
    return {x - other.x, y - other.y, z - other.z};
  }
  Point operator+(const Point& other) const {
    return {x + other.x, y + other.y, z + other.z};
  }
  Point operator*(const double value) const {
    return {x * value, y * value, z * value};
  }

  Point operator/(const double value) const {
    return {x / value, y / value, z / value};
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
  Point& operator/=(const double value) {
    x /= value;
    y /= value;
    z /= value;
    return *this;
  }
  Point& operator*=(const double value) {
    x *= value;
    y *= value;
    z *= value;
    return *this;
  }
  double length() const {
    return sqrt(length_sq());
  }
  double length_sq() const {
    return x * x + y * y + z * z;
  }
  Point normalize() const {
    const double norm = length();
    if (norm > 0) {
      return {x / norm, y / norm, z / norm};
    } else {
      return {x, y, z};
    }
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

}
#endif

#endif //CODEBALL_POINT_H
