/*!
 * Copyright 2016 by Muellerlab, UC Berkeley
 */
#pragma once

#include <math.h>
#include <limits>
#include <cerrno>

class Vec3f;

//declarations:
Vec3f operator*(const float lhs, const Vec3f rhs);
Vec3f operator*(const Vec3f rhs, const float lhs);
Vec3f operator*(const int lhs, const Vec3f rhs);
Vec3f operator*(const Vec3f rhs, const int lhs);


//! 3D vector class
/*!
 * A 3D vector class, to make passing arguments easier, and allow easy addition etc. of vectors.
 */
class Vec3f {
 public:
  float x, y, z;  //!< the three components of the vector

  Vec3f(void)
      : x(std::numeric_limits<float>::quiet_NaN()),
        y(std::numeric_limits<float>::quiet_NaN()),
        z(std::numeric_limits<float>::quiet_NaN()) {
  }  //!<Initialises all members to NaN
  Vec3f(float xin, float yin, float zin)
      : x(xin),
        y(yin),
        z(zin) {
  }  //!< Initialise vector

  Vec3f(const float in[3])
      : x(in[0]),
        y(in[1]),
        z(in[2]) {
  }  //!< Initialise vector

  Vec3f(Vec3f const &in)
      : x(in.x),
        y(in.y),
        z(in.z) {
  }  //!< Initialise from Vec3f

  //!Getter function, index 0 <-> x, 1 <-> y, 2 <-> z
  inline float operator[](int i) const {
    switch (i) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
        break;
    }
    //we're doing something wrong if we get here
    return std::numeric_limits<float>::quiet_NaN();
  }

  //!Setter function, index 0 <-> x, 1 <-> y, 2 <-> z
  inline float & operator[](int i) {
    switch (i) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
        break;
    }
    //we're doing something wrong if we get here
    //fail loudly:
    x = y = z = std::numeric_limits<float>::quiet_NaN();
    return x;
  }

  //!Calculate the dot product of two vectors
  inline float Dot(const Vec3f rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z;
  }

  //!Calculate the cross product of two vectors
  inline Vec3f Cross(const Vec3f rhs) const {
    return Vec3f(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z,
                x * rhs.y - y * rhs.x);
  }

  //!Calculate the Euclidean norm of the vector, squared (= sum of squared elements).
  inline float GetNorm2Squared(void) const {
    return this->Dot(*this);
  }

  //!Calculate the Euclidean norm of the vector.
  inline float GetNorm2(void) const {
    return sqrtf(GetNorm2Squared());
  }

  inline float Test(void) const {
    return x;
  }

  //!Get the unit vector pointing along the same direction as this vector. Will fail for zero vectors.
  inline Vec3f GetUnitVector(void) const {
    float const n = this->GetNorm2();
    return (*this) / n;
  }

  inline Vec3f operator+(const Vec3f rhs) const {
    return Vec3f(x + rhs.x, y + rhs.y, z + rhs.z);
  }
  inline Vec3f operator-(const Vec3f rhs) const {
    return Vec3f(x - rhs.x, y - rhs.y, z - rhs.z);
  }
  inline Vec3f operator/(const float rhs) const {
    return Vec3f(x / rhs, y / rhs, z / rhs);
  }
  inline Vec3f operator+() const {
    return (*this);
  }  //mostly used to write things prettily, contrasting to the below.
  inline Vec3f operator-() const {
    return (*this) * float(-1);
  }

  inline Vec3f operator+=(const Vec3f rhs) {
    (*this) = (*this) + rhs;
    return *this;
  }
  inline Vec3f operator-=(const Vec3f rhs) {
    (*this) = (*this) - rhs;
    return *this;
  }
  inline Vec3f operator*=(const float &rhs) {
    *this = (*this) * rhs;
    return *this;
  }
  inline Vec3f operator/=(const float &rhs) {
    *this = (*this) / rhs;
    return *this;
  }
};


//Multiply with scalar of same type as Vec3f:
inline Vec3f operator*(const Vec3f lhs, const float rhs) {
  return Vec3f(rhs * lhs.x, rhs * lhs.y, rhs * lhs.z);
}

inline Vec3f operator*(const float lhs, const Vec3f rhs) {
  return Vec3f(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

//Multiply with scalar of integer type:
inline Vec3f operator*(const Vec3f lhs, const int rhs) {
  return Vec3f(rhs * lhs.x, rhs * lhs.y, rhs * lhs.z);
}

inline Vec3f operator*(const int lhs, const Vec3f rhs) {
  return Vec3f(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}
