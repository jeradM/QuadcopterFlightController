#include "FC_Quaternion.h"

FC_Quaternion FC_Quaternion::operator+(const FC_Quaternion &q1) {
  return FC_Quaternion(_w + q1._w, _x + q1._x, _y + q1._y, _z + q1._z);
}

FC_Quaternion FC_Quaternion::cross_with(FC_Quaternion q) {
  float nw = (w * q.w) - (x * q.x) - (y * q.y) - (z * q.z);
  float nx = (w * q.x) + (x * q.w) + (y * q.z) - (z * q.y);
  float ny = (w * q.y) - (x * q.z) + (y * q.w) + (z * q.x);
  float nz = (w * q.z) + (x * q.y) - (y * q.x) + (z * q.w);
  
  return FC_Quaternion(nw, nx, ny, nz);
}

FC_Quaternion FC_Quaternion::times_scalar(float s) {
  return FC_Quaternion(w * s, x * s, y * s, z * s);
}

void FC_Quaternion::normalize() {
  float mag_sqr = (w * w) + (x * x) + (y * y) + (z * z);
  
  if ((fabs(mag_sqr) - 1.0f) > TOLERANCE) {
    float magnitude = sqrt(mag_sqr);
    w /= magnitude;
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
  }
}

FC_Quaternion FC_Quaternion::conjugate() {
  return FC_Quaternion(w, -x, -y, -z);
}

FC_Quaternion FC_Quaternion::rate_derivative(float rx, float ry, float rz) {
  FC_Quaternion q(0.0f, rx, ry, rz);
  FC_Quaternion h = times_scalar(0.5f);
  return h.cross_with(q);
}