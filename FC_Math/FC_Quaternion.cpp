#include "FC_Quaternion.h"
#include "FC_Math.h"
#include <math.h>

// Filter to update Quaternion
// w  - Angular velocity from gyro (rad.s-1)
// a  - Vector from accelerometer
// dt -  Delta time since last update
void FC_Quaternion::update(float *w, float *a, float dt) {
  // Angular Velocity in Quaternion form
  FC_Quaternion qW(0, w[0], w[1], w[2]);
  
  // Normalize accel vector
  float an[3];
  float a_norm = sqrt(square(a[0]) + squarea[1] + squarea[2]);
  an[0] = a[0] / a_norm;
  an[1] = a[1] / a_norm;
  an[2] = a[2] / a_norm;
  
  // Compute Objective Function
  float f1 = (2.0f * ((x * z) - (w * y))) - an[0];
  float f2 = (2.0f * ((w * x) + (y * z))) - an[1];
  float f3 = (1.0f - (2.0f * (square(x) - square(y)))) - an[2];
  
  // Compute Jacobian Matrix
  float j11_24 = 2.0f * y;
  float j12_23 = 2.0f * z;
  float j13_22 = 2.0f * w;
  float j14_21 = 2.0f * x;
  float j32    = 4.0f * x;
  float j33    = 4.0f * y;
  
  // Compute Gradient Matrix (Jacobian_Transpose * Objective)
  float grad_matrix_1 = (j14_21 * f2) - (j11_24 * f1);  
  float grad_matrix_2 = (j12_23 * f1) + (j13_22 * f2) - (j32 * f3);
  float grad_matrix_3 = (j12_23 * f2) - (j13_22 * f1) - (j33 * f3);
  float grad_matrix_4 = (j14_21 * f1) + (j11_24 * f2);
  
  // Normalized Gradient Quaternion
  FC_Quaternion q_grad(grad_matrix_1, grad_matrix_2, grad_matrix_3, grad_matrix_4);
  q_grad.normalize();
  
  // Quaternion derivative of angular velocity
  FC_Quaternion rate_der = times_scalar(0.5f).times(qW);
  
  // Gyro error factor
  float beta = (-1.0f) * (sqrt(3.0f / 4.0f) * radians(2.0f));
  
  // Calculated Quaternion Derivative (gyro/accel)
  FC_Quaternion deriv = rate_der.plus(q_grad.times_scalar(beta)).times_scalar(dt);
  
  // Integrate Attitude Quaternion and normalize
  w += deriv.w;
  x += deriv.x;
  y += deriv.y;
  z += deriv.z;
  normalize();
  
}

// Quaternion Multiplication
// Q1 = (s, V) * Q2 = (t, U)
// Q1Q2 = (st - u.v, sU + tV + VxU)
FC_Quaternion FC_Quaternion::times(FC_Quaternion q) {
  float nw = (w * q.w) - (x * q.x) - (y * q.y) - (z * q.z);
  float nx = (w * q.x) + (x * q.w) + (y * q.z) - (z * q.y);
  float ny = (w * q.y) - (x * q.z) + (y * q.w) + (z * q.x);
  float nz = (w * q.z) + (x * q.y) - (y * q.x) + (z * q.w);
  
  return FC_Quaternion(nw, nx, ny, nz);
}

// Add two Quaternions
FC_Quaternion FC_Quaternion::plus(FC_Quaternion q) {
  float nw = w + q.w;
  float nx = x + q.x;
  float ny = y + q.y;
  float nz = z + q.z;
  return FC_Quaternion(nw, nx, ny, nz);
}

// Multiply Quaternion by a scalar
FC_Quaternion FC_Quaternion::times_scalar(float s) {
  return FC_Quaternion(w * s, x * s, y * s, z * s);
}

// Normalize quaternion -- ||q|| == 1
void FC_Quaternion::normalize() {
  float mag_sqr = square(w) + square(x) + square(y) + square(z);
  
  if ((fabs((mag_sqr) - 1.0f)) > TOLERANCE) {
    float magnitude = sqrt(mag_sqr);
    w /= magnitude;
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
  }
}

// Quaternion Conjugate
// q* = (s, -V)
FC_Quaternion FC_Quaternion::conjugate() {
  return FC_Quaternion(w, -x, -y, -z);
}

// Convert Quaternion to Euler Angles
void FC_Quaternion::to_euler(float *eulers) {
  // Euler Roll Angle
  eulers[0] = atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  // Euler Pitch Angle
  eulers[1] = asin_safe(2.0f * (w * y - z * x));
  // Euler Yaw Angle
  eulers[2] = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}