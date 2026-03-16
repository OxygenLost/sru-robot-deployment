#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rl_nav_quat {

// All quaternion functions use (w, x, y, z) convention, stored as Eigen::Vector4f.

inline Eigen::Vector4f normalize(const Eigen::Vector4f& q, float eps = 1e-9f) {
    float n = q.norm();
    return (n > eps) ? (q / n) : q;
}

inline Eigen::Vector3f normalize3(const Eigen::Vector3f& v, float eps = 1e-9f) {
    float n = v.norm();
    return (n > eps) ? (v / n) : v;
}

inline Eigen::Vector4f yaw_quat(const Eigen::Vector4f& quat) {
    float qw = quat(0), qx = quat(1), qy = quat(2), qz = quat(3);
    float yaw = std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    Eigen::Vector4f result;
    result(0) = std::cos(yaw / 2.0f);
    result(1) = 0.0f;
    result(2) = 0.0f;
    result(3) = std::sin(yaw / 2.0f);
    return normalize(result);
}

inline Eigen::Vector4f quat_conjugate(const Eigen::Vector4f& q) {
    return Eigen::Vector4f(q(0), -q(1), -q(2), -q(3));
}

inline Eigen::Vector4f quat_inv(const Eigen::Vector4f& q) {
    return normalize(quat_conjugate(q));
}

inline Eigen::Vector4f quat_mul(const Eigen::Vector4f& q1, const Eigen::Vector4f& q2) {
    float w1 = q1(0), x1 = q1(1), y1 = q1(2), z1 = q1(3);
    float w2 = q2(0), x2 = q2(1), y2 = q2(2), z2 = q2(3);

    float ww = (z1 + x1) * (x2 + y2);
    float yy = (w1 - y1) * (w2 + z2);
    float zz = (w1 + y1) * (w2 - z2);
    float xx = ww + yy + zz;
    float qq = 0.5f * (xx + (z1 - x1) * (x2 - y2));
    float w = qq - ww + (z1 - y1) * (y2 - z2);
    float x = qq - xx + (x1 + w1) * (x2 + w2);
    float y = qq - yy + (w1 - x1) * (y2 + z2);
    float z = qq - zz + (z1 + y1) * (w2 - x2);

    return Eigen::Vector4f(w, x, y, z);
}

inline Eigen::Vector3f quat_apply(const Eigen::Vector4f& quat, const Eigen::Vector3f& vec) {
    Eigen::Vector3f xyz(quat(1), quat(2), quat(3));
    Eigen::Vector3f t = 2.0f * xyz.cross(vec);
    return vec + quat(0) * t + xyz.cross(t);
}

inline Eigen::Matrix3f matrix_from_quat(const Eigen::Vector4f& q) {
    float r = q(0), i = q(1), j = q(2), k = q(3);
    float two_s = 2.0f / q.squaredNorm();

    Eigen::Matrix3f m;
    m(0, 0) = 1.0f - two_s * (j * j + k * k);
    m(0, 1) = two_s * (i * j - k * r);
    m(0, 2) = two_s * (i * k + j * r);
    m(1, 0) = two_s * (i * j + k * r);
    m(1, 1) = 1.0f - two_s * (i * i + k * k);
    m(1, 2) = two_s * (j * k - i * r);
    m(2, 0) = two_s * (i * k - j * r);
    m(2, 1) = two_s * (j * k + i * r);
    m(2, 2) = 1.0f - two_s * (i * i + j * j);
    return m;
}

// Subtract frame transforms: T_12 = T_01^{-1} * T_02
// Returns (t12, q12). If t02/q02 are nullptr, treats them as identity/zero.
inline std::pair<Eigen::Vector3f, Eigen::Vector4f> subtract_frame_transforms(
    const Eigen::Vector3f& t01, const Eigen::Vector4f& q01,
    const Eigen::Vector3f* t02 = nullptr, const Eigen::Vector4f* q02 = nullptr)
{
    Eigen::Vector4f q10 = quat_inv(q01);

    Eigen::Vector4f q12;
    if (q02) {
        q12 = quat_mul(q10, *q02);
    } else {
        q12 = q10;
    }

    Eigen::Vector3f t12;
    if (t02) {
        t12 = quat_apply(q10, *t02 - t01);
    } else {
        t12 = quat_apply(q10, -t01);
    }

    return {t12, q12};
}

// Transform a single point using position and quaternion rotation.
inline Eigen::Vector3f transform_point(
    const Eigen::Vector3f& point,
    const Eigen::Vector3f& pos,
    const Eigen::Vector4f& quat)
{
    Eigen::Matrix3f rot = matrix_from_quat(quat);
    return rot * point + pos;
}

}  // namespace rl_nav_quat
