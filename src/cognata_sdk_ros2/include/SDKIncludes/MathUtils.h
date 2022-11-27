//
// MathUtils.h - utility math funcions
//
// Copyright Cognata Ltd. (c) 2022 - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// All trade-marks, trade names, service marks and logos referenced herein belong to their respective companies
// Proprietary and confidential

#ifndef __MATHUTILS_H__
#define __MATHUTILS_H__

#include <string>
#include <vectors.pb.h>
#include <cmath>

const float DEG2RAD = 0.0174532925f;
const float RAD2DEG = 57.295779513f;
const double g_epsilon = std::numeric_limits<double>::epsilon();
#if defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif
#include "Eigen/Dense"
#include "Eigen/Geometry"
#if defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
namespace Cognata
{
namespace SDK
{

Cognata::SDK::SDKVector3 Vector3fToSDKVec(const Eigen::Vector3f& v1);

Eigen::Vector3f transformPointByRotationDegree(Eigen::Vector3f point, Eigen::Vector3f rotation);

Eigen::Vector3f transformPointByHeading(Eigen::Vector3f point, float angle);

std::string toString(const Cognata::SDK::SDKVector3& vec);

std::string toString(const Eigen::Vector3f& vec);

static inline Eigen::Vector3f SDKVecToVector3f(const Cognata::SDK::SDKVector3& v1)
{
    return Eigen::Vector3f(v1.x(), v1.y(), v1.z());
}

static inline float transformDirectionByHeading(float rotation, float angle)
{
    return static_cast<float>(std::fmod(rotation + angle, 360));
}

static inline bool fuzzyCompare(double x, double y)
{
    return std::abs(x - y) < g_epsilon;
}

template<class T>
static inline T clamp(T val, T min, T max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

} // namespace SDK
} // namespace Cognata

#endif // __MATHUTILS_H__
