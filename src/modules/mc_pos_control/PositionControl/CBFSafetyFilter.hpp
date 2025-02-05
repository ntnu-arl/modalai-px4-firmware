/**
 * @file CBFSafetyFilter.hpp
 */
#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/tof_obstacles_chunk.h>
#include <uORB/topics/vehicle_attitude.h>
#include <qpOASES.hpp>

#include <vector>

using namespace matrix;
using namespace qpOASES;
// USING_NAMESPACE_QPOASES;

#define NV 5
#define NC 5

class CBFSafetyFilter
{
public:
    CBFSafetyFilter();
	~CBFSafetyFilter() = default;

    void updateObstacles();
    void updateAttitude();

    void filter(Vector3f& acceleration_setpoint, const Vector3f& velocity, uint64_t timestamp);

    void setEpsilon(float epsilon) { _epsilon = epsilon; }
    void setPole0(float pole0) { _pole0 = pole0; }
    void setKappa(float kappa) { _kappa = kappa; }
    void setGamma(float gamma) { _gamma = gamma; }
    void setAlpha(float alpha) { _alpha = alpha; }
    void setAlphaFov(float alpha) { _alpha_fov = alpha; }

private:
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _tof_obstacles_chunk_sub{ORB_ID(tof_obstacles_chunk)};
    int _prev_obstacles_chunk_id = -1;

    Quatf _attitude;
    Vector3f _body_velocity;
    Vector3f _vehicle_velocity;
    Vector3f _body_acceleration_setpoint;

    std::vector<Vector3f> _obstacles;
    std::vector<float> _nu1;

    float _epsilon;
    float _pole0;
    float _kappa;
    float _gamma;
    float _alpha;
    float _alpha_fov;
    const float _fov_h = 40.f / 180.f * 3.1415f;  // TODO set as param

    const float max_acc_xy = 2.f;  // TODO set as param
    const float max_acc_z = 3.f;  // TODO set as param
    void clampAccSetpoint(Vector3f& acceleration_setpoint);

    float saturate(float x);
    float saturateDerivative(float x);
    float kappaFunction(float h, float alpha);

    QProblem qp;
    real_t xOpt[NV];
};
