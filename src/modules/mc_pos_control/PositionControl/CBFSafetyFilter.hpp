/**
 * @file CBFSafetyFilter.hpp
 */
#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/tof_obstacles_chunk.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/cbf_debug.h>
#include <qpOASES.hpp>

#include <containers/Array.hpp>

using namespace matrix;
using namespace qpOASES;

#define NV 5  // nb of QP variables
#define NC 5  // nb of QP consntraints
#define CBF_MAX_OBSTACLES 200

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

    void getDebug(cbf_debug_s& debug_msg)
    {
      debug_msg.h = _debug_msg.h;
      memcpy(debug_msg.input, _debug_msg.input, 3 * sizeof(debug_msg.input[0]));
      memcpy(debug_msg.output, _debug_msg.output, 3 * sizeof(debug_msg.input[0]));
    };

private:
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _tof_obstacles_chunk_sub{ORB_ID(tof_obstacles_chunk)};

    Quatf _attitude;
    Vector3f _body_velocity;
    Vector3f _vehicle_velocity;
    Vector3f _body_acceleration_setpoint;

    px4::Array<Vector3f, CBF_MAX_OBSTACLES> _obstacles;
    px4::Array<float, CBF_MAX_OBSTACLES> _nu1;

    cbf_debug_s _debug_msg;

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
