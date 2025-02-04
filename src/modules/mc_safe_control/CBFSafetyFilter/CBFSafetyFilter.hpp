#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

#include <vector>


using namespace matrix;


class CBFSafetyFilter
{
public:

    CBFSafetyFilter();

    void setPosition(const Vector3f& position) { _position = position; }
    void setAttitude(const Quatf& attitude) { _attitude = attitude; }
    void setLinearVelocity(const Vector3f& velocity) {
        Dcmf R_IB(_attitude);
        Dcmf R_BI = R_IB.transpose();
        _local_velocity = R_BI * velocity;
        _velocity = velocity;
    }
    void update(Vector3f& acceleration_setpoint, uint64_t timestamp);

    void setEpsilon(float epsilon) { _epsilon = epsilon; }
    void setPole0(float pole0) { _pole0 = pole0; }
    void setKappa(float kappa) { _kappa = kappa; }
    void setGamma(float gamma) { _gamma = gamma; }
    void setAlpha(float alpha) { _alpha = alpha; }
    void setAlphaFov(float alpha) { _alpha_fov = alpha; }

    std::vector<Vector3f>& obstacles() { return _obstacles; }

private:

    Vector3f _position;
    Vector3f _local_velocity;
    Vector3f _velocity;
    Quatf _attitude;
    std::vector<Vector3f> _obstacles;
//     std::vector<Vector3f> _rel_pos;
    std::vector<float> _nu1;

    float _epsilon = 1.f;
    float _pole0 = -1.f;
    float _kappa = 10.f;
    float _gamma = 40.f;
    float _alpha = 1.f;
    float _fov_h = 40.f / 180.f * 3.1415f;
    float _alpha_fov = 1f;

    float saturate(float x);
    float saturateDerivative(float x);
    float kappaFunction(float h, float alpha);
};
