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
    void setLinearVelocity(const Vector3f& velocity) { _velocity = velocity; }
    void update(Vector3f& acceleration_setpoint, uint64_t timestamp);

    void setEpsilon(float epsilon) { _epsilon = epsilon; }
    void setPole0(float pole0) { _pole0 = pole0; }
    void setKappa(float kappa) { _kappa = kappa; }
    void setGamma(float gamma) { _gamma = gamma; }
    void setAlpha(float alpha) { _alpha = alpha; }

    std::vector<Vector3f>& obstacles() { return _obstacles; }

private:

    Vector3f _position;
    Vector3f _velocity;
    std::vector<Vector3f> _obstacles;
    std::vector<Vector3f> _rel_pos;
    std::vector<float> _h1;

    float _epsilon = 1.f;
    float _pole0 = -1.f;
    float _kappa = 10.f;
    float _gamma = 40.f;
    float _alpha = 1.f;
    // float _zero_eps = 1e-5f;

    float saturate(float x);
    float saturateDerivative(float x);
};