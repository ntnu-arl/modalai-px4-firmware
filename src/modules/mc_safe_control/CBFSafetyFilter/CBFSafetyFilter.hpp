#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

#include <vector>


using namespace matrix;


class CBFSafetyFilter
{
public:

    CBFSafetyFilter();

    void update(const Vector3f& position, const Vector3f& velocity, Vector3f& acceleration_setpoint);

    std::vector<Vector3f>& obstacles() { return _obstacles; }

private:

    std::vector<Vector3f> _obstacles;
    std::vector<Vector3f> _pos_diff;
    std::vector<float> _h1;

    float _epsilon_sq = 0.2f*0.2f;
    float _lambda0 = -0.1f;
    float _kappa = 0.1f;
    float _gamma = 0.1f;
    float _alpha = 0.1f;
    float _zero_eps = 1e-5f;

    float saturate(float x);
    float saturate_derivative(float x);
};