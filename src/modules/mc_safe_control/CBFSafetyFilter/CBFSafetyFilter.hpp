#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

#include <vector>


using namespace matrix;


class CBFSafetyFilter
{
public:

    CBFSafetyFilter();

    void update(Vector3f& acceleration);

    std::vector<Vector3f>& obstacles() { return _obstacles; }

private:

    std::vector<Vector3f> _obstacles;
};