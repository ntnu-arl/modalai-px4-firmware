/**
 * @file CBFSafetyFilter.cpp
 */
#include <CBFSafetyFilter.hpp>

#include <px4_platform_common/module.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/Publication.hpp>
#include <math.h>
#include <string.h>

static struct debug_vect_s dbg;
static orb_advert_t pub_dbg;

CBFSafetyFilter::CBFSafetyFilter() {
    qp = QProblem(_nV, _nC);
    qp.setPrintLevel(PL_NONE);

    dbg.x = 0.0f;
    dbg.y = 0.0f;
    dbg.z = 0.0f;
    pub_dbg = orb_advertise(ORB_ID(debug_vect), &dbg);
}

void CBFSafetyFilter::updateObstacles() {
    tof_obstacles_chunk_s tof_obstacles_chunk;
    if (_tof_obstacles_chunk_sub.update(&tof_obstacles_chunk))
    {
        if (_prev_obstacles_chunk_id < 0 || (int)tof_obstacles_chunk.chunk_id <= _prev_obstacles_chunk_id) {
            _obstacles.clear();
        }
        _prev_obstacles_chunk_id = (int)tof_obstacles_chunk.chunk_id;
        for (int i = 0; i < tof_obstacles_chunk.num_points_chunk; i++)
        {
            _obstacles.emplace_back(
                tof_obstacles_chunk.points_x[i],
                tof_obstacles_chunk.points_y[i],
                tof_obstacles_chunk.points_z[i]
            );
        }
    }
}

void CBFSafetyFilter::updateAttitude() {
    if (_vehicle_attitude_sub.updated())
    {
        vehicle_attitude_s vehicle_attitude;
        if (_vehicle_attitude_sub.copy(&vehicle_attitude))
            _attitude = Quatf(vehicle_attitude.q);
    }
}

void CBFSafetyFilter::filter(Vector3f& acceleration_setpoint, const Vector3f& velocity, uint64_t timestamp) {
    // pass through if no obstacles are recorded
    updateObstacles();
    const size_t n = _obstacles.size();
    if (n == 0) return;

    // compute local state
    Eulerf euler_current(_attitude);
    Dcmf R_WB(_attitude);
    Dcmf R_BW = R_WB.transpose();
    Dcmf R_VW(Eulerf(0.f, 0.f, -euler_current.psi()));
    Dcmf R_BV = R_BW * R_VW.transpose();

    _body_acceleration_setpoint = R_BW * acceleration_setpoint;
    _body_velocity = R_BW * velocity;
    _vehicle_velocity = R_VW * velocity;


    // composite collision CBF
    // nu1_i
    _nu1.resize(n);
    for(size_t i = 0; i < n; i++) {
        float nu_i0 = _obstacles[i].norm_squared() - (_epsilon * _epsilon);
        float Lf_nu_i0 = -2.f * _obstacles[i].dot(_body_velocity);
        float nu_i1 = Lf_nu_i0 - _pole0 * nu_i0;
        _nu1[i] = nu_i1;
    }

    // h(x)
    float exp_sum = 0.f;
    for(size_t i = 0; i < n; i++) {
        exp_sum += expf(-_kappa * saturate(_nu1[i] / _gamma));
    }
    float h = -(_gamma / _kappa) * logf(exp_sum);

    // L_{f}h(x)
    float Lf_h = 0.f;
    for(size_t i = 0; i < n; i++) {
        float Lf_nu_i1 = 2.f * (_body_velocity + _pole0 * _obstacles[i]).dot(_body_velocity);
        float lambda_i = expf(-_kappa * saturate(_nu1[i] / _gamma)) * saturateDerivative(_nu1[i] / _gamma);
        Lf_h += lambda_i * Lf_nu_i1;
    }
    Lf_h /= exp_sum;

    // L_{g}h(x)
    Vector3f Lg_h(0.f, 0.f, 0.f);
    for(size_t i = 0; i < n; i++) {
        Vector3f Lg_nu_i1 = -2.f * _obstacles[i];
        float lambda_i = expf(-_kappa * saturate(_nu1[i] / _gamma)) * saturateDerivative(_nu1[i] / _gamma);
        Lg_h += lambda_i * Lg_nu_i1;
    }
    Lg_h /= exp_sum;

    // L_{g}h(x) * u, u = k_n(x) = a
    float Lg_h_u = Lg_h.dot(_body_acceleration_setpoint);


    // horizontal FoV CBF
    Vector3f e1(sinf(_fov_h), cosf(_fov_h), 0.f);
    Vector3f e2(sinf(_fov_h), -cosf(_fov_h), 0.f);
    float h1 = (e1).dot(_vehicle_velocity);
    float h2 = (e2).dot(_vehicle_velocity);
    float Lf_h1 = 0.f;
    float Lf_h2 = 0.f;
    Vector3f Lg_h1 = R_BV * e1;
    Vector3f Lg_h2 = R_BV * e2;


    // solve QP
    // quadratic cost x^T*H*x
    real_t H[_nV*_nV] = {1.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 1.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 3.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0};
    // linear cost matrix g*x
    real_t g[_nV] = { 0.0, 0.0, 0.0, 50.0, 50.0 };
    // constraint matrix A
    real_t A[_nC*_nV] = {(real_t)Lg_h(0), (real_t)Lg_h(1), (real_t)Lg_h(2), 0.0, 0.0,
                      0.0, 0.0, 0.0, 1.0, 0.0,
                      (real_t)Lg_h1(0), (real_t)Lg_h1(1), (real_t)Lg_h1(2), 1.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 1.0,
                      (real_t)Lg_h2(0), (real_t)Lg_h2(1), (real_t)Lg_h2(2), 0.0, 1.0};
    // bounds on Ax
    real_t lbA[_nC] = { (real_t)(-Lf_h - kappaFunction(h, _alpha) - Lg_h_u), 0.0, (real_t)(-Lf_h1 - _alpha_fov * h1), 0.0, (real_t)(-Lf_h2 - _alpha_fov * h2) };
    real_t* ubA = NULL;
    // bounds on x
    real_t* lb = NULL;
    real_t* ub = NULL;

    returnValue qp_status = qp.init(H, g, A, lb, ub, lbA, ubA, _nWSR);

    switch(qp_status) {
        case SUCCESSFUL_RETURN: {
            qp.getPrimalSolution(xOpt);
            Vector3f acceleration_correction(xOpt[0], xOpt[1], xOpt[2]);
            _body_acceleration_setpoint += acceleration_correction;
            acceleration_setpoint = R_WB * _body_acceleration_setpoint;
            break;
        }
        case RET_MAX_NWSR_REACHED:
            PX4_ERR("QP could not be solved within the given number of working set recalculations");
            break;
        default:
            PX4_ERR("QP failed: returned %d", qp_status);
            break;
    }

    clampAccSetpoint(acceleration_setpoint);

    dbg.timestamp = timestamp;
    dbg.x = h;
    dbg.y = h1;
    dbg.z = h2;
    orb_publish(ORB_ID(debug_vect), pub_dbg, &dbg);
}

void CBFSafetyFilter::clampAccSetpoint(Vector3f& acceleration_setpoint) {
    acceleration_setpoint(0) = math::constrain(acceleration_setpoint(0), -max_acc_xy, max_acc_xy);
    acceleration_setpoint(1) = math::constrain(acceleration_setpoint(1), -max_acc_xy, max_acc_xy);
    acceleration_setpoint(2) = math::constrain(acceleration_setpoint(2), -max_acc_z, max_acc_z);
}

float CBFSafetyFilter::saturate(float x) {
    return tanh(x);
}

float CBFSafetyFilter::saturateDerivative(float x) {
    float th = tanh(x);
    return 1.f - (th * th);
}

float CBFSafetyFilter::kappaFunction(float h, float alpha) {
    float a = alpha;
    float b = 1.f/alpha;
    if (h>=0.f) {
        return alpha * h;
    }
    else {
        return a * b * ( h / (b + abs(h)) );
    }
}
