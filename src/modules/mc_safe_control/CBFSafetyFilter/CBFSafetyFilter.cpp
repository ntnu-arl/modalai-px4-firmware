#include <CBFSafetyFilter.hpp>

#include <px4_platform_common/module.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/debug_vect.h>
#include <qpOASES.hpp>

#include <math.h>
#include <string.h>


static struct debug_vect_s dbg;
static orb_advert_t pub_dbg;

CBFSafetyFilter::CBFSafetyFilter() {
    // _obstacles.emplace_back(10.f, 1.f, -15.f);
    // _obstacles.emplace_back(10.f, 1.f, -14.f);
    // _obstacles.emplace_back(10.f, 1.f, -13.f);
    // _obstacles.emplace_back(10.f, 1.f, -12.f);
    // _obstacles.emplace_back(10.f, 1.f, -11.f);
    // _obstacles.emplace_back(10.f, 1.f, -10.f);
    // _obstacles.emplace_back(10.f, 1.f, -9.f);
    // _obstacles.emplace_back(10.f, 1.f, -8.f);
    // _obstacles.emplace_back(10.f, 1.f, -7.f);
    // _obstacles.emplace_back(10.f, 1.f, -6.f);
    // _obstacles.emplace_back(10.f, 1.f, -5.f);

    dbg.x = 0.0f;
    dbg.y = 0.0f;
    dbg.z = 0.0f;
    pub_dbg = orb_advertise(ORB_ID(debug_vect), &dbg);
}

void CBFSafetyFilter::update(Vector3f& acceleration_setpoint, uint64_t timestamp) {
    const size_t n = _obstacles.size();

    if (n == 0) return;

    // PX4_INFO("Obstacle: %f, %f, %f", (double)_obstacles[0](0), (double)_obstacles[0](1), (double)_obstacles[0](2));

    Dcmf R_IB(_attitude);
    Dcmf R_BI = R_IB.transpose();
    Vector3f local_accel_setpoint = R_BI * acceleration_setpoint;
    Eulerf euler_current(_attitude);
    Eulerf euler_IV(0.f, 0.f, euler_current.psi());
    Dcmf R_IV(euler_IV);
    Dcmf R_VI = R_IV.transpose();

    _nu1.resize(n);

    // ====================================================================
    // ====================== composite collision CBF =====================
    // ====================================================================

    // _nu1 = {nu_{i, 1}, i=0...n-1}
    for(size_t i = 0; i < n; i++) {
        float nu_i0 = _obstacles[i].norm_squared() - (_epsilon * _epsilon);
        // float nu_i0 = _obstacles[i].norm_() - (_epsilon);
        float Lf_nu_i0 = -2.f * _obstacles[i].dot(_local_velocity);
        // float Lf_nu_i0 = -1.f * _obstacles[i].dot(_local_velocity);
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
        float Lf_nu_i1 = 2.f * (_local_velocity + _pole0 * _obstacles[i]).dot(_local_velocity);
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
    float Lg_h_u = Lg_h.dot(local_accel_setpoint);


    // ===================================================================
    // ======================== horizontal FoV CBF =======================
    // ===================================================================

    Vector3f e1(sinf(_fov_h), cosf(_fov_h), 0.f);
    Vector3f e2(sinf(_fov_h), -cosf(_fov_h), 0.f);
    float h1 = (e1).dot(R_VI * _velocity);
    float h2 = (e2).dot(R_VI * _velocity);
    float Lf_h1 = 0.f;
    float Lf_h2 = 0.f;
    Vector3f Lg_h1 = (R_VI * R_IB).transpose() * e1;
    Vector3f Lg_h2 = (R_VI * R_IB).transpose() * e2;

    // analytical QP solution from: https://arxiv.org/abs/2206.03568
//     float eta = 0.f;
//     float Lg_h_mag2 = Lg_h.norm_squared();
//     if (Lg_h_mag2 > 1e-5f) {
//         eta = -(Lf_h + Lg_h_u + _alpha*h) / Lg_h_mag2;
//     }
//     Vector3f local_correction = (eta > 0.f ? eta : 0.f) * Lg_h;
//     dbg.x = local_correction(0);
//     dbg.y = local_correction(1);
//     dbg.z = local_correction(2);
//     local_accel_setpoint += local_correction;

//     acceleration_setpoint = R_IB * local_accel_setpoint;

    USING_NAMESPACE_QPOASES

    // Hessian
    real_t H[5*5] = {1.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0};
    // linear cost matrix g*x
    real_t  g[5] = { 0.0, 0.0, 0.0, 50.0, 50.0 };
    // constraint matrix A
    real_t  A[5*5] = {(real_t)Lg_h(0), (real_t)Lg_h(1), (real_t)Lg_h(2), (real_t)0.0, (real_t)0.0,
                      (real_t)0.0, (real_t)0.0, (real_t)0.0, (real_t)1.0, (real_t)0.0,
                      (real_t)Lg_h1(0), (real_t)Lg_h1(1), (real_t)Lg_h1(2), (real_t)1.0, (real_t)0.0,
                      (real_t)0.0, (real_t)0.0, (real_t)0.0, (real_t)0.0, (real_t)1.0,
                      (real_t)Lg_h2(0), (real_t)Lg_h2(1), (real_t)Lg_h2(2), 0.0, (real_t)1.0};
    // bounds on Ax
    real_t  lbA[5] = { (real_t)(-Lf_h - _alpha * h - Lg_h_u), 0.0, (real_t)(-Lf_h1 - _alpha_fov * h1), 0.0, (real_t)(-Lf_h2 - _alpha_fov * h2) };
    real_t* ubA = NULL;
    // bounds on x
    real_t* lb = NULL;
    real_t* ub = NULL;
    int_t nWSR = 50;

    int_t nV = 5;
    int_t nC = 5;
    QProblem qp(nV, nC);
    qp.setPrintLevel(PL_NONE);
    returnValue qp_status = qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

    switch(qp_status) {
    case SUCCESSFUL_RETURN: {
        real_t xOpt[5];
        qp.getPrimalSolution(xOpt);
        Vector3f acceleration_correction(xOpt[0], xOpt[1], xOpt[2]);
        local_accel_setpoint += acceleration_correction;
        acceleration_setpoint = R_IB * local_accel_setpoint;
        break;
    }
    case RET_MAX_NWSR_REACHED:
        PX4_ERR("QP could not be solved within the given number of working set recalculations");
        break;
    default:
        PX4_ERR("QP initialisation failed: returned %d", qp_status);
        break;
    }

    dbg.timestamp = timestamp;
    dbg.x = h;
    dbg.y = h1;
    dbg.z = h2;
    orb_publish(ORB_ID(debug_vect), pub_dbg, &dbg);
}


float CBFSafetyFilter::saturate(float x) {
    return tanh(x);
}

float CBFSafetyFilter::saturateDerivative(float x) {
    float th = tanh(x);
    return 1.f - (th * th);
}
