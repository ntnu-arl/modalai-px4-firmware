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
    qp = QProblem(NV, NC);
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
        // TODO make this part of the message
        size_t max_chunk_size = 20;

        // if we get a new pointcloud with fewer points than the previous, remove the overflow
        for (size_t i = _obstacles.size() ; i >= tof_obstacles_chunk.num_points_total ; i--)
        {
            _obstacles.remove(i);
        }

        for (
            size_t i_obs = tof_obstacles_chunk.chunk_id * max_chunk_size, i_chnk = 0 ;
            i_chnk < tof_obstacles_chunk.num_points_chunk ;
            i_obs++, i_chnk++
        ) {
            // if the obstacle array is already large enough, replace
            if (i_obs < _obstacles.size())
            {
                _obstacles[i_obs] = Vector3f(
                    tof_obstacles_chunk.points_x[i_chnk],
                    tof_obstacles_chunk.points_y[i_chnk],
                    tof_obstacles_chunk.points_z[i_chnk]
                );
            }
            // else, pushback
            else
            {
                _obstacles.push_back(Vector3f(
                    tof_obstacles_chunk.points_x[i_chnk],
                    tof_obstacles_chunk.points_y[i_chnk],
                    tof_obstacles_chunk.points_z[i_chnk]
                ));
            }
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
    // TODO reset obstacle with timer
    // pass through if no obstacles are recorded
    updateAttitude();
    updateObstacles();
    const size_t n = _obstacles.size();
    if (n == 0) return;

    // compute local state
    Dcmf R_WB(_attitude);
    Dcmf R_BW = R_WB.transpose();
    Eulerf euler_current(_attitude);
    Eulerf euler_WV(0.f, 0.f, euler_current.psi());
    Dcmf R_WV(euler_WV);
    Dcmf R_VW = R_WV.transpose();
    Dcmf R_BV = R_BW * R_WV;

    _body_acceleration_setpoint = R_BW * acceleration_setpoint;
    _body_velocity = R_BW * velocity;
    _vehicle_velocity = R_VW * velocity;


    // composite collision CBF
    // nu1_i
    for(size_t i = 0; i < n; i++) {
        float nu_i0 = _obstacles[i].norm_squared() - (_epsilon * _epsilon);
        float Lf_nu_i0 = -2.f * _obstacles[i].dot(_body_velocity);
        _nu1[i] = Lf_nu_i0 - _pole0 * nu_i0;
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

    // analytical QP solution from: https://arxiv.org/abs/2206.03568
    // float eta = 0.f;
    // float Lg_h_mag2 = Lg_h.norm_squared();
    // if (Lg_h_mag2 > 1e-5f) {
    //     eta = -(Lf_h + Lg_h_u + _alpha*h) / Lg_h_mag2;
    // }
    // Vector3f local_correction = (eta > 0.f ? eta : 0.f) * Lg_h;
    // dbg.x = local_correction(0);
    // dbg.y = local_correction(1);
    // dbg.z = local_correction(2);
    // local_accel_setpoint += local_correction;
    // acceleration_setpoint = R_IB * local_accel_setpoint;

    _debug_msg.h = h;
    // _debug_msg.virtual_obstacle = ; // TODO: marvin
    _debug_msg.input[0] = acceleration_setpoint(0);
    _debug_msg.input[1] = acceleration_setpoint(1);
    _debug_msg.input[2] = acceleration_setpoint(2);

    // solve QP
    // quadratic cost x^T*H*x
    real_t H[NV*NV] = {1.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 3.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0};
    // linear cost matrix g*x
    real_t  g[NV] = { 0.0, 0.0, 0.0, 50.0, 50.0 };
    // constraint matrix A
    real_t  A[NC*NV] = {(real_t)Lg_h(0), (real_t)Lg_h(1), (real_t)Lg_h(2), (real_t)0.0, (real_t)0.0,
                      (real_t)0.0, (real_t)0.0, (real_t)0.0, (real_t)1.0, (real_t)0.0,
                      (real_t)Lg_h1(0), (real_t)Lg_h1(1), (real_t)Lg_h1(2), (real_t)1.0, (real_t)0.0,
                      (real_t)0.0, (real_t)0.0, (real_t)0.0, (real_t)0.0, (real_t)1.0,
                      (real_t)Lg_h2(0), (real_t)Lg_h2(1), (real_t)Lg_h2(2), 0.0, (real_t)1.0};
    // bounds on Ax
    real_t  lbA[NC] = { (real_t)(-Lf_h - kappaFunction(h, _alpha) - Lg_h_u), 0.0, (real_t)(-Lf_h1 - _alpha_fov * h1), 0.0, (real_t)(-Lf_h2 - _alpha_fov * h2) };
    real_t* ubA = NULL;
    // bounds on x
    real_t* lb = NULL;
    real_t* ub = NULL;
    int_t nWSR = 50;

    qp = QProblem(NV, NC);
    qp.setPrintLevel(PL_NONE);
    returnValue qp_status = qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

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
    
    _debug_msg.output[0] = acceleration_setpoint(0);
    _debug_msg.output[1] = acceleration_setpoint(1);
    _debug_msg.output[2] = acceleration_setpoint(2);

    dbg.timestamp = timestamp;
    dbg.x = h;
    dbg.y = h1;
    dbg.z = (float) n;
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
