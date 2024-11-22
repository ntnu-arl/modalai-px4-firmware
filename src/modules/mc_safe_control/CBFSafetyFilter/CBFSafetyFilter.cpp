#include <CBFSafetyFilter.hpp>

#include <math.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/debug_vect.h>
#include <string.h>


static struct debug_vect_s dbg;
static orb_advert_t pub_dbg;

CBFSafetyFilter::CBFSafetyFilter() {
    // _obstacles.emplace_back(10.f, 0.f, -15.f);
    // _obstacles.emplace_back(10.f, 0.f, -14.f);
    // _obstacles.emplace_back(10.f, 0.f, -13.f);
    // _obstacles.emplace_back(10.f, 0.f, -12.f);
    // _obstacles.emplace_back(10.f, 0.f, -11.f);
    _obstacles.emplace_back(10.f, 3.f, -10.f);
    _obstacles.emplace_back(5.f, -3.f, -8.f);
    _obstacles.emplace_back(15.f, 0.f, -9.f);
    // _obstacles.emplace_back(10.f, 0.f, -9.f);
    // _obstacles.emplace_back(10.f, 0.f, -8.f);
    // _obstacles.emplace_back(10.f, 0.f, -7.f);
    // _obstacles.emplace_back(10.f, 0.f, -6.f);
    // _obstacles.emplace_back(10.f, 0.f, -5.f);

    dbg.x = 0.0f;
    dbg.y = 0.0f;
    dbg.z = 0.0f;
    pub_dbg = orb_advertise(ORB_ID(debug_vect), &dbg);
}

void CBFSafetyFilter::update(Vector3f& acceleration_setpoint, uint64_t timestamp) {
    const size_t n = _obstacles.size();

    if (n == 0) return;

    _h1.resize(n);
    _rel_pos.resize(n);

    // _h1 = {h_{i, 1}, i=0...n-1}
    for(size_t i = 0; i < n; i++) {
        _rel_pos[i] = _position - _obstacles[i];
        float hi0 = _rel_pos[i].norm_squared() - (_epsilon * _epsilon);
        float Lf_hi0 = 2.f * _rel_pos[i].dot(_velocity);
        float hi1 = Lf_hi0 - _lambda0 * hi0;
        _h1[i] = hi1;
        
        dbg.x = hi0;
        dbg.y = hi1;
    }

    // h(x)
    float exp_sum = 0.f;
    for(size_t i = 0; i < n; i++) {
        exp_sum += expf(-_kappa * saturate(_h1[i] / _gamma));
    }
    float h = -(_gamma / _kappa) * logf(exp_sum);
    dbg.z = h;

    // L_{g}h(x)
    Vector3f Lg_h(0.f, 0.f, 0.f);
    for(size_t i = 0; i < n; i++) {
        Vector3f Lg_hi1 = 2.f * _rel_pos[i];
        float phi_i = saturate_derivative(_h1[i] / _gamma) * expf(-_kappa * saturate(_h1[i] / _gamma));
        Lg_h += phi_i * Lg_hi1;
    }
    Lg_h /= exp_sum;
    // L_{g}h(x) * u, u = k_n(x) = a
    float Lg_h_u = Lg_h.dot(acceleration_setpoint);

    // L_{f}h(x)
    float Lf_h = 0.f;
    for(size_t i = 0; i < n; i++) {
        float Lf_hi1 = 2.f * (_velocity - _lambda0 * _rel_pos[i]).dot(_velocity);
        float phi_i = saturate_derivative(_h1[i] / _gamma) * expf(-_kappa * saturate(_h1[i] / _gamma));
        Lf_h += phi_i * Lf_hi1;
    }
    Lf_h /= exp_sum;

    // analytical QP solution from: https://arxiv.org/abs/2206.03568
    float eta = 0.f;
    float Lg_h_mag2 = Lg_h.norm_squared();
    if (Lg_h_mag2 > _zero_eps) {
        eta = -(Lf_h + Lg_h_u + _alpha*h) / Lg_h_mag2;
    }
    Vector3f acceleration_correction = (eta > 0.f ? eta : 0.f) * Lg_h;
    acceleration_setpoint += acceleration_correction;

    dbg.timestamp = timestamp;
    dbg.x = acceleration_setpoint(0);
    dbg.y = acceleration_setpoint(1);
    dbg.z = acceleration_setpoint(2);
    orb_publish(ORB_ID(debug_vect), pub_dbg, &dbg);
}


float CBFSafetyFilter::saturate(float x) {
    return tanh(x);
}

float CBFSafetyFilter::saturate_derivative(float x) {
    float th = tanh(x);
    return 1.f - (th * th);
}