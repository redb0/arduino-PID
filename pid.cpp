#include "pid.h"

PID::PID(float p, float i, float d) {
    k_i = i;
    k_p = p;
    k_d = d;
    _u1 = 0;
    _u2 = 0;
    _past_u = 0;
    _past_err = 0;
    _past_past_err = 0;
    _sampling_time = 0.1;
}

PID::PID() {
    k_i = 0;
    k_p = 0;
    k_d = 0;
    _u1 = 0;
    _u2 = 0;
    _past_u = 0;
    _past_err = 0;
    _past_past_err = 0;
    _sampling_time = 0.1;
}

void PID::reset() {
    k_i = 0;
    k_p = 0;
    k_d = 0;
    _u1 = 0;
    _u2 = 0;
    _past_u = 0;
    _past_err = 0;
    _past_past_err = 0;
    _sampling_time = 0.1;
}

void PID::setOutputLimits(float *u1, float *u2) {
    if (*u1 < *u2) {
        _u1 = u1;
        _u2 = u2;
    } else {
        return;
    }
}

void PID::setSamplingTime(float *t) {
    if (*t <= 0) return;
    //_sampling_time = (float)*t / (float)1000;
    _sampling_time = *t;
    k_i = k_i * _sampling_time;
    k_d = k_d / _sampling_time;
}

void PID::setCoefficients(float p, float i, float d) {
    if ((p < 0) || (i < 0) || (d < 0)
            || (_sampling_time <= 0)) return;
    //float sampling_time_sec = ((float)*_sampling_time) / 1000;

    k_p = p;
    k_i = i * _sampling_time;
    k_d = d / _sampling_time;
}

float PID::update(float *set_point, float *value_obj) {
    //u = past_u + k_p * (err - past_err) + k_i * err + k_d * (err - 2*past_err - past_past_err)
    float err = *set_point - *value_obj;
    float u;
    u = _past_u + k_p * (err - _past_err) + k_i * err + k_d * (err - 2 * _past_err + _past_past_err);
    _past_past_err = _past_err;
    _past_err = err;
    //_past_u = u;
    if (*_u1 < *_u2) {
        if (u < *_u1) u = *_u1;
        if (u > *_u2) u = *_u2;
    }
    _past_u = u;
    return u;
}

float PID::getKd() {return k_d;}
float PID::getKi() {return k_i;}
float PID::getKp() {return k_p;}
float PID::getSamplingTime() {return _sampling_time;}
