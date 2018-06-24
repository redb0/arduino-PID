#include "pid.h"

PID::PID(float p, float i, float d, float s_t,
         bool no_dk, bool pom, bool direction, bool mode) {
    _u1 = 0;
    _u2 = 0;
    _past_u = 0;
    _past_err = 0;
    _past_past_err = 0;
    _last_input = 0;
    _last_last_input = 0;

    if (s_t > 0) {
        _sampling_time = s_t;
    } else {
        _sampling_time = 0.1;
    }
    setCoefficients(p, i, d, direction);

    _no_dk = no_dk;
    _pom = pom;
    _mode = mode;
    //with_initialization = false;
}

PID::PID(float p, float i, float d) {
    _u1 = 0;
    _u2 = 0;
    _past_u = 0;
    _past_err = 0;
    _past_past_err = 0;
    _last_input = 0;
    _last_last_input = 0;

    _sampling_time = 0.1;
    k_i = i * _sampling_time;
    k_p = p;
    k_d = d / _sampling_time;

    _no_dk = !NO_D_K;
    _pom = PoE;
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
    _last_input = 0;
    _last_last_input = 0;
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
    _last_input = 0;
    _last_last_input = 0;
    _sampling_time = 0.1;
}

void PID::setMode(bool mode) {
    _mode = mode;
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
    k_i = k_i * (*t / _sampling_time);
    k_d = k_d * _sampling_time / (*t);
    _sampling_time = *t;
}

void PID::setCoefficients(float p, float i, float d, bool direction) {
    if ((p < 0) || (i < 0) || (d < 0) || (_sampling_time <= 0)) return;

    k_p = p;
    k_i = i * _sampling_time;
    k_d = d / _sampling_time;
    if (direction == REVERSE) {
        setReverseDirection();
    }
}

void PID::setReverseDirection() {
    k_p = 0 - k_p;
    k_i = 0 - k_i;
    k_d = 0 - k_d;
}

void PID::setInputOutput(float input, float last_input, float object_value) {
    //with_initialization = true;
    _last_input = input;
    _last_input = last_input; // ??? = 0
    _last_last_input = 0;

    _past_u = object_value;
    if (*_u1 < *_u2) {
        if (_past_u < *_u1) _past_u = *_u1;
        if (_past_u > *_u2) _past_u = *_u2;
    }
}

float PID::update(float *set_point, float *value_obj) {
    float err = *set_point - *value_obj;
    float u = 0;

    //if (_mode == NOT_ACTIVE) return 0;

    // Classic PID
    // U(t) = U(t-1) + P + I + D
    // P = Kp * (E(t) - E(t-1))
    // I = Ki * E(t)
    // D = Kd * (E(t) - 2 * E(t-1) + E(t-2))
    // with NO_D_K
    // D = -[Kd * (X(t) - 2 * X(t-1) + X(t-2))]
    // with PoM
    // P = -[Kp * (X(t) - X(t-1))]

    if (_mode == NOT_ACTIVE) {
        u += _past_u + k_i * err;
        if (u > *_u2) u = *_u2;
        if (u < *_u1) u = *_u1;
        if (_no_dk == NO_D_K) {
            u -= k_d * (*value_obj - 2 * _last_input + _last_last_input);
        } else {
            u += k_d * (err - 2 * _past_err + _past_past_err);
        }
        if (_pom == PoM) {
            u -= k_p * (*value_obj - _last_input);
        } else {
            u += k_p * (err - _past_err);
        }
    }

    //u = _past_u + k_p * (err - _past_err) + k_i * err + k_d * (err - 2 * _past_err + _past_past_err);
    _last_last_input = _last_input;
    _last_input = *value_obj;
    _past_past_err = _past_err;
    _past_err = err;

    if (_mode == NOT_ACTIVE) return 0;

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
