#ifndef PID_H
#define PID_H


class PID
{
public:
    PID(float p, float i, float d);
    PID();
    void reset();
    float update(float *set_point, float *value_obj);
    void setOutputLimits(float *u1, float *u2);
    void setCoefficients(float p, float i, float d);
    //void setSamplingTime(unsigned long *t);
    void setSamplingTime(float *t);

    float getKi();
    float getKp();
    float getKd();
    float getSamplingTime();

private:
    float k_i;
    float k_p;
    float k_d;

    float _past_u;
    float _past_err;
    float _past_past_err;

    float _sampling_time;

    float *_u1;
    float *_u2;
};

#endif // PID_H
