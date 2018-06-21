#ifndef PID_H
#define PID_H

//вкл/выкл контроллера
//http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-onoff/
//инициализация контроллера
//http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-initialization/
//направление ??? (почти готово)
//http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/
//http://brettbeauregard.com/blog/
//https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp
class PID
{
public:
    #define NO_D_K true
    #define PoM false
    #define PoE true

    PID(float p, float i, float d, bool no_dk, bool pom);
    PID(float p, float i, float d);
    PID();
    void reset();
    float update(float *set_point, float *value_obj);
    void setOutputLimits(float *u1, float *u2);
    void setCoefficients(float p, float i, float d);
    //void setSamplingTime(unsigned long *t);
    void setSamplingTime(float*);

    float getKi();
    float getKp();
    float getKd();
    float getSamplingTime();

private:
    bool _no_dk;
    bool _pom;

    float k_i;
    float k_p;
    float k_d;

    float _past_u;
    float _past_err;
    float _past_past_err;

    float _last_input;
    float _last_last_input;

    float _sampling_time;

    float *_u1;
    float *_u2;
};

#endif // PID_H
