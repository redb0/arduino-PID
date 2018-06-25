#ifndef PID_H
#define PID_H

//вкл/выкл контроллера ???
//http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-onoff/

//инициализация контроллера !
//http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-initialization/

//направление !
//http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/

//http://brettbeauregard.com/blog/
//https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp
class PID
{
public:
    #define NO_D_K true
    #define PoM false
    #define PoE true

    #define DIRECT true
    #define REVERSE false

    #define ACTIVE true
    #define NOT_ACTIVE false

    PID(float p, float i, float d, float s_t,
        bool no_dk, bool pom, bool, bool);
    PID(float p, float i, float d);
    PID();
    void reset();

    float update(float *set_point, float *value_obj); //
    void setOutputLimits(float *u1, float *u2);       // +
    void setCoefficients(float p, float i, float d,
                         bool direction);             // +
    void setSamplingTime(float*);                     // +
    void setReverseDirection();                       // +
    void setInputOutput(float, float, float);         // +
    //void initialize();                                //
    void setMode(bool);                               // +

    float getKi();
    float getKp();
    float getKd();
    float getSamplingTime();

private:
    bool _no_dk;            // флаг для нейтрализаци всплесков по управлению
    bool _pom;              // флаг дл использовани другого вида пропорциональной составлющей
    bool _mode;             // режим работы контроллера (true - ручной, false - автоматический)

    float k_i;              // интегральный коэффициент
    float k_p;              // пропорциональный коэффициент
    float k_d;              // дифференцальный коэффициент

    float _past_u;          // управление на прошлом такте u(t-1)
    float _past_err;        // ошибка на прошлом такте e(t-1)
    float _past_past_err;   // ошибка на такте t-2, e(t-2)

    float _last_input;      // значение объекта на прошлом такте, x(t-1)
    float _last_last_input; // значение объекта на такте t-2, x(t-2)

    float _sampling_time;   // время дискретизации в секундах

    float *_u1;             // ограничение снизу на управление
    float *_u2;             // ограничение сверху на управление
};

#endif // PID_H
