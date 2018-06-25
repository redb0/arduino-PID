#ifndef PID_H
#define PID_H

/**
 * @brief The PID class - библиотека с реализацией PID-контроллера.
 * При реализации PID-регулятора используется рекурентная формула:
 * U(t) = U(t-1) + P + I + D,
 * где U(t) - искомое управляющее воздействие в момент времени t;
 *     U(t-1) - управлющее воздействие на прошлой итерации t-1;
 *     P - пропорциональная составляющая;
 *     I - интегральна составляющая;
 *     D - дифференциальна составлющая.
 *
 * В стандартном представлении PID-регуляятора
 * составляющие представлены следующим образом:
 * E(t) = x*(t) - x(t)
 * P = Kp * (E(t) - E(t-1)),
 * I = Ki * E(t),
 * D = Kd * (E(t) - 2 * E(t-1) + E(t-2)),
 * где E(t) - сигнал рассогласования в текущий момент времени t;
 *     x*(t) - значенеи уставки в момент времени t;
 *     x(t) - значенеи выхода объекта в момент времени t;
 *     Kp - пропорциональный коэффициент;
 *     Ki - интегральный коэффициент;
 *     Kd - дифференциальный коэффициент.
 *
 * Подробнее о PID-регулторе: https://en.wikipedia.org/wiki/PID_controller.
 *
 * Подробную документацию на каждый метод смотрите в файле "pid.cpp".
 */
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

    float update(float *set_point, float *value_obj);
    void setOutputLimits(float *u1, float *u2);
    void setCoefficients(float p, float i, float d,
                         bool direction);
    void setSamplingTime(float*);
    void setReverseDirection();
    void setInputOutput(float, float, float);
    //void initialize();
    void setMode(bool);

    float getKi();
    float getKp();
    float getKd();
    float getSamplingTime();
    bool getMode();

private:
    bool _no_dk;            // флаг для нейтрализаци всплесков по управлению
    bool _pom;              // флаг дл использовани другого вида пропорциональной
                            // составлющей
    bool _mode;             // режим работы контроллера
                            // (true - ручной, false - автоматический)

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
