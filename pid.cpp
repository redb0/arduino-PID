#include "pid.h"

/**
 * @brief PID::PID - конструктор.
 * @param p         - пропорциональный коэффициент.
 * @param i         - интегральный коэффициент.
 * @param d         - дифференциальный коэффициент.
 * @param s_t       - врем дискретизации в секундах.
 * @param no_dk     - флаг (NO_D_K) для использования альтернативной
 *                    дифференциальной составляющей.
 * @param pom       - флаг для использования альтернативной
 *                    пропорциональной составляющей (PoE - стандартна
 *                    пропорциональная составляющая, основанная на изменении
 *                    ошибки. PoM - пропорциональная составляющая,
 *                    основанная на изменении значения выхода объекта.).
 * @param direction - флаг направления (DIRECT - прямое, REVERSE - обратное).
 * @param mode      - флаг режима работы (ACTIVE - автоматический,
 *                    NOT_ACTIVE - ручной).
 */
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
}

/**
 * @brief PID::PID - конструктор.
 * Устанавливает только коэффициенты. Время дискретизации по умолчанию 0.1 секунды.
 * Автоматический режим работы, используется классическая рекурентна формула.
 * @param p - пропорциональный коэффициент.
 * @param i - интегральный коэффициент.
 * @param d - дифференциальный коэффициент.
 */
PID::PID(float p, float i, float d) {
    _u1 = 0;
    _u2 = 0;
    _past_u = 0;
    _past_err = 0;
    _past_past_err = 0;
    _last_input = 0;
    _last_last_input = 0;

    _sampling_time = 0.1;
    setCoefficients(p, i, d, DIRECT);

    _no_dk = !NO_D_K;
    _pom = PoE;
    _mode = ACTIVE;
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

/**
 * @brief PID::setMode - установка режима работы.
 *
 * Предназначен для включения и выключения контроллера.
 * Режимы: автоматический (ACTIVE), ручной (NOT_ACTIVE).
 * В ручном режиме не происходит вычисление управление, но отслеживаются
 * значения входа выхода объекта.
 * @param mode - флаг режима работы. Если передана константа ACTIVE,
 *               то режим работы - автоматический (будет произведено вычисление
 *               значения управления). Если передано NOT_ACTIVE, то режим
 *               работы - ручной (управление не вычисляется).
 */
void PID::setMode(bool mode) {
    _mode = mode;
}

/**
 * @brief PID::setOutputLimits - установка ограничений на управление.
 *
 * При наличии ограничений управление будет лежать в диапазоне u1 < u < u2, u1 < u2.
 * @param u1 - указатель на значение ограничения снизу.
 * @param u2 - указатель на значение ограничения сверху.
 */
void PID::setOutputLimits(float *u1, float *u2) {
    if (*u1 < *u2) {
        _u1 = u1;
        _u2 = u2;
    } else {
        return;
    }
}

/**
 * @brief PID::setSamplingTime - метод установки времени дискретизации.
 *
 * Время принимается в секундах.
 */
void PID::setSamplingTime(float *t) {
    if (*t <= 0) return;
    //_sampling_time = (float)*t / (float)1000;
    k_i = k_i * (*t / _sampling_time);
    k_d = k_d * _sampling_time / (*t);
    _sampling_time = *t;
}

/**
 * @brief PID::setCoefficients - метод установки коэффициентов.
 *
 * Коэффициенты устанавливаются как положительные числа (для прямой и обратной зависимости).
 * Если зависимость обратная ставиться флаг REVERSE, иначе DIRECT.
 * @param p         - пропорциональный коэффициент.
 * @param i         - интегральный коэффициент.
 * @param d         - дифференциальный коэффициент.
 * @param direction - флаг обратного направления
 *                    (false - обратная зависимость, коэффициенты приводятся к виду -p, -i, -d,
 *                    true - прямая зависимость, коэффициенты остаются без изменения)
 */
void PID::setCoefficients(float p, float i, float d, bool direction) {
    if ((p < 0) || (i < 0) || (d < 0) || (_sampling_time <= 0)) return;

    k_p = p;
    k_i = i * _sampling_time;
    k_d = d / _sampling_time;
    if (direction == REVERSE) {
        setReverseDirection();
    }
}

/**
 * @brief PID::setReverseDirection - установить обратное направление в контроллере.
 *
 * Преобразует текущие коэффициенты к противоположному знаку.
 */
void PID::setReverseDirection() {
    k_p = 0 - k_p;
    k_i = 0 - k_i;
    k_d = 0 - k_d;
}

/**
 * @brief PID::setInputOutput - метод установки значений входа: u(t-1), u(t-2)
 *                              и значения объекта (выхода): x(t-1).
 *
 * Используется в случае, когда регултор включается в уже работающий процесс.
 * @param input        - значение входа в момент времени t-1, u(t-1).
 * @param last_input   - значение входа в момент времени t-2, u(t-2).
 * @param object_value - значение объекта (выхода) в момен времени t-1, x(t-1).
 */
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

/**
 * @brief PID::update - производит расчет нового значения управления (u(t)).
 *
 * Для расчета используются следующие формулы PID-регулятора:
 * 1) Классическая рекурентная формула ():
 *     U(t) = U(t-1) + P + I + D,
 *     P = Kp * (E(t) - E(t-1)),
 *     I = Ki * E(t),
 *     D = Kd * (E(t) - 2 * E(t-1) + E(t-2)),
 *     где E(t) = x*(t) - x(t),
 *     x*(t) - уставка, x(t) - значение объекта (выхода) в момент времени t.
 * 2) Сопротивление скачкам сигнала рассогласования:
 *     U(t) = U(t-1) + P + I - D,
 *     D = Kd * (x(t) - 2 * x(t-1) + x(t-2)).
 * 3) Пропорциональнаяя составляющая на основе значений объекта (выхода):
 *     U(t) = U(t-1) - P + I + D,
 *     P = Kp * (X(t) - X(t-1)).
 * Если установлен автоматический режим (ACTIVE) - будет рассчитано значение
 * управления.
 * Если установлен ручной режим работы (NOT_ACTIVE) - будет отслеживаться
 * значения входов и ошибки, в качестве управления будет возвращенн 0.
 * @param set_point - уставка (x*(t)).
 * @param value_obj - значение объекта на итерации t, x(t).
 * @return значение управления u(t), если ACTIVE, иначе 0.
 */
float PID::update(float *set_point, float *value_obj) {
    float err = *set_point - *value_obj;
    float u = 0;

    // Classic PID
    // U(t) = U(t-1) + P + I + D
    // P = Kp * (E(t) - E(t-1))
    // I = Ki * E(t)
    // D = Kd * (E(t) - 2 * E(t-1) + E(t-2))
    // with NO_D_K
    // D = -[Kd * (X(t) - 2 * X(t-1) + X(t-2))]
    // with PoM
    // P = -[Kp * (X(t) - X(t-1))]

    if (_mode) { // _mode == ACTIVE
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

    _last_last_input = _last_input;
    _last_input = *value_obj;
    _past_past_err = _past_err;
    _past_err = err;

    if (!_mode) return 0; // _mode == NOT_ACTIVE

    if (*_u1 < *_u2) {
        if (u < *_u1) u = *_u1;
        if (u > *_u2) u = *_u2;
    }
    _past_u = u;
    return u;
}

//Property
float PID::getKd() {return k_d;}
float PID::getKi() {return k_i;}
float PID::getKp() {return k_p;}
float PID::getSamplingTime() {return _sampling_time;}
bool PID::getMode() {return _mode;}
