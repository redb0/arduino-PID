/**
 * Пример использования PID-регулятора.
 * Пример подготовлен для адаптивного управления сервомотором, подключенному к цифровому пину 5.
 * Начальное положение сервомотора 0. Ограничения на управление -90 <= U(t) <= 90.
 * 
 * Уставка зависит от времени работы контроллера (меняется раз в 30 секунд) и задаетсяя функцией getSetPoint().
 * Текущее положение сервомотора эмулируется с помощью выражения X + U, в функции getX(int *u, int *x).
 * 
 * Результаты каждой итерации можно посмотреть в Serial порту.
 * 
 * По умолчанию используется формула вида (стандартная рекурентная):
 * U(t) = U(t-1) + Kp * (E(t) - E(t-1)) + Ki * E(t) + Kd * (E(t) - 2 * E(t-1) + E(t-2))
 * Раскомментируйте 24 строку и закомментируйте 27 для использования формулы вида:
 * U(t) = U(t-1) - Kp * (X(t) - X(t-1)) + Ki * E(t) - Kd * (X(t) - 2 * X(t-1) + X(t-2))
 */

#include <Servo.h>
#include <pid.h>

#define PIN_OUTPUT 5               // пин подключения сервомотора

float setPoint;                    // уставка
int u;                             // управляющее воздействие
int output;                        // выход объекта (положение сервомотора)
int last_u;                        // прошлое значение управляющего воздействия
int last_output;                   // прошлое значение выхода объекта
float samplingTime = 0.5;          // время дискретизации в секундах
unsigned long operating_time;      // таймер рабочего времени
unsigned long sampling_timer;      // таймер времени дискретизации
unsigned long last_sampling_timer; // прошлое значение таймера времени дискретизации
unsigned long time_period = (unsigned long)30*1000; // продолжительность периодов (время между изменением уставки)

//PID myPID(1.0, 0.15, 0.15, 1.0, NO_D_K, PoM, DIRECT, ACTIVE); // раскомментируйте эту строку и закомментируйте строку 20
                                                              // для использование PID-регуляятора с альтернативными 
                                                              // пропорциональной и дифференциальной составляющей
PID myPID(1.0, 0.15, 0.15); // использование стандартной рекурентной формулы PID-регулятора

Servo myservo;

float getSetPoint() {
  if (operating_time <= time_period) return (float)50;
  if ((operating_time > time_period) && (operating_time <= 2*time_period)) return (float)150;
  if ((operating_time > 2*time_period) && (operating_time <= 3*time_period)) return (float)100;
  if (operating_time > 3*time_period) return (float)30;
}

int getX(int *u, int *x) {  
  return *u + *x;
}

void setup() {
  Serial.begin(9600);
  myservo.attach(PIN_OUTPUT);
  myservo.write(0); // начально положение сервомотора
  
  float u1 = -90; // ограничения на управление
  float u2 = 90;
  myPID.setSamplingTime(&samplingTime);
  myPID.setOutputLimits(&u1, &u2);
  myPID.setMode(ACTIVE);
  
  operating_time = millis();
  last_sampling_timer = millis();
  last_u = 0;
  last_output = 0;

  Serial.print("K_p: ");
  Serial.println(myPID.getKp());
  Serial.print("K_i: ");
  Serial.println(myPID.getKi());
  Serial.print("K_d: ");
  Serial.println(myPID.getKd());
  Serial.print("Sampling Time: ");
  Serial.println(myPID.getSamplingTime());
  Serial.print("Mode: ");
  Serial.println(myPID.getMode());
  Serial.println("---------------");
}

void loop() {
  operating_time = millis();
  sampling_timer = millis();
  
  // контролируем время дискретизации
  if (sampling_timer - last_sampling_timer >= (unsigned long)(samplingTime*1000)) {
    last_sampling_timer = millis();

    output = getX(&last_u, &last_output);  // эмулируем измерение положения сервомотора
    setPoint = getSetPoint();              // получаем уставку
    
    float x1 = (float)output;
    u = (int)myPID.update(&setPoint, &x1); // вычисляем управляющее воздействие

    myservo.write(output);                 // изменяем положение сервомотора
    
    last_u = u;                            // обновляем старые значения
    last_output = output;

    Serial.print("Уставка: ");
    Serial.println(setPoint);
    Serial.print("Объект: ");
    Serial.println(output);
    Serial.print("Управление: ");
    Serial.println(u);
  }
}

