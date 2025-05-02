/*
Режимы работы:
RUN - основной режим, где робот следует по линии с учетом всех условий.
CALIBRATE - режим калибровки датчиков с сохранением в EEPROM.
DEBUG - отладочный вывод в Serial.
Для использования нужно раскомментировать нужный режим в начале кода и загрузить программу в Arduino. В калибровочном режиме следуйте инструкциям в Serial-мониторе.

Основной режим:
Робот ждет, пока оба датчика не перестанут видеть белое (бумагу убрали).
Через 85 секунд начинается движение.
Робот останавливается через 98 секунд или на втором перекрестке.
При потере линии более чем на 5 секунд робот останавливается.
После остановки вторая сервомашинка начинает вращаться.

Калибровочный режим:
c - калибровка белого (датчики должны быть на белом).
s - сохранение калибровки в EEPROM.
l - загрузка калибровки из EEPROM.
p - вывод текущих калибровочных значений.

Гистерезис:
Пороговые значения для белого рассчитываются как среднее значение + offset (WHITE_THRESHOLD_OFFSET).
Функция isOnWhite принимает третий параметр prevState (предыдущее состояние датчика)
Для входа на линию (когда ранее датчик был на поле): value < (threshold - HYSTERESIS)
Для выхода с линии (когда ранее датчик был на линии): value < (threshold + HYSTERESIS)
Вызов функции в runMainMode() передает предыдущее состояние датчиков

Когда датчик над полем (не на линии): Чтобы перейти в состояние "на линии", значение должно упасть ниже threshold - HYSTERESIS
Когда датчик на линии: Чтобы перейти в состояние "не на линии", значение должно подняться выше threshold + HYSTERESIS
Это создает "зону нечувствительности" между двумя порогами, что предотвращает дребезг при переходе через границу линии.

Защита от множественного счета одного перекрестка
Фильтрацию кратковременных ложных срабатываний определения перекрестка

Управление сервомашинками:
Первая сервомашинка поворачивает колесо в зависимости от того, какой датчик видит линию.
Вторая сервомашинка начинает вращаться после остановки робота.

*/

/*
 * Программа управления мобильным роботом с двумя аналоговыми датчиками линии
 * Особенности:
 * - Белая линия на черном фоне
 * - Ожидание 85 сек после удаления стартовой бумаги
 * - Работа в течение 98 сек
 * - Подсчет перекрестков
 * - Остановка на 2-м перекрестке
 * - Активация доп. сервопривода после остановки
 * - Защита от потери линии (5 сек)
 */

// ВНИМАНИЕ: при подключении датчиков пины 2,3,4 могут использоваться шилдом для управления моторами. Неверная конфигурация использования моторов может приводить к неверным показаниям.
//           пины 2,3,4 - цифровые, они не используются в данной программе, но все же стоит учитывать влияние на них.
//           пин 4 влияет на мотор 3, поэтому нельзя использовать датчик на 4м пине...

#include <AFMotor.h>          // Библиотека для управления моторами через L293D
#include <Servo.h>            // Библиотека для управления сервомашинками
#include <EEPROM.h>           // Библиотека для работы с EEPROM (сохранение калибровки)

// Режимы работы (раскомментируй нужный)
#define RUN                   // Основной режим
//#define CALIBRATE           // Калибровочный режим
//#define DEBUG               // Отладочный вывод

// Пины
#define LEFT_SENSOR_PIN A3    // Левый датчик линии
#define RIGHT_SENSOR_PIN A2   // Правый датчик линии
#define SERVO1_PIN 10         // Первая сервомашинка (поворотное колесо)
#define SERVO2_PIN 9          // Вторая сервомашинка (веселое действие)

// Параметры сервомашинок
#define SERVO1_MIN_ANGLE 70    // Минимальный угол сервомашинки
#define SERVO1_MAX_ANGLE 110   // Максимальный угол сервомашинки
#define SERVO1_CENTER_ANGLE 90 // ((SERVO1_MAX_ANGLE + SERVO1_MIN_ANGLE) / 2)  // Центральное положение

#define SERVO2_MIN_ANGLE 30    // Минимальный угол сервомашинки
#define SERVO2_MAX_ANGLE 150   // Максимальный угол сервомашинки

// Параметры движения
#define MOTOR_SPEED 150       // Скорость мотора (0-255)
#define NO_LINE_TIMEOUT 5000  // Таймаут отсутствия линии (5 секунд)
#define START_DELAY 85000     // Задержка перед стартом (85 секунд)
#define STOP_TIME 98000       // Время остановки (98 секунд после старта)
#define TIME_MAX_SPEED 50 // Время для включения максимального включения моторов (надо настраивать)
#define INTERVAL_MAX_SPEED 1000

// Калибровка датчиков
#define CALIBRATION_SAMPLES 50 // Количество измерений для калибровки
#define WHITE_THRESHOLD_OFFSET 20 // Отступ от калибровочного значения (чтобы порог был выше)
#define HYSTERESIS 15         // Гистерезис для датчиков линии

// Состояния
enum State {
  WAITING_FOR_START,         // Ожидание старта (датчики на белом)
  DELAY_BEFORE_MOVE,         // Задержка перед движением
  FOLLOWING_LINE,            // Движение по линии
  STOPPED,                   // Остановка
  LOST_LINE                  // Потеря линии
};

// Глобальные переменные
AF_DCMotor motor(3);         // Мотор подключен к M3
Servo servo1;                // Первая сервомашинка (поворотное колесо)
Servo servo2;                // Вторая сервомашинка

State currentState = WAITING_FOR_START;
unsigned long startTime = 0;
unsigned long lastLineTime = 0;
int intersectionsCount = 0;
bool isOnIntersection = false; // Флаг нахождения на перекрестке
unsigned long intersectionEnterTime = 0; // Время входа на перекресток
unsigned long lastSpeedingTime = 0; // время последнего ускорения (для интервального увеличения скорости)

// Калибровочные значения (хранятся в EEPROM)
struct CalibrationData {
  int leftWhite;
  int rightWhite;
};
CalibrationData calibration;

// Прототипы функций
void runMainMode();
void runCalibrationMode();
void readSensors(int &leftValue, int &rightValue);
bool isOnWhite(int sensorValue, int whiteThreshold, bool prevState);
void saveCalibration();
void loadCalibration();
void printSensorValues(int leftValue, int rightValue);

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  // Настройка пинов
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  //pinMode(4, INPUT);  //настройка на вход влияет на работу мотора на выходах М3
  pinMode(A4, INPUT);

  // Инициализация сервомашинок
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(SERVO1_CENTER_ANGLE);
  servo2.write(SERVO1_MIN_ANGLE);

  // Инициализация мотора
  motor.setSpeed(0);
  motor.run(RELEASE);

  // Загрузка калибровки
  loadCalibration();

  #ifdef RUN
  runMainMode();
  #elif defined(CALIBRATE)
  runCalibrationMode();
  #endif
}

void loop() {
  // Основной цикл не используется, так как режимы работают в своих функциях
}

// Основной режим работы
void runMainMode() {
  int leftValue, rightValue;
  bool leftOnWhite, rightOnWhite;
  bool prevLeftOnWhite = false;
  bool prevRightOnWhite = false;

  while (true) {
    readSensors(leftValue, rightValue);
     // Проверка с учетом гистерезиса и предыдущего состояния
    leftOnWhite = isOnWhite(leftValue, calibration.leftWhite, prevLeftOnWhite);
    rightOnWhite = isOnWhite(rightValue, calibration.rightWhite, prevRightOnWhite);

    #ifdef DEBUG
    printSensorValues(leftValue, rightValue);
    #endif

    switch (currentState) {
      case WAITING_FOR_START:
        // Старт, когда хотя бы один датчик не на белом (бумагу убрали)
        if (!leftOnWhite || !rightOnWhite) {
          currentState = DELAY_BEFORE_MOVE; // смена состояния
          startTime = millis();
          #ifdef DEBUG
          Serial.println("START detected");
          #endif
        }
        break;

      case DELAY_BEFORE_MOVE:
        // Ждем 85 секунд перед движением
        if (millis() - startTime >= START_DELAY) {
        currentState = FOLLOWING_LINE; // смена состояния
          motor.setSpeed(MOTOR_SPEED+50); // реализация включения увеличенной скорости мотора
          lastSpeedingTime = millis(); // считываем время последнего "рывка мотора"
          motor.run(FORWARD);
          delay(20);
          motor.setSpeed(MOTOR_SPEED);          
          #ifdef DEBUG
          Serial.println("Starting movement");
          #endif
        }
        break;

      case FOLLOWING_LINE: // езда робота по линии
        // проверка времени для "рывка мотора"
        if (millis() - lastSpeedingTime >= INTERVAL_MAX_SPEED){
          //motor.setSpeed(MOTOR_SPEED+50); // увеличение скорости мотора
          //delay(TIME_MAX_SPEED); // пауза для реализации "рывка" мотора
          //motor.setSpeed(MOTOR_SPEED); // возвращение скорости мотора к первоначальной
          //#ifdef DEBUG
         //Serial.println("Motor full speed");
          //#endif
        }

        // Проверка на остановку по времени (98 секунд)               
        if (millis() - startTime >= STOP_TIME) {
          motor.setSpeed(0);
          motor.run(RELEASE);
          currentState = STOPPED; // смена состояния
          #ifdef DEBUG
          Serial.println("Stopped by timer");
          #endif
          break;
        }

        // Проверка на перекресток (оба датчика на белом)
        if (leftOnWhite && rightOnWhite) {
          if (!isOnIntersection) {
            // Впервые обнаружили перекресток
            isOnIntersection = true;
            intersectionEnterTime = millis();
            #ifdef DEBUG
            Serial.println("Entering intersection");
            #endif
          }
        } else {
          if (isOnIntersection) {
            // Выйти с перекрестка можно когда хотя бы один датчик не на белом
            if (!leftOnWhite || !rightOnWhite) {
              // Перекресток считается пройденным только если провели на нем >50мс
              if (millis() - intersectionEnterTime > 50) {
                intersectionsCount++;
                #ifdef DEBUG
                Serial.print("Intersection passed. Total: ");
                Serial.println(intersectionsCount);
                #endif

                // Остановка на втором перекрестке
                if (intersectionsCount >= 2) {
                  motor.setSpeed(0);
                  motor.run(RELEASE);
                  currentState = STOPPED;
                  #ifdef DEBUG
                  Serial.println("Stopped at intersection");
                  #endif
                }
              }
              isOnIntersection = false;
            }
          }
        }

        // Проверка на потерю линии
        if (!leftOnWhite && !rightOnWhite) {
          if (millis() - lastLineTime > NO_LINE_TIMEOUT) {
            motor.setSpeed(0);
            motor.run(RELEASE);
            currentState = LOST_LINE;
            #ifdef DEBUG
            Serial.println("Line lost");
            #endif
            break;
          }
        } else {
          lastLineTime = millis();
        }

        // Управление сервомашинкой (поворот колеса)
        if (leftOnWhite && !prevLeftOnWhite) {
          servo1.write(SERVO1_MIN_ANGLE); // Поворот влево
          #ifdef DEBUG
          Serial.println("Turning left");
          #endif
        } else if (rightOnWhite && !prevRightOnWhite) {
          servo1.write(SERVO1_MAX_ANGLE); // Поворот вправо
          #ifdef DEBUG
          Serial.println("Turning right");
          #endif
        } else if (!leftOnWhite && !rightOnWhite) {
          servo1.write(SERVO1_CENTER_ANGLE); // Прямо
        }
        break;

      case LOST_LINE:
      case STOPPED:
        // После остановки включаем вторую сервомашинку
        static unsigned long lastServoTime = 0;
        static int servo2Angle = SERVO2_MIN_ANGLE;
        static int x = SERVO2_MIN_ANGLE;  // Направление: +1 (вверх), -1 (вниз)

        if (millis() - lastServoTime > 350){
            x = SERVO2_MIN_ANGLE + SERVO2_MAX_ANGLE - x;
            servo2.write(x);
            lastServoTime = millis();
        }
        
        //плавное вращение - оставил для справки
        // static int direction = 1;  // Направление: +1 (вверх), -1 (вниз)

        // if (millis() - lastServoTime > 20) { // Плавное вращение
        //   direction *= 1 - 2 * (servo2Angle >= SERVO2_MAX_ANGLE || servo2Angle <= SERVO2_MIN_ANGLE);
        //   servo2Angle += direction*5;
        
        //   servo2.write(servo2Angle);
        //   lastServoTime = millis();
        //   Serial.print("servo2Angle = ");
        //    Serial.println(servo2Angle);
        // }
        break;

    }

    prevLeftOnWhite = leftOnWhite;
    prevRightOnWhite = rightOnWhite;
    delay(10); // Небольшая задержка для стабильности
  }
}

// Калибровочный режим
void runCalibrationMode() {
  Serial.println("Calibration mode");
  Serial.println("Commands:");
  Serial.println("c - calibrate white");
  Serial.println("s - save calibration");
  Serial.println("l - load calibration");
  Serial.println("p - print current calibration");

  int leftSum = 0, rightSum = 0;
  int samplesCount = 0;

  while (true) {
    if (Serial.available()) {
      char command = Serial.read();
      switch (command) {
        case 'c': // Калибровка белого
          leftSum = 0;
          rightSum = 0;
          samplesCount = 0;
          Serial.println("Calibrating white... Place both sensors on white and wait");

          for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            int leftValue = analogRead(LEFT_SENSOR_PIN);
            int rightValue = analogRead(RIGHT_SENSOR_PIN);
            leftSum += leftValue;
            rightSum += rightValue;
            samplesCount++;
            delay(50);
          }

          calibration.leftWhite = leftSum / samplesCount + WHITE_THRESHOLD_OFFSET;
          calibration.rightWhite = rightSum / samplesCount + WHITE_THRESHOLD_OFFSET;

          Serial.println("Calibration complete:");
          Serial.print("Left white: "); Serial.println(calibration.leftWhite);
          Serial.print("Right white: "); Serial.println(calibration.rightWhite);
          break;

        case 's': // Сохранение калибровки
          saveCalibration();
          Serial.println("Calibration saved to EEPROM");
          break;

        case 'l': // Загрузка калибровки
          loadCalibration();
          Serial.println("Calibration loaded from EEPROM:");
          Serial.print("Left white: "); Serial.println(calibration.leftWhite);
          Serial.print("Right white: "); Serial.println(calibration.rightWhite);
          break;

        case 'p': // Печать текущей калибровки
          Serial.println("Current calibration:");
          Serial.print("Left white: "); Serial.println(calibration.leftWhite);
          Serial.print("Right white: "); Serial.println(calibration.rightWhite);
          break;
      }
    }

    // Вывод текущих значений датчиков
    int leftValue = analogRead(LEFT_SENSOR_PIN);
    int rightValue = analogRead(RIGHT_SENSOR_PIN);
    Serial.print("Sensors: L="); Serial.print(leftValue);
    Serial.print(" R="); Serial.println(rightValue);
    delay(1000);
  }
}

// Чтение значений датчиков
void readSensors(int &leftValue, int &rightValue) {
  leftValue = analogRead(LEFT_SENSOR_PIN);
  rightValue = analogRead(RIGHT_SENSOR_PIN);
}

// Проверка, находится ли датчик на белом
bool isOnWhite(int sensorValue, int whiteThreshold, bool prevState) {
  if (prevState) {
    // Если ранее было белое, используем нижний порог для выхода (белое + гистерезис)
    return sensorValue < (whiteThreshold + HYSTERESIS);
  } else {
    // Если ранее было не белое, используем верхний порог для входа (белое - гистерезис)
    return sensorValue < (whiteThreshold - HYSTERESIS);
  }
}

// Сохранение калибровки в EEPROM
void saveCalibration() {
  EEPROM.put(0, calibration);
}

// Загрузка калибровки из EEPROM
void loadCalibration() {
  EEPROM.get(0, calibration);
  // Если EEPROM пуста, используем значения по умолчанию
  if (calibration.leftWhite == -1 || calibration.rightWhite == -1) {
    calibration.leftWhite = 500;
    calibration.rightWhite = 500;
  }
}

// Вывод значений датчиков (только в DEBUG)
void printSensorValues(int leftValue, int rightValue) {
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 1000) {
    Serial.print("Sensors: L="); Serial.print(leftValue);
    Serial.print(" R="); Serial.println(rightValue);
    lastPrintTime = millis();
  }
}