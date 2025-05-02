#include <AFMotor.h>          // Библиотека для управления моторами через L293D
#include <Servo.h>            // Библиотека для управления сервомашинками

// Пины
#define LEFT_SENSOR_PIN A3    // Левый датчик линии
#define RIGHT_SENSOR_PIN A2   // Правый датчик линии
#define SERVO1_PIN 10         // Первая сервомашинка (поворотное колесо)
#define SERVO2_PIN 9          // Вторая сервомашинка (веселое действие)

// углы максимального поворота сервомашинок
#define SERVO_MIN_ANGLE 30    // Минимальный угол сервомашинки
#define SERVO_MAX_ANGLE 170   // Максимальный угол сервомашинки

// параметры езды
#define START_ANGLE_SERVO1 90
#define START_ANGLE_SERVO2 40

//параметры датчиков
#define WHITE_VALUE 25
#define WHITE_THRESHOLD_OFFSET 20 // отступ от значения датчика

AF_DCMotor motor(3);         // Мотор подключен к M3
Servo servo1; // сервопривод поворотного колеса
Servo servo2; // сервопривод веселого действия


int leftValue = analogRead(LEFT_SENSOR_PIN); // значение левого датчика
int rightValue = analogRead(RIGHT_SENSOR_PIN); // значение правого датчика

void readSensors(int &leftValue, int &rightValue);

void setup() {
  // подключение сервоприводов
  servo1.attach(SERVO1_PIN); // подключение поворотного колеса
  //servo2.attach(SERVO2_PIN); // подключение веселого действия
  
  // установка начального положения сервоприводов
  servo1.write(START_ANGLE_SERVO1);
  //servo2.write(START_ANGLE_SERVO2);

  // настройка мотора
  motor.setSpeed(0); 
  motor.run(RELEASE);
  motor.setSpeed(250);

  Serial.begin(9600);

}

void loop() {
  // получаем значение датчиков
  leftValue = analogRead(LEFT_SENSOR_PIN);
  rightValue = analogRead(RIGHT_SENSOR_PIN);
}