#include <AFMotor.h>          // Библиотека для управления моторами через L293D
#include <Servo.h>            // Библиотека для управления сервомашинками

#define SERVO1_PIN 10         // Первая сервомашинка (поворотное колесо)
#define SERVO2_PIN 9         // Вторая сервомашинка (веселое действие)

#define SERVO_MIN_ANGLE 30    // Минимальный угол сервомашинки
#define SERVO_MAX_ANGLE 170   // Максимальный угол сервомашинки


AF_DCMotor motor(3);         // Мотор подключен к M3
Servo servo2; // сервопривод веселого действия
Servo servo1; // сервопривод поворотного колеса

void readSensors(int &leftValue, int &rightValue);

void setup() {
  // подключение сервоприводов
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);  

  motor.setSpeed(0); 
  motor.run(RELEASE);
  motor.run(FORWARD);
  delay(5000);
  motor.setSpeed(250); 
  delay(5000);
  motor.setSpeed(0); 
}

void loop() {
  servo.write(170);
  delay(100);
  servo.write(10);
  delay(100);
}


void readSensors(int &leftValue, int &rightValue) {
  leftValue = analogRead(LEFT_SENSOR_PIN);
  rightValue = analogRead(RIGHT_SENSOR_PIN);
}