#include <Arduino.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

// Definición de pines de los sensores de línea
const int SENSOR_COUNT = 6;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5};
const int EMITTER_PIN = 11;

// Definición de pines de los encoders magnéticos
const int ENCODER_LEFT_CLK = 2;
const int ENCODER_RIGHT_CLK = 3;

// Definición de pines del motor
const int MOTOR_LEFT_DIR = 7;
const int MOTOR_RIGHT_DIR = 8;
const int MOTOR_LEFT_PWM = 9;
const int MOTOR_RIGHT_PWM = 10;

// Constantes de velocidad
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;
const int BASE_SPEED = 100;

// PID constants
float KP = 4;
float KI = 0;
float KD = 0;

// Variables globales
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
unsigned long lastLeftCount = 0;
unsigned long lastRightCount = 0;
unsigned long lastTime = 0;
SoftwareSerial bluetooth(4, 5); // RX, TX

// Funciones
void motorSetup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
}

void setLeftMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
    analogWrite(MOTOR_LEFT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
    analogWrite(MOTOR_LEFT_PWM, pwm);
  }
}

void setRightMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
    analogWrite(MOTOR_RIGHT_PWM, pwm);
  }
}

void setMotorPWM(int left, int right) {
  setLeftMotorPWM(left);
  setRightMotorPWM(right);
}

void leftEncoderISR() {
  leftEncoderCount++;
}

void rightEncoderISR() {
  rightEncoderCount++;
}

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
  qtr.setEmitterPin(EMITTER_PIN);

  // Configurar encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_CLK), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_CLK), rightEncoderISR, RISING);

  motorSetup();

  // Iniciar comunicación serial con Bluetooth
  Serial.begin(9600);
  bluetooth.begin(9600);
}

void loop() {
  // Leer valores de los sensores de línea
  int position = qtr.readLineBlack(sensorValues);

  // Calcular el error
  int error = position - 2500; // Suponemos que 2500 es el centro

  // Calcular la derivada (diferencia entre el error actual y el error anterior)
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Tiempo transcurrido desde la última vez
  lastTime = currentTime;
  int derivative = (error - lastError) / dt;

  // Acumular el error para la integral
  integral += error * dt;

  // Calcular la corrección del PID
  int motorSpeedAdjustment = KP * error + KI * integral + KD * derivative;
  int leftSpeed = BASE_SPEED - motorSpeedAdjustment;
  int rightSpeed = BASE_SPEED + motorSpeedAdjustment;

  // Limitar las velocidades
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  // Controlar los motores
  setMotorPWM(leftSpeed, rightSpeed);

  // Actualizar el error anterior
  lastError = error;

  // Mostrar información en el puerto serie
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print("  Error: ");
  Serial.print(error);
  Serial.print("  Adjustment: ");
  Serial.print(motorSpeedAdjustment);
  Serial.print("  Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print("  Right Speed: ");
  Serial.println(rightSpeed);

  // Controlar la comunicación Bluetooth
  if (bluetooth.available() > 0) {
    char command = bluetooth.read();
    if (command == 'K') {
      KP += 0.1;
    } else if (command == 'k') {
      KP -= 0.1;
    } else if (command == 'I') {
      KI += 0.001;
    } else if (command == 'i') {
      KI -= 0.001;
    } else if (command == 'D') {
      KD += 0.1;
    } else if (command == 'd') {
      KD -= 0.1;
    }
  }
}
