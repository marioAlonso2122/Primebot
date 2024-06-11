#include <Arduino.h>
#include <QTRSensors.h>
/**
 * Version Sencilla de PID para PrimeBot
 */

/**
 * Definicion de los Pines de sensores de la placa
 */

const int ENCODER_LEFT_CLK = 2;
const int ENCODER_RIGHT_CLK = 3;
const int ENCODER_LEFT_B = 4;
const int ENCODER_RIGHT_B = 5;
const int MOTOR_LEFT_DIR = 7;
const int MOTOR_RIGHT_DIR = 8;
const int MOTOR_LEFT_PWM = 9;
const int MOTOR_RIGHT_PWM = 10;
const int LED_RIGHT = 6;
const int LED_LEFT = 11;
const int EMITTER = 12;
const int SENSOR_RIGHT_MARK = A0;
const int SENSOR_1 = A1;
const int SENSOR_2 = A2;
const int SENSOR_3 = A3;
const int SENSOR_4 = A4;
const int SENSOR_LEFT_MARK = A5;
const int FUNCTION_PIN = A6;
const int BATTERY_VOLTS = A7;

/**
 * Variables Globales 
 */

// QTR Sensors
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

uint32_t updateTime;
uint32_t updateInterval = 100; // milisegundos


#define MaxSpeed 255 //Velocidad Maxima Motores
#define MinSpeed 0 //Velocidad minima Motores

// PID constants
const float Kp = 4;   // Constante Proporcional
const float Ki = 0;  // Constante Integral
const float Kd = 0;  // Constante Derivativa
int lastError = 0;      // Error anterior
int integral = 0;       // Acumulación de errores
int baseSpeed = 100;  // Velocidad base de los motores

/**
 * Configuración inicial de los pines de motores.
 */
void motorSetup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  digitalWrite(MOTOR_LEFT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR, 0);
  digitalWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_RIGHT_DIR, 0);
}

/**
 * Funciones para establecer el PWM de los motores.
 */

void setLeftMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_LEFT_DIR, 1);
    analogWrite(MOTOR_LEFT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, 0);
    analogWrite(MOTOR_LEFT_PWM, pwm);
  }
}

void setRightMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_RIGHT_DIR, 0);
    analogWrite(MOTOR_RIGHT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, 1);
    analogWrite(MOTOR_RIGHT_PWM, pwm);
  }
}

void setMotorPWM(int left, int right) {
  setLeftMotorPWM(left);
  setRightMotorPWM(right);
}


void setup() {

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(11);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); 

   for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

}

void loop() {
  int position = qtr.readLineBlack(sensorValues);
  Serial.println("Posicion:");
  Serial.println(position);
  
  int error = position - 2500;  // Suponemos que 2500 es el centro

  // Calcula la derivativa (diferencia entre el error actual y el error anterior)
  int derivative = error - lastError;

  // Acumula el error para la integral
  integral += error;

  // Calcula el ajuste del PID
  int motorSpeedAdjustment = Kp * error + Kd * derivative;
  int leftSpeed = baseSpeed - motorSpeedAdjustment;
  int rightSpeed = baseSpeed + motorSpeedAdjustment;

  //Comprobacion de Velocidades
  if (rightSpeed > MaxSpeed ) rightSpeed = MaxSpeed;
  if (leftSpeed > MaxSpeed ) leftSpeed = MaxSpeed; 
  if (rightSpeed < 0) rightSpeed = 0;    
  if (leftSpeed < 0)leftSpeed = 0;

  setMotorPWM(leftSpeed, rightSpeed); // Aplicación de las velocidades calculadas a los motores

}
