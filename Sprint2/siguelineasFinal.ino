#include <Arduino.h>
#include <QTRSensors.h>

// Definición de pines
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
const int EMITTER = 12; // Emisor para los sensores QTR
const int SENSOR_RIGHT_MARK = A0;
const int SENSOR_1 = A1;
const int SENSOR_2 = A2;
const int SENSOR_3 = A3;
const int SENSOR_4 = A4;
const int SENSOR_LEFT_MARK = A5;
const int FUNCTION_PIN = A6;
const int BATTERY_VOLTS = A7;


// QTR Sensors
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// PID constants
float Kp = 1.8;   // Constante Proporcional
float Ki = 0.02;   // Constante Integral
float Kd = 0.2;   // Constante Derivativa
float Kv = 2.60;   // Constante para las rectas
int lastError = 0;      // Error anterior
int integral = 0;       // Acumulación de errores
int baseSpeed = 185;     // Velocidad base de los motores
const int MaxSpeed = 255; // Velocidad máxima de los motores

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
  // Configuración de los sensores QTR
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(11);

  digitalWrite(LED_BUILTIN, HIGH); 
  // Calibración de los sensores
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); 

  // Inicialización de la comunicación serial
  Serial.begin(9600);
}

// Función para ajustar los valores del PID a través de Bluetooth
void controlador() {
  if (Serial.available() > 0) {
    char option = Serial.read();

    switch (option) {
      case 'P':
        Kp += 0.1;
        break;
      case 'p':
        Kp -= 0.1;
        break;
      case 'I':
        Ki += 0.01;
        break;
      case 'i':
        Ki -= 0.01;
        break;
      case 'D':
        Kd += 0.1;
        break;
      case 'd':
        Kd -= 0.1;
        break;
      case 'V':
        Kv += 0.01;
        break;
      case 'v':
        Kv -= 0.01;
        break;
      case 'B':
        baseSpeed += 1;
        break;
      case 'b':
        baseSpeed -= 1;
        break;
      default:
        break;
    }

    // Imprime los valores actualizados
    Serial.print("Kp: ");
    Serial.print(Kp);
    Serial.print(" Ki: ");
    Serial.print(Ki);
    Serial.print(" Kd: ");
    Serial.print(Kd);
    Serial.print(" Kv: ");
    Serial.print(Kv);
    Serial.print(" BaseSpeed: ");
    Serial.println(baseSpeed);
  }
}

void loop() {
  // Lectura de la posición de la línea
  int position = qtr.readLineBlack(sensorValues);
  int error = position - 2500;  // Suponemos que 2500 es el centro
  error = error / 10;

  // Calcula la derivativa (diferencia entre el error actual y el error anterior)
  int derivative = error - lastError;

  // Acumula el error para la integral
  integral += error;

  // Calcula el ajuste del PID
  int motorSpeedAdjustment = Kp * error + Ki * integral + Kd * derivative;
  int leftSpeed = baseSpeed - motorSpeedAdjustment;
  int rightSpeed = baseSpeed + motorSpeedAdjustment;

  // Comprobación de Velocidades
  if (rightSpeed > MaxSpeed) rightSpeed = MaxSpeed;
  if (leftSpeed > MaxSpeed) leftSpeed = MaxSpeed;
  if (rightSpeed < 0) rightSpeed = 0;
  if (leftSpeed < 0) leftSpeed = 0;

  // Configuración de la velocidad de los motores
  setMotorPWM(leftSpeed, rightSpeed);


  // Controlador para ajustar los valores del PID
  controlador();

  // Actualiza el error anterior
  lastError = error;

  // Pequeña pausa para evitar que el bucle se ejecute demasiado rápido
  delay(10);
}
