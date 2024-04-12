#include <Arduino.h>

/**
 * Pequeño programa para realizar lecturas de los números 0-15 
 * Del Switch Mini dip de 4 posiciones de la placa
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


/***
 * Variables Globales
 */

uint32_t updateTime;
uint32_t updateInterval = 100;  // en milisegundos

int decodeFunctionSwitch(int functionValue) {
  /**
   * Valores típicos de lectura para el Switch Dip de 4 posiciones (0-15)
   */
  const int adcReading[] = {660, 647, 630, 614, 590, 570, 545, 522, 461,
                            429, 385, 343, 271, 212, 128, 44,  0};

  if(functionValue > 1000){
    return 16;  // Botón reset pulsado
  }
  int result = 16;
  for (int i = 0; i < 16; i++) {
    if (functionValue > (adcReading[i] + adcReading[i + 1]) / 2) {
      result = i;
      break;
    }
  }
  return result;
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("hola\n"));
  updateTime = millis() + updateInterval;
}

int i;
void loop() {
  if (millis() > updateTime) {
    updateTime += updateInterval;
    int functionValue = analogRead(FUNCTION_PIN);
    int function = decodeFunctionSwitch(functionValue);
    if (function == 16) {
      digitalWrite(LED_BUILTIN, 1);
    } else {
      digitalWrite(LED_BUILTIN, 0);
    }
    Serial.print(functionValue);
    Serial.print(F(" => "));
    Serial.print(function);
    Serial.println();
  }
}
