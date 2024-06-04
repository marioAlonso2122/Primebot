#include <Wire.h>
#include <OPT3101.h>

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

OPT3101 sensor;
int16_t distances[3];

const int umbralFrontal = 50; // Umbral de distancia frontal en mm
const int umbralLateral = 110; // Umbral de distancia lateral en mm
const int velocidadBase = 75;  // Velocidad base de los motores

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
  Serial.begin(115200);
  Wire.begin();

  // Espera a que el puerto serial se abra antes de imprimir mensajes
  while (!Serial) {}

  sensor.init();
  if (sensor.getLastError()) {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) {}
  }

  sensor.setFrameTiming(128); // Ajusta el tiempo de muestreo para una respuesta más rápida
  sensor.setChannel(0);
  sensor.setBrightness(OPT3101Brightness::Adaptive);

  sensor.startSample();
  motorSetup();
}

void loop() {
  if (sensor.isSampleDone()) {
    sensor.readOutputRegs();

    distances[sensor.channelUsed] = sensor.distanceMillimeters;

    if (sensor.channelUsed == 2) {
      int16_t distanciaIzquierda = distances[0]; // Usar el primer canal como lateral izquierdo
      int16_t distanciaFrontal = distances[1]; // Usar el segundo canal como frontal
      int16_t distanciaDerecha = distances[2]; // Usar el tercer canal como lateral derecho

      Serial.print("Distancia Frontal: ");
      Serial.println(distanciaFrontal);
      Serial.print("Distancia Izquierda: ");
      Serial.println(distanciaIzquierda);
      Serial.print("Distancia Derecha: ");
      Serial.println(distanciaDerecha);

      seguirParedIzquierda(distanciaIzquierda, distanciaFrontal, distanciaDerecha);
    }

    sensor.nextChannel();
    sensor.startSample();
  }
}

void seguirParedIzquierda(int16_t distanciaIzquierda, int16_t distanciaFrontal, int16_t distanciaDerecha) {
  if (distanciaFrontal < umbralFrontal && distanciaIzquierda < 50 && distanciaDerecha < 50) {
    // Girar 180 grados
    detener();
    setMotorPWM(125, -125); // Girar en el lugar
    delay(1000); // Ajustar el tiempo para un giro de 180 grados
    sensor.startSample();
    while (!sensor.isSampleDone()) {}
    sensor.readOutputRegs();
    Serial.println("Obstáculo en todos los lados, girando 180 grados");
  } else if (distanciaFrontal < umbralFrontal) {
    // Girar a la derecha hasta que no haya obstáculo frontal
    detener();
    setMotorPWM(125, -125); // Girar a la derecha
    delay(350);
    sensor.startSample();
    while (!sensor.isSampleDone()) {}
    sensor.readOutputRegs();
    distanciaFrontal = sensor.distanceMillimeters;

    while (distanciaFrontal < umbralFrontal) {
      setMotorPWM(125, -125); // Continuar girando a la derecha
      delay(50);
      sensor.startSample();
      while (!sensor.isSampleDone()) {}
      sensor.readOutputRegs();
      distanciaFrontal = sensor.distanceMillimeters;
    }

    setMotorPWM(velocidadBase, velocidadBase); // Seguir adelante después de girar
    Serial.println("Obstáculo frontal, girando a la derecha");
  } else if (distanciaIzquierda > 300) {
    // Avanzar un poco más para asegurarse de que el robot ha pasado el obstáculo
    setMotorPWM(velocidadBase, velocidadBase); 
    delay(700); // Ajusta el tiempo según sea necesario para avanzar más allá del obstáculo
    detener();
    
    // Girar 90 grados a la izquierda
    setMotorPWM(-125, 125); 
    delay(200); // Ajusta el tiempo para un giro de 90 grados
    setMotorPWM(velocidadBase, velocidadBase); // Avanzar un poco más

    // Avanzar un poco y verificar si hay un hueco a la izquierda
    delay(700); 
    sensor.startSample();
    while (!sensor.isSampleDone()) {}
    sensor.readOutputRegs();
    distanciaIzquierda = distances[0]; // Actualizar la distancia lateral izquierda

    if (distanciaIzquierda > 300) {
      // Si aún hay hueco, girar nuevamente a la izquierda
      detener();
      setMotorPWM(-125, 125);
      delay(350); // Ajusta el tiempo para otro giro de 90 grados
      setMotorPWM(velocidadBase, velocidadBase); // Avanzar hasta encontrar una pared

      // Verificación continua mientras avanza
      while (true) {
        sensor.startSample();
        while (!sensor.isSampleDone()) {}
        sensor.readOutputRegs();
        distanciaFrontal = sensor.distanceMillimeters;
        if (distanciaFrontal < umbralFrontal) {
          detener();
          break;
        }
        setMotorPWM(velocidadBase, velocidadBase);
        Serial.println("Avanzando hasta encontrar pared");
      }
    }
  } else {
    // Seguir la pared de la izquierda
    if (distanciaIzquierda < umbralLateral) {
      // Demasiado cerca de la pared izquierda, gira un poco a la derecha
      setMotorPWM(velocidadBase, velocidadBase / 2);
      Serial.println("Girando un poco a la derecha");
    } else {
      // Demasiado lejos de la pared izquierda, gira un poco a la izquierda
      setMotorPWM(velocidadBase / 2, velocidadBase);
      Serial.println("Girando un poco a la izquierda");
    }
  }
}

void detener() {
  setMotorPWM(0, 0);
}
