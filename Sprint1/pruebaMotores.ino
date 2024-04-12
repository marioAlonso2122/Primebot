/**
 * Definicion Pines Placa
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


/****/

/***
 * Variables Globales
 */

uint32_t updateTime;
uint32_t updateInterval = 100;  // in milliseconds

int decodeFunctionSwitch(int functionValue) {
  
  /**
   * Valores obtenidos según la posición del minidip switch
   */
   
  const int adcReading[] = {660, 647, 630, 614, 590, 570, 545, 522, 461,
                            429, 385, 343, 271, 212, 128, 44,  0};

  if (functionValue > 1000) {
    return 16;  // Valor tras pulsar el boton
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

int getFunctionSwitch() {
  int functionValue = analogRead(FUNCTION_PIN);
  int function = decodeFunctionSwitch(functionValue);
  return function;
}

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
  Serial.println(F("Empieza la Prueba de Motorers\n"));
  motorSetup();
  updateTime = millis() + updateInterval;
}

void motorAction(int function) {
  switch (function) {
    case 0:
      setMotorPWM(0, 0);
      Serial.println("Motores Apagados");
      break;
    case 1:
      setMotorPWM(63, 63);
      Serial.println("Adelante 25%");
      break;
    case 2:
      setMotorPWM(127, 127);
      Serial.println("Adelante 50%");
      break;
    case 3:
      setMotorPWM(195, 195);
      Serial.println("Adelante 75%");
      break;
    case 4:
      setMotorPWM(-63, -63);
      Serial.println("Atras 25%");
      break;
    case 5:
      setMotorPWM(-127, -127);
      Serial.println("Atras 50%");
      break;
    case 6:
      setMotorPWM(-195, -195);
      Serial.println("Atras 75%");
      break;
    case 7:
      setMotorPWM(-63, 63);
      Serial.println("Izquierda 25%");
      break;
    case 8:
      setMotorPWM(-127, 127);
      Serial.println("Izquierda 50%");
      break;
    case 9:
      setMotorPWM(63, -63);
      Serial.println("Derecha 25%");
      break;
    case 10:
      setMotorPWM(127, -127);
      Serial.println("Derecha 50%");
      break;
    case 11:
      setMotorPWM(0, 63);
      Serial.println("Pivot Izquierda 25%");
      break;
    case 12:
      setMotorPWM(63, 0);
      Serial.println("Pivot Derecha 25%");
      break;
    case 13:
      setMotorPWM(63, 127);
      Serial.println("Curva a la izquierda");
      break;
    case 14:
      setMotorPWM(127, 63);
      Serial.println("Curva a la derecha");
      break;
    case 15:
      setMotorPWM(195, 127);
      Serial.println("Gran curva a la derecha");
      break;
    default:
      setMotorPWM(0, 0);
      break;
  }
}

void runRobot() {
  int function = getFunctionSwitch();
  
  // Tiempo que funcionan los motores
  uint32_t endTime = millis() + 2000;
  motorAction(function);
  while (endTime > millis()) {
    if (getFunctionSwitch() == 16) {
      break;  // Parar si se pulsa el boton
    }
  }
  // Apagar los motores
  setMotorPWM(0, 0);

  delay(500);
}

void loop() {
  if (getFunctionSwitch() == 16) {
    
    while (getFunctionSwitch() == 16) {
      delay(20);
    }
    
    delay(500);
    runRobot();
  }
}
