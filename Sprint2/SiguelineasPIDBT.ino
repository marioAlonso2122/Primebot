#include <QTRSensors.h>
#define MaxSpeed 255 //Velocidad Maxima Motores
#define MinSpeed 0 //Velocidad minima Motores

//Programa de las primeras pruebas de PID siguelíneas para PrimeBot antes de montarlo en el PCB.
// La definición de pines es diferente ya que no se ejecutó en la PCB si no en protoboard.

// Motor driver
const int AIN1 = 9; 
const int AIN2 = 10;
const int PWMA = 11; 
const int BIN1 = 12; 
const int BIN2 = 13;
const int PWMB = 8;
const int STBY = 7;

// QTR Sensors
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


// PID constants
float Kp = 0;   // Constante Proporcional
float Ki = 0;  // Constante Integral
float Kd = 1;  // Constante Derivativa
float kv; // Constante para las rectas
int lastError = 0;      // Error anterior
int integral = 0;       // Acumulación de errores
int baseSpeed = 70;  // Velocidad base de los motores

void setup() {

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);


  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Led del arduino como que está en modo calibracion


  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}


// Modificaciones parametros via Bluetooth
void controlador(){
    char option = Serial.read();
    if (option == 'w') { inicio=true;   Serial.println("Start");}
    if (option == 's') { inicio=false; Serial.println("Stop");   analogWrite(3, 0);   analogWrite(11, 0);   }
     if (option == 'P'){ Kp+=0.01; }
     if (option == 'p'){Kp-=0.01; }
     if (option == 'I'){ Ki+=0.001; }
     if (option == 'i'){Ki-=0.001; }
     if (option == 'D'){ Kd+=0.1;}
     if (option == 'd'){Kd-=0.1; }
     if (option == 'V'){ Kv+=0.001;}
     if (option == 'v'){Kv-=0.001;}
     if (option == 'B'){ base_iz+=1;base_der+=1;}
     if (option == 'b'){base_iz-=1;base_der-=1;}   
     Serial.print(Kp); Serial.print(' '); Serial.print(Ki); Serial.print(' '); 
     Serial.print(Kd); Serial.print(' '); Serial.println(Kv); Serial.print(' '); 
     Serial.println(baseSpeed);
  }



void loop() {
  int position = qtr.readLineBlack(sensorValues);
  Serial.println("Posicion:");
  Serial.println(position);
  
  int error = position - 2500;  // Suponemos que 2500 es el centro
  error = error /10;
  // Calcula la derivativa (diferencia entre el error actual y el error anterior)
  int derivative = error - lastError;

  // Acumula el error para la integral
  integral += error;

  // Calcula el ajuste del PID
  int motorSpeedAdjustment = Kp * error + Kd * derivative;
  int leftSpeed = baseSpeed - motorSpeedAdjustment;
  int rightSpeed = baseSpeed + motorSpeedAdjustment;

  //Comprobacion de Velocidades
  if (rightSpeed > MaxSpeed ) rightSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftSpeed > MaxSpeed ) leftSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightSpeed < 0) rightSpeed = 0;    
  if (leftSpeed < 0)leftSpeed = 0;

  setMotorSpeeds(leftSpeed, rightSpeed);

  Serial.println("Vder:");
  Serial.println(rightSpeed);
  Serial.println("Vizq:");
  Serial.println(leftSpeed);
  Serial.println("Correcion:");
  Serial.println(motorSpeedAdjustment);
  delay(10);

  // Cuando Serial está disponible cargamos los cambios del PID 
  if (Serial.available()>0){
    controlador();
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA, leftSpeed);

  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB, rightSpeed);
}
