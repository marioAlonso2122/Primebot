#include <QTRSensors.h>

//Prueba básica de los Sensores Siguelíneas de Pololu QTR-8A

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(11);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Encendemos el LED de Arduino para marcar que está en modo calibración

  // Llamamos a la función 'calibrate' para realizar la calibración
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // Apagamos el LED de Arduino para indicar que ya ha terminado la calibración

  // Iniciamos la comunicación Serial e imprimimos los valores mínimos de la caslibración
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // Imprimimos los valores máximos que han leído el modo calibración
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  // Leemos el valor de la línea a través de los sensores ya calibrados
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Imprimimos los valores de cada sensor en tiempo real
  // 0 indica máxima reflectancia
  // 1000 indica mínima reflectancia
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
}
