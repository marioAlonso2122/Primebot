#include <QTRSensors.h>

// Prueba básica de los Sensores Siguelíneas de Pololu QTR-8A

QTRSensors qtr; // Crea una instancia de la clase QTRSensors para interactuar con los sensores

const uint8_t SensorCount = 6; // Número de sensores que se están utilizando
uint16_t sensorValues[SensorCount]; // Arreglo para almacenar los valores de los sensores

void setup()
{
  qtr.setTypeAnalog(); // Configura los sensores como analógicos
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount); // Establece los pines que se utilizan para los sensores
  qtr.setEmitterPin(11); // Establece el pin del emisor IR

  delay(500); // Espera para que los sensores se estabilicen
  pinMode(LED_BUILTIN, OUTPUT); // Configura el LED incorporado como salida
  digitalWrite(LED_BUILTIN, HIGH); // Enciende el LED para indicar el inicio de la calibración

  // Calibración de los sensores
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate(); // Ejecuta la calibración
  }
  digitalWrite(LED_BUILTIN, LOW); // Apaga el LED para señalar el fin de la calibración

  // Inicia la comunicación Serial
  Serial.begin(9600);
  // Imprime los valores mínimos obtenidos durante la calibración
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // Imprime los valores máximos obtenidos durante la calibración
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000); // Espera antes de iniciar el bucle principal
}

void loop()
{
  // Lee la posición de la línea respecto a los sensores
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Imprime los valores de reflectancia de cada sensor en tiempo real
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position); // Imprime la posición calculada de la línea

  delay(250); // Pequeña pausa para hacer las lecturas menos frecuentes
}
