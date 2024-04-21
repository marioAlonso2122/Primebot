// Prueba básica de los sensores de distancia OPT de Pololu en PrimeBot
#include <OPT3101.h>
#include <Wire.h>

OPT3101 sensor;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  // Esperamos a que el Serial esté abierto para empezar a imprimir mensajes
  while (!Serial) {}

  sensor.init();
  if (sensor.getLastError())
  {
    Serial.print(F("Error al Inicializar OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) {}
  }

  // Indica las lecturas medias que harán los sensores en el momento de realizar una lectura.
  sensor.setFrameTiming(256);

  // Utilizamos el canal del medio TX1
  sensor.setChannel(1);

  // Configuramos el sensor para que él solo seleccione el brillo 
  sensor.setBrightness(OPT3101Brightness::Adaptive);
}

void loop()
{
  //Pedimos al Sensor que haga una medición y que nos muestre la distancia en milímetros detectada.
  // Podríamos medir otras variables pero en el uso de este robot no nos interesa.
  sensor.sample();
  Serial.print(sensor.distanceMillimeters);
  Serial.println();
}
