#include <OPT3101.h> // Incluye la biblioteca OPT3101
#include <Wire.h> // Incluye la biblioteca Wire para la comunicación I2C

OPT3101 sensor; // Crea una instancia del objeto OPT3101

void setup()
{
  Serial.begin(9600); // Inicia la comunicación serial a 9600 baudios
  Wire.begin(); // Inicia la comunicación I2C

  // Espera a que el puerto serial se abra antes de imprimir mensajes
  // Esto solo es necesario en placas con USB nativo.
  while (!Serial) {}

  sensor.init(); // Inicializa el sensor OPT3101
  if (sensor.getLastError()) // Verifica si hay errores al inicializar el sensor
  {
    Serial.print(F("Failed to initialize OPT3101: error "));
    Serial.println(sensor.getLastError());
    while (1) {} // Se queda en un bucle infinito si hay un error
  }

  // Establece el número de lecturas a promediar
  sensor.setFrameTiming(256); // Puedes ajustar este valor según sea necesario

  // Establece el canal a utilizar (en este caso, el canal medio)
  sensor.setChannel(1); // 1 significa utilizar el canal medio

  // Establece el brillo del sensor (en este caso, Adaptable)
  sensor.setBrightness(OPT3101Brightness::Adaptive); // Puedes ajustar esto según tus necesidades
}

void loop()
{
  sensor.sample(); // Realiza una muestra

  // Calcula la distancia en centímetros
  float distancia_cm = sensor.distanceMillimeters / 10.0;

  // Imprime la distancia en centímetros
  Serial.println(distancia_cm);

  // Espera un breve periodo de tiempo antes de la próxima lectura
  delay(500);
}
