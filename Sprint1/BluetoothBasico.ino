// Definición del pin al que está conectado el LED.
const int led = 13;

// Variable para almacenar la opción recibida a través del puerto serie.
int option;

// Función de configuración que se ejecuta una sola vez al iniciar el Arduino.
void setup(){
  // Inicia la comunicación serial a 9600 bits por segundo.
  Serial.begin(9600);
  // Configura el pin del LED como salida.
  pinMode(led, OUTPUT); 
}

// Función loop que se ejecuta repetidamente en un bucle infinito.
void loop(){
  // Verifica si hay datos disponibles para leer en el puerto serie.
  if (Serial.available()>0){
    // Lee el primer byte de datos disponibles.
    char option = Serial.read();
    // Verifica si la opción está entre '1' y '9'
    if (option >= '1' && option <= '9')
    {
      // Convierte el carácter de la opción en un número entero.
      option -= '0';
      // Realiza un bucle para hacer parpadear el LED tantas veces como el número recibido.
      for(int i=0;i<option;i++){
        digitalWrite(led, HIGH); // Enciende el LED.
        delay(100);              // Espera 100 milisegundos.
        digitalWrite(led, LOW);  // Apaga el LED.
        delay(200);              // Espera 200 milisegundos.
      }
    }
  }
}
