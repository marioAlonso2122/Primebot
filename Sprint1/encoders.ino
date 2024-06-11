const int C1 = 3; // Entrada de la señal A del encoder.
const int C2 = 2; // Entrada de la señal B del encoder.

volatile int n = 0; // Contador de los cambios de estado
volatile byte ant = 0; // Estado anterior de las señales A y B
volatile byte act = 0; // Estado actual de las señales A y B

unsigned long lastTime = 0; // Tiempo anterior para el muestreo
unsigned long sampleTime = 100; // Intervalo de muestreo en milisegundos

double P = 0; // Posición angular del encoder en grados
double R = 1238; // Resolución del encoder en conteos por vuelta completa

void setup()
{
  Serial.begin(9600);
  pinMode(C1, INPUT); // Configura pin C1 como entrada
  pinMode(C2, INPUT); // Configura pin C2 como entrada

  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE); // Interrupción para cambios en C1
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE); // Interrupción para cambios en C2
  
  Serial.println("Numero de conteos");
}

void loop() {
  if (millis() - lastTime >= sampleTime || lastTime == 0)
  { // Actualiza la posición cada sampleTime milisegundos
      lastTime = millis();
      P = (n * 360.0) / R; // Calcula la posición en grados
      Serial.print("Posicion en grados: "); Serial.println(P);
  }
}

// Función de interrupción para leer los cambios de estado del encoder cuadrático
void encoder(void)
{
  ant = act; // Guarda el estado anterior
  
  // Actualiza el estado actual basado en las lecturas de los pines
  if (digitalRead(C1)) bitSet(act, 1); else bitClear(act, 1);
  if (digitalRead(C2)) bitSet(act, 0); else bitClear(act, 0);
  
  // Incrementa o decrementa el contador 'n' según la transición detectada
  if (ant == 2 && act == 0) n++;
  if (ant == 0 && act == 1) n++;
  if (ant == 3 && act == 2) n++;
  if (ant == 1 && act == 3) n++;

  if (ant == 1 && act == 0) n--;
  if (ant == 3 && act == 1) n--;
  if (ant == 0 && act == 2) n--;
  if (ant == 2 && act == 3) n--;
}
