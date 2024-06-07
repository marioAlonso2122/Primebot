#include <Wire.h>
#include <OPT3101.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

// Definición de pines
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
const int BLUETOOTH_RX = 12; 
const int BLUETOOTH_TX = 13; 

// QTR Sensors
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

// PID constants
float Kp = 1.8;
float Ki = 0.02;
float Kd = 0.2;
float Kv = 2.60;
int lastError = 0;
int integral = 0;
int baseSpeed = 60;
const int MaxSpeed = 125;

// Variables Globales
uint32_t updateTime;
uint32_t updateInterval = 100;

SoftwareSerial BTSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Coordenadas de las estaciones de origen y destino
int origen[7][2] = {
  {0, 3}, {0, 2}, {0, 1}, {1, 0}, {2, 0}, {3, 0}, {4, 0}
};
int destino[7][2] = {
  {1, 4}, {2, 4}, {3, 4}, {4, 4}, {5, 3}, {5, 2}, {5, 1}
};

// Estado actual del robot
int posX, posY;
int destinoX, destinoY;
int direccion; // 0 = x+, 1 = y-, 2 = x-, 3 = y+

// Array para los puntos bloqueados y la trayectoria
bool blocked[5][6] = {false};

// Nodo para el algoritmo A*
struct Nodo {
  int x, y;
  int f, g, h;
  Nodo* parent;
};

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
  // Configuración de los sensores QTR
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4, SENSOR_LEFT_MARK, SENSOR_RIGHT_MARK}, SensorCount);
  qtr.setEmitterPin(EMITTER);

  // Inicialización de la comunicación serial
  Serial.begin(9600);
  BTSerial.begin(9600);  // Inicialización de la comunicación serial Bluetooth

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  motorSetup(); // Configuración de los motores
  pedirBloqueos(); // Pedir puntos bloqueados
  pedirEstaciones(); // Pedir nuevas estaciones de origen y destino
  calcularTrayectoria(); // Calcular nueva trayectoria
}


void mostrarRuta(char grid[5][6]) {
  // Imprime la cuadrícula
  Serial.println("Ruta del robot:");
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 6; x++) {
      Serial.print(grid[y][x]);
      Serial.print(' ');
    }
    Serial.println();
  }
}

void bloquearEsquinas() {
  // Bloquear las esquinas de la cuadrícula
  blocked[0][0] = true;
  blocked[0][5] = true;
  blocked[4][0] = true;
  blocked[4][5] = true;
}

void pedirBloqueos() {
  // Bloquear las esquinas
  bloquearEsquinas();

  // Pedir puntos bloqueados adicionales
  Serial.println("Ingrese el número de puntos bloqueados:");
  int numPuntosBloqueados = readIntFromSerial();

  for (int i = 0; i < numPuntosBloqueados; i++) {
    Serial.print("Ingrese la coordenada X del punto bloqueado ");
    Serial.print(i + 1);
    Serial.println(" (0-5):");
    int xBloqueado = readIntFromSerial();

    Serial.print("Ingrese la coordenada Y del punto bloqueado ");
    Serial.print(i + 1);
    Serial.println(" (0-4):");
    int yBloqueado = readIntFromSerial();

    blocked[4 - yBloqueado][xBloqueado] = true; // Marca el punto como bloqueado
  }

  // Mostrar los puntos bloqueados ingresados por el usuario
  char grid[5][6] = { 
    {' ', '.', '.', '.', '.', ' '},
    {'.', '.', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '.'},
    {' ', '.', '.', '.', '.', ' '}
  }; 

  // Marca los puntos bloqueados
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 6; x++) {
      if (blocked[y][x]) {
        grid[y][x] = 'X'; // Punto bloqueado
      }
    }
  }
  
  mostrarRuta(grid);
}

void limpiarBloqueosEstaciones() {
  for (int i = 0; i < 7; i++) {
    blocked[4 - origen[i][1]][origen[i][0]] = false;
    blocked[4 - destino[i][1]][destino[i][0]] = false;
  }
}

void pedirEstaciones() {
  limpiarBloqueosEstaciones();
  // Bloquear las esquinas nuevamente
  bloquearEsquinas();

  // Pedir estación de salida
  int estacionSalida = -1;
  while (estacionSalida < 0 || estacionSalida >= 7) {
    Serial.println("Ingrese el número de la estación de salida (1-7):");
    estacionSalida = readIntFromSerial() - 1;
    if (estacionSalida < 0 || estacionSalida >= 7) {
      Serial.println("Número de estación de salida inválido. Intente de nuevo.");
    }
  }
  posX = origen[estacionSalida][0];
  posY = origen[estacionSalida][1];

  // Establecer dirección inicial
  if (estacionSalida <= 3) {
    direccion = 0; // x+
  } else {
    direccion = 3; // y+
  }

  // Pedir estación de destino
  char estacionDestino = ' ';
  while (estacionDestino < 'A' || estacionDestino > 'G') {
    Serial.println("Ingrese la letra de la estación de destino (A-G):");
    estacionDestino = readCharFromSerial();
    if (estacionDestino < 'A' || estacionDestino > 'G') {
      Serial.println("Letra de estación de destino inválida. Intente de nuevo.");
    }
  }
  destinoX = destino[estacionDestino - 'A'][0];
  destinoY = destino[estacionDestino - 'A'][1];

  // Bloquear las estaciones no seleccionadas sin sobrescribir puntos bloqueados
  for (int i = 0; i < 7; i++) {
    if (i != estacionSalida) {
      blocked[4 - origen[i][1]][origen[i][0]] = true;
    }
    if (i != (estacionDestino - 'A')) {
      blocked[4 - destino[i][1]][destino[i][0]] = true;
    }
  }

  // Mostrar ruta en la consola
  char grid[5][6] = { 
    {' ', '.', '.', '.', '.', ' '},
    {'.', '.', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '.'},
    {' ', '.', '.', '.', '.', ' '}
  }; 

  // Marca la posición de salida
  grid[4 - posY][posX] = 'S';

  // Marca la posición de destino
  grid[4 - destinoY][destinoX] = 'D';

  // Marca los puntos bloqueados
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 6; x++) {
      if (blocked[y][x]) {
        grid[y][x] = 'X'; // Punto bloqueado
      }
    }
  }

  mostrarRuta(grid);
}

int readIntFromSerial() {
  while (Serial.available() == 0) {}
  int value = Serial.parseInt();
  clearSerial();
  return value;
}

char readCharFromSerial() {
  while (Serial.available() == 0) {}
  char value = Serial.read();
  clearSerial();
  return value;
}

void clearSerial() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

int calcularH(int x, int y, int destX, int destY) {
  return abs(x - destX) + abs(y - destY);
}

Nodo* encontrarMenorF(Nodo* lista[], int tam) {
  int menor = 0;
  for (int i = 1; i < tam; i++) {
    if (lista[i]->f < lista[menor]->f) {
      menor = i;
    }
  }
  return lista[menor];
}

bool estaEnLista(Nodo* lista[], int tam, int x, int y) {
  for (int i = 0; i < tam; i++) {
    if (lista[i]->x == x && lista[i]->y == y) {
      return true;
    }
  }
  return false;
}

void liberarNodos(Nodo* nodos[], int tam) {
  for (int i = 0; i < tam; i++) {
    delete nodos[i];
  }
}

void calcularTrayectoria() {
  Nodo* abierta[50]; // Reducido tamaño de las listas para optimizar memoria
  Nodo* cerrada[50];
  int tamAbierta = 0;
  int tamCerrada = 0;

  Nodo* start = new Nodo{posX, posY, 0, 0, calcularH(posX, posY, destinoX, destinoY), nullptr};
  abierta[tamAbierta++] = start;

  while (tamAbierta > 0) {
    Nodo* actual = encontrarMenorF(abierta, tamAbierta);

    for (int i = 0; i < tamAbierta; i++) {
      if (abierta[i] == actual) {
        abierta[i] = abierta[--tamAbierta];
        break;
      }
    }
    cerrada[tamCerrada++] = actual;

    if (actual->x == destinoX && actual->y == destinoY) {
      Nodo* temp = actual;

      // Crear una lista para almacenar la ruta desde el origen hasta el destino
      char ruta[100] = "";
      Nodo* camino[50]; // Reducido tamaño de las listas para optimizar memoria
      int index = 0;

      // Recorrer el camino desde el destino hasta el origen
      while (temp->parent != nullptr) {
        camino[index++] = temp;
        temp = temp->parent;
      }

      // Crear una nueva cuadrícula para mostrar la ruta
      char grid[5][6] = { 
        {' ', '.', '.', '.', '.', ' '},
        {'.', '.', '.', '.', '.', '.'},
        {'.', '.', '.', '.', '.', '.'},
        {'.', '.', '.', '.', '.', '.'},
        {' ', '.', '.', '.', '.', ' '}
      }; 

      // Marca la posición de salida
      grid[4 - posY][posX] = 'S';

      // Marca la posición de destino
      grid[4 - destinoY][destinoX] = 'D';

      // Marca los puntos bloqueados
      for (int y = 0; y < 5; y++) {
        for (int x = 0; x < 6; x++) {
          if (blocked[y][x]) {
            grid[y][x] = 'X'; // Punto bloqueado
          }
        }
      }

      // Imprimir la ruta en el orden correcto
      Serial.print("Ruta a seguir: ");
      for (int i = index - 1; i >= 0; i--) {
        int dx = camino[i]->x - camino[i]->parent->x;
        int dy = camino[i]->y - camino[i]->parent->y;

        // Marca la trayectoria en la cuadrícula
        grid[4 - camino[i]->y][camino[i]->x] = '*';

        if (direccion == 0) { // x+
          if (dx == 1) {
            strcat(ruta, "Recto ");
          } else if (dy == -1) {
            strcat(ruta, "Derecha ");
            direccion = 1;
          } else if (dy == 1) {
            strcat(ruta, "Izquierda ");
            direccion = 3;
          } else if (dx == -1) {
            strcat(ruta, "Giro 180 ");
            direccion = 2;
          }
        } else if (direccion == 1) { // y-
          if (dy == -1) {
            strcat(ruta, "Recto ");
          } else if (dx == 1) {
            strcat(ruta, "Izquierda ");
            direccion = 0;
          } else if (dx == -1) {
            strcat(ruta, "Derecha ");
            direccion = 2;
          } else if (dy == 1) {
            strcat(ruta, "Giro 180 ");
            direccion = 3;
          }
        } else if (direccion == 2) { // x-
          if (dx == -1) {
            strcat(ruta, "Recto ");
          } else if (dy == -1) {
            strcat(ruta, "Izquierda ");
            direccion = 1;
          } else if (dy == 1) {
            strcat(ruta, "Derecha ");
            direccion = 3;
          } else if (dx == 1) {
            strcat(ruta, "Giro 180 ");
            direccion = 0;
          }
        } else if (direccion == 3) { // y+
          if (dy == 1) {
            strcat(ruta, "Recto ");
          } else if (dx == 1) {
            strcat(ruta, "Derecha ");
            direccion = 0;
          } else if (dx == -1) {
            strcat(ruta, "Izquierda ");
            direccion = 2;
          } else if (dy == -1) {
            strcat(ruta, "Giro 180 ");
            direccion = 1;
          }
        }
      }
      Serial.println(ruta);
      mostrarRuta(grid);
      liberarNodos(cerrada, tamCerrada);
      liberarNodos(abierta, tamAbierta);

      // Ejecutar los movimientos calculados
      ejecutarTrayectoria(ruta);

      return;
    }

    int movimientos[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };
    for (int i = 0; i < 4; i++) {
      int nx = actual->x + movimientos[i][0];
      int ny = actual->y + movimientos[i][1];

      if (nx >= 0 && nx < 6 && ny >= 0 && ny < 5 && !blocked[4 - ny][nx]) {
        if (estaEnLista(cerrada, tamCerrada, nx, ny)) {
          continue;
        }

        Nodo* vecino = new Nodo{nx, ny, 0, 0, 0, actual};
        vecino->g = actual->g + 1;
        vecino->h = calcularH(nx, ny, destinoX, destinoY);
        vecino->f = vecino->g + vecino->h;

        bool mejorRuta = true;
        for (int j = 0; j < tamAbierta; j++) {
          if (abierta[j]->x == vecino->x && abierta[j]->y == vecino->y) {
            if (vecino->g < abierta[j]->g) {
              abierta[j]->g = vecino->g;
              abierta[j]->f = vecino->f;
              abierta[j]->parent = actual;
            } else {
              mejorRuta = false;
            }
            break;
          }
        }

        if (mejorRuta) {
          abierta[tamAbierta++] = vecino;
        } else {
          delete vecino;
        }
      }
    }
  }

  Serial.println("No se encontró una ruta válida.");
  liberarNodos(cerrada, tamCerrada);
  liberarNodos(abierta, tamAbierta);
}

void seguirLinea() {
  int position = qtr.readLineBlack(sensorValues);
  int error = position - 2500;  // Suponemos que 2500 es el centro
  int derivative = error - lastError;
  integral += error;

  int motorSpeedAdjustment = Kp * error + Ki * integral + Kd * derivative;
  int leftSpeed = baseSpeed - motorSpeedAdjustment;
  int rightSpeed = baseSpeed + motorSpeedAdjustment;

  leftSpeed = constrain(leftSpeed, 0, MaxSpeed);
  rightSpeed = constrain(rightSpeed, 0, MaxSpeed);

  setMotorPWM(leftSpeed, rightSpeed);
  lastError = error;
}

bool cruceDetectado(uint16_t *sensorValues) {
  // Consideramos que hay un cruce cuando todos los sensores detectan la línea negra
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 800) { // Ajusta el umbral según sea necesario
      return false;
    }
  }
  Serial.println("Estoy en un cruce");
  return true;
}

void ejecutarTrayectoria(char* ruta) {
  char* movimiento = strtok(ruta, " ");
  while (movimiento != NULL) {
    if (strcmp(movimiento, "Recto") == 0) {
      while (true) {
        seguirLinea();
        if (cruceDetectado(sensorValues)) {
          break;
        }
      }
      // Pausar 500ms en el cruce
      setMotorPWM(0, 0);
      delay(500);
      // Avanzar sin seguir el PID para evitar múltiples detecciones del cruce
      setMotorPWM(baseSpeed, baseSpeed);
      delay(300);
      // Detener el motor brevemente
      setMotorPWM(0, 0);
      delay(500);
    } else if (strcmp(movimiento, "Derecha") == 0) {
      Serial.println("Me muevo Derecha");
      setMotorPWM(100, -100);
      delay(300);
      setMotorPWM(0, 0);
      delay(500);
      // Seguir la línea usando el PID hasta el siguiente cruce
      while (true) {
        seguirLinea();
        if (cruceDetectado(sensorValues)) {
          break;
        }
      }
      // Pausar 500ms en el cruce
      setMotorPWM(0, 0);
      delay(500);
    } else if (strcmp(movimiento, "Izquierda") == 0) {
      setMotorPWM(-100, 100);
      delay(300);
      // Detener el motor brevemente
      setMotorPWM(0, 0);
      delay(500);
      // Seguir la línea usando el PID hasta el siguiente cruce
      while (true) {
        seguirLinea();
        if (cruceDetectado(sensorValues)) {
          break;
        }
      }
      // Pausar 500ms en el cruce
      setMotorPWM(0, 0);
      delay(500);
    }
    movimiento = strtok(NULL, " ");
  }
}

void loop() {
  // El loop principal queda vacío
}
