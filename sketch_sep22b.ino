#include <Servo.h>
#include <SoftwareSerial.h>
#include "WiFiEsp.h"
#include "WiFiEspUdp.h"

// =====================================================
// WIFI / UDP
// =====================================================

SoftwareSerial espSerial(10, 11);
WiFiEspUDP udp;

char ssid[] = "LabEquiposMedicos";
char pass[] = "Materiales2021#";

const unsigned int PUERTO_UDP = 4210;

// =====================================================
// PINES
// =====================================================

#define ENA 5
#define IN1 2
#define IN2 4
#define IN3 7
#define IN4 8
#define ENB 6

#define SERVO_PIN A2
#define TRIG_PIN A1
#define ECHO_PIN A0

// =====================================================
// MOVIMIENTO
// =====================================================

#define VELOCIDAD 100
#define TIEMPO_CELDA 700
#define TIEMPO_GIRO_90 600
#define DISTANCIA_MINIMA 8

// =====================================================
// ULTRASONIDO / SERVO
// =====================================================

#define ANGULO_DERECHA 0
#define ANGULO_CENTRO 90
#define ANGULO_IZQUIERDA 180

Servo servoUS;

// =====================================================
// GRILLA
// =====================================================

#define FILAS 6
#define COLUMNAS 6
#define MAX_NODOS 24

int8_t grilla[FILAS][COLUMNAS] = {
  {0,0,0,0,0,0},
  {0,1,1,1,0,0},
  {0,0,0,1,0,0},
  {0,1,0,1,0,0},
  {0,1,0,0,0,0},
  {0,0,0,1,1,0}
};

// =====================================================
// POSICIÓN DEL ROBOT
// =====================================================

int8_t posX = 0;
int8_t posY = 0;

const int8_t objetivoX = 5;
const int8_t objetivoY = 5;

// 0=arriba, 1=derecha, 2=abajo, 3=izquierda
int8_t orientacionActual = 2;

// =====================================================
// RECEPCIÓN DEL GRID POR UDP
// =====================================================

void recibirGridPorWiFi() {

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("ESPERANDO GRID POR UDP..."));
  Serial.println(F("================================"));

  unsigned long inicio = millis();

  while (millis() - inicio < 30000) {

    int packetSize = udp.parsePacket();

    if (packetSize > 0) {

      Serial.println();
      Serial.println(F(">>> PAQUETE UDP RECIBIDO <<<"));

      Serial.print(F("Tamano del paquete: "));
      Serial.println(packetSize);

      Serial.print(F("IP origen: "));
      Serial.println(udp.remoteIP());

      Serial.print(F("Puerto origen: "));
      Serial.println(udp.remotePort());

      char buffer[MAX_NODOS + 1];
      int len = udp.read(buffer, MAX_NODOS);
      buffer[len] = '\0';

      Serial.print(F("Bytes leidos: "));
      Serial.println(len);

      Serial.print(F("Contenido recibido: "));
      Serial.println(buffer);

      if (len != FILAS * COLUMNAS) {
        Serial.println(F(">>> ERROR: TAMANO INCORRECTO <<<"));
        return;
      }

      bool valido = true;
      for (int i = 0; i < FILAS * COLUMNAS; i++) {
        if (buffer[i] != '0' && buffer[i] != '1') {
          valido = false;
          Serial.print(F("Caracter invalido en posicion "));
          Serial.print(i);
          Serial.print(F(": "));
          Serial.println(buffer[i]);
          break;
        }
      }

      if (!valido) {
        Serial.println(F(">>> GRID INVALIDO <<<"));
        return;
      }

      for (int i = 0; i < FILAS * COLUMNAS; i++) {
        int fila = i / COLUMNAS;
        int col = i % COLUMNAS;
        grilla[fila][col] = buffer[i] - '0';
      }

      Serial.println();
      Serial.println(F("================================"));
      Serial.println(F(">>> GRID RECIBIDO CORRECTAMENTE"));
      Serial.println(F("================================"));

      Serial.println(F("Grid recibido:"));
      for (int f = 0; f < FILAS; f++) {
        for (int c = 0; c < COLUMNAS; c++) {
          Serial.print(grilla[f][c]);
          Serial.print(' ');
        }
        Serial.println();
      }

      return;
    }

    delay(10);
  }

  Serial.println();
  Serial.println(F(">>> TIMEOUT ESPERANDO GRID <<<"));
  Serial.println(F("Se mantiene la grilla por defecto."));
}

// =====================================================
// ESTRUCTURA A*
// =====================================================

struct Nodo {
  int8_t x;
  int8_t y;
  int8_t g;
  int8_t f;
  int8_t padre;   // índice dentro de abiertos (-1 si inicio)
};

Nodo abiertos[MAX_NODOS];
bool visitado[FILAS][COLUMNAS];

int numAbiertos = 0;

int8_t metaX;
int8_t metaY;

int8_t caminoX[FILAS * COLUMNAS];
int8_t caminoY[FILAS * COLUMNAS];
int largoCamino = 0;

// =====================================================
// HEURÍSTICA MANHATTAN
// =====================================================

inline int heuristica(int x, int y) {
  return abs(x - metaX) + abs(y - metaY);
}

// =====================================================
// FUNCIONES AUXILIARES A*
// =====================================================

int enAbiertos(int x, int y) {
  for (int i = 0; i < numAbiertos; i++) {
    if (abiertos[i].x == x && abiertos[i].y == y) return i;
  }
  return -1;
}

int indiceMenorF() {
  int mejor = 0;
  for (int i = 1; i < numAbiertos; i++) {
    if (abiertos[i].f < abiertos[mejor].f) mejor = i;
  }
  return mejor;
}

void quitarDeAbiertos(int idx) {
  abiertos[idx] = abiertos[numAbiertos - 1];
  numAbiertos--;
}

// =====================================================
// ALGORITMO A*
// =====================================================

bool aEstrella(int inicioX, int inicioY, int objetivoXLocal, int objetivoYLocal) {

  metaX = objetivoXLocal;
  metaY = objetivoYLocal;

  numAbiertos = 0;
  largoCamino = 0;

  // Limpiar matriz de visitados
  for (int f = 0; f < FILAS; f++) {
    for (int c = 0; c < COLUMNAS; c++) {
      visitado[f][c] = false;
    }
  }

  Nodo inicio;
  inicio.x = inicioX;
  inicio.y = inicioY;
  inicio.g = 0;
  inicio.f = heuristica(inicioX, inicioY);
  inicio.padre = -1;

  abiertos[numAbiertos++] = inicio;

  int8_t dx[4] = {-1, 1, 0, 0};
  int8_t dy[4] = {0, 0, -1, 1};

  while (numAbiertos > 0) {

    int idxActual = indiceMenorF();
    Nodo actual = abiertos[idxActual];

    // Reconstruir camino usando cadena de padres ANTES de quitar
    if (actual.x == metaX && actual.y == metaY) {

      // Reconstruir hacia atrás
      int8_t tmpX[FILAS * COLUMNAS];
      int8_t tmpY[FILAS * COLUMNAS];
      int n = 0;

      int idx = idxActual;
      // Nota: el padre apunta dentro de abiertos, pero como quitamos
      // elementos, los índices pueden invalidarse. Para simplificar,
      // reconstruimos usando una copia temporal del nodo actual.
      Nodo nodoRecon = actual;
      tmpX[n] = nodoRecon.x;
      tmpY[n] = nodoRecon.y;
      n++;

      // Como no guardamos cadena completa de padres estables,
      // usamos búsqueda por cercanía sobre visitado para reconstruir.
      // Mejor: guardamos el camino parcial en cada nodo.

      // Solución simple: guardar camino completo en cada nodo no es viable.
      // En su lugar, usamos el padre como índice en abiertos ANTES de quitarlo.
      // Pero como quitamos con swap, los índices cambian.

      // Para que funcione correctamente, cambiamos estrategia:
      // reconstruimos el camino buscando el nodo padre por (x,y)
      // en la lista original. Pero ya no está.

      // --- Estrategia alternativa robusta ---
      // Volvemos a ejecutar A* guardando el camino en cada paso.
      // Pero para no complicar, usamos el método clásico:
      // reconstruir desde el nodo actual usando el campo padre
      // mientras los índices sean válidos (no se ha hecho swap aún).

      // Como hemos quitado el nodo actual de abiertos, los índices
      // de los padres pueden haberse movido. Para evitar esto,
      // NO usamos swap en quitarDeAbiertos durante la reconstrucción.

      // Simplificación: guardamos el camino en el momento de encontrar la meta
      // usando una lista auxiliar de nodos padre.

      // --- Implementación correcta ---
      // Mejor guardar camino desde el inicio en cada nodo. Pero eso ocupa mucho.

      // Solución final: usar búsqueda hacia atrás por posición en una copia
      // de la lista de abiertos original. Para ello, mantenemos una copia.

      // Como esto se complica, optamos por reconstruir el camino
      // ejecutando de nuevo A* pero guardando el camino en cada nodo.

      // --- REIMPLEMENTACIÓN ---
      // Para no alargar, usamos el enfoque de guardar el camino completo
      // en una variable global durante la expansión.

      // Este bloque se reemplaza por la lógica de abajo.
      break;
    }

    // Marcar como visitado
    visitado[actual.x][actual.y] = true;
    quitarDeAbiertos(idxActual);

    for (int i = 0; i < 4; i++) {
      int nx = actual.x + dx[i];
      int ny = actual.y + dy[i];

      if (nx < 0 || nx >= FILAS || ny < 0 || ny >= COLUMNAS) continue;
      if (grilla[nx][ny] == 1) continue;
      if (visitado[nx][ny]) continue;

      int gTentativo = actual.g + 1;
      int idxAbierto = enAbiertos(nx, ny);

      if (idxAbierto == -1) {
        if (numAbiertos < MAX_NODOS) {
          Nodo vecino;
          vecino.x = nx;
          vecino.y = ny;
          vecino.g = gTentativo;
          vecino.f = gTentativo + heuristica(nx, ny);
          vecino.padre = -1; // no usado en esta versión
          abiertos[numAbiertos++] = vecino;
        }
      } else if (gTentativo < abiertos[idxAbierto].g) {
        abiertos[idxAbierto].g = gTentativo;
        abiertos[idxAbierto].f = gTentativo + heuristica(nx, ny);
      }
    }
  }

  return false;
}

// =====================================================
// NOTA: El algoritmo A* anterior necesita reconstruir el camino.
// Para que funcione correctamente sin consumir mucha memoria,
// implementamos una versión simplificada que guarda el camino
// directamente usando BFS con predecesores en una matriz.
// =====================================================

// =====================================================
// A* SIMPLIFICADO CON MATRIZ DE PREDECESORES
// =====================================================

int8_t predX[FILAS][COLUMNAS];
int8_t predY[FILAS][COLUMNAS];
int8_t costoG[FILAS][COLUMNAS];
bool enListaAbierta[FILAS][COLUMNAS];

bool aEstrellaSimple(int inicioX, int inicioY, int objX, int objY) {

  metaX = objX;
  metaY = objY;
  largoCamino = 0;

  // Inicializar matrices
  for (int f = 0; f < FILAS; f++) {
    for (int c = 0; c < COLUMNAS; c++) {
      costoG[f][c] = 127;
      predX[f][c] = -1;
      predY[f][c] = -1;
      visitado[f][c] = false;
      enListaAbierta[f][c] = false;
    }
  }

  // Lista de abiertos simple (coordenadas)
  int8_t listaX[MAX_NODOS];
  int8_t listaY[MAX_NODOS];
  int numLista = 0;

  listaX[numLista] = inicioX;
  listaY[numLista] = inicioY;
  numLista++;

  costoG[inicioX][inicioY] = 0;
  enListaAbierta[inicioX][inicioY] = true;

  int8_t dx[4] = {-1, 1, 0, 0};
  int8_t dy[4] = {0, 0, -1, 1};

  while (numLista > 0) {

    // Encontrar el nodo con menor f = g + h
    int mejorIdx = 0;
    int mejorF = 32767;

    for (int i = 0; i < numLista; i++) {
      int g = costoG[listaX[i]][listaY[i]];
      int h = abs(listaX[i] - metaX) + abs(listaY[i] - metaY);
      int f = g + h;
      if (f < mejorF) {
        mejorF = f;
        mejorIdx = i;
      }
    }

    int cx = listaX[mejorIdx];
    int cy = listaY[mejorIdx];

    // Quitar de la lista (swap con último)
    listaX[mejorIdx] = listaX[numLista - 1];
    listaY[mejorIdx] = listaY[numLista - 1];
    numLista--;

    enListaAbierta[cx][cy] = false;
    visitado[cx][cy] = true;

    // ¿Meta alcanzada?
    if (cx == metaX && cy == metaY) {

      // Reconstruir camino hacia atrás
      int8_t tmpX[FILAS * COLUMNAS];
      int8_t tmpY[FILAS * COLUMNAS];
      int n = 0;

      int px = cx, py = cy;
      while (px != -1 && py != -1) {
        tmpX[n] = px;
        tmpY[n] = py;
        n++;
        int8_t ax = predX[px][py];
        int8_t ay = predY[px][py];
        px = ax;
        py = ay;
      }

      largoCamino = n;
      for (int i = 0; i < n; i++) {
        caminoX[i] = tmpX[n - 1 - i];
        caminoY[i] = tmpY[n - 1 - i];
      }

      return true;
    }

    // Expandir vecinos
    for (int i = 0; i < 4; i++) {
      int nx = cx + dx[i];
      int ny = cy + dy[i];

      if (nx < 0 || nx >= FILAS || ny < 0 || ny >= COLUMNAS) continue;
      if (grilla[nx][ny] == 1) continue;
      if (visitado[nx][ny]) continue;

      int gTentativo = costoG[cx][cy] + 1;

      if (gTentativo < costoG[nx][ny]) {
        costoG[nx][ny] = gTentativo;
        predX[nx][ny] = cx;
        predY[nx][ny] = cy;

        if (!enListaAbierta[nx][ny]) {
          if (numLista < MAX_NODOS) {
            listaX[numLista] = nx;
            listaY[numLista] = ny;
            numLista++;
            enListaAbierta[nx][ny] = true;
          }
        }
      }
    }
  }

  return false;
}

// =====================================================
// MOTORES
// =====================================================

void detener() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void avanzar() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, VELOCIDAD);
  analogWrite(ENB, VELOCIDAD);
}

void retroceder() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, VELOCIDAD);
  analogWrite(ENB, VELOCIDAD);
}

void girarIzquierda() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, VELOCIDAD);
  analogWrite(ENB, VELOCIDAD);
}

void girarDerecha() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, VELOCIDAD);
  analogWrite(ENB, VELOCIDAD);
}

// =====================================================
// ORIENTACIÓN
// =====================================================

void girarHaciaOrientacion(int deseada) {

  int diferencia = (deseada - orientacionActual + 4) % 4;

  if (diferencia == 0) return;

  if (diferencia == 1) {
    girarDerecha();
    delay(TIEMPO_GIRO_90);
    detener();
  } else if (diferencia == 3) {
    girarIzquierda();
    delay(TIEMPO_GIRO_90);
    detener();
  } else if (diferencia == 2) {
    girarDerecha();
    delay(TIEMPO_GIRO_90 * 2);
    detener();
  }

  delay(150);
  orientacionActual = deseada;
}

int obtenerOrientacion(int actualX, int actualY, int siguienteX, int siguienteY) {
  int dx = siguienteX - actualX;
  int dy = siguienteY - actualY;

  if (dx == -1 && dy == 0) return 0;
  if (dx == 0 && dy == 1) return 1;
  if (dx == 1 && dy == 0) return 2;
  if (dx == 0 && dy == -1) return 3;

  return -1;
}

// =====================================================
// ULTRASONIDO
// =====================================================

float medirDistanciaCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duracion = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duracion == 0) return 999.0;

  return duracion * 0.0343 / 2.0;
}

// =====================================================
// MARCAR OBSTÁCULO LATERAL
// =====================================================

void marcarObstaculoLateral(int direccion, float distancia) {

  if (distancia >= DISTANCIA_MINIMA) return;

  int8_t dx[4] = {-1, 0, 1, 0};
  int8_t dy[4] = {0, 1, 0, -1};

  int obstaculoX = posX + dx[direccion];
  int obstaculoY = posY + dy[direccion];

  if (obstaculoX >= 0 && obstaculoX < FILAS &&
      obstaculoY >= 0 && obstaculoY < COLUMNAS) {

    grilla[obstaculoX][obstaculoY] = 1;

    Serial.print(F("Obstaculo lateral marcado: ("));
    Serial.print(obstaculoX);
    Serial.print(',');
    Serial.print(obstaculoY);
    Serial.println(')');
  }
}

// =====================================================
// ESCANEAR LADOS
// =====================================================

void escanearLadosYActualizarGrilla() {

  detener();

  servoUS.write(ANGULO_IZQUIERDA);
  delay(500);
  float distanciaIzquierda = medirDistanciaCM();

  servoUS.write(ANGULO_DERECHA);
  delay(500);
  float distanciaDerecha = medirDistanciaCM();

  servoUS.write(ANGULO_CENTRO);
  delay(500);

  Serial.print(F("Distancia izquierda: "));
  Serial.print(distanciaIzquierda);
  Serial.println(F(" cm"));

  Serial.print(F("Distancia derecha: "));
  Serial.print(distanciaDerecha);
  Serial.println(F(" cm"));

  int direccionIzquierda = (orientacionActual + 3) % 4;
  int direccionDerecha = (orientacionActual + 1) % 4;

  marcarObstaculoLateral(direccionIzquierda, distanciaIzquierda);
  marcarObstaculoLateral(direccionDerecha, distanciaDerecha);
}

// =====================================================
// RETROCEDER UN POCO
// =====================================================

void retrocederUnPoco() {
  Serial.println(F("Retrocediendo un poco..."));
  detener();
  delay(300);
  retroceder();
  delay(500);
  detener();
  delay(200);
}

// =====================================================
// AVANZAR UNA CELDA
// =====================================================

bool avanzarUnaCeldaConDeteccion() {

  unsigned long tiempoInicial = millis();
  avanzar();

  while (millis() - tiempoInicial < TIEMPO_CELDA) {

    float distanciaDuranteAvance = medirDistanciaCM();

    Serial.print(F("Distancia durante avance: "));
    Serial.print(distanciaDuranteAvance);
    Serial.println(F(" cm"));

    if (distanciaDuranteAvance < DISTANCIA_MINIMA) {
      detener();
      Serial.println(F(">>> OBSTACULO DETECTADO DURANTE EL AVANCE"));
      retrocederUnPoco();
      return false;
    }

    delay(40);
  }

  return true;
}

// =====================================================
// IMPRIMIR CAMINO
// =====================================================

void imprimirCamino() {
  Serial.print(F("Camino: "));

  for (int i = 0; i < largoCamino; i++) {
    Serial.print('(');
    Serial.print(caminoX[i]);
    Serial.print(',');
    Serial.print(caminoY[i]);
    Serial.print(')');

    if (i < largoCamino - 1) Serial.print(F(" -> "));
  }

  Serial.println();
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  detener();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
  delay(500);

  Serial.println();
  Serial.println(F("======================="));
  Serial.println(F("ROBOT AUTONOMO A*"));
  Serial.println(F("======================="));

  Serial.print(F("Inicio: ("));
  Serial.print(posX);
  Serial.print(',');
  Serial.print(posY);
  Serial.println(')');

  Serial.print(F("Meta: ("));
  Serial.print(objetivoX);
  Serial.print(',');
  Serial.print(objetivoY);
  Serial.println(')');

  delay(3000);

  Serial.println();
  Serial.println(F("Inicializando ESP-01..."));

  espSerial.begin(9600);
  delay(2000);

  while (espSerial.available()) espSerial.read();

  WiFi.init(&espSerial);

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("ERROR: No se detecta ESP8266"));
    while (true) detener();
  }

  Serial.println(F("ESP8266 detectado correctamente."));

  Serial.print(F("Conectando a: "));
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Intentando conexion..."));
    WiFi.begin(ssid, pass);
    delay(2000);
  }

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("WIFI CONECTADO"));
  Serial.println(F("================================"));

  Serial.print(F("IP del ESP-01: "));
  Serial.println(WiFi.localIP());

  Serial.print(F("Puerto UDP: "));
  Serial.println(PUERTO_UDP);

  if (udp.begin(PUERTO_UDP)) {
    Serial.println(F("UDP iniciado correctamente."));
  } else {
    Serial.println(F("ERROR: No se pudo iniciar UDP."));
  }

  recibirGridPorWiFi();

  servoUS.attach(SERVO_PIN);
  servoUS.write(90);
  delay(500);

  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("SETUP COMPLETADO"));
  Serial.println(F("================================"));
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================

void loop() {

  // 1. COMPROBAR META
  if (posX == objetivoX && posY == objetivoY) {
    detener();

    Serial.println();
    Serial.println(F("======================="));
    Serial.println(F("META ALCANZADA"));
    Serial.println(F("======================="));

    while (true) detener();
  }

  // 2. CALCULAR CAMINO
  bool encontrado = aEstrellaSimple(posX, posY, objetivoX, objetivoY);

  if (!encontrado) {
    detener();
    Serial.println();
    Serial.println(F("SIN CAMINO DISPONIBLE"));
    while (true) detener();
  }

  imprimirCamino();

  if (largoCamino < 2) {
    detener();
    return;
  }

  // 3. SIGUIENTE CELDA
  int siguienteX = caminoX[1];
  int siguienteY = caminoY[1];

  Serial.println();
  Serial.print(F("Posicion actual: ("));
  Serial.print(posX);
  Serial.print(',');
  Serial.print(posY);
  Serial.println(')');

  Serial.print(F("Siguiente celda: ("));
  Serial.print(siguienteX);
  Serial.print(',');
  Serial.print(siguienteY);
  Serial.println(')');

  // 4. ORIENTACIÓN
  int orientacionDeseada = obtenerOrientacion(posX, posY, siguienteX, siguienteY);

  if (orientacionDeseada == -1) {
    detener();
    Serial.println(F("ERROR DE ORIENTACION"));
    while (true) detener();
  }

  // 5. GIRAR
  girarHaciaOrientacion(orientacionDeseada);

  // 6. SENSOR AL FRENTE
  servoUS.write(90);
  delay(300);

  // 7. MEDIR DISTANCIA
  float distancia = medirDistanciaCM();

  Serial.print(F("Distancia: "));
  Serial.print(distancia);
  Serial.println(F(" cm"));

  // 8. OBSTÁCULO
  if (distancia < DISTANCIA_MINIMA) {
    detener();

    Serial.println(F(">>> OBSTACULO DETECTADO"));

    grilla[siguienteX][siguienteY] = 1;

    Serial.print(F("Marcando celda: ("));
    Serial.print(siguienteX);
    Serial.print(',');
    Serial.print(siguienteY);
    Serial.println(')');

    Serial.println(F("Recalculando camino..."));

    escanearLadosYActualizarGrilla();
    delay(500);
    return;
  }

  // 9. CELDA LIBRE
  Serial.println(F("Celda libre -> avanzar"));

  bool avanceCompleto = avanzarUnaCeldaConDeteccion();
  delay(200);

  if (!avanceCompleto) {
    grilla[siguienteX][siguienteY] = 1;

    Serial.print(F("Marcando celda durante avance: ("));
    Serial.print(siguienteX);
    Serial.print(',');
    Serial.print(siguienteY);
    Serial.println(')');

    escanearLadosYActualizarGrilla();
    Serial.println(F("Recalculando camino..."));
    return;
  }

  // 10. ACTUALIZAR POSICIÓN
  posX = siguienteX;
  posY = siguienteY;

  Serial.print(F("Nueva posicion: ("));
  Serial.print(posX);
  Serial.print(',');
  Serial.print(posY);
  Serial.println(')');

  delay(300);
}