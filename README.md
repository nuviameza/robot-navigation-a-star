
# Autonomous Robot Navigation with A*

Proyecto final de la cátedra de **Inteligencia Artificial aplicada a la Robótica — FIUNA**.

El proyecto implementa un sistema de navegación autónoma para un robot diferencial utilizando:

- algoritmo A* sobre una grilla de ocupación;
- cámara de profundidad ZED 2i;
- ArUco para localización externa;
- YOLOv8n para detección de objetos;
- ESP8266 para comunicación UDP;
- sensor ultrasónico HC-SR04 para detección local de obstáculos;
- Arduino Uno para planificación y control del robot.

## Funcionamiento

La cámara ZED 2i genera una representación del entorno a partir de información de profundidad. Esta representación se reduce a una grilla de `6 × 6` celdas y se transmite al Arduino mediante ESP8266.

El Arduino ejecuta A* utilizando conectividad de cuatro vecinos y distancia Manhattan como heurística.

Durante el desplazamiento, el HC-SR04 detecta obstáculos no incluidos inicialmente en el mapa. Cuando esto ocurre, el robot actualiza localmente la grilla y vuelve a ejecutar A* desde su posición actual.

## Archivos principales

- `sketch_sep22b.ino`: firmware del Arduino. Incluye la implementación de A*, control de motores, lectura del sensor ultrasónico, control del servomotor y lógica de replanificación.

- `zed_robot_mapeos.py`: módulo de percepción externa. Incluye procesamiento de la cámara ZED 2i, generación de la grilla de ocupación, detección ArUco, detección de objetos mediante YOLOv8n y envío de la grilla al robot.

- `informe.pdf`: informe técnico final del proyecto, con la descripción de la arquitectura, metodología, implementación, resultados y conclusiones.

## Parámetros principales

Grilla: 6 × 6
PWM motores: 100
TIEMPO_CELDA: 700 ms
TIEMPO_GIRO_90: 600 ms
DISTANCIA_MINIMA: 8 cm
UDP port: 4210
Estructura
arduino/
    sketch_sep22b.ino
perception/
    zed_robot_mapeos.py
report/
    informe.tex
source_zips/
    archivos originales del proyecto

Limitaciones
El robot no dispone de encoders, por lo que la posición interna se estima a partir de los movimientos ejecutados y tiempos previamente calibrados. La posición obtenida mediante ArUco no se utiliza actualmente para corregir continuamente la posición del Arduino.

Autores:

Francisco Santiago Moreno Schribertschnik

Antonella Prado

Nuvia Meza


Facultad de Ingeniería
Universidad Nacional de Asunción
Paraguay







