"""
ZED 2i - Sistema integrado: tracking del robot (ArUco) + mapa de obstaculos
=============================================================================
Corre en Windows nativo (requiere pyzed). Una sola conexion a la camara.

Por que no usamos homografia por clicks (a diferencia del script UVC de tu
companero): con pyzed ya tenemos profundidad real en metros. Para saber donde
esta el ArUco en el mundo, consultamos directamente el punto 3D de la nube de
puntos en el pixel del marcador -- no hace falta calibrar con un objeto de
tamano conocido. La homografia solo es necesaria cuando NO hay profundidad
(como en una webcam UVC pura).

Secciones:
  A. Configuracion
  B. Apertura de camara (pyzed) con sistema de coordenadas Z-up
  C. Deteccion ArUco -> posicion y orientacion 3D real del robot
  D. Profundidad -> occupancy grid 2D (obstaculos)
  E. Calibracion del nivel de piso (una tecla, no clicks)
  F. Envio a WSL por socket (placeholder, completar con tu IP de WSL)
  G. Bucle principal

Teclas:
  F = calibrar nivel de piso (apuntar a piso vacio y presionar)
  Q = salir
"""

import pyzed.sl as sl
import numpy as np
import cv2
import math
import socket
import json
from ultralytics import YOLO

# ------------------------------------------------------------------
# A. CONFIGURACION
# ------------------------------------------------------------------
ARUCO_DICT = cv2.aruco.DICT_4X4_50
GRID_SIZE_METERS = 5.0       # area cubierta: 5x5 m (ajustar a tu espacio real)
GRID_RESOLUTION = 0.05       # cada celda: 5cm x 5cm
GRID_CELLS = int(GRID_SIZE_METERS / GRID_RESOLUTION)
ROBOT_HEIGHT_MAX = 1.5       # altura maxima considerada "obstaculo relevante"
OBSTACLE_MARGIN = 0.05       # ignora ruido de +-5cm sobre el nivel de piso

ESP01_IP = "192.168.1.104"          # completar con la IP que te dio AT+CIFSR en el ESP-01
ESP01_PORT = 4210
ARDUINO_FILAS = 6              # debe coincidir con FILAS/COLUMNAS del sketch Arduino
ARDUINO_COLUMNAS = 6

YOLO_MODEL_PATH = "yolov8n.pt"   # se descarga solo la primera vez
YOLO_CONFIANZA_MIN = 0.5
YOLO_CLASES_INTERES = None       # None = todas las clases COCO; o lista ej: ["person", "chair"]


# ------------------------------------------------------------------
# B. APERTURA DE CAMARA
# ------------------------------------------------------------------
def abrir_camara():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # evita NEURAL a proposito
    init_params.coordinate_units = sl.UNIT.METER
    # Z-up: el SDK usa el IMU para alinear el eje Z con la gravedad,
    # asi el eje Z siempre representa "altura real" sin importar el
    # angulo de inclinacion de la camara en el tripode.
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Error al abrir la camara: {status}")

    return zed


# ------------------------------------------------------------------
# C. DETECCION ARUCO -> POSICION 3D REAL
# ------------------------------------------------------------------
def crear_detector_aruco():
    diccionario = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    parametros = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(diccionario, parametros)


def punto_3d_en_pixel(point_cloud_np, x_px, y_px):
    """Devuelve (X, Y, Z) en metros del punto 3D en ese pixel, o None si invalido."""
    h, w = point_cloud_np.shape[:2]
    if 0 <= y_px < h and 0 <= x_px < w:
        pt = point_cloud_np[y_px, x_px]
        x, y, z = pt[0], pt[1], pt[2]
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            return float(x), float(y), float(z)
    return None


def detectar_robot(frame_bgr, point_cloud_np, detector):
    """
    Detecta el marcador ArUco del robot y devuelve su posicion y orientacion
    en coordenadas reales del mundo (metros, sistema Z-up de la camara).
    """
    corners, ids, _ = detector.detectMarkers(frame_bgr)
    if ids is None or len(ids) == 0:
        return None

    # Tomamos el primer marcador detectado (ajustar si hay mas de un robot)
    puntos_px = corners[0].reshape(4, 2).astype(int)
    top_left_px, top_right_px = puntos_px[0], puntos_px[1]
    centro_px = puntos_px.mean(axis=0).astype(int)

    centro_3d = punto_3d_en_pixel(point_cloud_np, centro_px[0], centro_px[1])
    tl_3d = punto_3d_en_pixel(point_cloud_np, top_left_px[0], top_left_px[1])
    tr_3d = punto_3d_en_pixel(point_cloud_np, top_right_px[0], top_right_px[1])

    if centro_3d is None or tl_3d is None or tr_3d is None:
        return None

    # Orientacion: vector entre dos esquinas del marcador, proyectado
    # sobre el plano del piso (X, Y) -- ya en coordenadas reales, no en pixeles.
    dx = tr_3d[0] - tl_3d[0]
    dy = tr_3d[1] - tl_3d[1]
    theta = math.degrees(math.atan2(dy, dx)) % 360.0

    return {
        "id": int(ids.flatten()[0]),
        "x": centro_3d[0],
        "y": centro_3d[1],
        "z": centro_3d[2],
        "theta_deg": theta,
        "pixel_centro": (int(centro_px[0]), int(centro_px[1])),
    }


# ------------------------------------------------------------------
# D. PROFUNDIDAD -> OCCUPANCY GRID
# ------------------------------------------------------------------
def calcular_occupancy_grid(point_cloud_np, floor_z):
    """
    Filtra la nube de puntos por altura real (Z, ya alineado con la
    gravedad) y proyecta lo que queda como obstaculo a un grid 2D.
    Como el sistema es Z-up, X e Y ya son coordenadas metricas del
    plano del piso -- no hace falta homografia.
    """
    grid = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.uint8)

    xs = point_cloud_np[:, :, 0].flatten()
    ys = point_cloud_np[:, :, 1].flatten()
    zs = point_cloud_np[:, :, 2].flatten()

    altura_relativa = zs - floor_z
    valid = (
        np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        & (altura_relativa > OBSTACLE_MARGIN)
        & (altura_relativa < ROBOT_HEIGHT_MAX)
    )
    xs, ys = xs[valid], ys[valid]
    if len(xs) == 0:
        return grid

    # Centrar el grid en el origen de la camara (offset a la mitad del area)
    col = ((xs + GRID_SIZE_METERS / 2) / GRID_SIZE_METERS * GRID_CELLS).astype(int)
    row = ((ys + GRID_SIZE_METERS / 2) / GRID_SIZE_METERS * GRID_CELLS).astype(int)

    dentro = (col >= 0) & (col < GRID_CELLS) & (row >= 0) & (row < GRID_CELLS)
    grid[row[dentro], col[dentro]] = 255

    return grid


def reducir_grid_para_arduino(grid_fino):
    """
    Agrupa el grid fino (GRID_CELLS x GRID_CELLS) en bloques, para producir
    el grid grueso (ARDUINO_FILAS x ARDUINO_COLUMNAS) que usa el A* del
    Arduino. Si CUALQUIER celda de un bloque esta ocupada, el bloque
    completo se marca como obstaculo (mas conservador/seguro).
    """
    bloque_h = GRID_CELLS // ARDUINO_FILAS
    bloque_w = GRID_CELLS // ARDUINO_COLUMNAS
    grid_reducido = np.zeros((ARDUINO_FILAS, ARDUINO_COLUMNAS), dtype=np.uint8)

    for fila in range(ARDUINO_FILAS):
        for col in range(ARDUINO_COLUMNAS):
            bloque = grid_fino[
                fila * bloque_h:(fila + 1) * bloque_h,
                col * bloque_w:(col + 1) * bloque_w
            ]
            grid_reducido[fila, col] = 1 if np.any(bloque > 0) else 0

    return grid_reducido


# ------------------------------------------------------------------
# E. CALIBRACION DEL NIVEL DE PISO
# ------------------------------------------------------------------
def calibrar_piso(point_cloud_np):
    """
    Toma una franja central-inferior de la imagen (asumida piso vacio
    en el momento de calibrar) y devuelve la altura Z tipica del piso.
    Presionar 'F' apuntando a piso despejado antes de operar.
    """
    h, w = point_cloud_np.shape[:2]
    franja = point_cloud_np[int(h * 0.7):h, int(w * 0.3):int(w * 0.7), 2]
    valores = franja[np.isfinite(franja)]
    if len(valores) == 0:
        return None
    return float(np.median(valores))


# ------------------------------------------------------------------
# F. ENVIO AL ESP-01 (envio puntual, no continuo)
# ------------------------------------------------------------------
def enviar_a_arduino(grid_fino):
    """
    Reduce el grid fino a 6x6 y lo manda como 36 bytes crudos ('0'/'1')
    al ESP-01, que actua como servidor TCP (AT+CIPSERVER). El Arduino
    los recibe via espSerial y los interpreta con recibirGridPorWiFi().
    Envio puntual: se llama una vez, no en loop continuo.
    """
    grid_reducido = reducir_grid_para_arduino(grid_fino)
    data = "".join(str(int(v)) for v in grid_reducido.flatten()).encode("ascii")

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(data, (ESP01_IP, ESP01_PORT))
        print(f"\nGrid enviado al Arduino ({len(data)} bytes)")
        return True
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        print(f"\n[ESP-01] No se pudo enviar: {e}")
        return False


# ------------------------------------------------------------------
# H. DETECCION DE OBJETOS (YOLO) -> CLASIFICAR OBSTACULOS EN 3D
# ------------------------------------------------------------------
def cargar_yolo():
    return YOLO(YOLO_MODEL_PATH)


def detectar_objetos(frame_bgr, point_cloud_np, modelo):
    """
    Corre YOLO sobre el frame, y para cada deteccion valida devuelve su
    clase, confianza, caja en pixeles, y posicion 3D real (consultando
    la nube de puntos en el centro de la caja, igual que con el ArUco).
    """
    resultados = modelo(frame_bgr, verbose=False)[0]
    objetos = []

    for box in resultados.boxes:
        conf = float(box.conf[0])
        if conf < YOLO_CONFIANZA_MIN:
            continue

        clase_id = int(box.cls[0])
        nombre_clase = modelo.names[clase_id]

        if YOLO_CLASES_INTERES is not None and nombre_clase not in YOLO_CLASES_INTERES:
            continue

        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

        pos_3d = punto_3d_en_pixel(point_cloud_np, cx, cy)
        if pos_3d is None:
            continue

        objetos.append({
            "clase": nombre_clase,
            "confianza": conf,
            "bbox_px": (int(x1), int(y1), int(x2), int(y2)),
            "x": pos_3d[0],
            "y": pos_3d[1],
            "z": pos_3d[2],
        })

    return objetos


def dibujar_objetos(frame_bgr, objetos):
    for obj in objetos:
        x1, y1, x2, y2 = obj["bbox_px"]
        cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 200, 255), 2)
        etiqueta = f"{obj['clase']} {obj['confianza']:.2f} ({obj['x']:.1f},{obj['y']:.1f}m)"
        cv2.putText(frame_bgr, etiqueta, (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)


# ------------------------------------------------------------------
# G. BUCLE PRINCIPAL
# ------------------------------------------------------------------
def main():
    zed = abrir_camara()
    detector = crear_detector_aruco()
    modelo_yolo = cargar_yolo()

    image = sl.Mat()
    point_cloud = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    floor_z = None
    print("Presiona 'F' apuntando a piso despejado para calibrar.")
    print("Presiona 'Q' para salir.")

    try:
        while True:
            if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

            frame_bgr = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)
            pc_np = point_cloud.get_data()

            objetos = detectar_objetos(frame_bgr, pc_np, modelo_yolo)
            dibujar_objetos(frame_bgr, objetos)

            robot_info = detectar_robot(frame_bgr, pc_np, detector)
            if robot_info:
                cv2.circle(frame_bgr, robot_info["pixel_centro"], 6, (0, 0, 255), -1)
                cv2.putText(
                    frame_bgr,
                    f"X={robot_info['x']:.2f}m Y={robot_info['y']:.2f}m Th={robot_info['theta_deg']:.0f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )

            grid_vis = None
            if floor_z is not None:
                grid = calcular_occupancy_grid(pc_np, floor_z)
                grid_vis = grid
                grid_grande = cv2.resize(
                    grid, (GRID_CELLS * 4, GRID_CELLS * 4),
                    interpolation=cv2.INTER_NEAREST
                )
                cv2.imshow("Occupancy Grid", grid_grande)

            cv2.putText(frame_bgr, "F=calibrar piso | M=enviar mapa | Q=salir", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.imshow("ZED - RGB + ArUco", frame_bgr)

            tecla = cv2.waitKey(1) & 0xFF
            if tecla == ord('f'):
                floor_z = calibrar_piso(pc_np)
                print(f"\nNivel de piso calibrado: Z = {floor_z:.3f} m")
            elif tecla == ord('m'):
                if grid_vis is not None:
                    enviar_a_arduino(grid_vis)
                else:
                    print("\nCalibra el piso primero (tecla F) antes de enviar el mapa.")
            elif tecla == ord('q'):
                break
    finally:
        zed.close()
        cv2.destroyAllWindows()
        print("\nCamara cerrada.")


if __name__ == "__main__":
    main()
