#!/usr/bin/env python3
"""
==============================================================
PROGRAMA: Nodo ROS 2 transmisor Serial hacia Arduino
==============================================================

Este nodo:

Se suscribe a un tópico llamado 'receive_msg'
Recibe mensajes tipo std_msgs/String
Valida que el formato sea LETRA:NUMERO
Envía el mensaje validado al Arduino por puerto serial

Ejemplo de mensaje válido:
    P:90
    C:45

Flujo del sistema:

Publicador ROS 2  --->  Este nodo (transmitter)  --->  Arduino
"""

# ================= IMPORTACIONES =================

import rclpy                      # Librería principal de ROS 2 en Python
import serial                     # Librería para comunicación serial (pyserial)

from std_msgs.msg import String   # Tipo de mensaje que usará el tópico


# Variables globales (compartidas entre funciones)
global arduino, node


# ==============================================================
# FUNCIÓN CALLBACK
# Se ejecuta automáticamente cada vez que llega un mensaje
# al tópico 'receive_msg'
# ==============================================================

def send_serial_callback(msg: String):

    global arduino, node

    # Extrae el texto contenido en el mensaje ROS
    data = msg.data.strip()  # strip() elimina espacios o saltos de línea

    print(f'Recibido del publicador: {data}')

    # ----------------------------------------------------------
    # VALIDACIÓN 1: Verificar que exista el carácter ":"
    # ----------------------------------------------------------
    # El formato esperado es LETRA:NUMERO
    # Ejemplo: P:90

    if ":" not in data:
        node.get_logger().error(
            "Formato inválido. Use LETRA:NUMERO, por ejemplo A:15"
        )
        return  # Sale de la función sin enviar nada al Arduino

    # ----------------------------------------------------------
    # Separar letra y número
    # ----------------------------------------------------------
    letra, numero = data.split(":", 1)

    # Limpieza de espacios extra
    letra = letra.strip()
    numero = numero.strip()

    # ----------------------------------------------------------
    # VALIDACIÓN 2: La primera parte debe ser una letra
    # ----------------------------------------------------------
    if not letra.isalpha():
        node.get_logger().error(
            "La primera parte debe ser una letra"
        )
        return

    # ----------------------------------------------------------
    # VALIDACIÓN 3: La segunda parte debe ser un número
    # ----------------------------------------------------------
    if not numero.isdigit():
        node.get_logger().error(
            "La segunda parte debe ser un número"
        )
        return

    # ----------------------------------------------------------
    # Preparar mensaje para Arduino
    # ----------------------------------------------------------
    # Se agrega '\n' porque el Arduino usa readStringUntil('\n')
    serial_msg = f"{letra}:{numero}\n"

    print(f'Enviando al Arduino: {serial_msg}')

    # Enviar datos al puerto serial
    # encode('utf-8') convierte texto a bytes
    arduino.write(serial_msg.encode('utf-8'))


# ==============================================================
# FUNCIÓN PARA INICIALIZAR EL PUERTO SERIAL
# ==============================================================

def init_serial():

    global arduino, node

    """
    serial.Serial(
        puerto,
        velocidad,
        timeout
    )
    """

    arduino = serial.Serial(
        '/dev/ttyACM0',  # Puerto donde está conectado el Arduino
        9600,            # Velocidad (debe coincidir con Serial.begin en Arduino)
        timeout=1.0
    )


# ==============================================================
# FUNCIÓN PRINCIPAL
# ==============================================================

def main(args=None):

    global arduino, node

    # Inicializa ROS 2
    rclpy.init(args=args)

    # ----------------------------------------------------------
    # Crear el nodo
    # ----------------------------------------------------------
    node = rclpy.create_node('ejemplo_transmisorSerial')

    # ----------------------------------------------------------
    # Inicializar comunicación serial
    # ----------------------------------------------------------
    init_serial()

    # ----------------------------------------------------------
    # Crear suscriptor
    # ----------------------------------------------------------
    # Parámetros:
    # (tipo_mensaje, nombre_topico, callback, tamaño_cola)
    sub = node.create_subscription(
        String,                 # Tipo de mensaje
        'receive_msg',          # Nombre del tópico
        send_serial_callback,   # Función que se ejecuta al recibir mensaje
        10                      # Tamaño de cola
    )

    # ----------------------------------------------------------
    # Mantener el nodo activo
    # ----------------------------------------------------------
    # spin() evita que el nodo termine
    rclpy.spin(node)

    # ----------------------------------------------------------
    # Cierre
    # ----------------------------------------------------------
    node.destroy_node()
    rclpy.shutdown()


# ==============================================================
# Punto de entrada
# ==============================================================

if __name__ == '__main__':
    main()