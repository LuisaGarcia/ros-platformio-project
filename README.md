# Brazo Robótico Serial con ROS 2 y PlatformIO

Este proyecto implementa el control de un brazo robótico de 4 grados de libertad usando ROS 2 y un microcontrolador programado con PlatformIO mediante comunicación serial.

El sistema permite mover cada articulación enviando comandos desde ROS 2 hacia el microcontrolador.

---

# Descripción del Proyecto

El sistema está dividido en dos partes principales:

1. El firmware desarrollado en **PlatformIO**, que controla los servomotores del brazo.
2. El nodo de **ROS 2**, que maneja la comunicación serial y valida los comandos enviados.

La comunicación se realiza mediante mensajes con el siguiente formato:

LETRA:NUMERO

Ejemplo:

P:90
C:45
H:120
R:30

Donde:

- La letra identifica el servo.
- El número representa el ángulo (0° – 180°).

El nodo ROS 2 valida el formato antes de enviar el comando al microcontrolador.

---

## Componentes del Hardware

- 4 Servomotores (SG90)
- Microcontrolador (ej. Arduino Uno)
- Estructura mecánica del brazo robótico
- Cable USB para comunicación con ROS 2

---

## Arquitectura del Sistema

ROS 2 Publisher
→ Nodo Transmisor Serial (ROS 2)
→ Microcontrolador (PlatformIO)
→ Servomotores

---

## Firmware (PlatformIO) - Main.cpp

El firmware:

- Inicializa comunicación serial a 9600 baudios.
- Espera comandos en formato `LETRA:NUMERO`.
- Valida:
  - Que la letra sea válida (P, C, H, R).
  - Que el ángulo esté entre 0 y 180.
- Mueve el servo correspondiente.
- Responde con mensajes de estado por serial.

### Conexión de Servos

| Servo | Pin |
|-------|------|
| P     | 3    |
| C     | 5    |
| H     | 6    |
| R     | 9    |


---
## Nodo ROS 2 - transmitter.py
Este nodo actúa como puente entre ROS 2 y el microcontrolador.

Funcionalidad

Se suscribe al tópico:
/receive_msg

Tipo de mensaje:
std_msgs/msg/String

- Valida el formato LETRA:NUMERO.
- Envía el comando al puerto serial.
- Evita enviar comandos mal formateados.

---
## Configuración e instalación

### Configuración del Arduino

Compilar y cargar el código de la carpeta ~/firmware/BrazoLuisa en el Arduino UNO


### Configuración ROS2

Navegar al directorio del workspace y compilar el proyecto

```bash
cd ~/ros_workspace
colcon build
source install/setup.bash
```

### Ejecución del sistema

Para lanzar el sistema completo se debe configurar el puerto USB que este usando el arduino dentro del archivo transmitter.py
`'/dev/ttyACM0'`
Se deben dar permisos al puerto ```sudo chmod 777 /dev/ttyACM0```

Una vez este todo conectado, el arduino programado y el USB configurado, se debe ejecutar el nodo transmisor

```bash
ros2 run brazorobotros transmitter
```

Luego crear una terminal nueva, ir al directorio del workspace y ejecutar
```bash
source install/setup.bash
```
Ejecutar el publicador para enviar el comando al brazo
```bash
ros2 topic pub /receive_msg std_msgs/msg/String "{data: 'C:100'}" --once
```

## Autor
Luisa Fernanda García Vargas - luisa.garcia@javeriana.edu.co

