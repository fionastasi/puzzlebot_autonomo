# Módulo de Control

Esta rama contiene la lógica de movimiento autónomo del robot.

## Estructura

- : Nodo que sigue la línea negra central
- : Nodo que calcula y estima la posición del robot

## Comunicación ROS2

### Publicaciones (`publishers`)
- : Velocidad de las ruedas
- : Estado actual (seguimiento, parado, lento, etc.)

### Subscripciones (`subscribers`)
-
-
-

## Control

- El seguidor de línea usa procesamiento de imagen y controlador proporcional
- Odometría estimada mediante encoder simulado
