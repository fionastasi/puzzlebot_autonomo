# Módulo de Visión Computacional

Esta rama contiene los módulos relacionados con la detección de semáforos y señales viales.

## Estructura

- : Nodo para detección de semáforos (rojo, amarillo, verde)
- : Nodo de reconocimiento de señales estáticas
- : Imágenes utilizadas para entrenar/clasificar señales

## 🛰Comunicación ROS2

### Publicaciones (`publishers`)
- : Color del semáforo detectado
- : Tipo de señal identificada

### Subscripciones (`subscribers`)
- `: Imagen de la cámara del robot

## Notas

- Los clasificadores fueron entrenados con imágenes reales tomadas de la pista
- El sistema opera en tiempo real a _ FPS
