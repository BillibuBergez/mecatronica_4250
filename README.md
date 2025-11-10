# Mecatrónica [4250] - Robot Auto-Balancín

## Objetivos

Los objetivos corresponden a la construcción de un robot autobalancín con componentes básicos y la implementación de PID. Todas las especificaciones se encuentran en el archivo [**base**](base/base.pdf)

## Presentación 1 [22 de Septiembre]

La carpeta [**avance_1**](avance_1) corresponde a los avances iniciales relacionados con la primera presentación.

### Contenido del avance:
- **Prototipos iniciales**: Se desarrollaron dos prototipos, siendo el segundo el seleccionado para continuar el desarrollo. Próximamente se evaluarán mejoras adicionales según los requerimientos.
- **Código de movimiento**: Se incluye el programa para el control de movimiento de las ruedas, específicamente diseñado para los **motores DC** del prototipo final.

### Observaciones y mejoras identificadas:
- **Material de las ruedas**: Implementar un material rugoso para mejorar la tracción
- **Refuerzos estructurales**: Añadir soportes en zonas críticas susceptibles a daños por impactos
- **Optimización del motor**: Utilizar el motorreductor amarillo por su ligereza y favorable relación torque/velocidad angular

## Presentación 2 [20 de Octubre]

La carpeta [**avance_2**](avance_2) corresponde a los avances para la segunda entrega del curso, donde se realizan varios cambios y avances, tanto en el diseño, como la incorporación de componentes y el código.

### Contenido del avance:

- **Cambio de Diseño:** Se realizó una revisión completa del diseño inicial, implementando mejoras sustanciales en la configuración mecánica y electrónica del sistema.

- **Sistema de Propulsión:** Se incorporaron motores DC con caja reductora ("motores amarillos"), seleccionados por su óptima relación torque-velocidad angular y mayor eficiencia energética para aplicaciones de balanceo.

- **Configuración Mecánica:**
  - Se implementó montaje con separadores para evitar interferencias con componentes soldados
  - Se adoptó diseño estructural abierto que minimiza uso de material, mejora ventilación y facilita acceso al cableado

- **Electrónica del Sistema:**
  - Control de motores mediante puente H L298N
  - Sensor de movimiento: Módulo GY-521 (MPU-6050)
  - Unidad de control: Arduino Uno
  - Alimentación: Banco de baterías de 12V CC

- **Ventajas del Nuevo Diseño:**
  - Mayor eficiencia energética
  - Mejor disipación térmica
  - Accesibilidad para mantenimiento
  - Optimización de peso y materiales
