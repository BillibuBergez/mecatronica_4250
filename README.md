

# Mecatrónica [4250] - Robot Auto-Balancín

## Indice de Contenidos
- [Objetivos](##Objetivos)
- [Avance 1](##Avance-1)
- [Avance 2](##Avance-2)

## Objetivos

El objetivo principal de este proyecto es el diseño y la construcción de un robot autobalancín funcional. El núcleo del desafío consiste en la implementación de un controlador **PID (Proporcional-Integral-Derivativo)** para mantener el equilibrio dinámico del sistema.

Para lograr esto, se integrarán componentes mecatrónicos esenciales, utilizando un microcontrolador **Arduino UNO** como unidad central de procesamiento. Este gestionará la lógica de control programada en C++ y la comunicación entre los sensores (encargados de medir la inclinación) y los actuadores (motores DC seleccionados para las ruedas). Además, el proyecto incluye el diseño de una **estética original y atractiva**, como parte de los requerimientos.

Es importante definir el alcance: el foco está puesto en la **integración exitosa de los componentes** y en la **demostración de un sistema de autobalanceo funcional**, más que en la optimización de la eficiencia o el rendimiento del diseño. Por lo tanto, aunque el prototipo pueda presentar ciertas ineficiencias, el objetivo se centra en la aplicación práctica de los conceptos de control.

Todas las especificaciones y requerimientos de diseño se encuentran detallados en el documento [**Base**](base.pdf).

## Avance 1

La carpeta [**avance_1**](avance_1) corresponde a los avances iniciales relacionados con la primera presentación. Donde en esta se inicia con el proyecto, donde se busca obtener la base del robot con las ruedas y motores implementados, además de unos soportes para resistir los impactos al momento de cae especialmente en las primeras etapas del prototipo. Se realiza una presentación al respecto en el archivo [**Presentación**](avance_1/Presentación.pdf). Se imprime utilizando una impresora 3D, utilizando un material de PLA.

### Contenido del avance:
- **Prototipos iniciales**: Se desarrollaron dos prototipos, siendo el segundo el seleccionado para continuar el desarrollo. Próximamente se evaluarán mejoras adicionales según los requerimientos.

<center><img src="img/11.jpeg" alt="Prototipo Inicial 1" width="400">
<img src="img/1.jpeg" alt="Prototipo Inicial 2" width="400">

- **Código de movimiento**: Se incluye el programa para el control de movimiento de las ruedas, específicamente diseñado para los **motores DC** del prototipo seleccionado.

```cpp
#include <Stepper.h>
int stepsPerRevolution = 2048;
int motSpeed = 5;
// Pin al que está conectado el potenciómetro
const int PotPin = A0;

// Variable para almacenar el valor leído del potenciómetro
int PotVal;

Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);
Stepper myStepper2(stepsPerRevolution, 4, 6, 5, 7);

void setup() {
  myStepper.setSpeed(motSpeed);
  myStepper2.setSpeed(motSpeed);
}

void loop() {
  for (int i = 0; i < stepsPerRevolution; i++) {
    PotVal = analogRead(PotPin);
    if (PotVal <= 60){
      myStepper.step(-1);      // un paso motor 1
      myStepper2.step(1);     // un paso motor 2
    }
      else { 
        myStepper.step(1);      // un paso motor 1
        myStepper2.step(-1);     // un paso motor 2
    }
  }
}
```

### Observaciones y mejoras identificadas:
- **Material de las ruedas**: Implementar un material rugoso para mejorar la tracción
- **Refuerzos estructurales**: Añadir soportes en zonas críticas susceptibles a daños por impactos
- **Optimización del motor**: Utilizar el motorreductor amarillo por su ligereza y favorable relación torque/velocidad angular.

<img src="img/reductora.jpg" alt="Motor con Recuctora" width="400">


## Avance 2
La carpeta [**avance_2**](avance_2) corresponde a los avances para la segunda entrega del curso, donde se realizan varios cambios y avances, tanto en el diseño, como la incorporación de componentes y el código. Dentro del diseño estetico se elige a un personaje curioso de la pelicula de Cars, nuestro querido personaje ¡Guido!, que nos pareción ideal y atractivo (*no es que inicialmente queríamos R2D2*), sin embargo posteriormente se incorporará para la carcasa con su respectivo diseño. Además, se realiza una presentación para mostrar los avances de este apartado, esto se encuentra en [**Presentación 2**](avance_2/presentación2.pdf).

<img src="img/guido.png" alt="Guido" width="400">

### Contenido del avance:

<img src="img/21.jpeg" alt="Prototipo 2" width="400">

- **Cambio de Diseño:** Se realizó una revisión completa del diseño inicial, implementando mejoras sustanciales en la configuración mecánica y electrónica del sistema. Donde se incorpora un portapilas, particularmente cada una de ellas de unos 4 volteos en serie, siendo un total de 12 volteos aproximadamente, más que suficiente para un correcto funcionamiento del sistema. Además se incorpora un sistema de sensibilidad dado por un giroscopio que permite generar información sobre la inclinación del sistema que posteriormente se utilizará ára que úeda autorregularse a través de un PID.

- **Sistema de Propulsión:** Se incorporaron motores DC con caja reductora ("motores amarillos"), seleccionados por su óptima relación torque-velocidad angular y mayor eficiencia energética para aplicaciones de balanceo.

- **Configuración Mecánica:**
  - Se implementó montaje con separadores para evitar interferencias con componentes soldados
  - Se adoptó diseño estructural abierto que minimiza uso de material, mejora ventilación y facilita acceso al cableado

- **Electrónica del Sistema:**
  - Control de motores mediante puente H L298N
  - Sensor de movimiento: Módulo GY-521 (MPU-6050)
  - Unidad de control: Arduino Uno
  - Alimentación: Banco de baterías de 12V CC

<img src="img/h.jpg" alt="Puente H" width="400">  <img src="img/gyro.jpg" alt="Gyroscopio" width="400">  
<img src="img/arduino.jpg" alt="Arduino Uno" width="400">  <img src="img/soporte.jpeg" alt="Soporte de Pilas" width="400">  

- **Ventajas del Nuevo Diseño:**
  - Mayor eficiencia energética
  - Mejor disipación térmica
  - Accesibilidad para mantenimiento
  - Optimización de peso y materiales

- **Código de Movimiento**

Aquí se incorporan códifos y funciones relacionadas al movimeinto de los motores para permitir el movimiento de las ruedas en función de lo que uno vaya necesitando o para más adelante reaccionar en función de lo que arroge el sensor y permita un PID funcional que permita el equilibrio.

```cpp
const int PinIN1 = 7;
const int PinIN2 = A0;
const int PinIN3 = 6;
const int PinIN4 = A1;
 
void setup() {
  // inicializar la comunicación serial a 9600 bits por segundo:
  Serial.begin(9600);
  // configuramos los pines como salida
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  pinMode(PinIN3, OUTPUT);
  pinMode(PinIN4, OUTPUT);
}
// Este loop es sólo para demostrar el funcionamiento de las funciones definidas
void loop() {
  
  MotorHorario();
  Serial.println("Giro del Motor en sentido horario");
  delay(1000);
  
  MotorAntihorario();
  Serial.println("Giro del Motor en sentido antihorario");
  delay(1000);
  
  MotorStop();
  Serial.println("Motor Detenido");
  delay(3000);
  
}
 
//función para girar el motor en sentido horario
void MotorHorario()
{
  digitalWrite (PinIN1, HIGH);
  digitalWrite (PinIN2, LOW);
  digitalWrite (PinIN3, HIGH);
  digitalWrite (PinIN4, LOW);
}

//función para girar el motor en sentido antihorario
void MotorAntihorario()
{
  digitalWrite (PinIN1, LOW);
  digitalWrite (PinIN2, HIGH);
  digitalWrite (PinIN3, LOW);
  digitalWrite (PinIN4, HIGH);
}
 
//función para apagar el motor
void MotorStop()
{
  digitalWrite (PinIN1, LOW);
  digitalWrite (PinIN2, LOW);
  digitalWrite (PinIN3, LOW);
  digitalWrite (PinIN4, LOW);
}
```
- **Código de Sensor**

Este código no es modificado y es el entregado en clases, que corresponde a la acción del sensor y la respuesta que arroja.

```cpp
#include <Wire.h>

const int MPU_ADDR = 0x68;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

float roll = 0, pitch = 0, yaw = 0;
unsigned long lastTime;
float dt;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power management
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("=== MPU-6050 Calibration ===");
  Serial.println("Place the sensor still and press any key + ENTER to start...");
  while (!Serial.available());  // Wait for user input
  while (Serial.available()) Serial.read();  // Clear input buffer

  calibrateGyro();
  Serial.println("Calibration complete!");

  lastTime = millis();
}

void loop() {
  readMPU6050();

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float gyroZ = (gz - gz_offset) / 131.0;
  yaw += gyroZ * dt;

  // Convert int16_t to float
  float ax_f = (float)ax;
  float ay_f = (float)ay;
  float az_f = (float)az;

  // Avoid division by zero
  float denominator = sqrt(ay_f * ay_f + az_f * az_f);
  if (denominator < 0.0001) denominator = 0.0001;

  // Compute angles in degrees
  roll = atan2(ay_f, az_f) * 180.0 / PI;
  pitch = atan2(-ax_f, denominator) * 180.0 / PI;

  // Print angles
  Serial.print("Roll: "); Serial.print(roll, 2);
  Serial.print(" | Pitch: "); Serial.print(pitch, 2);
  Serial.print(" | Yaw: "); Serial.println(yaw, 2);

  delay(100);
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void calibrateGyro() {
  long sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    readMPU6050();
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(5);
  }

  gx_offset = sumX / samples;
  gy_offset = sumY / samples;
  gz_offset = sumZ / samples;
}

```
 
### Posibles Mejoras
Mencionar que es importante la implementación de un switch para que las baterías no estén en constante consumo. Y permitir prender o apagar como sea convenienteLas fijaciones a presión parecen ser adecuadas, pero sólo óptimas para un prototipo final, pues no permite desarmar. Una implementación modular podría ser una mejora a considerar.
 
## Avance 3 [11 de Noviembre]
