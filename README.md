# Mecatrónica [4250] - Robot Auto-Balancín

## Índice de Contenidos
- [Objetivos](#objetivos)
- [Avance 1](#avance-1)
- [Avance 2](#avance-2)
- [Avance 3](#avance-3)

## Objetivos

El objetivo principal de este proyecto es el diseño y construcción de un robot autobalancín funcional. El núcleo del desafío consiste en implementar un controlador **PID (Proporcional-Integral-Derivativo)** para mantener el equilibrio dinámico del sistema.

Para lograr este objetivo, se integrarán componentes mecatrónicos esenciales utilizando un microcontrolador **Arduino UNO** como unidad central de procesamiento. Este gestionará la lógica de control programada en C++ y la comunicación entre los sensores (encargados de medir la inclinación) y los actuadores (motores DC seleccionados para las ruedas). Además, el proyecto incluye el diseño de una **estética original y atractiva** como parte de los requerimientos.

Es importante definir que el alcance del proyecto se centra en la **integración exitosa de los componentes** y en la **demostración de un sistema de autobalanceo funcional**, priorizando esto sobre la optimización de la eficiencia o el rendimiento del diseño. Por lo tanto, aunque el prototipo pueda presentar ciertas ineficiencias, el objetivo principal radica en la aplicación práctica de los conceptos de control.

Todas las especificaciones y requerimientos de diseño se detallan en el documento [**Base**](base.pdf).

## Avance 1

La carpeta [**avance_1**](avance_1) contiene los avances iniciales correspondientes a la primera presentación. En esta etapa se establecieron las bases del robot, implementando las ruedas y motores, junto con soportes diseñados para resistir impactos durante las primeras fases de desarrollo del prototipo. La documentación de esta fase se presenta en el archivo [**Presentación**](avance_1/Presentación.pdf). Las piezas se fabricaron mediante impresión 3D utilizando material PLA.

### Contenido del avance

- **Prototipos iniciales**: Se desarrollaron dos prototipos, seleccionando el segundo para continuar con el desarrollo. Posteriormente se evaluarán mejoras adicionales según los requerimientos.

<center>
<img src="img/11.jpeg" alt="Prototipo Inicial 1" width="400">
<img src="img/1.jpeg" alt="Prototipo Inicial 2" width="400">
</center>

- **Código de movimiento**: Se implementó un programa para el control de movimiento de las ruedas, específicamente diseñado para los **motores DC** del prototipo seleccionado.

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

### Observaciones

- **Material de las ruedas**: Se recomienda implementar un material rugoso para mejorar la tracción
- **Refuerzos estructurales**: Es necesario añadir soportes en zonas críticas susceptibles a daños por impactos
- **Optimización del motor**: Se propone utilizar el motorreductor amarillo por su ligereza y favorable relación torque/velocidad angular

<img src="img/reductora.jpg" alt="Motor con Reductora" width="400">

## Avance 2

La carpeta [**avance_2**](avance_2) documenta los progresos correspondientes a la segunda entrega del curso, donde se implementaron cambios significativos tanto en el diseño como en la incorporación de componentes y código. 

En cuanto al diseño estético, se seleccionó al personaje **Guido** de la película Cars, considerándolo ideal y atractivo para el proyecto (*inicialmente se contempló R2D2*). Posteriormente se desarrollará una carcasa personalizada con este diseño. La presentación de estos avances se encuentra en [**Presentación 2**](avance_2/presentación2.pdf).

<img src="img/guido.jpg" alt="Guido" width="400">

### Contenido del avance:

<img src="img/21.jpeg" alt="Prototipo 2" width="400">

- **Cambio de Diseño**: Se realizó una revisión completa del diseño inicial, implementando mejoras sustanciales en la configuración mecánica y electrónica del sistema. Se incorporó un portapilas que utiliza cuatro baterías en serie, totalizando aproximadamente 12 voltios, suficiente para el correcto funcionamiento del sistema. Además, se integró un giroscopio que proporciona información sobre la inclinación del sistema, la cual se utilizará posteriormente para la autorregulación mediante un controlador PID.

- **Sistema de Propulsión**: Se incorporaron motores DC con caja reductora ("motores amarillos"), seleccionados por su óptima relación torque-velocidad angular y mayor eficiencia energética para aplicaciones de balanceo.

- **Configuración Mecánica**:
  - Implementación de montaje con separadores para evitar interferencias con componentes soldados
  - Adopción de diseño estructural abierto que minimiza el uso de material, mejora la ventilación y facilita el acceso al cableado

- **Electrónica del Sistema**:
  - Control de motores mediante puente H L298N
  - Sensor de movimiento: Módulo GY-521 (MPU-6050)
  - Unidad de control: Arduino Uno
  - Alimentación: Banco de baterías de 12V CC

<img src="img/h.jpg" alt="Puente H" width="400"><img src="img/gyro.jpg" alt="Gyroscopio" width="400">
<img src="img/arduino.jpg" alt="Arduino Uno" width="400"><img src="img/soporte.jpeg" alt="Soporte de Pilas" width="400">

- **Ventajas del Nuevo Diseño**:
  - Mayor eficiencia energética
  - Mejor disipación térmica
  - Accesibilidad para mantenimiento
  - Optimización de peso y materiales

- **Código de Movimiento**:

Se implementaron funciones para controlar el movimiento de los motores, permitiendo el giro de las ruedas en diferentes direcciones. Este código servirá como base para la implementación futura del control PID que permitirá el equilibrio del robot.

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

- **Código del Sensor**:

Se utilizó el código proporcionado en clases para la lectura del sensor MPU-6050, que proporciona los datos de orientación necesarios para el sistema de control.

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

### Observaciones

- **Interruptor de alimentación**: Es importante implementar un switch para evitar el consumo constante de las baterías, permitiendo encender y apagar el sistema según sea conveniente
- **Sistema de fijación**: Las fijaciones a presión son adecuadas para el prototipo actual, pero una implementación modular podría ser una mejora a considerar para el prototipo final, facilitando el desarme y mantenimiento

## Avance 3

### Contenido del avance

### Observaciones

