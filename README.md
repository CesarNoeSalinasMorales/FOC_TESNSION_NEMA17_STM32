# Accionamiento de un motor a pasos bipolar NEMA 17 mediante FOC en modo tensión

# Descripción del proyecto #

Este repositorio contiene los códigos fuente desarrollados para la implementación de un sistema 
de accionamiento y posicionamiento de un motor a pasos bipolar NEMA 17 mediante Control Orientado 
al Campo (FOC) en modo tensión.
El sistema utiliza un microcontrolador STM32F411 Black Pill, dos módulos de potencia BTS7960 y un 
sensor magnético AS5600 para la medición de la posición angular del rotor.

## Hardware utilizado
- Microcontrolador STM32F411 Black Pill.
- Motor a pasos bipolar NEMA 17 (17HS8501).
- Dos módulos BTS7960.
- Sensor magnético AS5600.

## Contenido del repositorio

 | Descripción |
| 01_Acc_Motor_2senoidales.ino | Accionamiento del motor con los 2 bts7960 con señales senoidales. |
| 02_ControlPosicionKP.ino | Un control proporcional simple  para la posicion. |
| 03_Calibracion.ino | Implementa una calibracion simple del sensor. |
| 04_Cuadraturas_Calibracion.ino | Implementa una calibracion automatica de 4 cuadrantes. |
| 05_PROP_CAL.ino | Implementa una calibracion con el control proporsional de posicion. |
| 06_FOC_FINAL_DETALLES.ino | Implementa el control foc por tension çon un control proporcional 
para posicion. |

# Entorno de desarrollo
- Arduino IDE.
# Uso
Cada programa debe abrirse y cargarse de manera independiente en el microcontrolador, 
siguiendo las conexiones eléctricas correspondientes al sistema.
# Autoría

Código fuente desarrollado como parte de un proyecto de tesis de Ingeniería en Control 
y Automatización.
