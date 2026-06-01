# UAV_Telemetry
## Sistema embebido STM32 + FreeRTOS para adquisición y retransmisión de datos UAV

Repositorio del código firmware desarrollado para el Trabajo de Fin de Grado. El proyecto implementa un sistema embebido basado en **STM32F407** y **FreeRTOS** capaz de adquirir datos de distintos sensores, estructurarlos en formato JSON y retransmitirlos mediante un módulo WiFi ESP32 hacia una estación en tierra usando MQTT.

## Descripción general

El objetivo del proyecto es desarrollar un prototipo funcional de sistema embarcado para aplicaciones UAV, encargado de recoger información procedente de sensores inerciales, ambientales y de posicionamiento, procesarla en tiempo real y enviarla de forma inalámbrica a una estación receptora.

El sistema se apoya en una arquitectura multitarea mediante **FreeRTOS**, separando las responsabilidades principales del firmware en tareas independientes. Esto permite mejorar la organización del código, asignar prioridades a los procesos críticos y facilitar la ampliación futura del sistema.

## Características principales

* Firmware desarrollado para **STM32F407G-DISC1**.
* Uso de **FreeRTOS** para planificación multitarea.
* Adquisición de datos de una unidad sensorial **Adafruit 10DoF**.
* Recepción de datos GPS desde módulo **GY-GPS6MV2**.
* Comunicación con módulo **ESP32 WiFi** mediante UART.
* Retransmisión de datos mediante protocolo **MQTT**.
* Formateo de mensajes en estructura **JSON**.
* Uso de interrupciones, DMA y buffers para mejorar la recepción de datos serie.
* Separación modular del código en sensores, comunicación y utilidades comunes.
* Preparado para integración con una estación en tierra desarrollada en Unreal Engine.

## Arquitectura del sistema

El sistema está formado por los siguientes bloques principales:

```text
+-------------------+       +-------------------+
|   Sensor 10DoF    |       |    Módulo GPS     |
| IMU + barómetro   |       |   GY-GPS6MV2      |
+---------+---------+       +---------+---------+
          |                           |
          | I2C                       | UART
          |                           |
+---------v---------------------------v---------+
|              STM32F407 + FreeRTOS             |
|                                               |
|  - Lectura de sensores                        |
|  - Procesamiento de datos                     |
|  - Formateo JSON                              |
|  - Gestión de tareas                          |
|  - Comunicación UART                          |
+--------------------+--------------------------+
                     |
                     | UART / DMA
                     |
+--------------------v--------------------------+
|                  ESP32 WiFi                   |
|          Retransmisión de datos MQTT          |
+--------------------+--------------------------+
                     |
                     | WiFi / MQTT
                     |
+--------------------v--------------------------+
|              Estación en tierra               |
|        Visualización y registro de datos      |
+-----------------------------------------------+
```

## Hardware utilizado

| Componente          | Función                                                                           |
| ------------------- | --------------------------------------------------------------------------------- |
| STM32F407G-DISC1    | Microcontrolador principal del sistema                                            |
| Adafruit 10DoF IMU  | Lectura de acelerómetro, giroscopio, magnetómetro, temperatura, presión y altitud |
| GY-GPS6MV2          | Obtención de coordenadas GPS, altitud y datos de posicionamiento                  |
| ESP32 WiFi          | Comunicación inalámbrica y envío de mensajes MQTT                                 |
| Batería Li-Ion/LiPo | Alimentación del prototipo                                                        |
| TP4056              | Módulo de carga de batería                                                        |
| Módulo boost        | Elevación/regulación de tensión                                                   |
| Filtro LC           | Filtrado de ruido en la alimentación                                              |

## Software y herramientas

* **STM32CubeIDE**
* **STM32CubeMX**
* **FreeRTOS / CMSIS-RTOS**
* **HAL Drivers STM32**
* **Lenguaje C**
* **MQTT**
* **JSON**
* **Doxygen** para documentación del código

## Estructura del proyecto

La estructura general del firmware se organiza de forma modular:

```text
Core/
├── Inc/
│   ├── main.h
│   ├── freertos.h
│   ├── sensor/
│   │   ├── 10DoFSensor.h
│   │   └── gps.h
│   ├── communication/
│   │   ├── wifi.h
│   │   ├── mqtt.h
│   │   └── uart_dma.h
│   └── common/
│       ├── formatted_data.h
│       └── circular_buffer.h
│
├── Src/
│   ├── main.c
│   ├── freertos.c
│   ├── sensor/
│   │   ├── 10DoFSensor.c
│   │   └── gps.c
│   ├── communication/
│   │   ├── wifi.c
│   │   ├── mqtt.c
│   │   └── uart_dma.c
│   └── common/
│       ├── formatted_data.c
│       └── circular_buffer.c
```

> La estructura exacta puede variar ligeramente según la versión del proyecto exportada desde STM32CubeIDE.

## Funcionamiento del firmware

El firmware se basa en varias tareas FreeRTOS encargadas de ejecutar procesos de forma periódica e independiente.

### Tareas principales

| Tarea                     | Función principal                                                                       |
| ------------------------- | --------------------------------------------------------------------------------------- |
| Lectura de sensores 10DoF | Obtiene datos de acelerómetro, giroscopio, magnetómetro, temperatura, presión y altitud |
| Lectura GPS               | Procesa la información recibida desde el módulo GPS                                     |
| Formateo de datos         | Agrupa los datos en una estructura común y genera mensajes JSON                         |
| Comunicación WiFi/MQTT    | Envía los datos al ESP32 para su retransmisión inalámbrica                              |
| Gestión UART              | Controla la recepción y transmisión de datos mediante interrupciones y DMA              |

## Formato de datos

Los datos se estructuran en mensajes JSON para facilitar su recepción, interpretación y visualización en la estación en tierra.

Ejemplo simplificado de mensaje enviado:

```json
{
  "accel": {
    "x": 0.01,
    "y": -0.02,
    "z": 9.81
  },
  "gyro": {
    "x": 0.15,
    "y": 0.03,
    "z": -0.08
  },
  "mag": {
    "x": 32.5,
    "y": -14.2,
    "z": 41.8
  },
  "barometer": {
    "temperature": 24.5,
    "pressure": 1013.2,
    "altitude": 82.4
  },
  "gps": {
    "latitude": 40.4168,
    "longitude": -3.7038,
    "altitude": 650.0
  }
}
```

## Comunicación MQTT

El sistema utiliza el módulo ESP32 como interfaz WiFi. El STM32 envía los datos procesados al ESP32 mediante UART, y este se encarga de publicarlos en un broker MQTT.

La comunicación está pensada para integrarse con una estación en tierra capaz de:

* Suscribirse al topic MQTT correspondiente.
* Recibir los mensajes JSON.
* Interpretar los datos recibidos.
* Mostrar valores en tiempo real.
* Representar gráficas de los últimos datos recibidos.

## Configuración del proyecto

Para abrir y compilar el proyecto:

1. Clonar el repositorio:

```bash
git clone https://github.com/usuario/nombre-del-repositorio.git
```

2. Abrir **STM32CubeIDE**.

3. Seleccionar:

```text
File > Import > Existing Projects into Workspace
```

4. Elegir la carpeta del repositorio.

5. Compilar el proyecto desde STM32CubeIDE.

6. Programar la placa STM32F407G-DISC1 mediante ST-LINK.

## Requisitos previos

* STM32CubeIDE instalado.
* Drivers ST-LINK instalados.
* Placa STM32F407G-DISC1.
* Módulos sensores conectados según el esquemático del proyecto.
* Módulo ESP32 configurado para conectarse a la red WiFi y al broker MQTT.
* Broker MQTT activo.
* Cliente o estación receptora suscrita al topic correspondiente.

## Validación del prototipo

Durante el desarrollo se han realizado pruebas de validación sobre los aspectos principales del sistema:

* Correcto formato de los mensajes JSON.
* Estabilidad de la conexión WiFi y MQTT.
* Recepción de mensajes en la estación en tierra.
* Frecuencia de envío de datos próxima a la configurada.
* Funcionamiento continuo durante varias horas.
* Ausencia de pérdidas significativas en la retransmisión MQTT.
* Comprobación de las tensiones de alimentación.
* Verificación del funcionamiento de las tareas FreeRTOS.

Algunas pruebas permitieron detectar limitaciones o puntos de mejora, como la necesidad de mejorar el procesamiento de los datos del magnetómetro o sustituir retardos simples por mecanismos más deterministas como `vTaskDelayUntil()`.

## Mejoras futuras

Entre las posibles mejoras del sistema se incluyen:

* Diseño de una PCB dedicada para sustituir el montaje en protoboard.
* Reducción del tamaño total del sistema.
* Integración directa del microcontrolador STM32F407 en placa propia.
* Sustitución de sensores por modelos de mayores prestaciones.
* Mejora del procesamiento de datos del magnetómetro.
* Uso de `vTaskDelayUntil()` para aumentar el determinismo temporal de las tareas.
* Optimización de la comunicación UART mediante DMA y buffers circulares.
* Integración de una batería plana adaptada al diseño final.
* Ampliación del software de estación en tierra.
* Automatización del arranque del broker MQTT desde la estación receptora.
* Preparación de datos para futuros modelos de aprendizaje automático.

## Documentación

El código fuente está documentado mediante comentarios compatibles con **Doxygen**. Esta documentación permite generar automáticamente información sobre:

* Archivos fuente.
* Funciones.
* Estructuras de datos.
* Parámetros.
* Variables globales.
* Dependencias entre módulos.

Para generar la documentación, se puede utilizar Doxygen desde la raíz del proyecto si se dispone de un archivo de configuración `Doxyfile`.

## Estado del proyecto

El proyecto se encuentra en estado de prototipo funcional. El sistema permite adquirir datos de sensores, procesarlos y retransmitirlos mediante WiFi/MQTT hacia una estación en tierra.

Aunque el prototipo cumple con los objetivos principales, todavía existen aspectos pendientes de optimización antes de poder considerarse una versión final integrable en un UAV real.

## Autor

**Lorenzo Polo Arévalo**

Trabajo de Fin de Grado
Sistema embebido basado en STM32 y FreeRTOS para adquisición y transmisión de datos en aplicaciones UAV.

## Licencia

### **CC BY-NC 4.0**

# **Attribution-NonCommercial 4.0 International**

## **You are free to:**

1. **Share** — copy and redistribute the material in any medium or format
2. **Adapt** — remix, transform, and build upon the material
3. The licensor cannot revoke these freedoms as long as you follow the license terms.

## **Under the following terms:**

1. **Attribution** — You must give [appropriate credit](https://creativecommons.org/licenses/by-nc/4.0/deed.en#ref-appropriate-credit), provide a link to the license, and [indicate if changes were made](https://creativecommons.org/licenses/by-nc/4.0/deed.en#ref-indicate-changes). You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
2. **NonCommercial** — You may not use the material for [commercial purposes](https://creativecommons.org/licenses/by-nc/4.0/deed.en#ref-commercial-purposes).
3. **No additional restrictions** — You may not apply legal terms or [technological measures](https://creativecommons.org/licenses/by-nc/4.0/deed.en#ref-technological-measures) that legally restrict others from doing anything the license permits.

## **Notices:**

You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable [exception or limitation](https://creativecommons.org/licenses/by-nc/4.0/deed.en#ref-exception-or-limitation).

No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as [publicity, privacy, or moral rights](https://creativecommons.org/licenses/by-nc/4.0/deed.en#ref-publicity-privacy-or-moral-rights) may limit how you use the material.
