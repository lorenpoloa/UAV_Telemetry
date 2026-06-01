/**
 * @file freertos.c
 * @brief Planificación FreeRTOS de adquisición, empaquetado y envío MQTT de telemetría.
 *
 * @details
 * Este módulo organiza el firmware como una arquitectura desacoplada tipo
 * "data pipeline": cada tarea tiene una responsabilidad única y se comunica con
 * el resto mediante buffers globales protegidos por mutex y flags de estado.
 *
 * La separación entre adquisición, construcción de paquetes y publicación MQTT
 * evita que una operación de red bloquee directamente la lectura de sensores.
 * Esta decisión mejora la robustez temporal del sistema en un entorno embebido
 * donde las comunicaciones WiFi pueden introducir retardos variables.
 *
 * @author Lorenzo
 * @date 14 May 2026
 */

#include "cmsis_os.h"
#include "sensors/sensor10DoF.h"
#include "sensors/gps.h"
#include "communication/wifi.h"
#include "main.h"
#include "common/formatter_data.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/**
 * @brief UART asociada al módulo WiFi.
 *
 * @details
 * Se declara como externa porque la inicialización física del periférico UART
 * se realiza en el código generado por STM32CubeMX. Este módulo solo necesita
 * la referencia para inicializar y operar el driver WiFi.
 */
extern UART_HandleTypeDef huart2;

/**
 * @brief UART asociada al módulo GPS.
 *
 * @details
 * Se reutiliza el manejador generado por STM32CubeMX para mantener separada la
 * configuración de bajo nivel del periférico y la lógica de adquisición GPS.
 */
extern UART_HandleTypeDef huart3;

/**
 * @def WIFI_TEST_SSID
 * @brief SSID de la red WiFi usada por el sistema durante la ejecución.
 *
 * @details
 * Se mantiene como macro para simplificar la configuración de conexión en un
 * entorno embebido sin almacenamiento persistente de credenciales.
 */
#define WIFI_TEST_SSID      ""

/**
 * @def WIFI_TEST_PASS
 * @brief Contraseña de la red WiFi usada por el sistema durante la ejecución.
 */
#define WIFI_TEST_PASS      ""

/**
 * @def SERVER_HOST
 * @brief Dirección IP del broker MQTT local.
 *
 * @details
 * El uso de dirección IP directa evita depender de resolución DNS en el módulo
 * WiFi y reduce una posible fuente de fallo durante las pruebas locales.
 */
#define SERVER_HOST         ""

/**
 * @def PAYLOAD_LENGHT
 * @brief Tamaño reservado para cada payload JSON de telemetría.
 *
 * @details
 * El valor se dimensiona para mensajes MQTT compactos, reduciendo consumo de
 * RAM sin recurrir a reserva dinámica de memoria.
 */
#define PAYLOAD_LENGHT 128

/**
 * @brief Última lectura válida del sensor IMU/10DoF.
 *
 * @details
 * Actúa como buffer compartido entre la tarea de adquisición IMU y la tarea de
 * empaquetado. Su acceso se protege mediante @ref imuMutex.
 */
static Sensor10DoF_t imu_data;

/**
 * @brief Paquete de telemetría GPS preparado para serialización.
 */
static GpsDataPacket_t gpsPacket;

/**
 * @brief Paquete de telemetría inercial preparado para serialización.
 */
static ImuDataPacket_t imuPacket;

/**
 * @brief Paquete de telemetría ambiental preparado para serialización.
 */
static EnvDataPacket_t envPacket;

/**
 * @brief Buffer JSON del paquete GPS pendiente de envío.
 *
 * @details
 * Se usa un buffer estático para evitar fragmentación de memoria y mantener un
 * comportamiento determinista en FreeRTOS.
 */
static char payload_gpsPacket[PAYLOAD_LENGHT]={0};

/**
 * @brief Buffer JSON del paquete IMU pendiente de envío.
 *
 * @details
 * El buffer queda desacoplado del paquete binario para que la tarea WiFi pueda
 * copiar una versión estable antes de publicar por MQTT.
 */
static char payload_imuPacket[PAYLOAD_LENGHT]={0};

/**
 * @brief Buffer JSON del paquete ambiental pendiente de envío.
 */
static char payload_envPacket[PAYLOAD_LENGHT]={0};

/**
 * @brief Indica que existe una nueva muestra IMU disponible para empaquetado.
 *
 * @details
 * Permite desacoplar la frecuencia de lectura del sensor de la frecuencia de
 * construcción de paquetes sin bloquear la tarea de adquisición más tiempo del
 * necesario.
 */
static uint8_t imuDataUpdated = 0;

/**
 * @brief Indica que existe una nueva muestra GPS disponible para empaquetado.
 */
static uint8_t gpsDataUpdated = 0;

/**
 * @brief Solicita reconstruir los paquetes derivados de la última muestra IMU.
 */
static uint8_t imuPacketToUpdate = 0;

/**
 * @brief Solicita reconstruir el paquete derivado de la última muestra GPS.
 */
static uint8_t gpsPacketToUpdate = 0;

/**
 * @brief Indica que el payload IMU/ambiental está listo para publicación MQTT.
 */
static uint8_t imuPacketToSend = 0;

/**
 * @brief Indica que el payload GPS está listo para publicación MQTT.
 */
static uint8_t gpsPacketToSend = 0;

/**
 * @brief Último estado reportado por el driver WiFi.
 *
 * @details
 * Se declara como volatile porque su valor representa estado externo del módulo
 * de comunicaciones y se consulta de forma recurrente desde la tarea WiFi.
 */
volatile wifi_status_t wifi_st;

/**
 * @brief Último estado reportado por el driver GPS.
 *
 * @details
 * Se conserva global para facilitar diagnóstico y reutilización del estado de
 * inicialización del módulo GPS.
 */
volatile gps_status_t gps_st;

/**
 * @brief Mutex de protección para el buffer compartido de datos IMU.
 */
osMutexId_t imuMutex;

/**
 * @brief Mutex de protección para el buffer compartido de datos GPS.
 */
osMutexId_t gpsMutex;

/**
 * @brief Mutex de protección para payloads JSON y flags de publicación.
 */
osMutexId_t packetMutex;

/**
 * @brief Handle de la tarea de adquisición IMU.
 */
osThreadId_t imuTaskHandle;

/**
 * @brief Handle de la tarea de adquisición GPS.
 */
osThreadId_t gpsTaskHandle;

/**
 * @brief Handle de la tarea de construcción de paquetes de telemetría.
 */
osThreadId_t packetTaskHandle;

/**
 * @brief Handle de la tarea de conexión WiFi y publicación MQTT.
 */
osThreadId_t wifiTaskHandle;

/**
 * @brief Atributos RTOS de la tarea de adquisición IMU.
 *
 * @details
 * La pila se dimensiona con margen para las llamadas al driver 10DoF y la
 * prioridad normal permite que tareas de comunicación puedan ejecutarse sin
 * bloquear indefinidamente la adquisición.
 */
const osThreadAttr_t imuTask_attributes =
{
    .name = "imuTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal
};

/**
 * @brief Atributos RTOS de la tarea WiFi/MQTT.
 *
 * @details
 * La prioridad superior reduce el riesgo de acumulación de payloads cuando la
 * publicación MQTT necesita atenderse con rapidez frente a tareas de menor
 * criticidad temporal.
 */
const osThreadAttr_t wifiTask_attributes =
{
    .name = "wifiTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal
};

/**
 * @brief Atributos RTOS de la tarea GPS.
 *
 * @details
 * El GPS se ejecuta con menor prioridad porque su frecuencia útil de actualización
 * es inferior a la de la IMU y no requiere una cadencia tan estricta.
 */
const osThreadAttr_t gpsTask_attributes =
{
    .name = "gpsTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityBelowNormal
};

/**
 * @brief Atributos RTOS de la tarea de empaquetado.
 *
 * @details
 * Su prioridad superior permite transformar datos recientes en payloads JSON con
 * baja latencia antes de que la tarea WiFi los publique.
 */
const osThreadAttr_t packetTask_attributes =
{
    .name = "packetTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal
};

/**
 * @brief Tarea FreeRTOS de adquisición del sensor IMU/10DoF.
 *
 * @param argument Argumento genérico proporcionado por CMSIS-RTOS. No se utiliza.
 *
 * @details
 * Inicializa el sensor y mantiene actualizada la última muestra disponible. Si
 * una lectura falla, se solicita un reset del sensor para recuperar el bus o el
 * estado interno del driver sin detener el resto del sistema.
 *
 * La tarea solo mantiene bloqueado el mutex durante la sección crítica de lectura
 * y actualización del buffer compartido. Esto reduce la contención con la tarea
 * de empaquetado.
 */
void StartImuTask(void *argument)
{
    Sensor10DoF_Init();

    for(;;)
    {
        osMutexAcquire(imuMutex, osWaitForever);

        if(Sensor10DoF_Read(&imu_data) != SENSOR_OK)
        {
            Sensor10DoF_Reset();
        }else{
        	imuDataUpdated = 1;
        }
        osMutexRelease(imuMutex);

        osDelay(100);
    }
}

/**
 * @brief Tarea FreeRTOS de inicialización y procesado periódico del GPS.
 *
 * @param argument Argumento genérico proporcionado por CMSIS-RTOS. No se utiliza.
 *
 * @details
 * Reintenta la inicialización del GPS hasta obtener un estado válido. Este patrón
 * evita que un arranque lento del módulo GPS impida la ejecución del resto del
 * firmware.
 *
 * Durante la ejecución periódica, la tarea procesa la entrada NMEA/UART mediante
 * el driver GPS y marca la muestra como disponible para la etapa de empaquetado.
 */
void StartGpsTask(void *argument)
{

	gps_st = GPS_Init(&huart3);


    while(gps_st != GPS_OK)
    {
    	gps_st = GPS_Init(&huart3);
        osDelay(100);
    }


    for(;;)
    {
        osMutexAcquire(gpsMutex, osWaitForever);
        GPS_Process();
        gpsDataUpdated = 1;
        osMutexRelease(gpsMutex);

        osDelay(500);
    }
}

/**
 * @brief Tarea FreeRTOS de fusión lógica y serialización de telemetría.
 *
 * @param argument Argumento genérico proporcionado por CMSIS-RTOS. No se utiliza.
 *
 * @details
 * Esta tarea actúa como etapa intermedia entre adquisición y comunicación. Copia
 * muestras recientes bajo mutex, libera rápidamente los recursos compartidos y
 * construye los paquetes fuera de las secciones críticas siempre que sea posible.
 *
 * El diseño evita que la generación de JSON o la publicación MQTT accedan
 * directamente a las estructuras internas de los drivers, reduciendo acoplamiento
 * y facilitando cambios de formato en la telemetría.
 */
void StartPacketTask(void *argument)
{


    static Sensor10DoF_t imu;
    static gps_data_t gps;

    GpsDataPacket_Init(&gpsPacket);
    ImuDataPacket_Init(&imuPacket);
    EnvDataPacket_Init(&envPacket);


    for(;;)
    {
        osMutexAcquire(imuMutex, osWaitForever);
        if(imuDataUpdated)
        {
            imu = imu_data;
            imuDataUpdated = 0;
            imuPacketToUpdate = 1;
        }
        osMutexRelease(imuMutex);

        osMutexAcquire(gpsMutex, osWaitForever);
        if(gpsDataUpdated)
        {
        	gps = GPS_GetData();
        	gpsDataUpdated = 0;
        	gpsPacketToUpdate = 1;
        }
        osMutexRelease(gpsMutex);

        if(imuPacketToUpdate)
        {
			imuPacket.accel[0] = imu.accel_x_ms2;
			imuPacket.accel[1] = imu.accel_y_ms2;
			imuPacket.accel[2] = imu.accel_z_ms2;

			imuPacket.gyro[0] = imu.gyro_x_dps;
			imuPacket.gyro[1] = imu.gyro_y_dps;
			imuPacket.gyro[2] = imu.gyro_z_dps;

			imuPacket.mag[0] = imu.heading_deg;
			imuPacket.mag[1] = imu.roll_deg;
			imuPacket.mag[2] = imu.pitch_deg;

			envPacket.heading = imu.heading_deg;
			envPacket.temperature = imu.temperature_c;
			envPacket.pressure = imu.pressure_hpa;
        }

        if(gpsPacketToUpdate){
			gpsPacket.altitude = gps.altitude;
			gpsPacket.latitude = gps.latitude;
			gpsPacket.longitude = gps.longitude;

        }

        osMutexAcquire(packetMutex, osWaitForever);
        if(imuPacketToUpdate)
        {
			ImuData_to_json(payload_imuPacket,
					sizeof(payload_imuPacket),
					&imuPacket);

			EnvData_to_json(payload_envPacket,
					sizeof(payload_envPacket),
					&envPacket);

			imuPacketToUpdate = 0;
			imuPacketToSend = 1;
        }

        if(gpsPacketToUpdate)
        {
			GpsData_to_json(payload_gpsPacket,
					sizeof(payload_gpsPacket),
					&gpsPacket);

			gpsPacketToUpdate = 0;
			gpsPacketToSend = 1;
        }

        osMutexRelease(packetMutex);

        osDelay(50);
    }
}

/**
 * @brief Tarea FreeRTOS de conexión WiFi y publicación MQTT.
 *
 * @param argument Argumento genérico proporcionado por CMSIS-RTOS. No se utiliza.
 *
 * @details
 * Inicializa el módulo WiFi, garantiza la conexión al punto de acceso y configura
 * una sesión MQTT antes de entrar en el bucle de publicación.
 *
 * Los payloads se copian a buffers temporales antes de llamar al driver WiFi para
 * minimizar el tiempo de bloqueo de @ref packetMutex. Esta decisión evita que una
 * publicación lenta impida a la tarea de empaquetado actualizar nuevos mensajes.
 */
void StartWifiTask(void *argument)
{
	static char payload_gpsTemp[PAYLOAD_LENGHT]={0};
	static char payload_imuTemp[PAYLOAD_LENGHT]={0};
	static char payload_envTemp[PAYLOAD_LENGHT]={0};

    wifi_st = WIFI_Init(&huart2);
    osDelay(1000);

    while(wifi_st != WIFI_OK)
    {
    	wifi_st = WIFI_Reset();
        wifi_st = WIFI_Init(&huart2);
        osDelay(1000);
    }
    wifi_st = WIFI_ConnectAP(WIFI_TEST_SSID, WIFI_TEST_PASS);
    osDelay(1000);

    while(wifi_st != WIFI_OK)
    {
    	wifi_st = WIFI_ConnectAP(WIFI_TEST_SSID, WIFI_TEST_PASS);
        osDelay(1000);
    }

    WIFI_MQTT_Disconnect();
    osDelay(500);

    wifi_st = WIFI_MQTT_UserConfig("stm32_client");
    osDelay(500);

    wifi_st = WIFI_MQTT_Connect(SERVER_HOST, 1883);
    osDelay(500);


    for(;;)
    {
        if(gpsPacketToSend)
        {
            osMutexAcquire(packetMutex, osWaitForever);

            strcpy(payload_gpsTemp, payload_gpsPacket);

            osMutexRelease(packetMutex);

            wifi_st = WIFI_MQTT_Publish("telemetry",payload_gpsTemp,0,0);

            gpsPacketToSend = 0;

            osDelay(10);

        }

        if(imuPacketToSend)
        {
            osMutexAcquire(packetMutex, osWaitForever);

            strcpy(payload_imuTemp, payload_imuPacket);

            osMutexRelease(packetMutex);

            wifi_st = WIFI_MQTT_Publish("telemetry",payload_imuTemp,0,0);
            osDelay(10);


            osMutexAcquire(packetMutex, osWaitForever);

            strcpy(payload_envTemp, payload_envPacket);

            osMutexRelease(packetMutex);

            wifi_st = WIFI_MQTT_Publish("telemetry",payload_envTemp,0,0);

            imuPacketToSend = 0;

        }

		if(wifi_st != WIFI_OK)
		{
		}

        osDelay(50);
    }
}

/**
 * @brief Inicializa los recursos FreeRTOS definidos por la aplicación.
 *
 * @details
 * Crea los mutex compartidos y lanza las tareas que forman la cadena de
 * adquisición, empaquetado y transmisión. La función conserva el patrón esperado
 * por STM32CubeMX para integrarse con la inicialización global del proyecto.
 */
void MX_FREERTOS_Init(void)
{
    imuMutex = osMutexNew(NULL);
    gpsMutex = osMutexNew(NULL);
    packetMutex = osMutexNew(NULL);

    imuTaskHandle = osThreadNew(StartImuTask,NULL,&imuTask_attributes);
    gpsTaskHandle = osThreadNew(StartGpsTask,NULL,&gpsTask_attributes);
    wifiTaskHandle = osThreadNew(StartWifiTask,NULL,&wifiTask_attributes);
    packetTaskHandle = osThreadNew(StartPacketTask,NULL,&packetTask_attributes);

}
