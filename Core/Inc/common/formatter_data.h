/**
 * @file formatter_data.h
 * @brief Define los paquetes de datos normalizados y la API de conversión a JSON.
 * @author Lorenzo
 * @date 15 may 2026
 *
 * @details
 * Este módulo actúa como frontera entre las tareas de adquisición y la capa de
 * comunicación. Las estructuras agrupan los datos por origen funcional
 * —IMU, entorno y GPS— para que FreeRTOS pueda intercambiar muestras completas
 * sin acoplar directamente los drivers de sensores con el formato usado por MQTT.
 *
 * Los campos de actualización y checksum permiten distinguir muestras nuevas y
 * validar la coherencia básica del paquete antes de serializarlo.
 */

#ifndef FORMATTER_DATA_H
#define FORMATTER_DATA_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Paquete normalizado con las magnitudes procedentes de la IMU.
 *
 * @details
 * Agrupa acelerómetro, giróscopo y magnetómetro en un único bloque para que la
 * tarea de publicación pueda tratar la lectura inercial como una muestra
 * atómica. El timestamp permite ordenar las muestras aunque se reciban con
 * distinta latencia dentro del pipeline.
 */
typedef struct
{
    /** @brief Aceleración en los tres ejes. */
    float accel[3];

    /** @brief Velocidad angular en los tres ejes. */
    float gyro[3];

    /** @brief Campo magnético en los tres ejes. */
    float mag[3];

    /** @brief Marca temporal asociada a la muestra IMU. */
    uint32_t imu_timestamp;

    /** @brief Valor de comprobación usado para detectar paquetes inconsistentes. */
    uint32_t checksum;

    /** @brief Indica si el paquete contiene una lectura IMU pendiente de consumir. */
    int imu_updated;


} ImuDataPacket_t;

/**
 * @brief Paquete normalizado con las magnitudes ambientales.
 *
 * @details
 * Mantiene juntas las variables ambientales calculadas por el módulo 10DoF para
 * evitar que temperatura, presión y rumbo pertenezcan a instantes distintos al
 * ser enviados por la capa de comunicación.
 */
typedef struct
{
    /** @brief Rumbo estimado a partir del magnetómetro. */
    float heading;

    /** @brief Temperatura medida por el sensor barométrico. */
    float temperature;

    /** @brief Presión atmosférica medida o calculada. */
    float pressure;

    /** @brief Marca temporal asociada a la muestra ambiental. */
    uint32_t env_timestamp;

    /** @brief Valor de comprobación usado para detectar paquetes inconsistentes. */
    uint32_t checksum;

    /** @brief Indica si el paquete contiene datos ambientales pendientes de consumir. */
    int env_updated;


} EnvDataPacket_t;

/**
 * @brief Paquete normalizado con la posición procedente del receptor GPS.
 *
 * @details
 * Se separa del paquete IMU porque la frecuencia y la validez de las tramas GPS
 * son independientes de las lecturas inerciales. El flag de actualización evita
 * publicar repetidamente una posición antigua cuando no llega una nueva trama
 * válida.
 */
typedef struct
{
    /** @brief Altitud calculada por el receptor GPS. */
    double altitude;

    /** @brief Latitud geográfica en grados decimales. */
    double latitude;

    /** @brief Longitud geográfica en grados decimales. */
    double longitude;

    /** @brief Marca temporal asociada a la muestra GPS. */
    uint32_t gps_timestamp;

    /** @brief Valor de comprobación usado para detectar paquetes inconsistentes. */
    uint32_t checksum;

    /** @brief Indica si el paquete contiene una lectura GPS pendiente de consumir. */
    int gps_updated;

} GpsDataPacket_t;


/**
 * @brief Serializa un paquete GPS en formato JSON.
 *
 * @param[out] buffer Buffer de salida donde se escribe la cadena JSON.
 * @param[in] buffer_size Tamaño disponible en el buffer de salida.
 * @param[in] data Paquete GPS que contiene la última muestra normalizada.
 *
 * @pre `buffer` debe apuntar a memoria válida y `buffer_size` debe ser suficiente
 *      para almacenar el JSON generado.
 * @pre `data` debe apuntar a un paquete inicializado.
 *
 * @note La función está pensada para preparar payloads MQTT sin exponer al
 *       publicador los detalles internos de la estructura GPS.
 */
void GpsData_to_json(char *buffer,
                       size_t buffer_size,
					   GpsDataPacket_t *data);

/**
 * @brief Serializa un paquete IMU en formato JSON.
 *
 * @param[out] buffer Buffer de salida donde se escribe la cadena JSON.
 * @param[in] buffer_size Tamaño disponible en el buffer de salida.
 * @param[in] data Paquete IMU que contiene la última muestra normalizada.
 *
 * @pre `buffer` debe apuntar a memoria válida y `buffer_size` debe ser suficiente
 *      para almacenar el JSON generado.
 * @pre `data` debe apuntar a un paquete inicializado.
 *
 * @note Centralizar la conversión evita duplicar el formato de telemetría en las
 *       tareas de FreeRTOS.
 */
void ImuData_to_json(char *buffer,
                       size_t buffer_size,
					   ImuDataPacket_t *data);

/**
 * @brief Serializa un paquete ambiental en formato JSON.
 *
 * @param[out] buffer Buffer de salida donde se escribe la cadena JSON.
 * @param[in] buffer_size Tamaño disponible en el buffer de salida.
 * @param[in] data Paquete ambiental que contiene la última muestra normalizada.
 *
 * @pre `buffer` debe apuntar a memoria válida y `buffer_size` debe ser suficiente
 *      para almacenar el JSON generado.
 * @pre `data` debe apuntar a un paquete inicializado.
 */
void EnvData_to_json(char *buffer,
                       size_t buffer_size,
					   EnvDataPacket_t *data);

/**
 * @brief Inicializa un paquete GPS con valores seguros.
 *
 * @param[out] packet Paquete GPS a inicializar.
 *
 * @post El paquete queda preparado para ser usado por el pipeline antes de que
 *       llegue la primera trama GPS válida.
 */
void GpsDataPacket_Init(GpsDataPacket_t *packet);

/**
 * @brief Inicializa un paquete IMU con valores seguros.
 *
 * @param[out] packet Paquete IMU a inicializar.
 *
 * @post El paquete queda preparado para ser usado por el pipeline antes de la
 *       primera lectura del sensor.
 */
void ImuDataPacket_Init(ImuDataPacket_t *packet);

/**
 * @brief Inicializa un paquete ambiental con valores seguros.
 *
 * @param[out] packet Paquete ambiental a inicializar.
 *
 * @post El paquete queda preparado para ser usado por el pipeline antes de la
 *       primera lectura barométrica o magnética.
 */
void EnvDataPacket_Init(EnvDataPacket_t *packet);

#endif
