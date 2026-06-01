/**
 * @file gps.h
 * @brief Interfaz pública del driver GPS basado en recepción UART.
 * @author Lorenzo
 * @date 13 may 2026
 *
 * @details
 * El módulo encapsula la recepción y el procesado de tramas NMEA para ofrecer al
 * resto del sistema una estructura de datos estable. La separación entre
 * callback UART, procesado y consulta de datos permite integrarlo en FreeRTOS
 * sin bloquear las tareas de adquisición o comunicación.
 */

#ifndef GPS_H
#define GPS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @def GPS_RX_BUFFER_SIZE
 * @brief Tamaño del buffer circular usado para desacoplar la UART del parser.
 *
 * @details
 * Se dimensiona por encima de una única sentencia NMEA para absorber ráfagas o
 * pequeñas latencias de planificación sin perder caracteres.
 */
#define GPS_RX_BUFFER_SIZE          512

/**
 * @def GPS_SENTENCE_MAX_LENGTH
 * @brief Longitud máxima esperada para una sentencia NMEA completa.
 *
 * @details
 * Limita el almacenamiento temporal de una trama individual y protege al parser
 * frente a entradas corruptas o sin terminador.
 */
#define GPS_SENTENCE_MAX_LENGTH     128

/**
 * @brief Códigos de estado devueltos por la API GPS.
 *
 * @details
 * Permiten diferenciar fallos de inicialización, ausencia temporal de datos y
 * errores generales sin obligar a las capas superiores a conocer detalles de la
 * UART o del parser.
 */
typedef enum
{
    /** @brief Operación completada correctamente. */
    GPS_OK = 0,

    /** @brief Error genérico durante una operación del módulo GPS. */
    GPS_ERROR,

    /** @brief La operación no recibió datos dentro del tiempo esperado. */
    GPS_TIMEOUT

} gps_status_t;

/**
 * @brief Estado de fix del receptor GPS.
 *
 * @details
 * Se mantiene como enum para que la validez de posición sea explícita y no
 * dependa de interpretar directamente campos numéricos de la trama NMEA.
 */
typedef enum
{
    /** @brief El receptor todavía no dispone de posición válida. */
    GPS_NO_FIX = 0,

    /** @brief El receptor dispone de posición válida. */
    GPS_FIX

} gps_fix_t;

/**
 * @brief Última solución de navegación calculada por el módulo GPS.
 *
 * @details
 * La estructura conserva tanto la información de posición como metadatos de
 * calidad. Esto permite que la aplicación decida si publica o descarta una
 * muestra sin tener que volver a analizar la sentencia NMEA original.
 */
typedef struct
{
    /** @brief Latitud en grados decimales. */
    float latitude;

    /** @brief Longitud en grados decimales. */
    float longitude;

    /** @brief Altitud proporcionada por el receptor GPS. */
    float altitude;

    /** @brief Velocidad en nudos, mantenida por compatibilidad con NMEA. */
    float speed_knots;

    /** @brief Velocidad convertida a km/h para telemetría de aplicación. */
    float speed_kmh;

    /** @brief Número de satélites usados o visibles según la sentencia procesada. */
    uint8_t satellites;

    /** @brief Estado de fix asociado a la posición almacenada. */
    gps_fix_t fix;

    /** @brief Indica si la última muestra procesada es apta para uso externo. */
    bool valid;

} gps_data_t;

/**
 * @brief Inicializa el módulo GPS y asocia la UART usada por el receptor.
 *
 * @param[in] huart Handler HAL de la UART conectada al módulo GPS.
 *
 * @return GPS_OK si la inicialización se completa correctamente.
 * @return GPS_ERROR si no es posible preparar la recepción.
 *
 * @pre `huart` debe estar inicializado por la capa HAL antes de llamar a esta
 *      función.
 */
gps_status_t GPS_Init(UART_HandleTypeDef *huart);

/**
 * @brief Callback que debe invocarse desde la interrupción de recepción UART.
 *
 * @param[in] huart Handler HAL que ha generado el evento de recepción.
 *
 * @details
 * La callback mantiene la captura de caracteres separada del procesado completo
 * de sentencias para reducir el trabajo realizado en contexto de interrupción.
 */
void GPS_UARTCallback(UART_HandleTypeDef *huart);

/**
 * @brief Procesa las tramas GPS acumuladas por la recepción UART.
 *
 * @details
 * Debe ejecutarse desde contexto de tarea o bucle principal para convertir las
 * sentencias recibidas en una muestra `gps_data_t` consultable por la aplicación.
 */
void GPS_Process(void);

/**
 * @brief Devuelve una copia de la última muestra GPS procesada.
 *
 * @return Estructura `gps_data_t` con el último estado conocido del receptor.
 *
 * @note La devolución por valor evita que el consumidor modifique accidentalmente
 *       el estado interno del driver.
 */
gps_data_t GPS_GetData(void);

/**
 * @brief Indica si la última muestra GPS disponible es válida.
 *
 * @retval true Existe una muestra válida con fix aceptable.
 * @retval false La muestra actual no debe usarse como posición fiable.
 */
bool GPS_DataValid(void);

/**
 * @brief Reinicia la recepción UART del GPS.
 *
 * @details
 * Se expone para recuperar la recepción tras errores o condiciones de parada sin
 * reinicializar toda la aplicación.
 */
void GPS_RestartReceive(void);

#endif
