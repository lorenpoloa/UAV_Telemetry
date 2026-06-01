/**
 * @file gps.c
 * @brief Driver no bloqueante para recepción y procesado de tramas NMEA GPS.
 * @author Lorenzo
 * @date 13 may 2026
 *
 * @details
 * El módulo recibe bytes por interrupción UART y los desacopla del parser usando
 * un búfer circular. Esta arquitectura evita realizar trabajo pesado dentro de
 * la ISR y permite que el procesado de sentencias NMEA se ejecute desde una
 * tarea o bucle principal con menor impacto temporal.
 */

#include "sensors/gps.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/** @defgroup GPS_Private Estado interno del driver GPS
 *  @brief Variables privadas usadas para desacoplar recepción UART y parsing.
 *  @{
 */

/** @brief UART asociada al módulo GPS inicializado. */
static UART_HandleTypeDef *gps_uart;

/** @brief Byte temporal usado por la recepción UART por interrupción. */
static uint8_t uart_rx_byte;

/** @brief Búfer circular de recepción para no perder bytes entre llamadas a GPS_Process(). */
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];

/** @brief Índice de escritura del búfer circular, actualizado desde la ISR. */
static volatile uint16_t rx_head = 0;

/** @brief Índice de lectura del búfer circular, consumido por el parser. */
static volatile uint16_t rx_tail = 0;

/** @brief Sentencia NMEA en construcción antes de identificar su tipo. */
static char sentence[GPS_SENTENCE_MAX_LENGTH];

/** @brief Posición actual dentro de @ref sentence. */
static uint16_t sentence_index = 0;

/** @brief Último estado GPS decodificado y disponible para el resto del sistema. */
static gps_data_t gps_data;

/** @} */

/**
 * @brief Inserta un byte recibido en el búfer circular.
 *
 * @param[in] byte Byte procedente de la UART GPS.
 *
 * @details
 * Si el búfer está lleno se descarta el byte nuevo para proteger la coherencia
 * de los índices. Esta política prioriza que el parser no lea memoria corrupta
 * frente a conservar una sentencia incompleta.
 */
static void gps_rx_push(uint8_t byte)
{
    uint16_t next =
        (rx_head + 1) % GPS_RX_BUFFER_SIZE;

    if(next != rx_tail)
    {
        gps_rx_buffer[rx_head] = byte;
        rx_head = next;
    }
}

/**
 * @brief Extrae un byte pendiente del búfer circular.
 *
 * @param[out] byte Puntero donde se almacena el byte leído.
 * @retval true Se ha extraído un byte válido.
 * @retval false No había datos pendientes.
 *
 * @details
 * La lectura se realiza fuera de la interrupción para mantener la ISR limitada
 * al mínimo trabajo imprescindible.
 */
static bool gps_rx_pop(uint8_t *byte)
{
    if(rx_head == rx_tail)
        return false;

    *byte = gps_rx_buffer[rx_tail];

    rx_tail =
        (rx_tail + 1) % GPS_RX_BUFFER_SIZE;

    return true;
}

/**
 * @brief Convierte una coordenada NMEA en grados decimales.
 *
 * @param[in] raw Coordenada en formato NMEA `ddmm.mmmm` o `dddmm.mmmm`.
 * @param[in] dir Hemisferio asociado a la coordenada (`N`, `S`, `E` o `W`).
 * @return Coordenada en grados decimales con signo geográfico.
 *
 * @details
 * El GPS entrega latitud y longitud en grados/minutos. La conversión se realiza
 * aquí para que el resto del sistema trabaje siempre con un formato directo de
 * representar en mapas o interfaces gráficas.
 */
static float gps_nmea_to_decimal(const char *raw,
                                 char dir)
{
    float value = atof(raw);

    int deg = (int)(value / 100);

    float minutes =
        value - (deg * 100);

    float decimal =
        deg + (minutes / 60.0f);

    if(dir == 'S' || dir == 'W')
        decimal *= -1.0f;

    return decimal;
}

/**
 * @brief Decodifica una sentencia GGA y actualiza posición, fix, satélites y altitud.
 *
 * @param[in,out] nmea Sentencia NMEA modificable.
 *
 * @details
 * Se usa GGA porque concentra la información de calidad de fix y altitud. La
 * sentencia se tokeniza in situ para evitar buffers adicionales en RAM.
 *
 * @warning `strtok()` modifica la cadena recibida; no se debe reutilizar la
 *          sentencia original después de llamar a esta función.
 */
static void gps_parse_gga(char *nmea)
{
    char *token;

    uint8_t field = 0;

    char lat[16] = {0};
    char lon[16] = {0};

    char lat_dir = 0;
    char lon_dir = 0;

    token = strtok(nmea, ",");

    while(token)
    {
        switch(field)
        {
            case 2:
                strncpy(lat, token, sizeof(lat));
                break;

            case 3:
                lat_dir = token[0];
                break;

            case 4:
                strncpy(lon, token, sizeof(lon));
                break;

            case 5:
                lon_dir = token[0];
                break;

            case 6:
                gps_data.fix =
                    atoi(token) ? GPS_FIX : GPS_NO_FIX;
                break;

            case 7:
                gps_data.satellites =
                    atoi(token);
                break;

            case 9:
                gps_data.altitude =
                    atof(token);
                break;
        }

        token = strtok(NULL, ",");

        field++;
    }

    gps_data.latitude =
        gps_nmea_to_decimal(lat, lat_dir);

    gps_data.longitude =
        gps_nmea_to_decimal(lon, lon_dir);

    gps_data.valid = true;
}

/**
 * @brief Decodifica una sentencia RMC y actualiza la velocidad.
 *
 * @param[in,out] nmea Sentencia NMEA modificable.
 *
 * @details
 * La velocidad se toma de RMC porque es el campo NMEA estándar para movimiento
 * sobre tierra. Se guarda en nudos y km/h para no repetir conversiones en las
 * capas superiores.
 *
 * @warning `strtok()` modifica la cadena recibida.
 */
static void gps_parse_rmc(char *nmea)
{
    char *token;

    uint8_t field = 0;

    token = strtok(nmea, ",");

    while(token)
    {
        switch(field)
        {
            case 7:

                gps_data.speed_knots =
                    atof(token);

                gps_data.speed_kmh =
                    gps_data.speed_knots * 1.852f;

                break;
        }

        token = strtok(NULL, ",");

        field++;
    }
}

/**
 * @brief Inicializa el driver GPS y arranca la recepción UART por interrupción.
 *
 * @param[in] huart UART conectada al módulo GPS.
 * @retval GPS_OK La recepción inicial se configuró correctamente.
 * @retval GPS_ERROR No se pudo iniciar la recepción por interrupción.
 *
 * @details
 * La recepción se inicia byte a byte para poder reconstruir sentencias NMEA sin
 * depender de DMA ni de longitudes fijas, ya que el GPS emite tramas ASCII de
 * tamaño variable.
 */
gps_status_t GPS_Init(UART_HandleTypeDef *huart)
{
	gps_status_t gps_st;
    gps_uart = huart;

    memset(&gps_data, 0, sizeof(gps_data));

    if(HAL_UART_Receive_IT(gps_uart, &uart_rx_byte,1) == HAL_OK ){
    	gps_st = GPS_OK;

    }else{
    	gps_st = GPS_ERROR;
    }

    return gps_st;
}

/**
 * @brief Callback que debe llamarse al completarse la recepción UART del GPS.
 *
 * @param[in] huart UART que ha generado la interrupción.
 *
 * @details
 * El callback filtra la UART recibida para que el driver ignore interrupciones
 * de otros periféricos. Tras guardar el byte, rearma inmediatamente la escucha
 * para mantener una recepción continua.
 */
void GPS_UARTCallback(UART_HandleTypeDef *huart)
{
    if(huart != gps_uart)
        return;

    gps_rx_push(uart_rx_byte);

    HAL_UART_Receive_IT(gps_uart,
                        &uart_rx_byte,
                        1);
}

/**
 * @brief Procesa los bytes pendientes y actualiza los datos GPS disponibles.
 *
 * @details
 * Esta función debe ejecutarse periódicamente desde contexto de tarea o bucle
 * principal. Separar el parser de la ISR evita bloquear otras interrupciones y
 * reduce el riesgo de perder datos en sistemas con FreeRTOS.
 */
void GPS_Process(void)
{
    uint8_t byte;

    while(gps_rx_pop(&byte))
    {
        if(byte == '$')
        {
            sentence_index = 0;
        }

        if(sentence_index <
           GPS_SENTENCE_MAX_LENGTH - 1)
        {
            sentence[sentence_index++] = byte;
        }

        if(byte == '\n')
        {
            sentence[sentence_index] = 0;

            if(strstr(sentence, "GPGGA"))
            {
                gps_parse_gga(sentence);
            }
            else if(strstr(sentence, "GPRMC"))
            {
                gps_parse_rmc(sentence);
            }

            sentence_index = 0;
        }
    }
}

/**
 * @brief Devuelve una copia del último estado GPS decodificado.
 *
 * @return Estructura con los últimos datos GPS disponibles.
 *
 * @details
 * Se devuelve por valor para evitar que otros módulos modifiquen directamente
 * el estado interno del driver.
 */
gps_data_t GPS_GetData(void)
{
    return gps_data;
}

/**
 * @brief Indica si ya se ha decodificado una sentencia GPS válida.
 *
 * @retval true Hay datos GPS procesados.
 * @retval false Aún no hay datos GPS válidos disponibles.
 */
bool GPS_DataValid(void)
{
    return gps_data.valid;
}

/**
 * @brief Rearma manualmente la recepción UART del GPS.
 *
 * @details
 * Esta función permite recuperar la escucha si una condición externa cancela o
 * interrumpe la recepción por interrupción. Se reutiliza el byte interno para
 * mantener encapsulado el estado de recepción.
 */
void GPS_RestartReceive(void)
{
    HAL_UART_Receive_IT(gps_uart, &uart_rx_byte, 1);
}
