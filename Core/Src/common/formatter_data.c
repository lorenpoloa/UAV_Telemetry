/**
 * @file formatter_data.c
 * @brief Conversión de paquetes de telemetría a cadenas de texto para publicación.
 * @author Lorenzo
 * @date 15 may 2026
 *
 * @details
 * Este módulo centraliza el formateo de los datos adquiridos antes de enviarlos
 * por el canal de comunicaciones. Se mantiene un formato compacto basado en
 * separadores para reducir el tamaño del payload y simplificar su transporte por
 * MQTT/AT, evitando caracteres que anteriormente podían interferir con el
 * encapsulado de comandos del módulo WiFi.
 */

#include "common/formatter_data.h"

#include <stdio.h>
#include <string.h>

/**
 * @brief Serializa el paquete inercial en el formato compacto de telemetría.
 *
 * @param[out] buffer Buffer donde se escribe la cadena generada.
 * @param[in] buffer_size Tamaño total disponible en @p buffer.
 * @param[in] data Paquete IMU con acelerómetro, giróscopo y magnetómetro.
 *
 * @details
 * El formato usa claves delimitadas con `|` y campos separados por `/` para
 * evitar comillas dobles dentro del payload. Esta decisión reduce problemas al
 * insertar la cadena dentro de comandos AT, donde las comillas forman parte de
 * la sintaxis del propio comando.
 *
 * @pre @p buffer debe apuntar a memoria válida con al menos @p buffer_size bytes.
 * @pre @p data debe apuntar a un paquete IMU válido.
 * @note La función no devuelve estado; si el buffer no es suficiente, `snprintf`
 *       truncará la salida manteniendo la terminación nula cuando el tamaño lo
 *       permita.
 */
void ImuData_to_json(char *buffer,
                       size_t buffer_size,
					   ImuDataPacket_t *data){

	int pos = 0;

	pos += snprintf(buffer + pos, buffer_size - pos, "{");

	pos += snprintf(buffer + pos, buffer_size - pos,
		    "|ax|:%.1f/|ay|:%.1f/|az|:%.1f/",
			data->accel[0], data->accel[1], data->accel[2]);
	pos += snprintf(buffer + pos, buffer_size - pos,
		    "|gx|:%.1f/|gy|:%.1f/|gz|:%.1f/",
			data->gyro[0], data->gyro[1], data->gyro[2]);
	pos += snprintf(buffer + pos, buffer_size - pos,
		    "|mx|:%.1f/|my|:%.1f/|mz|:%.1f",
			data->mag[0], data->mag[1], data->mag[2]);

	pos += snprintf(buffer + pos, buffer_size - pos, "}");
}

/**
 * @brief Serializa el paquete ambiental en el formato compacto de telemetría.
 *
 * @param[out] buffer Buffer donde se escribe la cadena generada.
 * @param[in] buffer_size Tamaño total disponible en @p buffer.
 * @param[in] data Paquete ambiental con rumbo, temperatura y presión.
 *
 * @details
 * El formato se mantiene coherente con el resto de tramas del sistema para que
 * el receptor pueda aplicar una única lógica de separación de campos, sin tratar
 * este paquete como un caso especial.
 *
 * @pre @p buffer debe apuntar a memoria válida con al menos @p buffer_size bytes.
 * @pre @p data debe apuntar a un paquete ambiental válido.
 */
void EnvData_to_json(char *buffer,
                       size_t buffer_size,
					   EnvDataPacket_t *data){

	snprintf(buffer, buffer_size,
	    "{|h|:%.2f/|t|:%.2f/|p|:%.2f}",
		data->heading, data->temperature, data->pressure);
}

/**
 * @brief Serializa el paquete GPS en el formato compacto de telemetría.
 *
 * @param[out] buffer Buffer donde se escribe la cadena generada.
 * @param[in] buffer_size Tamaño total disponible en @p buffer.
 * @param[in] data Paquete GPS con altitud, latitud y longitud.
 *
 * @details
 * Las coordenadas se limitan a cuatro decimales para contener el tamaño del
 * mensaje y porque la precisión resultante es suficiente para la visualización
 * general de posición usada por la interfaz del proyecto.
 *
 * @pre @p buffer debe apuntar a memoria válida con al menos @p buffer_size bytes.
 * @pre @p data debe apuntar a un paquete GPS válido.
 */
void GpsData_to_json(char *buffer,
                       size_t buffer_size,
					   GpsDataPacket_t *data){

	snprintf(buffer, buffer_size,
	    "{|al|:%.1lf/|la|:%.4lf/|lo|:%.4lf}",
		data->altitude, data->latitude, data->longitude);
}

/**
 * @brief Inicializa un paquete GPS a un estado conocido.
 *
 * @param[out] packet Paquete GPS que se reinicia.
 *
 * @details
 * Se borra toda la estructura antes de fijar los flags de control para evitar
 * que una lectura parcial o antigua se interprete como nueva telemetría válida.
 *
 * @pre @p packet debe apuntar a una estructura válida.
 */
void GpsDataPacket_Init(GpsDataPacket_t *packet)
{
    memset(packet,
           0,
           sizeof(GpsDataPacket_t));

    packet->checksum = 0;
    packet->gps_updated = 0;
}

/**
 * @brief Inicializa un paquete IMU a un estado conocido.
 *
 * @param[out] packet Paquete IMU que se reinicia.
 *
 * @details
 * El reinicio explícito del flag de actualización permite que las tareas
 * consumidoras distingan entre datos recién publicados y memoria simplemente
 * reservada o reutilizada.
 *
 * @pre @p packet debe apuntar a una estructura válida.
 */
void ImuDataPacket_Init(ImuDataPacket_t *packet)
{
    memset(packet,
           0,
           sizeof(ImuDataPacket_t));

    packet->checksum = 0;
    packet->imu_updated = 0;

}

/**
 * @brief Inicializa un paquete ambiental a un estado conocido.
 *
 * @param[out] packet Paquete ambiental que se reinicia.
 *
 * @details
 * Se utiliza el mismo patrón que en el resto de paquetes para mantener una
 * semántica uniforme entre productores y consumidores de datos en FreeRTOS.
 *
 * @pre @p packet debe apuntar a una estructura válida.
 */
void EnvDataPacket_Init(EnvDataPacket_t *packet)
{
    memset(packet,
           0,
           sizeof(EnvDataPacket_t));

    packet->checksum = 0;
    packet->env_updated = 0;

}
