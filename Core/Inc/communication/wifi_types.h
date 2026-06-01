/**
 * @file wifi_types.h
 * @brief Tipos compartidos por la interfaz WiFi y las capas de aplicación.
 * @author Lorenzo
 * @date 25 abr 2026
 *
 * @details
 * Este archivo separa los estados y códigos de retorno de la API principal para
 * que puedan reutilizarse en distintos módulos sin introducir dependencias
 * circulares con el driver WiFi completo.
 */
#ifndef WIFI_TYPES_H
#define WIFI_TYPES_H

#include <stdint.h>

/**
 * @brief Códigos de resultado de las operaciones WiFi.
 *
 * @details
 * Permiten a las tareas de FreeRTOS diferenciar entre fallos recuperables,
 * ausencia de conexión y recursos ocupados, manteniendo una política de
 * recuperación desacoplada de los comandos AT concretos.
 */
typedef enum
{
    /** @brief Operación completada correctamente. */
    WIFI_OK = 0,

    /** @brief Error genérico durante la operación. */
    WIFI_ERROR,

    /** @brief No se recibió respuesta dentro del tiempo esperado. */
    WIFI_TIMEOUT,

    /** @brief La operación requiere una conexión que todavía no existe. */
    WIFI_NOT_CONNECTED,

    /** @brief El módulo está ocupado o no puede aceptar una nueva operación. */
    WIFI_BUSY

} wifi_status_t;

/**
 * @brief Estado lógico de la conexión WiFi/MQTT.
 *
 * @details
 * El estado se expone a la aplicación para decidir si debe reintentar conexión,
 * publicar telemetría o esperar a que el módulo alcance una fase superior.
 */
typedef enum
{
    /** @brief El módulo no está asociado a una red WiFi. */
    WIFI_DISCONNECTED = 0,

    /** @brief El módulo está asociado al punto de acceso. */
    WIFI_CONNECTED,

    /** @brief El módulo dispone de dirección IP. */
    WIFI_GOT_IP,

    /** @brief La sesión MQTT está establecida. */
    WIFI_MQTT_CONNECTED

} wifi_state_t;

#endif /* WIFI_TYPES_H */
