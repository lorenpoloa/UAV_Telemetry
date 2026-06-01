/**
 * @file wifi.h
 * @brief Interfaz pública del driver WiFi basado en comandos AT.
 * @author Lorenzo
 * @date 25 abr 2026
 *
 * @details
 * El módulo encapsula las operaciones de red necesarias para el proyecto:
 * conexión al punto de acceso, transporte TCP, envío HTTP y publicación MQTT.
 * La API mantiene una interfaz de alto nivel para que las tareas de FreeRTOS no
 * dependan del formato concreto de los comandos AT ni de las respuestas del
 * módulo WiFi.
 */

#ifndef WIFI_H
#define WIFI_H

#include "main.h"
#include "wifi_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el driver WiFi y asocia la UART del módulo.
 *
 * @param[in] huart Handler HAL de la UART conectada al módulo WiFi.
 *
 * @return WIFI_OK si el módulo queda preparado para recibir comandos.
 * @return WIFI_ERROR si la UART o el módulo no pueden inicializarse.
 *
 * @pre `huart` debe estar inicializado por la capa HAL.
 */
wifi_status_t WIFI_Init(UART_HandleTypeDef *huart);

/**
 * @brief Reinicia el módulo WiFi.
 *
 * @return WIFI_OK si el reinicio se acepta correctamente.
 * @return WIFI_TIMEOUT si el módulo no responde.
 * @return WIFI_ERROR si la secuencia de reinicio falla.
 *
 * @details
 * Se usa para devolver el módem a un estado conocido cuando la comunicación AT
 * o la conexión de red dejan de ser fiables.
 */
wifi_status_t WIFI_Reset(void);

/**
 * @brief Comprueba la comunicación básica con el módulo WiFi.
 *
 * @return WIFI_OK si el módulo responde al comando de prueba.
 * @return WIFI_TIMEOUT si no hay respuesta.
 * @return WIFI_ERROR si la respuesta recibida no es válida.
 */
wifi_status_t WIFI_Test(void);

/**
 * @brief Configura el módulo WiFi en modo estación.
 *
 * @return WIFI_OK si el modo queda configurado correctamente.
 * @return WIFI_ERROR si el módulo rechaza la configuración.
 *
 * @details
 * El modo estación es necesario para que el dispositivo se conecte a la red
 * local y pueda alcanzar el broker MQTT o servicios HTTP externos.
 */
wifi_status_t WIFI_SetStationMode(void);

/**
 * @brief Conecta el módulo WiFi a un punto de acceso.
 *
 * @param[in] ssid Nombre de la red WiFi.
 * @param[in] pass Contraseña de la red WiFi.
 *
 * @return WIFI_OK si la asociación se completa correctamente.
 * @return WIFI_TIMEOUT si el punto de acceso no responde a tiempo.
 * @return WIFI_ERROR si las credenciales o la negociación fallan.
 *
 * @pre `ssid` y `pass` deben apuntar a cadenas válidas terminadas en cero.
 */
wifi_status_t WIFI_ConnectAP(const char *ssid, const char *pass);

/**
 * @brief Desconecta el módulo del punto de acceso actual.
 *
 * @return WIFI_OK si la desconexión se completa correctamente.
 * @return WIFI_ERROR si el módulo no acepta la orden.
 */
wifi_status_t WIFI_DisconnectAP(void);

/**
 * @brief Devuelve el estado lógico actual del driver WiFi.
 *
 * @return Estado de conexión almacenado por el módulo.
 *
 * @note El estado permite a la aplicación evitar publicaciones cuando todavía no
 *       existe conectividad suficiente.
 */
wifi_state_t  WIFI_GetState(void);

/**
 * @brief Solicita la dirección IP asignada al módulo.
 *
 * @param[out] ip Buffer donde se copiará la dirección IP.
 * @param[in] len Tamaño disponible en el buffer `ip`.
 *
 * @return WIFI_OK si se obtiene una dirección IP válida.
 * @return WIFI_NOT_CONNECTED si el módulo no tiene red activa.
 * @return WIFI_ERROR si no puede parsearse la respuesta.
 *
 * @pre `ip` debe apuntar a un buffer válido.
 */
wifi_status_t WIFI_GetIP(char *ip, uint16_t len);

/**
 * @brief Solicita el nivel RSSI de la conexión WiFi.
 *
 * @param[out] rssi Variable donde se almacena la potencia recibida.
 *
 * @return WIFI_OK si el RSSI se obtiene correctamente.
 * @return WIFI_NOT_CONNECTED si no hay conexión activa.
 * @return WIFI_ERROR si la respuesta no puede interpretarse.
 *
 * @pre `rssi` debe apuntar a memoria válida.
 */
wifi_status_t WIFI_GetRSSI(int *rssi);

/**
 * @brief Abre una conexión TCP con un host remoto.
 *
 * @param[in] host Nombre DNS o dirección IP del servidor remoto.
 * @param[in] port Puerto TCP remoto.
 *
 * @return WIFI_OK si la conexión se establece correctamente.
 * @return WIFI_TIMEOUT si el host no responde.
 * @return WIFI_ERROR si el módulo rechaza la conexión.
 */
wifi_status_t WIFI_TCPConnect(const char *host, uint16_t port);

/**
 * @brief Cierra la conexión TCP activa.
 *
 * @return WIFI_OK si la conexión se cierra correctamente.
 * @return WIFI_ERROR si no puede cerrarse la sesión.
 */
wifi_status_t WIFI_TCPClose(void);

/**
 * @brief Envía datos sin procesar por la conexión activa.
 *
 * @param[in] data Cadena de datos a transmitir.
 *
 * @return WIFI_OK si el módulo acepta el envío.
 * @return WIFI_NOT_CONNECTED si no existe transporte disponible.
 * @return WIFI_ERROR si se produce un fallo durante la transmisión.
 */
wifi_status_t WIFI_SendRaw(const char *data);

/**
 * @brief Envía una cadena JSON por la conexión activa.
 *
 * @param[in] json Cadena JSON terminada en cero.
 *
 * @return WIFI_OK si el envío se completa correctamente.
 * @return WIFI_NOT_CONNECTED si no existe transporte disponible.
 * @return WIFI_ERROR si el módulo rechaza los datos.
 *
 * @details
 * Esta función mantiene separada la generación del contenido JSON de la mecánica
 * de transmisión por el módem.
 */
wifi_status_t WIFI_SendJSON(const char *json);

/**
 * @brief Ejecuta la secuencia de reconexión WiFi definida por el driver.
 *
 * @return WIFI_OK si se recupera la conectividad.
 * @return WIFI_ERROR si la reconexión falla.
 *
 * @details
 * Centralizar la reconexión evita que varias tareas intenten recuperar el módem
 * con secuencias distintas.
 */
wifi_status_t WIFI_Reconnect(void);

/**
 * @brief Procesa tareas internas periódicas del módulo WiFi.
 *
 * @details
 * Se expone para que una tarea de FreeRTOS pueda mantener el estado del driver
 * sin bloquear al resto del sistema.
 */
void WIFI_Process(void);

/**
 * @brief Realiza una petición HTTP POST con contenido JSON.
 *
 * @param[in] host Servidor HTTP de destino.
 * @param[in] path Ruta del recurso al que se envía la petición.
 * @param[in] json Cuerpo JSON de la petición.
 *
 * @return WIFI_OK si la petición se envía correctamente.
 * @return WIFI_NOT_CONNECTED si no existe conectividad suficiente.
 * @return WIFI_ERROR si falla la conexión o el envío.
 *
 * @details
 * La función ofrece una vía de telemetría alternativa a MQTT manteniendo el
 * mismo formato de datos de aplicación.
 */
wifi_status_t WIFI_HTTP_PostJSON(const char *host,
                                 const char *path,
                                 const char *json);

/**
 * @brief Configura el identificador de cliente MQTT.
 *
 * @param[in] client_id Identificador usado por el broker para reconocer el nodo.
 *
 * @return WIFI_OK si la configuración se acepta correctamente.
 * @return WIFI_ERROR si el módulo rechaza el identificador.
 *
 * @pre `client_id` debe apuntar a una cadena válida terminada en cero.
 */
wifi_status_t WIFI_MQTT_UserConfig(const char *client_id);

/**
 * @brief Establece la conexión MQTT con el broker.
 *
 * @param[in] broker Nombre DNS o dirección IP del broker MQTT.
 * @param[in] port Puerto TCP del broker MQTT.
 *
 * @return WIFI_OK si la sesión MQTT queda establecida.
 * @return WIFI_TIMEOUT si el broker no responde.
 * @return WIFI_ERROR si falla la negociación MQTT.
 */
wifi_status_t WIFI_MQTT_Connect(const char *broker, uint16_t port);

/**
 * @brief Publica un payload en un topic MQTT.
 *
 * @param[in] topic Topic MQTT de destino.
 * @param[in] payload Contenido que se publicará.
 * @param[in] qos Nivel QoS solicitado para la publicación.
 * @param[in] retain Flag retain enviado al broker.
 *
 * @return WIFI_OK si el módulo acepta la publicación.
 * @return WIFI_NOT_CONNECTED si no existe sesión MQTT activa.
 * @return WIFI_ERROR si el comando de publicación falla.
 *
 * @details
 * La función permite a las tareas publicar telemetría sin conocer el comando AT
 * concreto ni la sintaxis interna del módulo WiFi.
 */
wifi_status_t WIFI_MQTT_Publish(const char *topic,
                                const char *payload,
                                uint8_t qos,
                                uint8_t retain);

/**
 * @brief Suscribe el cliente MQTT a un topic.
 *
 * @param[in] topic Topic MQTT que se desea recibir.
 * @param[in] qos Nivel QoS solicitado para la suscripción.
 *
 * @return WIFI_OK si la suscripción se completa correctamente.
 * @return WIFI_NOT_CONNECTED si no existe sesión MQTT activa.
 * @return WIFI_ERROR si el broker o el módulo rechazan la suscripción.
 */
wifi_status_t WIFI_MQTT_Subscribe(const char *topic,
                                  uint8_t qos);

/**
 * @brief Cierra la sesión MQTT activa.
 *
 * @return WIFI_OK si la desconexión se completa correctamente.
 * @return WIFI_ERROR si el módulo no confirma el cierre de sesión.
 */
wifi_status_t WIFI_MQTT_Disconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_H */
