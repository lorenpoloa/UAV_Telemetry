/**
 * @file wifi.c
 * @brief Driver WiFi/MQTT basado en comandos AT con RX DMA circular y TX DMA sincronizado.
 *
 * @details
 * La arquitectura separa recepción y transmisión en dos canales de hardware:
 * recepción DMA circular permanente para no perder respuestas asíncronas del
 * módulo WiFi, y transmisión DMA sincronizada mediante semáforo para que las
 * tareas FreeRTOS esperen sin ocupar CPU durante el envío físico por UART.
 */

#include "cmsis_os2.h"
#include "communication/wifi.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/** @brief UART asociada al módulo WiFi inicializado. */
static UART_HandleTypeDef *wifi_uart;

/** @brief Estado lógico actual de la conexión WiFi/MQTT. */
static wifi_state_t wifi_state = WIFI_DISCONNECTED;

/** @brief Tamaño del búfer circular usado por la recepción DMA continua. */
#define RING_BUFFER_SIZE 2048

/** @brief Búfer circular donde el DMA deposita respuestas y eventos del módulo WiFi. */
static uint8_t ring_buffer[RING_BUFFER_SIZE];

/** @brief Índice de lectura software dentro del búfer circular DMA. */
static uint32_t read_index = 0;

/**
 * @brief Semáforo usado para despertar a la tarea cuando termina la transmisión DMA.
 *
 * @details
 * El semáforo evita esperas activas durante el envío de comandos AT y mantiene
 * determinista la transición entre comando enviado y búsqueda de respuesta.
 */
static osSemaphoreId_t wifi_tx_sem = NULL;

/** @brief SSID almacenado para permitir reconexión automática. */
static char cached_ssid[64] = {0};
/** @brief Contraseña almacenada para permitir reconexión automática. */
static char cached_pass[64] = {0};
/** @brief Broker MQTT almacenado para reconstruir la sesión si se pierde. */
static char cached_broker[64] = {0};
/** @brief Puerto MQTT almacenado para reconstruir la sesión si se pierde. */
static uint16_t cached_port = 1883;
/** @brief Identificador MQTT almacenado para reconfigurar el cliente tras una reconexión. */
static char cached_client_id[64] = {0};

/**
 * @brief Obtiene la posición actual de escritura del DMA de recepción.
 *
 * @return Índice donde el DMA escribirá el siguiente byte dentro del búfer circular.
 *
 * @details
 * La HAL expone el contador restante del DMA, por lo que se convierte a índice
 * de escritura para comparar directamente con el índice software de lectura.
 */
static uint32_t get_dma_write_index(void)
{
    return RING_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(wifi_uart->hdmarx);
}

/**
 * @brief Descarta las respuestas acumuladas hasta el instante actual.
 *
 * @details
 * Antes de enviar un comando nuevo se sincroniza el índice de lectura con el DMA
 * para que la búsqueda posterior se asocie a la respuesta del comando actual y
 * no a mensajes antiguos que hayan quedado en el búfer.
 */
static void flush_ring_buffer(void)
{
    read_index = get_dma_write_index();
}

/**
 * @brief Busca una cadena objetivo en los datos recibidos desde la última lectura.
 *
 * @param[in] target Texto que debe aparecer en la respuesta del módulo WiFi.
 * @retval 1 Se encontró la cadena y se avanzó el índice de lectura.
 * @retval 0 La cadena no está disponible en los datos pendientes.
 *
 * @details
 * La búsqueda se realiza sobre una copia lineal para poder usar `strstr()` aunque
 * el DMA esté trabajando sobre un búfer circular. Solo se consume hasta el final
 * de la coincidencia para no descartar respuestas posteriores.
 */
static int search_in_ring_buffer(const char *target)
{
    uint32_t write_index = get_dma_write_index();
    if (read_index == write_index) return 0;

    static char eval_buf[RING_BUFFER_SIZE];
    uint32_t eval_idx = 0;
    uint32_t temp_read = read_index;

    while (temp_read != write_index && eval_idx < (sizeof(eval_buf) - 1))
    {
        eval_buf[eval_idx++] = ring_buffer[temp_read];
        temp_read = (temp_read + 1) % RING_BUFFER_SIZE;
    }
    eval_buf[eval_idx] = '\0';

    char *match_ptr = strstr(eval_buf, target);
    if (match_ptr)
    {
        uint32_t bytes_consumed = (match_ptr - eval_buf) + strlen(target);
        read_index = (read_index + bytes_consumed) % RING_BUFFER_SIZE;
        return 1;
    }
    return 0;
}

/**
 * @brief Callback HAL ejecutado al finalizar una transmisión UART por DMA.
 *
 * @param[in] huart UART que ha completado la transmisión.
 *
 * @details
 * La ISR solo libera el semáforo de transmisión para despertar a la tarea que
 * envió el comando. Mantener el callback mínimo reduce latencia y evita procesar
 * respuestas AT dentro de contexto de interrupción.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == wifi_uart->Instance)
    {
        if (wifi_tx_sem != NULL)
        {
            osSemaphoreRelease(wifi_tx_sem);
        }
    }
}

/**
 * @brief Envía un comando AT y espera una respuesta esperada.
 *
 * @param[in] cmd Comando AT completo, incluyendo terminación `\r\n`.
 * @param[in] expected Cadena que confirma la operación esperada.
 * @param[in] timeout Tiempo máximo de espera en ticks del kernel.
 * @retval WIFI_OK La respuesta esperada se recibió dentro del tiempo límite.
 * @retval WIFI_ERROR Falló la transmisión o el módulo devolvió `ERROR`.
 * @retval WIFI_TIMEOUT No llegó respuesta concluyente antes del timeout.
 *
 * @details
 * La función limpia errores UART y sincroniza el búfer RX antes de transmitir
 * para asociar la respuesta al comando actual. El envío usa DMA para no bloquear
 * la CPU, pero la tarea queda dormida hasta que el hardware confirma el fin de
 * transmisión.
 */
static wifi_status_t wifi_send_cmd(const char *cmd,
                                   const char *expected,
                                   uint32_t timeout)
{
    __HAL_UART_CLEAR_OREFLAG(wifi_uart);
    __HAL_UART_CLEAR_NEFLAG(wifi_uart);
    __HAL_UART_CLEAR_FEFLAG(wifi_uart);

    flush_ring_buffer();

    osSemaphoreAcquire(wifi_tx_sem, 0);

    if (HAL_UART_Transmit_DMA(wifi_uart, (uint8_t*)cmd, strlen(cmd)) != HAL_OK)
    {
        return WIFI_ERROR;
    }

    if (osSemaphoreAcquire(wifi_tx_sem, 500) != osOK)
    {
        HAL_UART_AbortTransmit(wifi_uart);
        return WIFI_ERROR;
    }

    uint32_t start_tick = osKernelGetTickCount();
    while ((osKernelGetTickCount() - start_tick) < timeout)
    {
        if (search_in_ring_buffer(expected)) return WIFI_OK;
        if (search_in_ring_buffer("ERROR"))  return WIFI_ERROR;
        osDelay(5);
    }

    return WIFI_TIMEOUT;
}

/**
 * @brief Inicializa el driver WiFi y prepara la recepción DMA circular.
 *
 * @param[in] huart UART conectada al módulo WiFi.
 * @retval WIFI_OK El módulo respondió y quedó en modo estación.
 * @retval WIFI_ERROR No se pudo comunicar o configurar el módulo.
 *
 * @details
 * La recepción DMA se arranca antes de probar el módulo para capturar cualquier
 * respuesta o evento que emita el ESP. Se desactivan interrupciones de mitad y
 * fin de recepción porque el consumo se realiza por software consultando el
 * índice del DMA.
 */
wifi_status_t WIFI_Init(UART_HandleTypeDef *huart)
{
    wifi_uart = huart;

    if (wifi_tx_sem == NULL)
    {
        wifi_tx_sem = osSemaphoreNew(1, 0, NULL);
    }

    HAL_UART_Receive_DMA(wifi_uart, ring_buffer, RING_BUFFER_SIZE);

    __HAL_DMA_DISABLE_IT(wifi_uart->hdmarx, DMA_IT_HT | DMA_IT_TC);

    if(WIFI_Test() != WIFI_OK) return WIFI_ERROR;

    wifi_send_cmd("ATE0\r\n", "OK", 1000);

    if(WIFI_SetStationMode() != WIFI_OK) return WIFI_ERROR;

    wifi_state = WIFI_DISCONNECTED;
    return WIFI_OK;
}

/**
 * @brief Comprueba que el módulo WiFi responde a comandos AT.
 *
 * @retval WIFI_OK El módulo respondió con `OK`.
 * @retval WIFI_ERROR Error devuelto por el módulo o fallo de transmisión.
 * @retval WIFI_TIMEOUT No hubo respuesta dentro del tiempo límite.
 */
wifi_status_t WIFI_Test(void) { return wifi_send_cmd("AT\r\n", "OK", 1000); }

/**
 * @brief Reinicia el módulo WiFi mediante comando AT.
 *
 * @retval WIFI_OK El módulo confirmó el reinicio con `ready`.
 * @retval WIFI_ERROR Error devuelto por el módulo o fallo de transmisión.
 * @retval WIFI_TIMEOUT No hubo respuesta dentro del tiempo límite.
 */
wifi_status_t WIFI_Reset(void) { wifi_state = WIFI_DISCONNECTED; return wifi_send_cmd("AT+RST\r\n", "ready", 4000); }

/**
 * @brief Configura el módulo WiFi en modo estación.
 *
 * @retval WIFI_OK Configuración aceptada.
 * @retval WIFI_ERROR Error devuelto por el módulo o fallo de transmisión.
 * @retval WIFI_TIMEOUT No hubo respuesta dentro del tiempo límite.
 */
wifi_status_t WIFI_SetStationMode(void) { return wifi_send_cmd("AT+CWMODE=1\r\n", "OK", 1000); }

/**
 * @brief Conecta el módulo a un punto de acceso WiFi.
 *
 * @param[in] ssid Nombre de la red WiFi.
 * @param[in] pass Contraseña de la red WiFi.
 * @retval WIFI_OK Conexión establecida y dirección IP obtenida.
 * @retval WIFI_ERROR Error de conexión o transmisión.
 * @retval WIFI_TIMEOUT No se obtuvo confirmación dentro del tiempo límite.
 *
 * @details
 * Las credenciales se almacenan localmente para que WIFI_Reconnect() pueda
 * reconstruir la conexión sin depender de que la aplicación vuelva a pasarlas.
 */
wifi_status_t WIFI_ConnectAP(const char *ssid, const char *pass)
{
    static char cmd[256];
    strncpy(cached_ssid, ssid, sizeof(cached_ssid)-1);
    strncpy(cached_pass, pass, sizeof(cached_pass)-1);

    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pass);
    wifi_status_t st = wifi_send_cmd(cmd, "WIFI GOT IP", 12000);

    if(st == WIFI_OK) wifi_state = WIFI_CONNECTED;
    else              wifi_state = WIFI_DISCONNECTED;

    return st;
}

/**
 * @brief Desconecta el módulo del punto de acceso actual.
 *
 * @retval WIFI_OK Desconexión aceptada.
 * @retval WIFI_ERROR Error devuelto por el módulo o fallo de transmisión.
 * @retval WIFI_TIMEOUT No hubo respuesta dentro del tiempo límite.
 */
wifi_status_t WIFI_DisconnectAP(void) { wifi_state = WIFI_DISCONNECTED; return wifi_send_cmd("AT+CWQAP\r\n", "OK", 3000); }

/**
 * @brief Devuelve el estado lógico actual del módulo WiFi/MQTT.
 *
 * @return Estado interno de conexión.
 */
wifi_state_t  WIFI_GetState(void) { return wifi_state; }

/**
 * @brief Configura el identificador de cliente MQTT.
 *
 * @param[in] client_id Identificador usado por el cliente MQTT del módulo.
 * @retval WIFI_OK Configuración aceptada.
 * @retval WIFI_ERROR Error devuelto por el módulo o fallo de transmisión.
 * @retval WIFI_TIMEOUT No hubo respuesta dentro del tiempo límite.
 *
 * @details
 * El identificador se guarda para poder repetir la configuración después de una
 * reconexión WiFi o reinicio del módulo.
 */
wifi_status_t WIFI_MQTT_UserConfig(const char *client_id)
{
    static char cmd[256];
    strncpy(cached_client_id, client_id, sizeof(cached_client_id)-1);
    snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"\",\"\",0,0,\"\"\r\n", client_id);
    return wifi_send_cmd(cmd, "OK", 3000);
}

/**
 * @brief Abre la conexión MQTT contra el broker indicado.
 *
 * @param[in] broker Dirección IP o nombre del broker MQTT.
 * @param[in] port Puerto TCP del broker MQTT.
 * @retval WIFI_OK Conexión MQTT establecida.
 * @retval WIFI_ERROR Error devuelto por el módulo o fallo de transmisión.
 * @retval WIFI_TIMEOUT No hubo respuesta dentro del tiempo límite.
 *
 * @details
 * El broker y puerto se conservan para recuperar la sesión tras pérdidas de red
 * sin obligar al resto de tareas a conocer la secuencia AT de reconexión.
 */
wifi_status_t WIFI_MQTT_Connect(const char *broker, uint16_t port)
{
    static char cmd[256];
    strncpy(cached_broker, broker, sizeof(cached_broker)-1);
    cached_port = port;

    snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%d,0\r\n", broker, port);
    wifi_status_t st = wifi_send_cmd(cmd, "OK", 6000);
    if (st == WIFI_OK) wifi_state = WIFI_MQTT_CONNECTED;
    return st;
}

/**
 * @brief Publica un payload en un topic MQTT.
 *
 * @param[in] topic Topic MQTT de destino.
 * @param[in] payload Cadena que se publica como contenido del mensaje.
 * @param[in] qos Nivel QoS solicitado al módulo WiFi.
 * @param[in] retain Flag retain del mensaje MQTT.
 * @retval WIFI_OK Publicación aceptada por el módulo.
 * @retval WIFI_ERROR Error devuelto por el módulo o fallo de transmisión.
 * @retval WIFI_TIMEOUT No hubo confirmación dentro del tiempo límite.
 *
 * @details
 * Si la publicación falla, el estado se degrada a WIFI_CONNECTED porque la red
 * puede seguir activa aunque la sesión MQTT haya quedado inválida. Esta decisión
 * permite que la capa superior intente reconstruir solo MQTT antes de reiniciar
 * toda la conexión WiFi.
 */
wifi_status_t WIFI_MQTT_Publish(const char *topic, const char *payload, uint8_t qos, uint8_t retain)
{
    static char large_cmd_buffer[512];

    snprintf(large_cmd_buffer, sizeof(large_cmd_buffer),
        "AT+MQTTPUB=0,\"%s\",\"%s\",%d,%d\r\n", topic, payload, qos, retain);

    wifi_status_t st = wifi_send_cmd(large_cmd_buffer, "OK", 1500);

    if (st != WIFI_OK) wifi_state = WIFI_CONNECTED;
    return st;
}

/**
 * @brief Cierra la sesión MQTT actual.
 *
 * @retval WIFI_OK Sesión MQTT liberada.
 * @retval WIFI_ERROR Error devuelto por el módulo o fallo de transmisión.
 * @retval WIFI_TIMEOUT No hubo respuesta dentro del tiempo límite.
 */
wifi_status_t WIFI_MQTT_Disconnect(void)
{
    if (wifi_state == WIFI_MQTT_CONNECTED) {
        wifi_state = WIFI_CONNECTED;
    }
    return wifi_send_cmd("AT+MQTTCLEAN=0\r\n", "OK", 3000);
}

/**
 * @brief Reconstruye la conexión WiFi y MQTT usando la configuración almacenada.
 *
 * @retval WIFI_OK Reconexión completada hasta MQTT.
 * @retval WIFI_ERROR No se pudo restaurar WiFi o MQTT.
 *
 * @details
 * La rutina distingue entre pérdida total de WiFi y pérdida solo de MQTT para
 * no reiniciar más capas de las necesarias. Esto reduce tiempo de recuperación
 * y evita cortes innecesarios cuando el punto de acceso sigue disponible.
 */
wifi_status_t WIFI_Reconnect(void)
{
    if (wifi_state == WIFI_DISCONNECTED)
    {
        WIFI_Reset();
        wifi_send_cmd("ATE0\r\n", "OK", 1000);
        WIFI_SetStationMode();
        if (WIFI_ConnectAP(cached_ssid, cached_pass) != WIFI_OK) return WIFI_ERROR;
    }
    if (wifi_state == WIFI_CONNECTED)
    {
        if (WIFI_MQTT_UserConfig(cached_client_id) == WIFI_OK)
        {
            if (WIFI_MQTT_Connect(cached_broker, cached_port) == WIFI_OK) return WIFI_OK;
        }
    }
    return WIFI_ERROR;
}
