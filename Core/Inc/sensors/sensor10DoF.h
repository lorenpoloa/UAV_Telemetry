/**
 * @file sensor10DoF.h
 * @brief Interfaz pública del módulo de adquisición del sensor 10DoF.
 *
 * @details
 * Este driver concentra la inicialización, recuperación y lectura de las
 * magnitudes inerciales, magnéticas y ambientales usadas por el sistema. La API
 * devuelve valores ya convertidos a unidades de aplicación para que las tareas
 * de FreeRTOS y la capa de telemetría no dependan de registros ni escalas
 * internas de cada sensor.
 */

#ifndef SENSOR10DOF_H
#define SENSOR10DOF_H

#include "main.h"
#include <stdint.h>

/**
 * @brief Estado genérico de las operaciones del módulo 10DoF.
 *
 * @details
 * La distinción entre error, timeout y bus ocupado permite aplicar estrategias
 * de recuperación distintas sin acoplar la aplicación al detalle del bus I2C.
 */
typedef enum
{
    /** @brief Operación completada correctamente. */
    SENSOR_OK = 0,

    /** @brief Error genérico durante la operación del sensor. */
    SENSOR_ERROR,

    /** @brief No se recibió respuesta dentro del tiempo esperado. */
    SENSOR_TIMEOUT,

    /** @brief El recurso compartido o bus todavía no está disponible. */
    SENSOR_BUSY
} SensorStatus_t;

/**
 * @brief Conjunto completo de medidas calculadas por el módulo 10DoF.
 *
 * @details
 * La estructura agrupa todas las magnitudes derivadas de una lectura para que la
 * aplicación pueda mover una muestra coherente entre tareas. Las unidades se
 * expresan en formato físico de alto nivel y no en cuentas ADC o registros del
 * sensor.
 */
typedef struct
{
    /** @brief Marca temporal de la muestra en milisegundos. */
    uint32_t timestamp_ms;

    /** @brief Aceleración en el eje X expresada en m/s². */
    float accel_x_ms2;

    /** @brief Aceleración en el eje Y expresada en m/s². */
    float accel_y_ms2;

    /** @brief Aceleración en el eje Z expresada en m/s². */
    float accel_z_ms2;

    /** @brief Velocidad angular en el eje X expresada en grados/s. */
    float gyro_x_dps;

    /** @brief Velocidad angular en el eje Y expresada en grados/s. */
    float gyro_y_dps;

    /** @brief Velocidad angular en el eje Z expresada en grados/s. */
    float gyro_z_dps;

    /** @brief Campo magnético en el eje X expresado en microteslas. */
    float mag_x_uT;

    /** @brief Campo magnético en el eje Y expresado en microteslas. */
    float mag_y_uT;

    /** @brief Campo magnético en el eje Z expresado en microteslas. */
    float mag_z_uT;

    /** @brief Rumbo horizontal estimado en grados. */
    float heading_deg;

    /** @brief Ángulo de roll estimado en grados. */
    float roll_deg;

    /** @brief Ángulo de pitch estimado en grados. */
    float pitch_deg;

    /** @brief Temperatura medida por el barómetro en grados Celsius. */
    float temperature_c;

    /** @brief Presión atmosférica en pascales. */
    float pressure_pa;

    /** @brief Presión atmosférica en hectopascales. */
    float pressure_hpa;

    /** @brief Presión atmosférica expresada en atmósferas. */
    float pressure_atm;

    /** @brief Altitud estimada a partir de la presión barométrica en metros. */
    float altitude_m;

} Sensor10DoF_t;

/**
 * @brief Inicializa los sensores y prepara el bus de comunicación.
 *
 * @return SENSOR_OK si todos los dispositivos quedan listos para lectura.
 * @return SENSOR_ERROR si algún dispositivo no responde o no queda configurado.
 * @return SENSOR_TIMEOUT si el bus no responde dentro del tiempo esperado.
 *
 * @details
 * Debe llamarse antes de solicitar lecturas para asegurar que las escalas,
 * registros y compensaciones iniciales están configuradas de forma coherente.
 */
SensorStatus_t Sensor10DoF_Init(void);

/**
 * @brief Lee una muestra completa del módulo 10DoF.
 *
 * @param[out] data Estructura donde se almacenan las magnitudes convertidas.
 *
 * @return SENSOR_OK si la lectura se completa correctamente.
 * @return SENSOR_ERROR si alguna magnitud no puede actualizarse.
 * @return SENSOR_TIMEOUT si un dispositivo no responde a tiempo.
 * @return SENSOR_BUSY si el módulo no puede atender la lectura en ese instante.
 *
 * @pre `data` debe apuntar a una estructura válida.
 *
 * @details
 * La función entrega una muestra agregada para reducir el acoplamiento entre las
 * tareas consumidoras y los distintos sensores físicos que forman el módulo.
 */
SensorStatus_t Sensor10DoF_Read(Sensor10DoF_t *data);

/**
 * @brief Reinicia el estado interno del módulo 10DoF.
 *
 * @details
 * Se utiliza como mecanismo de recuperación cuando la secuencia de lectura queda
 * en un estado no fiable y conviene volver a una condición conocida.
 */
void Sensor10DoF_Reset(void);

/**
 * @brief Ejecuta una recuperación manual del bus I2C usado por el módulo.
 *
 * @details
 * Esta operación está pensada para liberar el bus cuando un dispositivo mantiene
 * líneas bloqueadas tras una transacción incompleta o un reinicio inesperado.
 */
void Sensor10DoF_I2CRecover(void);

/**
 * @brief Ejecuta la calibración del módulo 10DoF.
 *
 * @details
 * La calibración reduce offsets sistemáticos, especialmente en el giróscopo, y
 * permite que las magnitudes publicadas sean estables cuando el sistema está en
 * reposo.
 */
void Sensor10DoF_Calibrate(void);
#endif
