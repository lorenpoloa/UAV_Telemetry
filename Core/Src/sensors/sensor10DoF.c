/**
 * @file sensor10DoF.c
 * @brief Driver de adquisición para IMU 10DoF mediante bus I2C.
 * @author Lorenzo
 * @date 27 may 2026
 *
 * @details
 * Este módulo agrupa la inicialización, recuperación y lectura de acelerómetro,
 * magnetómetro, giróscopo y barómetro. La lectura se entrega en unidades físicas
 * para que las capas superiores no dependan de registros, escalas ni coeficientes
 * internos de cada sensor.
 */

#include "sensors/sensor10DoF.h"
#include "i2c.h"
#include "gpio.h"
#include <math.h>
#include <string.h>

/** @brief Handle I2C generado por STM32CubeMX y compartido por los sensores 10DoF. */
extern I2C_HandleTypeDef hi2c1;

/** @defgroup Sensor10DoF_Addresses Direcciones I2C
 *  @brief Direcciones de 7 bits desplazadas para el formato esperado por la HAL.
 *  @{
 */
#define ACC_ADDR   (0x19 << 1) /**< Dirección I2C del acelerómetro. */
#define MAG_ADDR   (0x1E << 1) /**< Dirección I2C del magnetómetro. */
#define GYR_ADDR   (0x6B << 1) /**< Dirección I2C del giróscopo. */
#define BMP_ADDR   (0x77 << 1) /**< Dirección I2C del barómetro BMP180. */
/** @} */

/** @defgroup Sensor10DoF_Constants Constantes de conversión
 *  @brief Factores usados para convertir cuentas crudas a magnitudes físicas.
 *  @{
 */
#define G_CONST             9.80665f     /**< Gravedad estándar usada para convertir g a m/s². */

#define ACC_SENS_2G         0.001f       /**< Sensibilidad del acelerómetro en g/LSB para ±2 g. */
#define GYRO_SENS_250DPS    0.00875f     /**< Sensibilidad del giróscopo en dps/LSB para ±250 dps. */

#define MAG_XY_LSB_GAUSS    1100.0f      /**< Sensibilidad X/Y del magnetómetro en LSB/gauss. */
#define MAG_Z_LSB_GAUSS     980.0f       /**< Sensibilidad Z del magnetómetro en LSB/gauss. */
#define GAUSS_TO_UT         100.0f       /**< Factor de conversión de gauss a microteslas. */

#define BMP180_OSS          3            /**< Sobremuestreo del BMP180 priorizando precisión frente a latencia. */
#define BMP180_SEA_HPA      1013.25f     /**< Presión estándar al nivel del mar para estimar altitud relativa. */
/** @} */

/** @brief Coeficientes de calibración leídos de la EEPROM interna del BMP180. */
static int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
/** @brief Coeficientes sin signo de calibración del BMP180. */
static uint16_t AC4, AC5, AC6;

/** @brief Offset estático del eje X del giróscopo medido durante la calibración. */
static float gyro_x_offset = 0.0f;
/** @brief Offset estático del eje Y del giróscopo medido durante la calibración. */
static float gyro_y_offset = 0.0f;
/** @brief Offset estático del eje Z del giróscopo medido durante la calibración. */
static float gyro_z_offset = 0.0f;

/**
 * @brief Lee registros consecutivos de un dispositivo I2C.
 *
 * @param[in] dev Dirección I2C desplazada del dispositivo.
 * @param[in] reg Registro inicial de lectura.
 * @param[out] buf Buffer de destino.
 * @param[in] len Número de bytes a leer.
 * @return Estado HAL de la transacción.
 *
 * @details
 * La función encapsula la HAL para mantener en un único punto el timeout y el
 * tamaño de dirección de memoria usado por todos los sensores del módulo.
 */
static HAL_StatusTypeDef I2C_Read(uint16_t dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/**
 * @brief Escribe un byte en un registro de un dispositivo I2C.
 *
 * @param[in] dev Dirección I2C desplazada del dispositivo.
 * @param[in] reg Registro de destino.
 * @param[in] val Valor que se escribe.
 * @return Estado HAL de la transacción.
 *
 * @details
 * Centralizar las escrituras evita duplicar parámetros de HAL y facilita que la
 * secuencia de inicialización sea legible a nivel de registros relevantes.
 */
static HAL_StatusTypeDef I2C_Write(uint16_t dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/**
 * @brief Intenta recuperar el bus I2C generando pulsos manuales en SCL.
 *
 * @details
 * Algunos esclavos I2C pueden dejar SDA retenida tras un reset o una transferencia
 * interrumpida. La recuperación conmuta temporalmente SCL/SDA como GPIO para
 * liberar el bus antes de reinicializar el periférico I2C.
 */
void Sensor10DoF_I2CRecover(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_I2C_DeInit(&hi2c1);

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    for(int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    MX_I2C1_Init();
}

/**
 * @brief Configura el acelerómetro para lectura continua en rango ±2 g.
 *
 * @retval SENSOR_OK Configuración aplicada.
 * @retval SENSOR_ERROR Falló alguna escritura I2C.
 */
static SensorStatus_t ACC_Init(void)
{
    if(I2C_Write(ACC_ADDR, 0x20, 0x57) != HAL_OK) return SENSOR_ERROR;
    if(I2C_Write(ACC_ADDR, 0x23, 0x08) != HAL_OK) return SENSOR_ERROR;

    return SENSOR_OK;
}

/**
 * @brief Configura el magnetómetro para medición continua.
 *
 * @retval SENSOR_OK Configuración aplicada.
 * @retval SENSOR_ERROR Falló alguna escritura I2C.
 */
static SensorStatus_t MAG_Init(void)
{
    if(I2C_Write(MAG_ADDR, 0x00, 0x14) != HAL_OK) return SENSOR_ERROR;
    if(I2C_Write(MAG_ADDR, 0x01, 0x20) != HAL_OK) return SENSOR_ERROR;
    if(I2C_Write(MAG_ADDR, 0x02, 0x00) != HAL_OK) return SENSOR_ERROR;

    return SENSOR_OK;
}

/**
 * @brief Configura el giróscopo para habilitar sus tres ejes.
 *
 * @retval SENSOR_OK Configuración aplicada.
 * @retval SENSOR_ERROR Falló alguna escritura I2C.
 */
static SensorStatus_t GYR_Init(void)
{
    if(I2C_Write(GYR_ADDR, 0x20, 0x0F) != HAL_OK) return SENSOR_ERROR;
    if(I2C_Write(GYR_ADDR, 0x23, 0x00) != HAL_OK) return SENSOR_ERROR;

    return SENSOR_OK;
}

/**
 * @brief Lee y almacena los coeficientes de calibración del BMP180.
 *
 * @retval SENSOR_OK Coeficientes cargados correctamente.
 * @retval SENSOR_ERROR No se pudo leer la EEPROM de calibración.
 *
 * @details
 * Los coeficientes son necesarios para compensar temperatura y presión. Se leen
 * una sola vez durante la inicialización porque son constantes de fábrica.
 */
static SensorStatus_t BMP180_ReadCalibration(void)
{
    uint8_t b[22];

    if(I2C_Read(BMP_ADDR, 0xAA, b, 22) != HAL_OK)
        return SENSOR_ERROR;

    AC1 = (int16_t)((b[0] << 8) | b[1]);
    AC2 = (int16_t)((b[2] << 8) | b[3]);
    AC3 = (int16_t)((b[4] << 8) | b[5]);
    AC4 = (uint16_t)((b[6] << 8) | b[7]);
    AC5 = (uint16_t)((b[8] << 8) | b[9]);
    AC6 = (uint16_t)((b[10] << 8) | b[11]);
    B1  = (int16_t)((b[12] << 8) | b[13]);
    B2  = (int16_t)((b[14] << 8) | b[15]);
    MB  = (int16_t)((b[16] << 8) | b[17]);
    MC  = (int16_t)((b[18] << 8) | b[19]);
    MD  = (int16_t)((b[20] << 8) | b[21]);

    return SENSOR_OK;
}

/**
 * @brief Lee la temperatura cruda del BMP180.
 *
 * @param[out] UT Temperatura sin compensar.
 * @retval SENSOR_OK Lectura completada.
 * @retval SENSOR_ERROR Falló la transacción I2C.
 */
static SensorStatus_t BMP180_ReadUT(int32_t *UT)
{
    uint8_t b[2];

    if(I2C_Write(BMP_ADDR, 0xF4, 0x2E) != HAL_OK)
        return SENSOR_ERROR;

    HAL_Delay(5);

    if(I2C_Read(BMP_ADDR, 0xF6, b, 2) != HAL_OK)
        return SENSOR_ERROR;

    *UT = ((int32_t)b[0] << 8) | b[1];

    return SENSOR_OK;
}

/**
 * @brief Lee la presión cruda del BMP180.
 *
 * @param[out] UP Presión sin compensar ajustada por el sobremuestreo configurado.
 * @retval SENSOR_OK Lectura completada.
 * @retval SENSOR_ERROR Falló la transacción I2C.
 *
 * @details
 * El tiempo de espera depende del sobremuestreo. Con OSS alto se reduce el ruido
 * de presión a costa de aumentar la latencia de conversión.
 */
static SensorStatus_t BMP180_ReadUP(int32_t *UP)
{
    uint8_t b[3];

    if(I2C_Write(BMP_ADDR, 0xF4, 0x34 + (BMP180_OSS << 6)) != HAL_OK)
        return SENSOR_ERROR;

#if BMP180_OSS == 0
    HAL_Delay(5);
#elif BMP180_OSS == 1
    HAL_Delay(8);
#elif BMP180_OSS == 2
    HAL_Delay(14);
#else
    HAL_Delay(26);
#endif

    if(I2C_Read(BMP_ADDR, 0xF6, b, 3) != HAL_OK)
        return SENSOR_ERROR;

    *UP = ((((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2]) >> (8 - BMP180_OSS));

    return SENSOR_OK;
}

/**
 * @brief Calcula el offset estático del giróscopo.
 *
 * @retval SENSOR_OK Calibración completada.
 * @retval SENSOR_ERROR No se pudieron leer las muestras necesarias.
 *
 * @details
 * El promedio de varias muestras en reposo compensa el sesgo observado en los
 * ejes del giróscopo. Esta corrección es necesaria para que las lecturas en
 * quietud se aproximen a cero y no propaguen deriva a la telemetría.
 *
 * @pre El sensor debe permanecer inmóvil durante la calibración.
 */
static SensorStatus_t Sensor10DoF_CalibrateGyro_Internal(void)
{
    uint8_t b[6];
    int16_t raw_x, raw_y, raw_z;

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    const int samples = 500;

    HAL_Delay(500);

    for(int i = 0; i < samples; i++)
    {
        if(I2C_Read(GYR_ADDR, 0x28 | 0x80, b, 6) != HAL_OK)
            return SENSOR_ERROR;

        raw_x = (int16_t)((b[1] << 8) | b[0]);
        raw_y = (int16_t)((b[3] << 8) | b[2]);
        raw_z = (int16_t)((b[5] << 8) | b[4]);

        sum_x += raw_x * GYRO_SENS_250DPS;
        sum_y += raw_y * GYRO_SENS_250DPS;
        sum_z += raw_z * GYRO_SENS_250DPS;

        HAL_Delay(2);
    }

    gyro_x_offset = sum_x / samples;
    gyro_y_offset = sum_y / samples;
    gyro_z_offset = sum_z / samples;

    return SENSOR_OK;
}

/**
 * @brief Inicializa todos los sensores del módulo 10DoF.
 *
 * @retval SENSOR_OK Todos los dispositivos respondieron y quedaron configurados.
 * @retval SENSOR_ERROR Algún dispositivo no respondió, no pudo configurarse o falló la calibración.
 *
 * @details
 * La función verifica primero la presencia de todos los esclavos I2C antes de
 * configurar registros. Esto permite detectar fallos de cableado o bloqueo de
 * bus antes de producir lecturas parciales incoherentes.
 */
SensorStatus_t Sensor10DoF_Init(void)
{
    HAL_Delay(100);

    if(HAL_I2C_IsDeviceReady(&hi2c1, ACC_ADDR, 3, 100) != HAL_OK)
        return SENSOR_ERROR;

    if(HAL_I2C_IsDeviceReady(&hi2c1, MAG_ADDR, 3, 100) != HAL_OK)
        return SENSOR_ERROR;

    if(HAL_I2C_IsDeviceReady(&hi2c1, GYR_ADDR, 3, 100) != HAL_OK)
        return SENSOR_ERROR;

    if(HAL_I2C_IsDeviceReady(&hi2c1, BMP_ADDR, 3, 100) != HAL_OK)
        return SENSOR_ERROR;

    if(ACC_Init() != SENSOR_OK) return SENSOR_ERROR;
    if(MAG_Init() != SENSOR_OK) return SENSOR_ERROR;
    if(GYR_Init() != SENSOR_OK) return SENSOR_ERROR;
    if(BMP180_ReadCalibration() != SENSOR_OK) return SENSOR_ERROR;

    if(Sensor10DoF_CalibrateGyro_Internal() != SENSOR_OK)
        return SENSOR_ERROR;

    return SENSOR_OK;
}

/**
 * @brief Lee todos los sensores y rellena una muestra 10DoF completa.
 *
 * @param[out] d Estructura de destino con aceleración, orientación, campo magnético,
 *               velocidad angular, temperatura, presión y altitud.
 * @retval SENSOR_OK Muestra actualizada correctamente.
 * @retval SENSOR_ERROR Error de argumento, comunicación o compensación inválida.
 *
 * @details
 * La función entrega magnitudes ya convertidas para aislar al resto del proyecto
 * de escalas de registros. También calcula roll, pitch y heading compensado por
 * inclinación para que la interfaz pueda consumir directamente valores físicos.
 *
 * @pre Sensor10DoF_Init() debe haberse ejecutado correctamente.
 */
SensorStatus_t Sensor10DoF_Read(Sensor10DoF_t *d)
{
    uint8_t b[6];

    int16_t raw_x, raw_y, raw_z;
    int32_t UT, UP;
    int32_t X1, X2, B5, B6, X3, B3, p;
    uint32_t B4, B7;

    if(d == NULL)
        return SENSOR_ERROR;

    memset(d, 0, sizeof(Sensor10DoF_t));
    d->timestamp_ms = HAL_GetTick();

    if(I2C_Read(ACC_ADDR, 0x28 | 0x80, b, 6) != HAL_OK)
        return SENSOR_ERROR;

    raw_x = (int16_t)((b[1] << 8) | b[0]);
    raw_y = (int16_t)((b[3] << 8) | b[2]);
    raw_z = (int16_t)((b[5] << 8) | b[4]);

    raw_x >>= 4;
    raw_y >>= 4;
    raw_z >>= 4;

    d->accel_x_ms2 = raw_x * ACC_SENS_2G * G_CONST;
    d->accel_y_ms2 = raw_y * ACC_SENS_2G * G_CONST;
    d->accel_z_ms2 = raw_z * ACC_SENS_2G * G_CONST;

    d->roll_deg = atan2f(d->accel_y_ms2, d->accel_z_ms2) * 57.2957795f;
    d->pitch_deg = atan2f(
        -d->accel_x_ms2,
        sqrtf((d->accel_y_ms2 * d->accel_y_ms2) + (d->accel_z_ms2 * d->accel_z_ms2))
    ) * 57.2957795f;

    if(I2C_Read(MAG_ADDR, 0x03, b, 6) != HAL_OK)
        return SENSOR_ERROR;

    raw_x = (int16_t)((b[0] << 8) | b[1]);
    raw_z = (int16_t)((b[2] << 8) | b[3]);
    raw_y = (int16_t)((b[4] << 8) | b[5]);

    d->mag_x_uT = ((float)raw_x / MAG_XY_LSB_GAUSS) * GAUSS_TO_UT;
    d->mag_y_uT = ((float)raw_y / MAG_XY_LSB_GAUSS) * GAUSS_TO_UT;
    d->mag_z_uT = ((float)raw_z / MAG_Z_LSB_GAUSS) * GAUSS_TO_UT;

    float roll_rad = d->roll_deg * 0.0174532925f;
    float pitch_rad = d->pitch_deg * 0.0174532925f;

    float mx = d->mag_x_uT;
    float my = d->mag_y_uT;
    float mz = d->mag_z_uT;

    float mag_x_comp = mx * cosf(pitch_rad) + mz * sinf(pitch_rad);

    float mag_y_comp =
        mx * sinf(roll_rad) * sinf(pitch_rad) +
        my * cosf(roll_rad) -
        mz * sinf(roll_rad) * cosf(pitch_rad);

    d->heading_deg = atan2f(mag_y_comp, mag_x_comp) * 57.2957795f;

    if(d->heading_deg < 0.0f)
        d->heading_deg += 360.0f;

    if(I2C_Read(GYR_ADDR, 0x28 | 0x80, b, 6) != HAL_OK)
        return SENSOR_ERROR;

    raw_x = (int16_t)((b[1] << 8) | b[0]);
    raw_y = (int16_t)((b[3] << 8) | b[2]);
    raw_z = (int16_t)((b[5] << 8) | b[4]);

    d->gyro_x_dps = raw_x * GYRO_SENS_250DPS - gyro_x_offset;
    d->gyro_y_dps = raw_y * GYRO_SENS_250DPS - gyro_y_offset;
    d->gyro_z_dps = raw_z * GYRO_SENS_250DPS - gyro_z_offset;

    if(BMP180_ReadUT(&UT) != SENSOR_OK)
        return SENSOR_ERROR;

    if(BMP180_ReadUP(&UP) != SENSOR_OK)
        return SENSOR_ERROR;

    X1 = ((UT - AC6) * AC5) >> 15;
    X2 = ((int32_t)MC << 11) / (X1 + MD);
    B5 = X1 + X2;

    d->temperature_c = ((B5 + 8) >> 4) / 10.0f;

    B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;

    B3 = (((((int32_t)AC1 * 4 + X3) << BMP180_OSS) + 2) >> 2);

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;

    B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (50000 >> BMP180_OSS);

    if(B4 == 0)
        return SENSOR_ERROR;

    if(B7 < 0x80000000)
        p = (B7 << 1) / B4;
    else
        p = (B7 / B4) << 1;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

    p = p + ((X1 + X2 + 3791) >> 4);

    d->pressure_pa  = p;
    d->pressure_hpa = p / 100.0f;
    d->pressure_atm = d->pressure_hpa / BMP180_SEA_HPA;

    d->altitude_m = 44330.0f *
        (1.0f - powf(d->pressure_hpa / BMP180_SEA_HPA, 0.1903f));

    return SENSOR_OK;
}

/**
 * @brief Recupera el bus I2C y reinicializa el módulo 10DoF.
 *
 * @details
 * Se proporciona como rutina de recuperación ante errores de comunicación. La
 * función intenta dejar el bus y los sensores en un estado operativo sin exponer
 * a las capas superiores los detalles eléctricos de recuperación.
 */
void Sensor10DoF_Reset(void)
{
    Sensor10DoF_I2CRecover();
    Sensor10DoF_Init();
}

/**
 * @brief Ejecuta una nueva calibración del giróscopo.
 *
 * @details
 * Permite recalcular los offsets si cambia la temperatura, la alimentación o se
 * observa deriva tras el arranque. El sensor debe permanecer inmóvil durante la
 * llamada para que el promedio represente únicamente el sesgo estático.
 */
void Sensor10DoF_Calibrate(void)
{
    Sensor10DoF_CalibrateGyro_Internal();
}
