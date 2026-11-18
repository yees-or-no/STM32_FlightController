/**
 * bmp280.h - driver barometer BMP280 (I2C, STM32 HAL).
 *
 * Satu bus I2C1 dengan MPU6050 (alamat beda, tidak bentrok).
 * SDO ke GND -> 0x76, SDO ke VCC -> 0x77.
 *
 * Catatan: BME280 (chip ID 0x60) juga punya register tekanan yang sama,
 * tapi driver ini hanya menerima ID BMP280 (0x58). Cek label sensormu.
 *
 * Letakkan di Core/Inc.
 */
#ifndef BMP280_H
#define BMP280_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BMP280_ADDR
#define BMP280_ADDR            (0x76 << 1)
#endif

/* Koefisien low-pass ketinggian (0..1). Makin kecil = makin halus tapi lambat. */
#ifndef BMP280_ALT_LPF_ALPHA
#define BMP280_ALT_LPF_ALPHA   0.1f
#endif

typedef struct {
    /* koefisien kalibrasi dari sensor */
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    int32_t  t_fine;

    /* hasil */
    float temperature_C;
    float pressure_Pa;
    float p0_Pa;          /* tekanan referensi di permukaan tanah       */
    float altitude_m;     /* ketinggian mentah relatif terhadap p0      */
    float altitude_f;     /* ketinggian setelah low-pass                */
    uint8_t alt_valid;    /* 1 setelah altitude_f diisi pertama kali    */
} BMP280_t;

/* Cek ID, reset, baca koefisien, konfigurasi (suhu x2, tekanan x16,
 * IIR 4, mode normal). */
HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280_t *dev);

/* Baca suhu + tekanan (+ ketinggian jika p0 sudah diset). */
HAL_StatusTypeDef BMP280_Read(I2C_HandleTypeDef *hi2c, BMP280_t *dev);

/* Rata-ratakan n pembacaan sebagai p0 (tekanan "nol"). Blocking sekitar
 * n * 30 ms. Panggil saat diam di tanah. */
HAL_StatusTypeDef BMP280_CalibrateGround(I2C_HandleTypeDef *hi2c, BMP280_t *dev,
                                         uint16_t n);

#ifdef __cplusplus
}
#endif
#endif /* BMP280_H */
