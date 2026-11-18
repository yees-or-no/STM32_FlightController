/**
 * bmp280.c - lihat bmp280.h. Letakkan di Core/Src.
 * Rumus kompensasi: datasheet Bosch BMP280 (versi integer 32/64 bit).
 */
#include "bmp280.h"
#include <math.h>

#define REG_ID          0xD0
#define REG_RESET       0xE0
#define REG_CTRL_MEAS   0xF4
#define REG_CONFIG      0xF5
#define REG_PRESS_MSB   0xF7
#define REG_CALIB       0x88

#define BMP280_CHIP_ID  0x58
#define I2C_TIMEOUT_MS  100U

static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_regs(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                   uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT_MS);
}

/* BMP280: data kalibrasi little-endian */
static uint16_t u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t  s16le(const uint8_t *p) { return (int16_t)u16le(p); }

/* Hasil: suhu dalam 0.01 derajat C. Mengisi t_fine (dipakai untuk tekanan). */
static int32_t comp_temp(BMP280_t *d, int32_t adc_T)
{
    int32_t v1 = ((((adc_T >> 3) - ((int32_t)d->T1 << 1))) * (int32_t)d->T2) >> 11;
    int32_t t  = (adc_T >> 4) - (int32_t)d->T1;
    int32_t v2 = (((t * t) >> 12) * (int32_t)d->T3) >> 14;
    d->t_fine = v1 + v2;
    return (d->t_fine * 5 + 128) >> 8;
}

/* Hasil: tekanan dalam Pa format Q24.8 (bagi 256 untuk Pa). */
static uint32_t comp_press(const BMP280_t *d, int32_t adc_P)
{
    int64_t v1, v2, p;

    v1 = (int64_t)d->t_fine - 128000;
    v2 = v1 * v1 * (int64_t)d->P6;
    v2 = v2 + ((v1 * (int64_t)d->P5) << 17);
    v2 = v2 + (((int64_t)d->P4) << 35);
    v1 = ((v1 * v1 * (int64_t)d->P3) >> 8) + ((v1 * (int64_t)d->P2) << 12);
    v1 = (((((int64_t)1) << 47) + v1) * (int64_t)d->P1) >> 33;
    if (v1 == 0) return 0;   /* hindari pembagian nol */

    p  = 1048576 - adc_P;
    p  = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)d->P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)d->P8 * p) >> 19;
    p  = ((p + v1 + v2) >> 8) + (((int64_t)d->P7) << 4);
    return (uint32_t)p;
}

HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280_t *dev)
{
    HAL_StatusTypeDef st;
    uint8_t id = 0;

    dev->p0_Pa = 0.0f;
    dev->altitude_m = dev->altitude_f = 0.0f;
    dev->alt_valid = 0;

    st = read_regs(hi2c, REG_ID, &id, 1);
    if (st != HAL_OK) return st;
    if (id != BMP280_CHIP_ID) return HAL_ERROR;

    st = write_reg(hi2c, REG_RESET, 0xB6);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    /* 24 byte koefisien: T1..T3, P1..P9 */
    uint8_t c[24];
    st = read_regs(hi2c, REG_CALIB, c, sizeof(c));
    if (st != HAL_OK) return st;

    dev->T1 = u16le(&c[0]);  dev->T2 = s16le(&c[2]);  dev->T3 = s16le(&c[4]);
    dev->P1 = u16le(&c[6]);  dev->P2 = s16le(&c[8]);  dev->P3 = s16le(&c[10]);
    dev->P4 = s16le(&c[12]); dev->P5 = s16le(&c[14]); dev->P6 = s16le(&c[16]);
    dev->P7 = s16le(&c[18]); dev->P8 = s16le(&c[20]); dev->P9 = s16le(&c[22]);

    /* config: standby 0,5 ms (000), filter IIR koefisien 4 (010), SPI3W off
     *         0b000_010_00 = 0x08 */
    st = write_reg(hi2c, REG_CONFIG, 0x08);
    if (st != HAL_OK) return st;

    /* ctrl_meas: osrs_t x2 (010), osrs_p x16 (101), mode normal (11)
     *            0b010_101_11 = 0x57. Satu siklus sekitar 30 ms. */
    st = write_reg(hi2c, REG_CTRL_MEAS, 0x57);
    if (st != HAL_OK) return st;

    HAL_Delay(50);
    return HAL_OK;
}

HAL_StatusTypeDef BMP280_Read(I2C_HandleTypeDef *hi2c, BMP280_t *dev)
{
    uint8_t b[6];
    HAL_StatusTypeDef st = read_regs(hi2c, REG_PRESS_MSB, b, sizeof(b));
    if (st != HAL_OK) return st;

    int32_t adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    int32_t adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);

    /* 0x80000 = belum ada hasil (sensor baru mulai) */
    if (adc_P == 0x80000 || adc_T == 0x80000) return HAL_BUSY;

    dev->temperature_C = (float)comp_temp(dev, adc_T) / 100.0f;

    uint32_t p = comp_press(dev, adc_P);
    if (p == 0) return HAL_ERROR;
    dev->pressure_Pa = (float)p / 256.0f;

    if (dev->p0_Pa > 0.0f) {
        /* rumus barometrik standar: h = 44330 * (1 - (P/P0)^0.1903) */
        dev->altitude_m = 44330.0f *
            (1.0f - powf(dev->pressure_Pa / dev->p0_Pa, 0.1903f));

        if (!dev->alt_valid) {
            dev->altitude_f = dev->altitude_m;
            dev->alt_valid = 1;
        } else {
            dev->altitude_f += BMP280_ALT_LPF_ALPHA *
                               (dev->altitude_m - dev->altitude_f);
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef BMP280_CalibrateGround(I2C_HandleTypeDef *hi2c, BMP280_t *dev,
                                         uint16_t n)
{
    float sum = 0.0f;
    uint16_t got = 0;
    uint16_t tries = 0;

    dev->p0_Pa = 0.0f;
    dev->alt_valid = 0;

    while (got < n && tries < (uint16_t)(n * 3)) {
        tries++;
        if (BMP280_Read(hi2c, dev) == HAL_OK) {
            sum += dev->pressure_Pa;
            got++;
        }
        HAL_Delay(30);
    }
    if (got < n) return HAL_ERROR;

    dev->p0_Pa = sum / (float)got;
    return HAL_OK;
}
