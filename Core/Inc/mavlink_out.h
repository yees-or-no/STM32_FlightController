/**
 * mavlink_out.h - pemilih mode keluaran USB CDC + pengirim MAVLink v2.
 *
 * UBAH SATU BARIS INI untuk memilih apa yang dikirim lewat USB:
 *   OUT_PUTTY   -> tampilan teks tetap di PuTTY (tanpa library MAVLink)
 *   OUT_CSV     -> baris CSV untuk animasi_mpu.py (tanpa --mavlink)
 *   OUT_MAVLINK -> MAVLink biner untuk animasi_mpu.py --mavlink (butuh c_library_v2)
 *
 * Hanya satu mode aktif sekaligus, dan satu port COM hanya bisa dibuka satu program
 * (PuTTY ATAU Python). Di mode OUT_MAVLINK, PuTTY hanya menampilkan karakter acak.
 *
 * Pesan MAVLink yang dikirim:
 *   HEARTBEAT   1 Hz
 *   ATTITUDE    50 Hz  -> sudut Kalman (roll, pitch; yaw = 0 karena tanpa magnetometer)
 *   DEBUG_VECT  50 Hz  -> name "COMP", x = roll, y = pitch complementary (derajat)
 *   SCALED_IMU  50 Hz  -> accel (mG) dan gyro (mrad/s)
 *
 * Letakkan di Core/Inc.
 */
#ifndef MAVLINK_OUT_H
#define MAVLINK_OUT_H

#include "main.h"
#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OUT_PUTTY    0
#define OUT_CSV      1
#define OUT_MAVLINK  2

#define OUTPUT_MODE  OUT_MAVLINK      /* <<< pilih mode di sini */

#define MAVOUT_ENABLE (OUTPUT_MODE == OUT_MAVLINK)

void MavOut_Init(void);

/* Panggil di setiap putaran while(1). Fungsi ini mengatur laju kirimnya sendiri
 * dan tidak pernah blocking: kalau USB masih sibuk, langsung kembali. */
void MavOut_Service(const MPU6050_t *imu);

#ifdef __cplusplus
}
#endif
#endif
