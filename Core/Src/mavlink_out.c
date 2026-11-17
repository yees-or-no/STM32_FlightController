/**
 * mavlink_out.c - lihat mavlink_out.h
 *
 * Seluruh isi file ini hanya dikompilasi kalau OUTPUT_MODE == OUT_MAVLINK,
 * jadi di mode PuTTY/CSV library MAVLink TIDAK dibutuhkan.
 *
 * Library: https://github.com/mavlink/c_library_v2
 *   Salin folder "common", folder "minimal", dan semua .h di root repo itu
 *   ke Core/Inc/mavlink/. Header-only, tidak ada .c tambahan.
 *   Kalau ada error jumlah argumen di mavlink_msg_scaled_imu_pack, versi library-mu
 *   punya/tidak punya field "temperature": tambah/hapus argumen terakhir (0).
 */
#include "mavlink_out.h"

#if MAVOUT_ENABLE

#include "mavlink/common/mavlink.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define SYS_ID           1
#define COMP_ID          MAV_COMP_ID_IMU
#define STREAM_PERIOD    20U      /* 50 Hz */
#define HEARTBEAT_PERIOD 1000U
#define DEG2RAD          0.01745329252f

extern USBD_HandleTypeDef hUsbDeviceFS;

/* static: buffer harus tetap valid sampai USB selesai mengirim.
 * Satu siklus (heartbeat + 3 pesan) kira-kira 150 byte. */
static uint8_t  s_tx[512];
static uint16_t s_len;
static uint32_t t_stream, t_hb;

/* true kalau USB sudah terkonfigurasi dan transmit sebelumnya sudah selesai */
static uint8_t usb_ready(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return (hcdc != NULL && hcdc->TxState == 0);
}

/* tambahkan pesan ke buffer (belum dikirim) */
static void queue_msg(const mavlink_message_t *m)
{
    uint8_t tmp[MAVLINK_MAX_PACKET_LEN];
    uint16_t n = mavlink_msg_to_send_buffer(tmp, m);
    if ((s_len + n) <= sizeof(s_tx)) {
        memcpy(&s_tx[s_len], tmp, n);
        s_len += n;
    }
}

/* kirim seluruh isi buffer dalam satu transfer USB */
static void flush(void)
{
    if (s_len > 0) {
        CDC_Transmit_FS(s_tx, s_len);
        s_len = 0;
    }
}

void MavOut_Init(void)
{
    s_len = 0;
    t_stream = t_hb = HAL_GetTick();
}

void MavOut_Service(const MPU6050_t *d)
{
    if (!usb_ready()) return;   /* USB sibuk: coba lagi putaran berikutnya, tanpa menunggu */

    uint32_t now = HAL_GetTick();
    mavlink_message_t msg;
    s_len = 0;

    if ((now - t_hb) >= HEARTBEAT_PERIOD) {
        t_hb = now;
        mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &msg, MAV_TYPE_QUADROTOR,
                                   MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
        queue_msg(&msg);
    }

    if ((now - t_stream) >= STREAM_PERIOD) {
        t_stream = now;

        /* Kalman -> ATTITUDE (radian) */
        mavlink_msg_attitude_pack(SYS_ID, COMP_ID, &msg, now,
                                  d->KalmanAngleX * DEG2RAD,
                                  d->KalmanAngleY * DEG2RAD,
                                  0.0f,
                                  d->Gx * DEG2RAD, d->Gy * DEG2RAD, d->Gz * DEG2RAD);
        queue_msg(&msg);

        /* Complementary -> DEBUG_VECT (derajat) */
        mavlink_msg_debug_vect_pack(SYS_ID, COMP_ID, &msg, "COMP",
                                    (uint64_t)now * 1000ULL,
                                    d->CompAngleX, d->CompAngleY, 0.0f);
        queue_msg(&msg);

        /* Data sensor: mG dan mrad/s */
        mavlink_msg_scaled_imu_pack(SYS_ID, COMP_ID, &msg, now,
                                    (int16_t)(d->Ax * 1000.0f),
                                    (int16_t)(d->Ay * 1000.0f),
                                    (int16_t)(d->Az * 1000.0f),
                                    (int16_t)(d->Gx * DEG2RAD * 1000.0f),
                                    (int16_t)(d->Gy * DEG2RAD * 1000.0f),
                                    (int16_t)(d->Gz * DEG2RAD * 1000.0f),
                                    0, 0, 0, 0);
        queue_msg(&msg);
    }

    flush();
}

#else  /* bukan mode MAVLink: fungsi kosong supaya main.c tetap bisa dilink */

void MavOut_Init(void) {}
void MavOut_Service(const MPU6050_t *d) { (void)d; }

#endif /* MAVOUT_ENABLE */
