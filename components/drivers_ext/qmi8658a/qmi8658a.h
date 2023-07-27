#ifndef __QMI8658A_H__
#define __QMI8658A_H__

#include <stdint.h>

#include "nrf_twi_mngr.h"

#define QMI8658A_ADDR                   (0x6B)   // SA0 pulled down



#define QMI8658A_STATUS0                0x2E    //
#define QMI8658A_AX_L                   0x35
#define QMI8658A_AX_H                   0x36
#define QMI8658A_AY_L                   0x37
#define QMI8658A_AY_H                   0x38
#define QMI8658A_AZ_L                   0x39
#define QMI8658A_AZ_H                   0x3A
#define QMI8658A_GX_L                   0x3B
#define QMI8658A_GX_H                   0x3C
#define QMI8658A_GY_L                   0x3D
#define QMI8658A_GY_H                   0x3E
#define QMI8658A_GZ_L                   0x3F
#define QMI8658A_GZ_H                   0x40

#define QMI8658A_BYTES_IN_DATA          (QMI8658A_GZ_H - QMI8658A_AX_L + 1)
#define QMI8658A_SAMPLES_IN_DATA        (QMI8658A_BYTES_IN_DATA / 2)

#define QMI8658A_REG_CTRL1              0x02
#define QMI8658A_REG_CTRL1_ENABLE       0x70
#define QMI8658A_REG_CTRL1_DISABLE      0x61    // 0b0  spi
                                                // 0b1  address auto increment
                                                // 0b1  big endian
                                                // 0b0  int2 disabled
                                                // 0b0  int1 disabled
                                                // 0b0  resv
                                                // 0b0  resv
                                                // 0b1  disable internal 2MHz oscillator
#define QMI8658A_REG_RESET              0x60
#define QMI8658A_REG_RESET_VAL          0xB0    // 0xB0

typedef struct qmi8658a_ctx {
    nrf_twi_mngr_t * p_mngr;
    nrf_drv_twi_config_t const * p_config;
} qmi8658a_ctx_t;

extern const uint8_t qmi8658a_reg_addr[];
extern const char * const qmi8658a_reg_name[];
extern const uint8_t qmi8658a_init_cfg[];


#endif // __QMI8658A_H__
