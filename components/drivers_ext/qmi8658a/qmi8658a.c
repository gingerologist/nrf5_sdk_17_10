#include "nordic_common.h"
#include "nrf.h"

// #include "FreeRTOS.h"

#include "app_error.h"

#include "qmi8658a.h"

const uint8_t qmi8658a_reg_addr[] = {
  /* General Purpose (2)  */ 0x00, 0x01,
  /* Control (9)          */ 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
  /* Calibration (8)      */ 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12,
  /* Fifo (5)             */ 0x13, 0x14, 0x15, 0x16, 0x17,
  /* Status (3)           */ 0x2d, 0x2e, 0x2f,
  /* Timestamp (3)        */ 0x30, 0x31, 0x32,
  /* Data output (2 x 15) */
  0x33, 0x34, 0x35, 0x36, 0x37,
  0x38, 0x39, 0x3a, 0x3b, 0x3c,
  0x3d, 0x3e, 0x3f, 0x40, 0x49,
  0x4a, 0x4b, 0x4c, 0x4d, 0x4e,
  0x4f, 0x50, 0x51, 0x52, 0x53,
  0x54, 0x55, 0x56, 0x57, 0x58,
};

const char * const qmi8658a_reg_name[] = {
  "WHO_AM_I", "REVISION_ID",
  "CTRL 1", "CTRL 2", "CTRL 3", "CTRL 4", "CTRL 5", "CTRL 6", "CTRL 7", "CTRL 8", "CTRL 9",
  "CAL1_L", "CAL1_H", "CAL2_L", "CAL2_H", "CAL3_L", "CAL3_H", "CAL4_L", "CAL4_H",
  "FIFO_WTM_TH", "FIFO_CTRL", "FIFO_SMPL_CNT", "FIFO_STATUS", "FIFO_DATA",
  "STATUSINT", "STATUS0", "STATUS1",
  "TIMESTAMP_LOW", "TIMESTAMP_MID", "TIMESTAMP_HIGH",
  "TEMP_L", "TEMP_H",
  "AX_L", "AX_H", "AY_L", "AY_H", "AZ_L", "AZ_H",
  "GX_L", "GX_H", "GY_L", "GY_H", "GZ_L", "GZ_H",
  "dQW_L", "dQW_H", "dQX_L", "dQX_H", "dQY_L", "dQY_H", "dQZ_L", "dQZ_H",
  "dVX_L", "dVX_H", "dVY_L", "dVY_L", "dVZ_L", "dVZ_H",
  "AE_REG1", "AE_REG2"
};

const uint8_t qmi8658a_init_cfg[] = 
{
    QMI8658A_REG_CTRL1_ENABLE,     // 0x02 big endian (i2c is always BE), 
    0x07,   // 0x03 accel scale 2g, odr 58.75Hz
    0x07,   // 0x04 gyro 16dps, odr 58.75Hz
    0x00,   // 0x05 not used
    0x00,   // 0x06 no low pass filter
    0x00,   // 0x07 no motion on demand
    0x83,   // 0x08 syncSmpl mode, attitude disabled, gyro enabled and full, accel enabled
    0x00,   // 0x09 no engine enabled, int pin default
};


#if 0

static void qmi8658a_int2_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void qmi8658a_init(void);
static void qmi8658a_enable(void);
static void qmi8658a_disable(void);
static void qmi8658a_config(void);
static void qmi8658a_reset(void);
static void qmi8658a_read(uint8_t addr, uint8_t * p_data, uint8_t length);
static void qmi8658a_write(uint8_t addr, const uint8_t * p_data, uint8_t length);

static void qmi8658a_write_single(uint8_t addr, uint8_t val)
{
    ret_code_t err_code;
    uint8_t buf[2];
    buf[0] = addr;
    buf[1] = val;
    nrf_twi_mngr_transfer_t xfers[] =
    {
        NRF_TWI_MNGR_WRITE(QMI8658A_ADDR, buf, 2, 0),
    };
    
    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr,
                                    &qmi8658a_twi_config,
                                    xfers,
                                    sizeof(xfers) / sizeof(xfers[0]),
                                    NULL);
    APP_ERROR_CHECK(err_code);    
}

static void qmi8658a_write(uint8_t addr, const uint8_t * p_data, uint8_t length)
{
    ret_code_t err_code;
    nrf_twi_mngr_transfer_t xfers[] =
    {
        NRF_TWI_MNGR_WRITE(QMI8658A_ADDR,  &addr,       1, NRF_TWI_MNGR_NO_STOP),
        NRF_TWI_MNGR_WRITE(QMI8658A_ADDR, p_data,  length,                    0)
    };

    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr,
                                  &qmi8658a_twi_config,
                                  xfers,
                                  sizeof(xfers) / sizeof(xfers[0]),
                                  NULL);
    APP_ERROR_CHECK(err_code);
}

static void qmi8658a_read(uint8_t addr, uint8_t * p_data, uint8_t length)
{
  ret_code_t err_code;

  nrf_twi_mngr_transfer_t xfers[] =
  {
    NRF_TWI_MNGR_WRITE(QMI8658A_ADDR,  &addr,      1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ (QMI8658A_ADDR, p_data, length,                    0)
  };

  err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr,
                                  &qmi8658a_twi_config,
                                  xfers,
                                  sizeof(xfers) / sizeof(xfers[0]),
                                  NULL);
  APP_ERROR_CHECK(err_code);
}

static void qmi8658a_int2_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  BottomEvent_t be = { .type = BE_IMUINT2 };
  xQueueSendFromISR(m_beq, &be, NULL);
}

static void qmi8658a_init()
{
    ret_code_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = QMI8658A_SCL_PIN,
       .sda                = QMI8658A_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    APP_ERROR_CHECK(err_code);
    
//    qmi8658a_write(0x02, qmi8658a_init_cfg, sizeof(qmi8658a_init_cfg));

//    vTaskDelay(1);
//    
//    uint8_t out_data[sizeof(qmi8658a_init_cfg)] = {0};
//    qmi8658a_read(0x02, out_data, sizeof(out_data));
//    for (int i = 0; i < sizeof(out_data); i++)
//    {
//      NRF_LOG_INFO("init reg 0x%02x: 0x%02x", i + 0x02, out_data[i]);
//    }
    // qmi8658a_reset();
    // vTaskDelay(1);
    qmi8658a_disable();

    // set int2 to input, high accu (IN_EVENT, instead of PORT_EVENT)
    nrfx_gpiote_in_config_t int2_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    err_code = nrfx_gpiote_in_init(QMI8658A_INT2_PIN, &int2_cfg, qmi8658a_int2_handler);
    APP_ERROR_CHECK(err_code);
}

static void qmi8658a_reset()
{
//    uint8_t val = QMI8658A_REG_RESET_VAL;
//    qmi8658a_write(QMI8658A_REG_RESET, &val, 1);
    
    
    qmi8658a_write_single(0x60, 0xB0);
    vTaskDelay(10);
    
    uint8_t val = 0xff;
    qmi8658a_read(QMI8658A_REG_CTRL1, &val, 1);
    
    NRF_LOG_INFO("qmi8658a reset, CTRL1 0x%02x (expect 0x20)", val);
}

static void qmi8658a_config()
{    
    qmi8658a_write(QMI8658A_REG_CTRL1, qmi8658a_init_cfg, sizeof(qmi8658a_init_cfg));

    uint8_t out_data[sizeof(qmi8658a_init_cfg)] = {0};
    qmi8658a_read(0x02, out_data, sizeof(out_data));
    for (int i = 0; i < sizeof(out_data); i++)
    {
      NRF_LOG_INFO("qmi8658a config reg 0x%02x: 0x%02x", i + 0x02, out_data[i]);
    }
}

static void qmi8658a_enable()
{
//    uint8_t val = QMI8658A_REG_CTRL1_ENABLE;        // enable sensor and int2
//    qmi8658a_write(QMI8658A_REG_CTRL1, &val, 1);
    
    qmi8658a_write_single(QMI8658A_REG_CTRL1, QMI8658A_REG_CTRL1_ENABLE);
    
    uint8_t val = 0xff;
    qmi8658a_read(QMI8658A_REG_CTRL1, &val, 1);
    
    NRF_LOG_INFO("qmi8658a enabled, CTRL1 0x%02x (expect 0x%02x)", val, QMI8658A_REG_CTRL1_ENABLE);
}

static void qmi8658a_disable()
{
//    uint8_t val = QMI8658A_REG_CTRL1_DISABLE;       // disable sensor and int2
//    qmi8658a_write(QMI8658A_REG_CTRL1, &val, 1);
    
    qmi8658a_write_single(QMI8658A_REG_CTRL1, QMI8658A_REG_CTRL1_DISABLE);
    
    uint8_t val = 0xff;
    qmi8658a_read(QMI8658A_REG_CTRL1, &val, 1);
    
    NRF_LOG_INFO("qmi8658a disabled, CTRL1 0x%02x (expect 0x%02x)", val, QMI8658A_REG_CTRL1_DISABLE);
}

#endif
