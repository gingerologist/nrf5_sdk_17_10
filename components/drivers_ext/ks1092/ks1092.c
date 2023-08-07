#include <stdint.h>

#include "nrf_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_error.h"

#include "ks1092.h"


static uint8_t spi_tx[4];
static uint8_t spi_rx[4];

#if 0
static const uint8_t ks1092_reg_value[10][4] = {
  {0x20,0x01,0x20,0x00},  // 0: 360
  {0x20,0x01,0x30,0x10},  // 1: 540
  {0x20,0x01,0x22,0x02},  // 2: 680
  {0x20,0x01,0x34,0x14},  // 3: 720
  {0x20,0x01,0x32,0x12},  // 4: 1020
  {0x20,0x01,0x38,0x18},  // 5: 1080
  {0x20,0x01,0x36,0x16},  // 6: 1360
  {0x20,0x01,0x3c,0x1c},  // 7: 1440
  {0x20,0x01,0x00,0x20},  // 8: off
  {0x10,0x01,0xdd,0xdd},  // 9: read
};
#endif

// the last ones are off
static const uint8_t gain1_regval[] = { 0x20, 0x30, 0x22, 0x34, 0x32, 0x38, 0x36, 0x3c, 0x00 };
static const uint8_t gain2_regval[] = { 0x00, 0x10, 0x02, 0x14, 0x12, 0x18, 0x16, 0x1c, 0x20 };

static nrf_spi_mngr_transfer_t const xfers[] =
{
    NRF_SPI_MNGR_TRANSFER(spi_tx, 4, spi_rx, 4)
};

ret_code_t ks1092_write(ks1092_dev_t * p_dev, uint8_t gain1, uint8_t gain2)
{
    ret_code_t err_code;

    spi_tx[0] = 0x20;
    spi_tx[1] = 0x01;
    spi_tx[2] = gain1 > 8 ? gain1_regval[8] : gain1_regval[gain1];
    spi_tx[3] = gain2 > 8 ? gain2_regval[8] : gain2_regval[gain2];
    err_code = nrf_spi_mngr_perform(p_dev->p_spi_mngr,
                                    NULL,
                                    xfers,
                                    sizeof(xfers) / sizeof(xfers[0]),
                                    NULL);
    if (err_code)
    {
        return err_code;
    }

    spi_tx[0] = 0x10;
    spi_tx[1] = 0x01;
    err_code = nrf_spi_mngr_perform(p_dev->p_spi_mngr,
                                    NULL,
                                    xfers,
                                    sizeof(xfers) / sizeof(xfers[0]),
                                    NULL);
    if (err_code)
    {
        return err_code;
    }

    NRF_LOG_INFO("ks1092 regs: 0x%02x, 0x%02x", spi_rx[2], spi_rx[3]);
    return NRF_SUCCESS;
}

void ks1092_init(ks1092_dev_t * p_dev)
{
//    ret_code_t err_code;

//	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//    spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
//    spi_config.mode = NRF_DRV_SPI_MODE_1;

//    spi_config.ss_pin   = SPI_SS_PIN;
//    spi_config.miso_pin = SPI_MISO_PIN;
//    spi_config.mosi_pin = SPI_MOSI_PIN;
//    spi_config.sck_pin  = SPI_SCK_PIN;

//    err_code = nrf_drv_spi_init(&spi, &spi_config, NULL, NULL);
//	  APP_ERROR_CHECK(err_code);

//    nrf_gpio_cfg_output(KS1092_RST_PIN);
//    nrf_gpio_pin_set(KS1092_RST_PIN);
//    vTaskDelay(50);
//    nrf_gpio_pin_clear(KS1092_RST_PIN);
//    vTaskDelay(50);
//    nrf_gpio_pin_set(KS1092_RST_PIN);
//    vTaskDelay(50);

//    // switch off ks1092
//    ks1092_write(KS1092_OFF_INDEX);

    ret_code_t err_code;

    err_code = nrf_spi_mngr_init(p_dev->p_spi_mngr, p_dev->p_spi_cfg);
    APP_ERROR_CHECK(err_code);

    if (!nrfx_gpiote_is_init())
    {
        err_code = nrfx_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrfx_gpiote_out_init(p_dev->reset_pin, p_dev->p_reset_pin_cfg);
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_out_set(p_dev->reset_pin);
    vTaskDelay(50);
    nrfx_gpiote_out_clear(p_dev->reset_pin);
    vTaskDelay(50);
    nrfx_gpiote_out_set(p_dev->reset_pin);
    vTaskDelay(50);

    err_code = ks1092_write(p_dev, KS1092_CHAN_OFF, KS1092_CHAN_OFF);
    APP_ERROR_CHECK(err_code);
}
