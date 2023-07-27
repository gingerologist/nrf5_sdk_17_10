#ifndef __KS1092_H__
#define __KS1092_H__

#include "nrfx_gpiote.h"
#include "nrf_spi_mngr.h"

#define KS1092_OFF              0xFF

typedef struct ks1092_dev
{
    nrf_spi_mngr_t const * p_spi_mngr;
    nrf_drv_spi_config_t const * p_spi_cfg;
    nrfx_gpiote_pin_t reset_pin;
    nrfx_gpiote_out_config_t const * p_reset_pin_cfg;
} ks1092_dev_t;

void ks1092_init(ks1092_dev_t * p_dev);
ret_code_t ks1092_write(ks1092_dev_t * p_dev, uint8_t gain1, uint8_t gain2);

#endif // __KS1092_H__

