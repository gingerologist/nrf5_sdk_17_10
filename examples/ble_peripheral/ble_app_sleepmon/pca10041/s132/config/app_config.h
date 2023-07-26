#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

// CONFIG_GPIO_AS_PINRESET
// CONFIG_NFCT_PINS_AS_GPIOS

#define NRF_LOG_BACKEND_RTT_ENABLED                     1
#define NRF_LOG_BACKEND_UART_ENABLED                    0

#define NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED     0

#define BLE_EEG_ENABLED                                 1
#define BLE_EEG_BLE_OBSERVER_PRIO                       2

#define NRFX_TIMER_ENABLED 1
#define NRFX_TIMER1_ENABLED 1
#define TIMER_ENABLED 1
#define TIMER1_ENABLED 1

#define NRFX_PPI_ENABLED                                1
#define PPI_ENABLED 1

#define NRFX_SAADC_ENABLED                              1
#define NRFX_SAADC_CONFIG_RESOLUTION                    3 // 14 bit
#define SAADC_ENABLED   1
#define SAADC_CONFIG_RESOLUTION 3

#define NRFX_SPIM_ENABLED   1
#define SPI_ENABLED 1
#define SPI0_ENABLED 1
#define SPI0_USE_EASY_DMA 0

#define NRFX_TWIM_ENABLED   1
#define NRFX_TWI_ENABLED    1
#define TWI_ENABLED 1
#define TWI1_ENABLED 1
#define NRF_TWI_MNGR_ENABLED 1

// are these necessary?
#define HCI_UART_RX_PIN 0
#define HCI_UART_TX_PIN 0
#define HCI_UART_RTS_PIN 0
#define HCI_UART_CTS_PIN 0

#define NRF_QUEUE_ENABLED 1

#define NRF_SDH_BLE_GATT_MAX_MTU_SIZE 251
#define NRF_SDH_BLE_VS_UUID_COUNT       1

#define NRF_SDH_BLE_SERVICE_CHANGED     0   // no idea

#define NRF_SDH_DISPATCH_MODEL          0   // 0 interrupt, 2 polling (freertos must use 2)

#endif
