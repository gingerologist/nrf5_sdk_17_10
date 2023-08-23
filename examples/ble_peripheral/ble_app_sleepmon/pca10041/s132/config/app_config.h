#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

// CONFIG_GPIO_AS_PINRESET
// CONFIG_NFCT_PINS_AS_GPIOS

// RTT does not work yet
#define NRF_LOG_USE_RTT
// -DAPP_TIMER_V2,-DAPP_TIMER_V2_RTC1_ENABLED,-DCONFIG_GPIO_AS_PINRESET,

#ifdef NRF_LOG_USE_RTT
#define NRF_LOG_BACKEND_RTT_ENABLED                     1
#define NRF_LOG_BACKEND_UART_ENABLED                    0
#define NRF_LOG_BACKEND_UART_TX_PIN                     24
#define NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED     0
#else
#define NRF_LOG_BACKEND_RTT_ENABLED                     0
#define NRF_LOG_BACKEND_UART_ENABLED                    1
#define NRF_LOG_BACKEND_UART_TX_PIN                     24
#define NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED     1
#endif

#define BLE_EEG_ENABLED                                 1
#define BLE_EEG_BLE_OBSERVER_PRIO                       2

/*
 * Timer 1 is used for saadc sampling
 * Timer 2 is used for libuarte
 * Timer 3 is used for stimulus (gpiote)
 */
#define NRFX_TIMER_ENABLED                              1
#define NRFX_TIMER1_ENABLED                             1
#define NRFX_TIMER2_ENABLED                             1
#define NRFX_TIMER3_ENABLED                             1
#define TIMER_ENABLED                                   1
#define TIMER1_ENABLED                                  1
#define TIMER2_ENABLED                                  1
#define TIMER3_ENABLED                                  1


#define NRFX_PPI_ENABLED                                1
#define PPI_ENABLED                                     1

#define NRFX_SAADC_ENABLED                              1
#define NRFX_SAADC_CONFIG_RESOLUTION                    3 // 14 bit
#define SAADC_ENABLED                                   1
#define SAADC_CONFIG_RESOLUTION                         3

#define NRFX_SPIM_ENABLED                               1
#define SPI_ENABLED                                     1
#define SPI0_ENABLED                                    1
#define SPI0_USE_EASY_DMA                               0
#define NRF_SPI_MNGR_ENABLED                            1

#define NRFX_TWIM_ENABLED                               1
#define NRFX_TWI_ENABLED                                1
#define TWI_ENABLED                                     1
#define TWI1_ENABLED                                    1
#define NRF_TWI_MNGR_ENABLED                            1

// are these necessary?
#define HCI_UART_RX_PIN                                 0
#define HCI_UART_TX_PIN                                 0
#define HCI_UART_RTS_PIN                                0
#define HCI_UART_CTS_PIN                                0

#define NRF_QUEUE_ENABLED                               1

#define NRF_SDH_BLE_GATT_MAX_MTU_SIZE                   251
#define NRF_SDH_BLE_VS_UUID_COUNT                       1

#define NRF_SDH_BLE_SERVICE_CHANGED                     0   // no idea

#define NRF_SDH_DISPATCH_MODEL                          2   // 0 interrupt, 2 polling (freertos must use 2)

/* these are not in template project, but in libuarte example */
#define NRF_LIBUARTE_ASYNC_WITH_APP_TIMER               0
#define NRF_LIBUARTE_DRV_HWFC_ENABLED                   0
#define NRF_LIBUARTE_DRV_UARTE0                         1
#define NRF_LIBUARTE_DRV_UARTE1                         0


#endif
