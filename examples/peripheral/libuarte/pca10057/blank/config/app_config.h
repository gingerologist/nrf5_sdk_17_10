#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

#define NRF_LOG_BACKEND_RTT_ENABLED                     1
#define NRF_LOG_BACKEND_RTT_TEMP_BUFFER_SIZE            1024
#define NRF_LOG_BACKEND_UART_ENABLED                    0
#define NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED     0

#define NRF_LOG_DEFERRED                                0

// ==== Bare Metal Delay ====
#define NRFX_SYSTICK_ENABLED                            1

// ==== UARTE ====
#define NRFX_UARTE_ENABLED                              1
#define NRFX_UART_ENABLED                               1
#define UART_ENABLED                                    1
#define UART0_ENABLED                                   1
#define UART0_CONFIG_USE_EASY_DMA                       1
// #define UART1_ENABLED                                1
#endif
