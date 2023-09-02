/**
 * Copyright (c) 2018 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup libuarte_example_main main.c
 * @{
 * @ingroup libuarte_example
 * @brief Libuarte Example Application main file.
 *
 * This file contains the source code for a sample application using libuarte.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <bsp.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_systick.h"
#include "nrfx_uart.h"

#define HIGH				1
#define LOW					0

#define OWRESET             0xF0
#define OWWRITEBIT1         0xFF
#define OWWRITEBIT0         0x00

unsigned char OW_Data;

static nrfx_uart_t m_owuart = NRFX_UART_INSTANCE(0);

static nrfx_uart_config_t slow_config = {
    .pseltxd    = TX_PIN_NUMBER,
    .pselrxd    = RX_PIN_NUMBER,
    .pselcts    = NRF_UART_PSEL_DISCONNECTED,
    .pselrts    = NRF_UART_PSEL_DISCONNECTED,
    .p_context  = NULL,
    .hwfc       = NRF_UART_HWFC_DISABLED,
    .parity     = NRF_UART_PARITY_EXCLUDED,
    .baudrate   = NRF_UART_BAUDRATE_9600,
    .interrupt_priority = NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY,
    .txd_od_pu  = 1,
};

static nrfx_uart_config_t fast_config = {
    .pseltxd    = TX_PIN_NUMBER,
    .pselrxd    = RX_PIN_NUMBER,
    .pselcts    = NRF_UART_PSEL_DISCONNECTED,
    .pselrts    = NRF_UART_PSEL_DISCONNECTED,
    .p_context  = NULL,
    .hwfc       = NRF_UART_HWFC_DISABLED,
    .parity     = NRF_UART_PARITY_EXCLUDED,
    .baudrate   = UART_BAUDRATE_BAUDRATE_Baud115200,
    .interrupt_priority = NRFX_UART_DEFAULT_CONFIG_IRQ_PRIORITY,
    .txd_od_pu  = 1,
};

static unsigned char OW_Reset(void);
static void OW_SendDataBit(unsigned char TxBit);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 Function: OW_SendDataBit              				     																   //
//   Description: Send 1 Wire Data 1-Bit   					 																   //
//				  Write 1 - Sends 0xFF	 					 																   //
//				  Write 0 - Sends 0x00						 																   //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void OW_SendDataBit(unsigned char TxBit)
{
	// while(!(USI1ST1 & DRE));
	unsigned char USI1DR;
    
	if(TxBit == HIGH)
	{
		USI1DR = OWWRITEBIT1;
	}
	else if(TxBit == LOW)
	{
		USI1DR = OWWRITEBIT0;
	}	 
	else
	{
		USI1DR = TxBit;
	}
    
    nrfx_uart_tx(&m_owuart, &USI1DR, 1);
}// OW_SendDataBit


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	 Function:	  OW_Reset 								     																   //
//   Description: Sends the Reset Pulse and detect slave presence. Baud Rate should be set to 9600. Then 0xF0 will be sent,    //
//				  enough to reset the slaves. The if the Rx reads 0xF0, it means there are no slaves present.   			   // 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char OW_Reset(void)
{
	OW_Data = 0;

	// UART_SetBaud(UART1, FREQUENCY, DEFBAUD);									// Set Baud Rate to 9600
    nrfx_uart_uninit(&m_owuart);
    nrfx_uart_init(&m_owuart, &slow_config, NULL);
    nrfx_uart_rx_enable(&m_owuart);
	
    OW_SendDataBit(OWRESET);													// Send F0
	
    // OW_Data = UART_GetChar(UART1);										    // Read returned data
    nrfx_uart_rx(&m_owuart, &OW_Data, 1);
    
    NRF_LOG_INFO("OW Reset OW_Data: %d", OW_Data);
	
    // UART_SetBaud(UART1, FREQUENCY, FASTBAUD);								// Set UART to 115200Kbps 
    nrfx_uart_uninit(&m_owuart);
    nrfx_uart_init(&m_owuart, &fast_config, NULL);
    nrfx_uart_rx_enable(&m_owuart);
    
	if(OW_Data != OWRESET)													    // if send data is equal to the recieved data
	{	
		return (HIGH);															// Return 1
	}
	else
	{
		return (LOW);															// Return 0
	}
}// OW_Reset

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    nrfx_systick_init();

    bsp_board_init(BSP_INIT_LEDS);

    ret_code_t err_code = NRF_LOG_INIT(NULL); // NRF_LOG_INIT(app_timer_cnt_get);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("hello owuart");

    
    while (true)
    {
        NRF_LOG_FLUSH(); // or NRF_LOG_PROCESS by which only one log entry is processed
        nrfx_systick_delay_ms(1000);
        NRF_LOG_INFO("hello owuart");
    }
}


/** @} */
