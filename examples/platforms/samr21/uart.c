/*
 *  Copyright (c) 2017, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for UART communication.
 *
 */

#include "asf.h"

#include <openthread/platform/uart.h>

static struct usart_module sUsartInstance;

static uint16_t sRxByte;

void usartReadCallback(struct usart_module *const usartModule)
{
    otPlatUartReceived((uint8_t*)&sRxByte, 1);
    usart_read_job(&sUsartInstance, &sRxByte);
}

void usartWriteCallback(struct usart_module *const usartModule)
{
    otPlatUartSendDone();
}

otError otPlatUartEnable(void)
{
    struct usart_config configUsart;

    usart_get_config_defaults(&configUsart);

    configUsart.baudrate    = 115200;
    configUsart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
    configUsart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
    configUsart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
    configUsart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
    configUsart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;

    while (usart_init(&sUsartInstance, EDBG_CDC_MODULE, &configUsart)
           != STATUS_OK);

    usart_enable(&sUsartInstance);

    usart_register_callback(&sUsartInstance, usartWriteCallback,
                            USART_CALLBACK_BUFFER_TRANSMITTED);
    usart_register_callback(&sUsartInstance, usartReadCallback,
                            USART_CALLBACK_BUFFER_RECEIVED);

    usart_enable_callback(&sUsartInstance, USART_CALLBACK_BUFFER_TRANSMITTED);
    usart_enable_callback(&sUsartInstance, USART_CALLBACK_BUFFER_RECEIVED);

    usart_read_job(&sUsartInstance, &sRxByte);

    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    usart_disable(&sUsartInstance);

    return OT_ERROR_NONE;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    if (usart_write_buffer_job(&sUsartInstance, (uint8_t*)aBuf,
                               aBufLength) != STATUS_OK)
    {
        return OT_ERROR_FAILED;
    }

    return OT_ERROR_NONE;
}

