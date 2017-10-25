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
 * @brief
 *   This file includes the platform-specific initializers.
 */

#include "asf.h"

#include <openthread/platform/platform.h>

#include "platform-samr21.h"

otInstance *sInstance;

samr21UserRow *sUserRow = (samr21UserRow *)SAMR21_USER_ROW;

void board_init(void)
{
    struct port_config pin_conf;

    port_get_config_defaults(&pin_conf);
    pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
    port_pin_set_config(AT86RFX_SPI_SCK,  &pin_conf);
    port_pin_set_config(AT86RFX_SPI_MOSI, &pin_conf);
    port_pin_set_config(AT86RFX_SPI_CS,   &pin_conf);
    port_pin_set_config(AT86RFX_RST_PIN,  &pin_conf);
    port_pin_set_config(AT86RFX_SLP_PIN,  &pin_conf);
    port_pin_set_output_level(AT86RFX_SPI_SCK,  true);
    port_pin_set_output_level(AT86RFX_SPI_MOSI, true);
    port_pin_set_output_level(AT86RFX_SPI_CS,   true);
    port_pin_set_output_level(AT86RFX_RST_PIN,  true);
    port_pin_set_output_level(AT86RFX_SLP_PIN,  true);

    pin_conf.direction  = PORT_PIN_DIR_INPUT;
    port_pin_set_config(AT86RFX_SPI_MISO, &pin_conf);
}

void PlatformInit(int argc, char *argv[])
{
    system_clock_init();
    board_init();

    samr21AlarmInit();
    samr21RadioInit();
}

void PlatformDeinit(void)
{
}

void PlatformProcessDrivers(otInstance *aInstance)
{
    sInstance = aInstance;

    samr21UartProcess();
    samr21AlarmProcess(aInstance);
    samr21RadioProcess(aInstance);
}
