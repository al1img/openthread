/*
 *  Copyright (c) 2017, The OpenThr
ead Authors.
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
 *   This file implements the OpenThread platform abstraction for the non-volatile storage.
 */

#include "asf.h"

#include "utils/flash.h"

#include "openthread-core-samr21-config.h"

otError utilsFlashInit(void)
{
    struct nvm_config configNvm;

    nvm_get_config_defaults(&configNvm);

    configNvm.manual_page_write = false;

    enum status_code status;

    while ((status = nvm_set_config(&configNvm)) == STATUS_BUSY);

    if (status != STATUS_OK)
    {
        return OT_ERROR_FAILED;
    }

    return OT_ERROR_NONE;
}

uint32_t utilsFlashGetSize(void)
{
    return SETTINGS_CONFIG_PAGE_NUM * SETTINGS_CONFIG_PAGE_SIZE;
}

otError utilsFlashErasePage(uint32_t aAddress)
{
    if (nvm_erase_row(aAddress) != STATUS_OK)
    {
        return OT_ERROR_FAILED;
    }

    return OT_ERROR_NONE;
}

otError utilsFlashStatusWait(uint32_t aTimeout)
{
    otError error = OT_ERROR_BUSY;
    uint32_t start = otPlatAlarmMilliGetNow();

    do
    {
        if (nvm_is_ready())
        {
            error = OT_ERROR_NONE;
            break;
        }
    }
    while (aTimeout && ((otPlatAlarmMilliGetNow() - start) < aTimeout));

    return error;
}

uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    if ((aData == NULL) || (aAddress < SETTINGS_CONFIG_BASE_ADDRESS) ||
        ((aAddress - SETTINGS_CONFIG_BASE_ADDRESS + aSize) >
         utilsFlashGetSize()) || (aAddress & 3) || (aSize & 3))
    {
        return 0;
    }

    for (uint32_t i = 0; i < (aSize / sizeof(uint32_t)); i++)
    {
        *((volatile uint32_t *)aAddress) = *((uint32_t *)aData);
        aData += sizeof(uint32_t);
        aAddress += sizeof(uint32_t);
    }

    // check if write page command is required
    if ((aAddress) & (NVMCTRL_PAGE_SIZE - 1))
    {
        if (nvm_execute_command(NVM_COMMAND_WRITE_PAGE,
                                aAddress & (~(NVMCTRL_PAGE_SIZE - 1)), 0) !=
            STATUS_OK)
        {
            return 0;
        }
    }

    return aSize;
}

uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    if ((aData == NULL) || (aAddress < SETTINGS_CONFIG_BASE_ADDRESS) ||
        ((aAddress - SETTINGS_CONFIG_BASE_ADDRESS + aSize) >
         utilsFlashGetSize()))
    {
        return 0;
    }

    while (aSize--)
    {
        *aData++ = (*(uint8_t *)(aAddress++));
    }

    return aSize;
}
