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
 *   This file implements the OpenThread platform abstraction for radio communication.
 *
 */

#include <openthread/platform/radio.h>

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanId)
{
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    return true;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    return OT_ERROR_NONE;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    return OT_ERROR_NONE;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    return OT_ERROR_NONE;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    return OT_ERROR_NONE;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    return 0;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    return 0;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    return OT_RADIO_CAPS_ACK_TIMEOUT;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    return true;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    return OT_ERROR_NONE;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    return OT_ERROR_NONE;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    return OT_ERROR_NONE;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    return OT_ERROR_NONE;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

void otPlatRadioSetDefaultTxPower(otInstance *aInstance, int8_t aPower)
{
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    return 0;
}
