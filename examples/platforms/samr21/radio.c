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

#include "asf.h"

#include <phy.h>

#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/radio.h>

#define RSSI_BASE_VALUE (-94)

static otRadioFrame   sTransmitFrame;
static uint8_t        sTransmitPsdu[OT_RADIO_FRAME_MAX_SIZE];

static otRadioState   sState             = OT_RADIO_STATE_DISABLED;
static bool           sPromiscuous       = false;

static int8_t         sMaxRssi;
static uint32_t       sScanStartTime;
static uint16_t       sScanDuration;
static bool           sStartScan         = false;

void samr21RadioInit(void)
{
    sTransmitFrame.mLength = 0;
    sTransmitFrame.mPsdu   = sTransmitPsdu;

    PHY_Init();
}

void samr21RadioProcess(otInstance *aInstance)
{
    (void)aInstance;

    if (sStartScan)
    {
        if ((otPlatAlarmMilliGetNow() - sScanStartTime) < sScanDuration)
        {
            int8_t curRssi = PHY_EdReq();

            if (curRssi > sMaxRssi)
            {
                sMaxRssi = curRssi;
            }
        }
        else
        {
            sStartScan = false;
            otPlatRadioEnergyScanDone(aInstance, sMaxRssi);
        }
    }
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    (void)aInstance;

#ifdef CONF_IEEE_ADDRESS
    *((uint64_t*)aIeeeEui64) = CONF_IEEE_ADDRESS;
#else
    uint8_t* userRow = (uint8_t*)CONF_USER_ROW;

    for (uint8_t i = 0; i < OT_EXT_ADDRESS_SIZE; i++)
    {
        // TODO: define proper userRow struct
        aIeeeEui64[i] = userRow[2 + i];
    }
#endif
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanId)
{
    (void)aInstance;

    PHY_SetPanId(aPanId);
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
    (void)aInstance;

    PHY_SetIEEEAddr((uint8_t*)aAddress);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
    (void)aInstance;

    PHY_SetShortAddr(aAddress);
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    (void)aInstance;

    return (sState != OT_RADIO_STATE_DISABLED);
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    (void)aInstance;

    if (sState == OT_RADIO_STATE_SLEEP)
    {
        PHY_Wakeup();
    }

    sState = OT_RADIO_STATE_RECEIVE;

    PHY_SetRxState(true);

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    (void)aInstance;

    sState = OT_RADIO_STATE_DISABLED;

    PHY_SetRxState(false);

    return OT_ERROR_NONE;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    sState = OT_RADIO_STATE_SLEEP;

    PHY_Sleep();

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
    (void)aInstance;

    return &sTransmitFrame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    return RSSI_BASE_VALUE + sMaxRssi;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    return OT_RADIO_CAPS_ENERGY_SCAN | OT_RADIO_CAPS_TRANSMIT_RETRIES |
           OT_RADIO_CAPS_ACK_TIMEOUT;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    (void)aInstance;

    return sPromiscuous;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;

    sPromiscuous = aEnable;
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;
    (void)aEnable;
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    (void)aInstance;
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    (void)aInstance;
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    sScanStartTime = otPlatAlarmMilliGetNow();
    sScanDuration = aScanDuration;

    sMaxRssi = PHY_EdReq();

    sStartScan = true;

    return OT_ERROR_NONE;
}

void otPlatRadioSetDefaultTxPower(otInstance *aInstance, int8_t aPower)
{
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    return 0;
}
