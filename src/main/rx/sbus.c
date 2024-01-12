/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_SBUS

#include "build/debug.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sbus.h"
#include "rx/sbus_channels.h"

#include "fc/runtime_config.h"

/*
 * Observations
 *
 * FrSky X8R
 * time between frames: 6ms.
 * time to send frame: 3ms.
*
 * Futaba R6208SB/R6303SB
 * time between frames: 11ms.
 * time to send frame: 3ms.
 */

#define SBUS_BAUDRATE                 100000
#define SBUS_RX_REFRESH_RATE          11000
#define SBUS_TIME_NEEDED_PER_FRAME    3000

#define SBUS_FAST_BAUDRATE              200000
#define SBUS_FAST_RX_REFRESH_RATE       6000

#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)

#define SBUS_FRAME_BEGIN_BYTE 0x0F

#if !defined(SBUS_PORT_OPTIONS)
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

enum {
    DEBUG_SBUS_FRAME_FLAGS = 0,
    DEBUG_SBUS_STATE_FLAGS,
    DEBUG_SBUS_FRAME_TIME,
};

struct sbusFrame_s {
    uint8_t syncByte;
    sbusChannels_t channels;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
     * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
     * and
     * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
     */
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union sbusFrame_u {
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;

typedef struct sbusFrameData_s {
    sbusFrame_t frame;
    timeUs_t startAtUs;
    uint8_t position;
    bool done;
} sbusFrameData_t;

/*
 * Data structures for emulation of a momentary toggle switch.
 * To be used with DJI RC2 controllers, to facilitate a PREARM
 * setup.
 *
 * DJI controllers don't have a momentary toggle switch. Perform
 * edge detection on the channels to simulate this.
 */
typedef enum {
    RC2_EDGE_UNINITIALIZED,
    RC2_EDGE_WAITING_FOR_EDGE,
    RC2_EDGE_ACTIVE
} djiEdgeDetectionState_t;

typedef struct {
    djiEdgeDetectionState_t state;
    timeUs_t timeOfEdgeDetection;
    uint16_t value;
} djiEdgeDetection_t;

static uint8_t djiVirtualPreArmMonitorChannel = 0;
static uint8_t djiVirtualArmMonitorChannel = 0;

#define DJI_VIRTUAL_PREARM_CHANNEL (14)
#define DJI_VIRTUAL_PREARM_TIMEOUT_US (250000)

#define DJI_VIRTUAL_ARM_CHANNEL (15)
#define DJI_VIRTUAL_DISARM_WINDOW_US (150000)
#define DJI_VIRTUAL_DISARM_NUMBER_OF_EDGES_IN_WINDOW (2)

// Receive ISR callback
static void sbusDataReceive(uint16_t c, void *data)
{
    sbusFrameData_t *sbusFrameData = data;

    const timeUs_t nowUs = microsISR();

    const timeDelta_t sbusFrameTime = cmpTimeUs(nowUs, sbusFrameData->startAtUs);

    if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
        sbusFrameData->position = 0;
    }

    if (sbusFrameData->position == 0) {
        if (c != SBUS_FRAME_BEGIN_BYTE) {
            return;
        }
        sbusFrameData->startAtUs = nowUs;
    }

    if (sbusFrameData->position < SBUS_FRAME_SIZE) {
        sbusFrameData->frame.bytes[sbusFrameData->position++] = (uint8_t)c;
        if (sbusFrameData->position < SBUS_FRAME_SIZE) {
            sbusFrameData->done = false;
        } else {
            sbusFrameData->done = true;
            DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_TIME, sbusFrameTime);
        }
    }
}

STATIC_UNIT_TESTED void djiVirtualPreArm(rxRuntimeState_t *rxRuntimeState, const uint8_t frameStatus)
{
    static djiEdgeDetection_t toggleDetection = {
        .state = RC2_EDGE_UNINITIALIZED
    };

    // When having bad data, don't do edge detection; just stay inactive.
    if (frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED)) {
        rxRuntimeState->channelData[DJI_VIRTUAL_PREARM_CHANNEL] = SBUS_DIGITAL_CHANNEL_MIN;
        toggleDetection.state = RC2_EDGE_UNINITIALIZED;
        return;
    }

    const uint16_t channelValue = rxRuntimeState->channelData[djiVirtualPreArmMonitorChannel];
    const timeUs_t currentTimeUs = rxRuntimeState->lastRcFrameTimeUs;

    uint16_t virtualValue = SBUS_DIGITAL_CHANNEL_MIN;

    switch (toggleDetection.state) {
        case RC2_EDGE_UNINITIALIZED:
            toggleDetection.value = channelValue;
            toggleDetection.state = RC2_EDGE_WAITING_FOR_EDGE;
            // intentional fall-through.
        case RC2_EDGE_WAITING_FOR_EDGE:
            if (channelValue == toggleDetection.value) {
                break;
            }
            toggleDetection.timeOfEdgeDetection = currentTimeUs;
            toggleDetection.state = RC2_EDGE_ACTIVE;
            // intentional fall-through.
        case RC2_EDGE_ACTIVE:
            // On an active edge, raise the toggle channel for a given amount of time.
            if (currentTimeUs - toggleDetection.timeOfEdgeDetection <= DJI_VIRTUAL_PREARM_TIMEOUT_US) {
                virtualValue = SBUS_DIGITAL_CHANNEL_MAX;
            } else {
                // After the timeout, go back into edge detection state,
                // and deassert the channel value consequently.
                toggleDetection.state = RC2_EDGE_WAITING_FOR_EDGE;
            }
            break;
        default:
            break;
    }
    rxRuntimeState->channelData[DJI_VIRTUAL_PREARM_CHANNEL] = virtualValue;

    toggleDetection.value = channelValue;
}

STATIC_UNIT_TESTED void djiVirtualArm(rxRuntimeState_t *rxRuntimeState, const uint8_t frameStatus)
{
    static djiEdgeDetection_t toggleDetection = {
        .state = RC2_EDGE_UNINITIALIZED
    };
    static uint8_t outputState = 0;
    static uint8_t numberOfEdgesInWindow = 0;

    // If we have a failsafe or drop, retain the last known value.
    // Perform an edge detection on garbage data would be unwise.
    if (frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED)) {
        rxRuntimeState->channelData[DJI_VIRTUAL_ARM_CHANNEL] = outputState ?
            SBUS_DIGITAL_CHANNEL_MAX : SBUS_DIGITAL_CHANNEL_MIN;
        toggleDetection.state = RC2_EDGE_UNINITIALIZED;
        return;
    }

    const uint16_t channelValue = rxRuntimeState->channelData[djiVirtualArmMonitorChannel];
    const timeUs_t currentTimeUs = rxRuntimeState->lastRcFrameTimeUs;

    switch (toggleDetection.state) {
        case RC2_EDGE_UNINITIALIZED:
            toggleDetection.value = channelValue;
            toggleDetection.state = RC2_EDGE_WAITING_FOR_EDGE;
            outputState = 0;
            // intentional fall-through.
        case RC2_EDGE_WAITING_FOR_EDGE:
            if (channelValue == toggleDetection.value) {
                break;
            }
            // We detected a button press.
            toggleDetection.timeOfEdgeDetection = currentTimeUs;
            toggleDetection.state = RC2_EDGE_ACTIVE;

            // Update the toggle value to not double-count during
            // the active edge.
            toggleDetection.value = channelValue;
            numberOfEdgesInWindow = 1;
            // intentional fall-through.
        case RC2_EDGE_ACTIVE:
            if (!ARMING_FLAG(ARMED)) {
                outputState ^= 1;
                toggleDetection.state = RC2_EDGE_WAITING_FOR_EDGE;
                break;
            }
            // We are armed. Count edges while in the disarm window.
            if (currentTimeUs - toggleDetection.timeOfEdgeDetection <= DJI_VIRTUAL_DISARM_WINDOW_US) {
                if (channelValue != toggleDetection.value) {
                    numberOfEdgesInWindow++;
                }
                // When reaching the threshold, reset the output value.
                if (numberOfEdgesInWindow >= DJI_VIRTUAL_DISARM_NUMBER_OF_EDGES_IN_WINDOW) {
                    // forcefully set to disarm.
                    outputState = 0;
                    toggleDetection.state = RC2_EDGE_WAITING_FOR_EDGE;
                    break;
                }
            } else {
                // We were armed, but did not see enough edges.
                // Discard event by not modifying outputState.
                toggleDetection.state = RC2_EDGE_WAITING_FOR_EDGE;
                break;
            }
            break;
        default:
            break;
    }
    rxRuntimeState->channelData[DJI_VIRTUAL_ARM_CHANNEL] = outputState ?
        SBUS_DIGITAL_CHANNEL_MAX : SBUS_DIGITAL_CHANNEL_MIN;

    toggleDetection.value = channelValue;
}

static uint8_t sbusFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    sbusFrameData_t *sbusFrameData = rxRuntimeState->frameData;
    if (!sbusFrameData->done) {
        return RX_FRAME_PENDING;
    }
    sbusFrameData->done = false;

    DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_FLAGS, sbusFrameData->frame.frame.channels.flags);

    const uint8_t frameStatus = sbusChannelsDecode(rxRuntimeState, &sbusFrameData->frame.frame.channels);

    if (!(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
        rxRuntimeState->lastRcFrameTimeUs = sbusFrameData->startAtUs;
    }

    if (djiVirtualPreArmMonitorChannel) {
        djiVirtualPreArm(rxRuntimeState, frameStatus);
    }
    if (djiVirtualArmMonitorChannel) {
        djiVirtualArm(rxRuntimeState, frameStatus);
    }

    return frameStatus;
}

bool sbusInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    static uint16_t sbusChannelData[SBUS_MAX_CHANNEL];
    static sbusFrameData_t sbusFrameData;
    static uint32_t sbusBaudRate;

    djiVirtualPreArmMonitorChannel = rxConfig->djiVirtualPreArmMonitorChannel;
    djiVirtualArmMonitorChannel = rxConfig->djiVirtualArmMonitorChannel;

    rxRuntimeState->channelData = sbusChannelData;
    rxRuntimeState->frameData = &sbusFrameData;
    sbusChannelsInit(rxConfig, rxRuntimeState);

    rxRuntimeState->channelCount = SBUS_MAX_CHANNEL;

    if (rxConfig->sbus_baud_fast) {
        sbusBaudRate  = SBUS_FAST_BAUDRATE;
    } else {
        sbusBaudRate  = SBUS_BAUDRATE;
    }

    rxRuntimeState->rcFrameStatusFn = sbusFrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig, rxRuntimeState->serialrxProvider);
#else
    bool portShared = false;
#endif

    serialPort_t *sBusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        sbusDataReceive,
        &sbusFrameData,
        sbusBaudRate,
        portShared ? MODE_RXTX : MODE_RX,
        SBUS_PORT_OPTIONS | (rxConfig->serialrx_inverted ? 0 : SERIAL_INVERTED) | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

    if (rxConfig->rssi_src_frame_errors) {
        rssiSource = RSSI_SOURCE_FRAME_ERRORS;
    }

#ifdef USE_TELEMETRY
    if (portShared) {
        telemetrySharedPort = sBusPort;
    }
#endif

    return sBusPort != NULL;
}
#endif
