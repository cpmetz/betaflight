
#include <stdint.h>
#include <stdbool.h>

extern "C" {
    #include "platform.h"

    #include "pg/rx.h"
    #include "build/debug.h"
    #include "drivers/io.h"
    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"
    #include "flight/failsafe.h"
    #include "rx/rx.h"
    #include "rx/sbus_channels.h"
    #include "fc/rc_modes.h"
    #include "common/maths.h"
    #include "common/utils.h"
    #include "config/feature.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "io/beeper.h"
    #include "io/serial.h"

    bool sbusInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState);

    void djiVirtualPreArm(rxRuntimeState_t *rxRuntimeState, const uint8_t frameStatus);
    void djiVirtualArm(rxRuntimeState_t *rxRuntimeState, const uint8_t frameStatus);
}

#include "gtest/gtest.h"

extern "C" {

uint8_t armingFlags = 0;
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
static serialPortConfig_t *findSerialPortConfig_stub_retval;

const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) {
    return findSerialPortConfig_stub_retval;
}

rssiSource_e rssiSource;
uint32_t dummyTimeUs;

uint32_t micros(void) {return dummyTimeUs;}
uint32_t microsISR(void) {return micros();}

timeUs_t rxFrameTimeUs(void) { return 0; }

bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig)
{
    //TODO: implement
    (void) portConfig;
    return false;
}

serialPort_t * telemetrySharedPort = NULL;

serialPort_t dummyPort = {};
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return &dummyPort;}

}

#define SBUS_VAL0 (992)
#define SBUS_VAL1 (1812)
#define TEST_CHANNEL (6)

#define DJI_VIRTUAL_PREARM_CHANNEL (14)
#define DJI_VIRTUAL_ARM_CHANNEL (15)

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

#define DJI_VIRTUAL_PREARM_TIMEOUT_US (250000)
#define DJI_VIRTUAL_DISARM_WINDOW_US (150000)

TEST(SbusTest, TestVirtualFailsafeDeactivatesPreArm)
{
    rxConfig_t rxConfig;
    rxRuntimeState_t rxRuntimeState;
    uint8_t frameStatus = 0;

    rxConfig.djiVirtualPreArmMonitorChannel = TEST_CHANNEL;

    sbusInit(&rxConfig, &rxRuntimeState);

    uint16_t *cd = &rxRuntimeState.channelData[DJI_VIRTUAL_PREARM_CHANNEL];

    for (int i = 0; i < rxRuntimeState.channelCount; ++i) {
        rxRuntimeState.channelData[i] = SBUS_DIGITAL_CHANNEL_MAX;
    }

    djiVirtualPreArm(&rxRuntimeState, frameStatus | RX_FRAME_FAILSAFE);
    EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MIN);
}

TEST(SbusTest, TestVirtualPreArmSinglePress)
{
    rxConfig_t rxConfig;
    rxRuntimeState_t rxRuntimeState;
    uint8_t frameStatus = 0;

    rxConfig.djiVirtualPreArmMonitorChannel = TEST_CHANNEL;

    sbusInit(&rxConfig, &rxRuntimeState);

    uint16_t *cd = &rxRuntimeState.channelData[DJI_VIRTUAL_PREARM_CHANNEL];
    uint16_t *cv = &rxRuntimeState.channelData[TEST_CHANNEL];

    for (int i = 0; i < rxRuntimeState.channelCount; ++i) {
        rxRuntimeState.channelData[i] = SBUS_VAL0;
    }
    rxRuntimeState.lastRcFrameTimeUs = 0;

    *cv = SBUS_VAL0;
    // Reset state by sending failsafe.
    djiVirtualPreArm(&rxRuntimeState, frameStatus | RX_FRAME_FAILSAFE);
    djiVirtualPreArm(&rxRuntimeState, frameStatus);
    EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MIN);

    rxRuntimeState.lastRcFrameTimeUs += 100;
    *cv = SBUS_VAL1;
    djiVirtualPreArm(&rxRuntimeState, frameStatus);
    EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MAX);

    rxRuntimeState.lastRcFrameTimeUs += DJI_VIRTUAL_PREARM_TIMEOUT_US;
    djiVirtualPreArm(&rxRuntimeState, frameStatus);
    EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MAX);

    rxRuntimeState.lastRcFrameTimeUs += 1;
    djiVirtualPreArm(&rxRuntimeState, frameStatus);
    EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MIN);
}

TEST(SbusTest, TestVirtualPreArmMultiPressDuringWindowDoesNotExtendPreArmTime)
{
    rxConfig_t rxConfig;
    rxRuntimeState_t rxRuntimeState;
    uint8_t frameStatus = 0;

    rxConfig.djiVirtualPreArmMonitorChannel = TEST_CHANNEL;

    sbusInit(&rxConfig, &rxRuntimeState);

    uint16_t *cd = &rxRuntimeState.channelData[DJI_VIRTUAL_PREARM_CHANNEL];
    uint16_t *cv = &rxRuntimeState.channelData[TEST_CHANNEL];

    for (int i = 0; i < rxRuntimeState.channelCount; ++i) {
        rxRuntimeState.channelData[i] = SBUS_VAL0;
    }
    rxRuntimeState.lastRcFrameTimeUs = 0;

    *cv = SBUS_VAL0;
    // Reset state by sending failsafe.
    djiVirtualPreArm(&rxRuntimeState, frameStatus | RX_FRAME_FAILSAFE);
    djiVirtualPreArm(&rxRuntimeState, frameStatus);
    EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MIN);

    rxRuntimeState.lastRcFrameTimeUs += 100;
    const timeUs_t expectedTimeBeforeSwitch = rxRuntimeState.lastRcFrameTimeUs + DJI_VIRTUAL_PREARM_TIMEOUT_US;
    *cv = SBUS_VAL1;
    djiVirtualPreArm(&rxRuntimeState, frameStatus);
    EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MAX);

    for (int k = 0; k < 10; ++k) {
        rxRuntimeState.lastRcFrameTimeUs += 100;
        *cv = (k & 1) ? SBUS_VAL1 : SBUS_VAL0;
        djiVirtualPreArm(&rxRuntimeState, frameStatus);
        EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MAX);
    }
    rxRuntimeState.lastRcFrameTimeUs = expectedTimeBeforeSwitch;
    djiVirtualPreArm(&rxRuntimeState, frameStatus);
    EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MAX);

    rxRuntimeState.lastRcFrameTimeUs += 1;
    djiVirtualPreArm(&rxRuntimeState, frameStatus);
    EXPECT_EQ(*cd, SBUS_DIGITAL_CHANNEL_MIN);
}


