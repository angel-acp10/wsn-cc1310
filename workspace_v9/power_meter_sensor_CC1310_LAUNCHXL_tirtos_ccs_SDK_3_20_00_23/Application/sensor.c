/******************************************************************************

 @file sensor.c

 @brief TIMAC 2.0 Sensor Example Application

 Group: WCS LPC
 Target Device: cc13x0

 ******************************************************************************

 Copyright (c) 2016-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************


 *****************************************************************************/

/******************************************************************************
 Includes
 *****************************************************************************/
#include "PowerMeter/power_meter.h"
#include <string.h>
#include <stdint.h>
#include "mac_util.h"
#include "api_mac.h"
#include "jdllc.h"
#include "ssf.h"
#include "smsgs.h"
#include "sensor.h"
#include "config.h"
#include "board_led.h"
#include "board_lcd.h"
#include <xdc/runtime/System.h>
#include <inc/hw_fcfg1.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>


#ifdef FEATURE_NATIVE_OAD
#include "oad_client.h"
#endif /* FEATURE_NATIVE_OAD */

#ifdef OSAL_PORT2TIRTOS
#include <ti/sysbios/knl/Clock.h>
#else
#include "icall.h"
#endif

#ifdef FEATURE_SECURE_COMMISSIONING
#include "sm_ti154.h"
#endif

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

#if !defined(CONFIG_AUTO_START)
#if defined(AUTO_START)
#define CONFIG_AUTO_START 1
#else
#define CONFIG_AUTO_START 0
#endif
#endif

/* default MSDU Handle rollover */
#define MSDU_HANDLE_MAX 0x1F

/* App marker in MSDU handle */
#define APP_MARKER_MSDU_HANDLE 0x80

/* App Message Tracking Mask */
#define APP_MASK_MSDU_HANDLE 0x60

/* App Sensor Data marker for the MSDU handle */
#define APP_SENSOR_MSDU_HANDLE 0x40

/* App tracking response marker for the MSDU handle */
#define APP_TRACKRSP_MSDU_HANDLE 0x20

/* App config response marker for the MSDU handle */
#define APP_CONFIGRSP_MSDU_HANDLE 0x60

/* Reporting Interval Min and Max (in milliseconds) */
#define MIN_REPORTING_INTERVAL 1000
#define MAX_REPORTING_INTERVAL 360000

/* Polling Interval Min and Max (in milliseconds) */
#define MIN_POLLING_INTERVAL 1000
#define MAX_POLLING_INTERVAL 10000

/* Blink Time for Identify LED Request (in milliseconds) */
#define IDENTIFY_LED_TIME 1000

/* Inter packet interval in certification test mode */
#if CERTIFICATION_TEST_MODE
#if ((CONFIG_PHY_ID >= APIMAC_MRFSK_STD_PHY_ID_BEGIN) && (CONFIG_PHY_ID <= APIMAC_MRFSK_GENERIC_PHY_ID_BEGIN))
/*! Regular Mode */
#define SENSOR_TEST_RAMP_DATA_SIZE   75
#define CERT_MODE_INTER_PKT_INTERVAL 50
#elif ((CONFIG_PHY_ID >= APIMAC_MRFSK_GENERIC_PHY_ID_BEGIN + 1) && (CONFIG_PHY_ID <= APIMAC_MRFSK_GENERIC_PHY_ID_END))
/*! LRM Mode */
#define SENSOR_TEST_RAMP_DATA_SIZE   20
#define CERT_MODE_INTER_PKT_INTERVAL 300
#else
#error "PHY ID is wrong."
#endif
#endif

/******************************************************************************
 Global variables
 *****************************************************************************/
extern enum SENSOR_ACTION sensor_action;
extern power_meter_variables powerMeterData;

extern uint16_t id0_sensor_val;
extern uint16_t id1_sensor_val;
extern uint16_t id2_sensor_val;
extern uint16_t id3_sensor_val;
extern uint16_t type_sensor_val;
extern uint16_t battery_sensor_val;
extern uint16_t f0_sensor_val; // f0 to f9_sensor_val are general purpose
extern uint16_t f1_sensor_val; // variables to store the read data from the
extern uint16_t f2_sensor_val; // a generic sensor and send it to the collector
extern uint16_t f3_sensor_val; // which also has teh same variables
extern uint16_t f4_sensor_val;
extern uint16_t f5_sensor_val;
extern uint16_t f6_sensor_val;
extern uint16_t f7_sensor_val;
extern uint16_t f8_sensor_val;
extern uint16_t f9_sensor_val;

extern Semaphore_Handle semHandle; //semaphore to activate or deactivate the SENSORTASK

/* Task pending events */
uint16_t Sensor_events = 0;

/*! Sensor statistics */
Smsgs_msgStatsField_t Sensor_msgStats =
    { 0 };
extern bool initBroadcastMsg;
extern bool parentFound;
#ifdef POWER_MEAS
/*! Power Meas Stats fields */
Smsgs_powerMeastatsField_t Sensor_pwrMeasStats =
    { 0 };
#endif
/******************************************************************************
 Local variables
 *****************************************************************************/

static void *sem;

/*! Rejoined flag */
static bool rejoining = false;

/*! Collector's address */
static ApiMac_sAddr_t collectorAddr = {0};

/* Join Time Ticks (used for average join time calculations) */
static uint_fast32_t joinTimeTicks = 0;

/* End to end delay statistics timestamp */
static uint32_t startSensorMsgTimeStamp = 0;

/*! Device's Outgoing MSDU Handle values */
STATIC uint8_t deviceTxMsduHandle = 0;

STATIC Smsgs_configReqMsg_t configSettings;

#if !defined(OAD_IMG_A)
/*!
 Temp Sensor field - valid only if Smsgs_dataFields_tempSensor
 is set in frameControl.
 */
STATIC Smsgs_tempSensorField_t tempSensor =
    { 0 };

/*!
 Light Sensor field - valid only if Smsgs_dataFields_lightSensor
 is set in frameControl.
 */
STATIC Smsgs_lightSensorField_t lightSensor =
    { 0 };

/*!
 Humidity Sensor field - valid only if Smsgs_dataFields_humiditySensor
 is set in frameControl.
 */
STATIC Smsgs_humiditySensorField_t humiditySensor =
    { 0 };
#endif //OAD_IMG_A

/*!
 id0 Sensor field - valid only if Smsgs_dataFields_id0Sensor
 is set in frameControl.
 */
STATIC Smsgs_id0SensorField_t id0Sensor =
    { 0 };

/*!
 id1 Sensor field - valid only if Smsgs_dataFields_id1Sensor
 is set in frameControl.
 */
STATIC Smsgs_id1SensorField_t id1Sensor =
    { 0 };

/*!
 id2 Sensor field - valid only if Smsgs_dataFields_id2Sensor
 is set in frameControl.
 */
STATIC Smsgs_id2SensorField_t id2Sensor =
    { 0 };

/*!
 id3 Sensor field - valid only if Smsgs_dataFields_id3Sensor
 is set in frameControl.
 */
STATIC Smsgs_id3SensorField_t id3Sensor =
    { 0 };

/*!
 type Sensor field - valid only if Smsgs_dataFields_typeSensor
 is set in frameControl.
 */
STATIC Smsgs_typeSensorField_t typeSensor =
    { 0 };

/*!
 battery Sensor field - valid only if Smsgs_dataFields_batterySensor
 is set in frameControl.
 */
STATIC Smsgs_batterySensorField_t batterySensor =
    { 0 };

/*!
 f0 Sensor field - valid only if Smsgs_dataFields_f0Sensor
 is set in frameControl.
 */
STATIC Smsgs_f0SensorField_t f0Sensor =
    { 0 };

/*!
 f1 Sensor field - valid only if Smsgs_dataFields_f1Sensor
 is set in frameControl.
 */
STATIC Smsgs_f1SensorField_t f1Sensor =
    { 0 };

/*!
 f2 Sensor field - valid only if Smsgs_dataFields_f2Sensor
 is set in frameControl.
 */
STATIC Smsgs_f2SensorField_t f2Sensor =
    { 0 };

/*!
 f3 Sensor field - valid only if Smsgs_dataFields_f3Sensor
 is set in frameControl.
 */
STATIC Smsgs_f3SensorField_t f3Sensor =
    { 0 };

/*!
 f4 Sensor field - valid only if Smsgs_dataFields_f4Sensor
 is set in frameControl.
 */
STATIC Smsgs_f4SensorField_t f4Sensor =
    { 0 };

/*!
 f5 Sensor field - valid only if Smsgs_dataFields_f5Sensor
 is set in frameControl.
 */
STATIC Smsgs_f5SensorField_t f5Sensor =
    { 0 };

/*!
 f6 Sensor field - valid only if Smsgs_dataFields_f6Sensor
 is set in frameControl.
 */
STATIC Smsgs_f6SensorField_t f6Sensor =
    { 0 };

/*!
 f7 Sensor field - valid only if Smsgs_dataFields_f7Sensor
 is set in frameControl.
 */
STATIC Smsgs_f7SensorField_t f7Sensor =
    { 0 };

/*!
 f8 Sensor field - valid only if Smsgs_dataFields_f8Sensor
 is set in frameControl.
 */
STATIC Smsgs_f8SensorField_t f8Sensor =
    { 0 };

/*!
 f9 Sensor field - valid only if Smsgs_dataFields_f9Sensor
 is set in frameControl.
 */
STATIC Smsgs_f9SensorField_t f9Sensor =
    { 0 };




STATIC Llc_netInfo_t parentInfo = {0};

STATIC uint16_t lastRcvdBroadcastMsgId = 0;

#ifdef FEATURE_SECURE_COMMISSIONING
/* variable to store the current setting of auto Request Pib attribute
 * before it gets modified by SM module, in beacon mode
 */
static bool currAutoReq = 0;
#endif /* FEATURE_SECURE_COMMISSIONING */

#ifdef OAD_IMG_A
static bool Oad_hasSentResetRsp = false;
#endif /* OAD_IMG_A */

/******************************************************************************
 Local function prototypes
 *****************************************************************************/
static void initializeClocks(void);
static void dataCnfCB(ApiMac_mcpsDataCnf_t *pDataCnf);
static void dataIndCB(ApiMac_mcpsDataInd_t *pDataInd);
static uint8_t getMsduHandle(Smsgs_cmdIds_t msgType);

#if !defined(OAD_IMG_A)
static void processSensorMsgEvt(void);
static bool sendSensorMessage(ApiMac_sAddr_t *pDstAddr,
                              Smsgs_sensorMsg_t *pMsg);
static void readSensors(void);
#endif //OAD_IMG_A

#if SENSOR_TEST_RAMP_DATA_SIZE
static void processSensorRampMsgEvt(void);
#endif

static void processConfigRequest(ApiMac_mcpsDataInd_t *pDataInd);
static void processBroadcastCtrlMsg(ApiMac_mcpsDataInd_t *pDataInd);
static bool sendConfigRsp(ApiMac_sAddr_t *pDstAddr, Smsgs_configRspMsg_t *pMsg);
static uint16_t validateFrameControl(uint16_t frameControl);

static void jdllcJoinedCb(ApiMac_deviceDescriptor_t *pDevInfo,
                          Llc_netInfo_t  *pStartedInfo);
static void jdllcDisassocIndCb(ApiMac_sAddrExt_t *extAddress,
                               ApiMac_disassocateReason_t reason);
static void jdllcDisassocCnfCb(ApiMac_sAddrExt_t *extAddress,
                               ApiMac_status_t status);
static void jdllcStateChangeCb(Jdllc_states_t state);


#ifdef FEATURE_SECURE_COMMISSIONING
/* Security Manager callback functions */
static void smFailCMProcessCb(ApiMac_deviceDescriptor_t *devInfo,
                              bool rxOnIdle, bool keyRefreshment);
static void smSuccessCMProcessCb(ApiMac_deviceDescriptor_t *devInfo,
                                 bool keyRefreshment);
#endif

/******************************************************************************
 Callback tables
 *****************************************************************************/

/*! API MAC Callback table */
STATIC ApiMac_callbacks_t Sensor_macCallbacks =
    {
      /*! Associate Indicated callback */
      NULL,
      /*! Associate Confirmation callback */
      NULL,
      /*! Disassociate Indication callback */
      NULL,
      /*! Disassociate Confirmation callback */
      NULL,
      /*! Beacon Notify Indication callback */
      NULL,
      /*! Orphan Indication callback */
      NULL,
      /*! Scan Confirmation callback */
      NULL,
      /*! Start Confirmation callback */
      NULL,
      /*! Sync Loss Indication callback */
      NULL,
      /*! Poll Confirm callback */
      NULL,
      /*! Comm Status Indication callback */
      NULL,
      /*! Poll Indication Callback */
      NULL,
      /*! Data Confirmation callback */
      dataCnfCB,
      /*! Data Indication callback */
      dataIndCB,
      /*! Purge Confirm callback */
      NULL,
      /*! WiSUN Async Indication callback */
      NULL,
      /*! WiSUN Async Confirmation callback */
      NULL,
      /*! Unprocessed message callback */
      NULL
    };

STATIC Jdllc_callbacks_t jdllcCallbacks =
    {
      /*! Network Joined Indication callback */
      jdllcJoinedCb,
      /* Disassociation Indication callback */
      jdllcDisassocIndCb,
      /* Disassociation Confirm callback */
      jdllcDisassocCnfCb,
      /*! State Changed indication callback */
      jdllcStateChangeCb
    };
#ifdef FEATURE_SECURE_COMMISSIONING
STATIC SM_callbacks_t SMCallbacks =
    {
      /*! Security authentication failed callback */
      smFailCMProcessCb,
      /* Security authentication successful callback */
      smSuccessCMProcessCb
    };
#endif

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 Initialize this application.

 Public function defined in sensor.h
 */
#ifdef OSAL_PORT2TIRTOS
void Sensor_init(uint8_t macTaskId)
#else
void Sensor_init(void)
#endif
{
    uint32_t frameCounter = 0;

    /* Initialize the sensor's structures */
    memset(&configSettings, 0, sizeof(Smsgs_configReqMsg_t));
#if defined(TEMP_SENSOR)
    configSettings.frameControl |= Smsgs_dataFields_tempSensor;
#endif
#if defined(LIGHT_SENSOR)
    configSettings.frameControl |= Smsgs_dataFields_lightSensor;
#endif
#if defined(HUMIDITY_SENSOR)
    configSettings.frameControl |= Smsgs_dataFields_humiditySensor;
#endif
    configSettings.frameControl |= Smsgs_dataFields_msgStats;
    configSettings.frameControl |= Smsgs_dataFields_configSettings;


    //id0 sensor
    configSettings.frameControl |= Smsgs_dataFields_id0Sensor;
    //id1 sensor
    configSettings.frameControl |= Smsgs_dataFields_id1Sensor;
    //id2 sensor
    configSettings.frameControl |= Smsgs_dataFields_id2Sensor;
    //id3 sensor
    configSettings.frameControl |= Smsgs_dataFields_id3Sensor;
    //type sensor
    configSettings.frameControl |= Smsgs_dataFields_typeSensor;
    //battery sensor
    configSettings.frameControl |= Smsgs_dataFields_batterySensor;

    //f0 sensor
    configSettings.frameControl |= Smsgs_dataFields_f0Sensor;
    //f1 sensor
    configSettings.frameControl |= Smsgs_dataFields_f1Sensor;
    //f2 sensor
    configSettings.frameControl |= Smsgs_dataFields_f2Sensor;
    //f3 sensor
    configSettings.frameControl |= Smsgs_dataFields_f3Sensor;
    //f4 sensor
    configSettings.frameControl |= Smsgs_dataFields_f4Sensor;
    //f5 sensor
    configSettings.frameControl |= Smsgs_dataFields_f5Sensor;
    //f6 sensor
    configSettings.frameControl |= Smsgs_dataFields_f6Sensor;
    //f7 sensor
    configSettings.frameControl |= Smsgs_dataFields_f7Sensor;
    //f8 sensor
    configSettings.frameControl |= Smsgs_dataFields_f8Sensor;
    //f9 sensor
    configSettings.frameControl |= Smsgs_dataFields_f9Sensor;


    if(!CERTIFICATION_TEST_MODE)
    {
        configSettings.reportingInterval = CONFIG_REPORTING_INTERVAL;
    }
    else
    {
        /* start back to back data transmission at the earliest */
        configSettings.reportingInterval = 100;
    }
    configSettings.pollingInterval = CONFIG_POLLING_INTERVAL;

    /* Initialize the MAC */
#ifdef OSAL_PORT2TIRTOS
    sem = ApiMac_init(macTaskId, CONFIG_FH_ENABLE);
#else
    sem = ApiMac_init(CONFIG_FH_ENABLE);
#endif

    /* Initialize the Joining Device Logical Link Controller */
    Jdllc_init(&Sensor_macCallbacks, &jdllcCallbacks);

    /* Register the MAC Callbacks */
    ApiMac_registerCallbacks(&Sensor_macCallbacks);

    /* Initialize the platform specific functions */
    Ssf_init(sem);

#ifdef FEATURE_SECURE_COMMISSIONING
    /* Intialize the security manager and register callbacks */
    SM_init();
    SM_registerCallback(&SMCallbacks);
#endif /* FEATURE_SECURE_COMMISSIONING */
    ApiMac_mlmeSetReqUint8(ApiMac_attribute_phyCurrentDescriptorId,
                           (uint8_t)CONFIG_PHY_ID);

    ApiMac_mlmeSetReqUint8(ApiMac_attribute_channelPage,
                           (uint8_t)CONFIG_CHANNEL_PAGE);

    Ssf_getFrameCounter(NULL, &frameCounter);

#ifdef FEATURE_MAC_SECURITY
    /* Initialize the MAC Security */
    Jdllc_securityInit(frameCounter);
#endif /* FEATURE_MAC_SECURITY */

    /* Set the transmit power */
    ApiMac_mlmeSetReqUint8(ApiMac_attribute_phyTransmitPowerSigned,
                           (uint8_t)CONFIG_TRANSMIT_POWER);
    /* Set Min BE */
    ApiMac_mlmeSetReqUint8(ApiMac_attribute_backoffExponent,
                              (uint8_t)CONFIG_MIN_BE);
    /* Set Max BE */
    ApiMac_mlmeSetReqUint8(ApiMac_attribute_maxBackoffExponent,
                              (uint8_t)CONFIG_MAX_BE);
    /* Set MAC MAX CSMA Backoffs */
    ApiMac_mlmeSetReqUint8(ApiMac_attribute_maxCsmaBackoffs,
                              (uint8_t)CONFIG_MAC_MAX_CSMA_BACKOFFS);
    /* Set MAC MAX Frame Retries */
    ApiMac_mlmeSetReqUint8(ApiMac_attribute_maxFrameRetries,
                              (uint8_t)CONFIG_MAX_RETRIES);
#ifdef FCS_TYPE16
    /* Set the fcs type */
    ApiMac_mlmeSetReqBool(ApiMac_attribute_fcsType,
                           (bool)1);
#endif
    /* Initialize the app clocks */
    initializeClocks();

    if(CONFIG_AUTO_START)
    {
        /* Start the device */
        Util_setEvent(&Sensor_events, SENSOR_START_EVT);
    }

    // unique MAC address identifier obtaintion
    // itis sent in the collector message to identify the sensor in the network
    uint32_t macAddrlsb_32 = HWREG(FCFG1_BASE + FCFG1_O_MAC_15_4_0);
    uint32_t macAddrmsb_32 = HWREG(FCFG1_BASE + FCFG1_O_MAC_15_4_1);

    uint16_t macAddr_16[4];

    macAddr_16[0] = (macAddrlsb_32 & 0x0000ffff);       //lowest part
    macAddr_16[1] = (macAddrlsb_32 & 0xffff0000) >> 16;
    macAddr_16[2] = (macAddrmsb_32 & 0x0000ffff);
    macAddr_16[3] = (macAddrmsb_32 & 0xffff0000) >> 16; //biggest part


    id0Sensor.id0RawData = macAddr_16[0];         // id sensor
    id1Sensor.id1RawData = macAddr_16[1];
    id2Sensor.id2RawData = macAddr_16[2];
    id3Sensor.id3RawData = macAddr_16[3];

    //define the type of sensor
    typeSensor.typeRawData = DEF_SENSOR_TYPE;   // type sensor
}

/*!
 Application task processing.

 Public function defined in sensor.h
 */
void Sensor_process(void)
{
    /* Start the collector device in the network */
    if(Sensor_events & SENSOR_START_EVT)
    {
        ApiMac_deviceDescriptor_t devInfo;
        Llc_netInfo_t parentInfo;

        if(Ssf_getNetworkInfo(&devInfo, &parentInfo ) == true)
        {
            Ssf_configSettings_t configInfo;
#ifdef FEATURE_MAC_SECURITY
            ApiMac_status_t stat;
#endif /* FEATURE_MAC_SECURITY */

            /* Do we have config settings? */
            if(Ssf_getConfigInfo(&configInfo) == true)
            {
                /* Save the config information */
                configSettings.frameControl = configInfo.frameControl;
                configSettings.reportingInterval = configInfo.reportingInterval;
                configSettings.pollingInterval = configInfo.pollingInterval;

                /* Update the polling interval in the LLC */
                Jdllc_setPollRate(configSettings.pollingInterval);
            }

            /* Initially, setup the parent as the collector */
            if(parentInfo.fh == true && CONFIG_RX_ON_IDLE)
            {
                collectorAddr.addrMode = ApiMac_addrType_extended;
                memcpy(&collectorAddr.addr.extAddr,
                       parentInfo.devInfo.extAddress, APIMAC_SADDR_EXT_LEN);
            }
            else
            {
                collectorAddr.addrMode = ApiMac_addrType_short;
                collectorAddr.addr.shortAddr = parentInfo.devInfo.shortAddress;
            }

#ifdef FEATURE_MAC_SECURITY
            /* Put the parent in the security device list */
            stat = Jdllc_addSecDevice(parentInfo.devInfo.panID,
                                      parentInfo.devInfo.shortAddress,
                                      &parentInfo.devInfo.extAddress, 0);
            if(stat != ApiMac_status_success)
            {
                Ssf_displayError("Auth Error: 0x", (uint8_t)stat);
            }
#endif /* FEATURE_MAC_SECURITY */

#ifdef FEATURE_SECURE_COMMISSIONING
            if(!CONFIG_FH_ENABLE)
            {
                nvDeviceKeyInfo_t devKeyInfo;
                SM_seedKey_Entry_t * pSeedKeyEnty;
                if(Ssf_getDeviceKeyInfo(&devKeyInfo) == TRUE)
                {
                    /* Update the seedKeyTable and MAC Key Table */
                    /* Use its own ext address */
                    updateSeedKeyFromNV(&devInfo,&devKeyInfo);
                    pSeedKeyEnty = getEntryFromSeedKeyTable(devInfo.extAddress,devInfo.shortAddress);
                    /* Do not change the order below to lines */
                    /* Copy collector ext Address first */
                    memcpy(commissionDevInfo.extAddress, parentInfo.devInfo.extAddress, sizeof(ApiMac_sAddrExt_t));
                    addDeviceKey(pSeedKeyEnty,devKeyInfo.deviceKey, true);
                    LCD_WRITE_STRING("KeyInfo recovered", 6);
                }
            }
#endif
            Jdllc_rejoin(&devInfo, &parentInfo);
            rejoining = true;

        }
        else
        {
            /* Get Start Timestamp */
#ifdef OSAL_PORT2TIRTOS
            joinTimeTicks = Clock_getTicks();
#else
            joinTimeTicks = ICall_getTicks();
#endif
            Jdllc_join();
        }

        /* Clear the event */
        Util_clearEvent(&Sensor_events, SENSOR_START_EVT);
    }

    /* Is it time to send the next sensor data message? */
    if(Sensor_events & SENSOR_READING_TIMEOUT_EVT)
    {

#if !defined(OAD_IMG_A)

        /* In certification test mode, back to back data shall be sent */
        if(!CERTIFICATION_TEST_MODE)
        {
            /* Setup for the next message */
            Ssf_setReadingClock(configSettings.reportingInterval);
        }


#ifdef FEATURE_SECURE_COMMISSIONING
        /* if secure Commissioning feature is enabled, read
         * sensor data and send it only after the secure
         * commissioning process is done successfully.
         * else, do not read and send sensor data.
         */
        if(SM_Last_State != SM_CM_InProgress)
        {
#endif //FEATURE_SECURE_COMMISSIONING


#if SENSOR_TEST_RAMP_DATA_SIZE
        processSensorRampMsgEvt();
#else
        /* Read sensors */
        readSensors();

        /* Process Sensor Reading Message Event */
        processSensorMsgEvt();
#endif //SENSOR_TEST_RAMP_DATA_SIZE
#ifdef FEATURE_SECURE_COMMISSIONING
        }
#endif //FEATURE_SECURE_COMMISSIONING

#endif //OAD_IMG_A

        /* Clear the event */
        Util_clearEvent(&Sensor_events, SENSOR_READING_TIMEOUT_EVT);
    }

#if defined(OAD_IMG_A)
    if(Sensor_events & SENSOR_OAD_SEND_RESET_RSP_EVT )
    {
        /* send OAD reset response */
        if( false == Oad_hasSentResetRsp)
        {
            OADProtocol_Status_t  status;
            status = OADProtocol_sendOadResetRsp(&collectorAddr);

            if(OADProtocol_Status_Success == status)
            {
                //notify to user
                LCD_WRITE_STRING("Sent Reset Response", 6);
                Oad_hasSentResetRsp = true;
            }
            else
            {
                LCD_WRITE_STRING(" Failed to send Reset Response", 6);
            }
        }

        /* Clear the event */
        Util_clearEvent(&Sensor_events, SENSOR_OAD_SEND_RESET_RSP_EVT);
    }
#endif //OAD_IMG_A

#ifdef DISPLAY_PER_STATS
    if(Sensor_events & SENSOR_UPDATE_STATS_EVT)
    {
        Ssf_displayPerStats(&Sensor_msgStats);
        /* Clear the event */
        Util_clearEvent(&Sensor_events, SENSOR_UPDATE_STATS_EVT);
    }
#endif /* DISPLAY_PER_STATS */

    /* Process LLC Events */
    Jdllc_process();

    /* Allow the Specific functions to process */
    Ssf_processEvents();

#ifdef FEATURE_SECURE_COMMISSIONING
    /* Allow the security manager specific functions to process */
    SM_process();
#endif /* FEATURE_SECURE_COMMISSIONING */
    /*
     Don't process ApiMac messages until all of the sensor events
     are processed.
     */
#ifdef FEATURE_SECURE_COMMISSIONING
    /*only if there are no sensor events and security manager events to handle*/
    if((Sensor_events == 0) && (SM_events == 0))
#else
    if(Sensor_events == 0)
#endif
    {
        /* Wait for response message or events */
        ApiMac_processIncoming();
    }
}

/*!
 * @brief   Send MAC data request
 *
 * @param   type - message type
 * @param   pDstAddr - destination address
 * @param   rxOnIdle - true if not a sleepy device
 * @param   len - length of payload
 * @param   pData - pointer to the buffer
 *
 * @return  true if sent, false if not
 */
bool Sensor_sendMsg(Smsgs_cmdIds_t type, ApiMac_sAddr_t *pDstAddr,
                    bool rxOnIdle, uint16_t len, uint8_t *pData)
{
    bool ret = false;
    ApiMac_mcpsDataReq_t dataReq;

    /* Timestamp to compute end to end delay */
#ifdef OSAL_PORT2TIRTOS
    startSensorMsgTimeStamp = Clock_getTicks();
#else
    startSensorMsgTimeStamp = ICall_getTicks();
#endif
    /* Fill the data request field */
    memset(&dataReq, 0, sizeof(ApiMac_mcpsDataReq_t));

    memcpy(&dataReq.dstAddr, pDstAddr, sizeof(ApiMac_sAddr_t));

    if(pDstAddr->addrMode == ApiMac_addrType_extended)
    {
        dataReq.srcAddrMode = ApiMac_addrType_extended;
    }
    else
    {
        dataReq.srcAddrMode = ApiMac_addrType_short;
    }

    if(rejoining == true)
    {
        ApiMac_mlmeGetReqUint16(ApiMac_attribute_panId,
                                &(parentInfo.devInfo.panID));
    }

    dataReq.dstPanId = parentInfo.devInfo.panID;

    dataReq.msduHandle = getMsduHandle(type);

    dataReq.txOptions.ack = true;

    if(CERTIFICATION_TEST_MODE)
    {
        dataReq.txOptions.ack = false;
    }

    if(rxOnIdle == false)
    {
        dataReq.txOptions.indirect = true;
    }

    dataReq.msdu.len = len;
    dataReq.msdu.p = pData;

#ifdef FEATURE_MAC_SECURITY
#ifdef FEATURE_SECURE_COMMISSIONING
    {
        extern ApiMac_sAddrExt_t ApiMac_extAddr;
        SM_getSrcDeviceSecurityInfo(ApiMac_extAddr, SM_Sensor_SAddress, &dataReq.sec);
    }
#else
    Jdllc_securityFill(&dataReq.sec);
#endif /* FEATURE_SECURE_COMMISSIONING */
#endif /* FEATURE_MAC_SECURITY */

    if(type == Smsgs_cmdIds_sensorData || type == Smsgs_cmdIds_rampdata)
    {
        Sensor_msgStats.msgsAttempted++;
    }
    else if(type == Smsgs_cmdIds_trackingRsp)
    {
        Sensor_msgStats.trackingResponseAttempts++;
    }
    else if(type == Smsgs_cmdIds_configRsp)
    {
        Sensor_msgStats.configResponseAttempts++;
    }

    /* Send the message */
    if(ApiMac_mcpsDataReq(&dataReq) == ApiMac_status_success)
    {
        ret = true;
    }
    else
    {
        /* handle transaction overflow by retrying */
        if(type == Smsgs_cmdIds_sensorData || type == Smsgs_cmdIds_rampdata)
        {
            Ssf_setReadingClock(configSettings.reportingInterval);
            Sensor_msgStats.msgsAttempted++;
        }
    }

    return (ret);
}


#ifdef IDENTIFY_LED
/*!
 Send LED Identify Request to collector

 Public function defined in sensor.h
 */
void Sensor_sendIdentifyLedRequest(void)
{
    uint8_t cmdBytes[SMSGS_INDENTIFY_LED_REQUEST_MSG_LEN];

    /* send the response message directly */
    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_IdentifyLedReq;
    cmdBytes[1] = (uint8_t) IDENTIFY_LED_TIME;
    Sensor_sendMsg(Smsgs_cmdIds_IdentifyLedReq,
            &collectorAddr, true,
            SMSGS_INDENTIFY_LED_REQUEST_MSG_LEN,
            cmdBytes);
}
#endif
/******************************************************************************
 Local Functions
 *****************************************************************************/

/*!
 * @brief       Initialize the clocks.
 */
static void initializeClocks(void)
{
    /* Initialize the reading clock */
    Ssf_initializeReadingClock();
}

/*!
 * @brief      MAC Data Confirm callback.
 *
 * @param      pDataCnf - pointer to the data confirm information
 */
static void dataCnfCB(ApiMac_mcpsDataCnf_t *pDataCnf)
{
    uint16_t endToEndDelay = 0;
    /* Record statistics */
    if(pDataCnf->status == ApiMac_status_channelAccessFailure)
    {
        Sensor_msgStats.channelAccessFailures++;
    }
    else if(pDataCnf->status == ApiMac_status_noAck)
    {
        Sensor_msgStats.macAckFailures++;
#ifdef DISPLAY_PER_STATS
        Util_setEvent(&Sensor_events, SENSOR_UPDATE_STATS_EVT);
#endif
    }
    else if(pDataCnf->status != ApiMac_status_success)
    {
        Sensor_msgStats.otherDataRequestFailures++;
#ifdef DISPLAY_PER_STATS
        Util_setEvent(&Sensor_events, SENSOR_UPDATE_STATS_EVT);
#endif
        Ssf_displayError("dataCnf: ", pDataCnf->status);
    }
    else if(pDataCnf->status == ApiMac_status_success)
    {
        Ssf_updateFrameCounter(NULL, pDataCnf->frameCntr);
    }

    /* Make sure the message came from the app */
    if(pDataCnf->msduHandle & APP_MARKER_MSDU_HANDLE)
    {
        /* What message type was the original request? */
        if((pDataCnf->msduHandle & APP_MASK_MSDU_HANDLE)
           == APP_SENSOR_MSDU_HANDLE)
        {
            if(pDataCnf->status == ApiMac_status_success)
            {
                Sensor_msgStats.msgsSent++;
#ifdef DISPLAY_PER_STATS
                Util_setEvent(&Sensor_events, SENSOR_UPDATE_STATS_EVT);
#endif
                /* Calculate end to end delay */
#ifdef OSAL_PORT2TIRTOS
                if(Clock_getTicks() < startSensorMsgTimeStamp)
                {
                    endToEndDelay = Clock_getTicks() +
                                    (0xFFFFFFFF-startSensorMsgTimeStamp);
                }
                else
                {
                    endToEndDelay = Clock_getTicks() - startSensorMsgTimeStamp;
                }
#else
                if(ICall_getTicks() < startSensorMsgTimeStamp)
                {
                    endToEndDelay = ICall_getTicks() +
                                    (0xFFFFFFFF-startSensorMsgTimeStamp);
                }
                else
                {
                    endToEndDelay = ICall_getTicks() - startSensorMsgTimeStamp;
                }
#endif
                endToEndDelay = endToEndDelay/TICKPERIOD_MS_US;

                Sensor_msgStats.worstCaseE2EDelay =
                   (Sensor_msgStats.worstCaseE2EDelay > endToEndDelay) ?
                    Sensor_msgStats.worstCaseE2EDelay:endToEndDelay;
                Sensor_msgStats.avgE2EDelay =
                    (((uint32_t)Sensor_msgStats.avgE2EDelay *
                      (Sensor_msgStats.msgsSent - 1)) + endToEndDelay)/
                     Sensor_msgStats.msgsSent;

            }


#if CERTIFICATION_TEST_MODE
            {
                /* Setup for the next message */
                Ssf_setReadingClock(CERT_MODE_INTER_PKT_INTERVAL);
            }
#endif
        }
        if((pDataCnf->msduHandle & APP_MASK_MSDU_HANDLE)
           == APP_TRACKRSP_MSDU_HANDLE)
        {
            if(pDataCnf->status == ApiMac_status_success)
            {
                Sensor_msgStats.trackingResponseSent++;
            }
        }
        if((pDataCnf->msduHandle & APP_MASK_MSDU_HANDLE)
           == APP_CONFIGRSP_MSDU_HANDLE)
        {
            if(pDataCnf->status == ApiMac_status_success)
            {
                Sensor_msgStats.configResponseSent++;
            }
        }
    }
}

/*!
 * @brief      MAC Data Indication callback.
 *
 * @param      pDataInd - pointer to the data indication information
 */
static void dataIndCB(ApiMac_mcpsDataInd_t *pDataInd)
{
    uint8_t cmdBytes[SMSGS_TOGGLE_LED_RESPONSE_MSG_LEN];

    if((pDataInd != NULL) && (pDataInd->msdu.p != NULL)
       && (pDataInd->msdu.len > 0))
    {
        Smsgs_cmdIds_t cmdId = (Smsgs_cmdIds_t)*(pDataInd->msdu.p);

#ifdef FEATURE_MAC_SECURITY
        {
            if(Jdllc_securityCheck(&(pDataInd->sec)) == false)
            {
                /* reject the message */
                return;
            }
        }
#endif /* FEATURE_MAC_SECURITY */

        switch(cmdId)
        {
            case Smsgs_cmdIds_configReq:
                processConfigRequest(pDataInd);
                Sensor_msgStats.configRequests++;
                break;

            case Smsgs_cmdIds_trackingReq:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_TRACKING_REQUEST_MSG_LENGTH)
                {
                    /* Update stats */
                    Sensor_msgStats.trackingRequests++;

                    /* Indicate tracking message received */
                    Ssf_trackingUpdate(&pDataInd->srcAddr);

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_trackingRsp;
                    Sensor_sendMsg(Smsgs_cmdIds_trackingRsp,
                            &pDataInd->srcAddr, true,
                            1, cmdBytes);
                }
                break;

            case Smsgs_cmdIds_toggleLedReq:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_TOGGLE_LED_REQUEST_MSG_LEN)
                {

                    Sensor_sendMsg(Smsgs_cmdIds_toggleLedRsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_TOGGLE_LED_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            case Smgs_cmdIds_broadcastCtrlMsg:
                if(parentFound)
                {
                    /* Node has successfully associated with the network */
                    processBroadcastCtrlMsg(pDataInd);
                }
                break;
#ifdef POWER_MEAS
            case Smsgs_cmdIds_rampdata:
                Sensor_pwrMeasStats.rampDataRcvd++;
                break;
#endif

#ifdef FEATURE_NATIVE_OAD
            case Smsgs_cmdIds_oad:
                //Index past the Smsgs_cmdId
                OADProtocol_ParseIncoming((void*) &(pDataInd->srcAddr), pDataInd->msdu.p + 1);
                break;
#endif //FEATURE_NATIVE_OAD
#ifdef FEATURE_SECURE_COMMISSIONING
            case Smgs_cmdIds_CommissionStart:
                {
                    ApiMac_sec_t devSec;
                    extern ApiMac_sAddrExt_t ApiMac_extAddr;

                    /* Obtain MAC level security information. Use network key for SM */
                    Jdllc_securityFill(&devSec);

                    uint8_t *pBuf = pDataInd->msdu.p;
                    pBuf += sizeof(Smsgs_cmdIds_t);
                    SMMsgs_cmdIds_t CMMsgId = (SMMsgs_cmdIds_t)Util_buildUint16(pBuf[0], pBuf[1]);

                    /* read the current value */
                    ApiMac_mlmeGetReqBool(ApiMac_attribute_autoRequest, &currAutoReq);

                    /* beacon-mode of operation and autoRequest is set to true */
                    if((CONFIG_MAC_BEACON_ORDER != 15) && (currAutoReq == true))
                    {
                        /* if false enable explicit polling */
                        ApiMac_mlmeSetReqBool(ApiMac_attribute_autoRequest, false);
                        Util_setEvent(&Jdllc_events, JDLLC_POLL_EVT);
                    }

                    if ((SM_Last_State != SM_CM_InProgress) &&
                        (CMMsgId == SMMsgs_cmdIds_KeyRefreshRequest))
                    {
                        /* Kick off key refreshment process after successful commissioning */
                        SM_startKeyRefreshProcess(&parentInfo.devInfo, &devSec,
                                                  parentInfo.fh, true);
                    }
                    else
                    {
                        /* Kick off commissioning process to obtain security information */
                        SM_startCMProcess(&parentInfo.devInfo, &devSec, parentInfo.fh,
                                          true, SM_type_device, SM_SENSOR_AUTH_METHOD);
                    }
                }
                break;
            case Smgs_cmdIds_CommissionMsg:
                {
                    /* Process Security manager commissioning data */
                    SM_processCommData(pDataInd);
                }
                break;


#endif /* FEATURE_SECURE_COMMISSIONING */

            // id0 sensor
            case Smsgs_cmdIds_id0Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_ID0_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_id0Rsp;
                    cmdBytes[1] = Ssf_id0CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_id0Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_ID0_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // id1 sensor
            case Smsgs_cmdIds_id1Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_ID1_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_id1Rsp;
                    cmdBytes[1] = Ssf_id1CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_id1Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_ID1_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // id2 sensor
            case Smsgs_cmdIds_id2Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_ID2_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_id2Rsp;
                    cmdBytes[1] = Ssf_id2CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_id2Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_ID2_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // id3 sensor
            case Smsgs_cmdIds_id3Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_ID3_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_id3Rsp;
                    cmdBytes[1] = Ssf_id3CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_id3Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_ID3_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // type sensor
            case Smsgs_cmdIds_typeReq:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_TYPE_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_typeRsp;
                    cmdBytes[1] = Ssf_typeCMD();
                    Sensor_sendMsg(Smsgs_cmdIds_typeRsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_TYPE_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // battery sensor
            case Smsgs_cmdIds_batteryReq:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_BATTERY_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_batteryRsp;
                    cmdBytes[1] = Ssf_batteryCMD();
                    Sensor_sendMsg(Smsgs_cmdIds_batteryRsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_BATTERY_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f0 sensor
            case Smsgs_cmdIds_f0Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F0_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f0Rsp;
                    cmdBytes[1] = Ssf_f0CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f0Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F0_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f1 sensor
            case Smsgs_cmdIds_f1Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F1_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f1Rsp;
                    cmdBytes[1] = Ssf_f1CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f1Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F1_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f2 sensor
            case Smsgs_cmdIds_f2Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F2_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f2Rsp;
                    cmdBytes[1] = Ssf_f2CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f2Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F2_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f3 sensor
            case Smsgs_cmdIds_f3Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F3_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f3Rsp;
                    cmdBytes[1] = Ssf_f3CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f3Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F3_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f4 sensor
            case Smsgs_cmdIds_f4Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F4_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f4Rsp;
                    cmdBytes[1] = Ssf_f4CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f4Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F4_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f5 sensor
            case Smsgs_cmdIds_f5Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F5_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f5Rsp;
                    cmdBytes[1] = Ssf_f5CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f5Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F5_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f6 sensor
            case Smsgs_cmdIds_f6Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F6_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f6Rsp;
                    cmdBytes[1] = Ssf_f6CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f6Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F6_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f7 sensor
            case Smsgs_cmdIds_f7Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F7_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f7Rsp;
                    cmdBytes[1] = Ssf_f7CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f7Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F7_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f8 sensor
            case Smsgs_cmdIds_f8Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F8_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f8Rsp;
                    cmdBytes[1] = Ssf_f8CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f8Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F8_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;

            // f9 sensor
            case Smsgs_cmdIds_f9Req:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_F9_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_f9Rsp;
                    cmdBytes[1] = Ssf_f9CMD();
                    Sensor_sendMsg(Smsgs_cmdIds_f9Rsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_F9_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;



            default:
                /* Should not receive other messages */
                break;
        }
    }
}

/*!
 * @brief      Get the next MSDU Handle
 *             <BR>
 *             The MSDU handle has 3 parts:<BR>
 *             - The MSBit(7), when set means the the application sent the
 *               message
 *             - Bit 6, when set means that the app message is a config request
 *             - Bits 0-5, used as a message counter that rolls over.
 *
 * @param      msgType - message command id needed
 *
 * @return     msdu Handle
 */
static uint8_t getMsduHandle(Smsgs_cmdIds_t msgType)
{
    uint8_t msduHandle = deviceTxMsduHandle;

    /* Increment for the next msdu handle, or roll over */
    if(deviceTxMsduHandle >= MSDU_HANDLE_MAX)
    {
        deviceTxMsduHandle = 0;
    }
    else
    {
        deviceTxMsduHandle++;
    }

    /* Add the App specific bit */
    msduHandle |= APP_MARKER_MSDU_HANDLE;

    /* Add the message type bit */
    if(msgType == Smsgs_cmdIds_sensorData || msgType == Smsgs_cmdIds_rampdata)
    {
        msduHandle |= APP_SENSOR_MSDU_HANDLE;
    }
    else if(msgType == Smsgs_cmdIds_trackingRsp)
    {
        msduHandle |= APP_TRACKRSP_MSDU_HANDLE;
    }
    else if(msgType == Smsgs_cmdIds_configRsp)
    {
        msduHandle |= APP_CONFIGRSP_MSDU_HANDLE;
    }

    return (msduHandle);
}

/*!
 @brief  Build and send fixed size ramp data
 */
#if SENSOR_TEST_RAMP_DATA_SIZE
static void processSensorRampMsgEvt(void)
{
    uint8_t *pMsgBuf;
    uint16_t index;

    pMsgBuf = (uint8_t *)Ssf_malloc(SENSOR_TEST_RAMP_DATA_SIZE);
    if(pMsgBuf)
    {
        uint8_t *pBuf = pMsgBuf;
        *pBuf++ = (uint8_t)Smsgs_cmdIds_rampdata;
        for(index = 1; index < SENSOR_TEST_RAMP_DATA_SIZE; index++)
        {
            *pBuf++ = (uint8_t) (index & 0xFF);
        }

#ifndef POWER_MEAS
        Board_Led_toggle(board_led_type_LED2);
#endif

        Sensor_sendMsg(Smsgs_cmdIds_rampdata, &collectorAddr, true,
                SENSOR_TEST_RAMP_DATA_SIZE, pMsgBuf);

        Ssf_free(pMsgBuf);
    }

}
#endif

#if !defined(OAD_IMG_A)
/*!
 @brief   Build and send sensor data message
 */
static void processSensorMsgEvt(void)
{
    Smsgs_sensorMsg_t sensor;
    uint32_t stat;

    memset(&sensor, 0, sizeof(Smsgs_sensorMsg_t));

    ApiMac_mlmeGetReqUint32(ApiMac_attribute_diagRxSecureFail, &stat);
    Sensor_msgStats.rxDecryptFailures = (uint16_t)stat;

    ApiMac_mlmeGetReqUint32(ApiMac_attribute_diagTxSecureFail, &stat);
    Sensor_msgStats.txEncryptFailures = (uint16_t)stat;

    ApiMac_mlmeGetReqArray(ApiMac_attribute_extendedAddress,
    		               sensor.extAddress);

    /* fill in the message */
    sensor.frameControl = configSettings.frameControl;
    if(sensor.frameControl & Smsgs_dataFields_tempSensor)
    {
        memcpy(&sensor.tempSensor, &tempSensor,
               sizeof(Smsgs_tempSensorField_t));
    }
    if(sensor.frameControl & Smsgs_dataFields_lightSensor)
    {
        memcpy(&sensor.lightSensor, &lightSensor,
               sizeof(Smsgs_lightSensorField_t));
    }
    if(sensor.frameControl & Smsgs_dataFields_humiditySensor)
    {
        memcpy(&sensor.humiditySensor, &humiditySensor,
               sizeof(Smsgs_humiditySensorField_t));
    }
    if(sensor.frameControl & Smsgs_dataFields_msgStats)
    {
        memcpy(&sensor.msgStats, &Sensor_msgStats,
               sizeof(Smsgs_msgStatsField_t));
    }
    if(sensor.frameControl & Smsgs_dataFields_configSettings)
    {
        sensor.configSettings.pollingInterval = configSettings.pollingInterval;
        sensor.configSettings.reportingInterval = configSettings
                        .reportingInterval;
    }

    // id0 sensor
    if(sensor.frameControl & Smsgs_dataFields_id0Sensor)
    {
       memcpy(&sensor.id0Sensor, &id0Sensor,
              sizeof(Smsgs_id0SensorField_t));
    }

    // id1 sensor
    if(sensor.frameControl & Smsgs_dataFields_id1Sensor)
    {
       memcpy(&sensor.id1Sensor, &id1Sensor,
              sizeof(Smsgs_id1SensorField_t));
    }

    // id2 sensor
    if(sensor.frameControl & Smsgs_dataFields_id2Sensor)
    {
       memcpy(&sensor.id2Sensor, &id2Sensor,
              sizeof(Smsgs_id2SensorField_t));
    }

    // id3 sensor
    if(sensor.frameControl & Smsgs_dataFields_id3Sensor)
    {
       memcpy(&sensor.id3Sensor, &id3Sensor,
              sizeof(Smsgs_id0SensorField_t));
    }
    // type sensor
    if(sensor.frameControl & Smsgs_dataFields_typeSensor)
    {
       memcpy(&sensor.typeSensor, &typeSensor,
              sizeof(Smsgs_typeSensorField_t));
    }
    // battery sensor
    if(sensor.frameControl & Smsgs_dataFields_batterySensor)
    {
       memcpy(&sensor.batterySensor, &batterySensor,
              sizeof(Smsgs_batterySensorField_t));
    }

    // f0 sensor
    if(sensor.frameControl & Smsgs_dataFields_f0Sensor)
    {
       memcpy(&sensor.f0Sensor, &f0Sensor,
              sizeof(Smsgs_f0SensorField_t));
    }

    // f1 sensor
    if(sensor.frameControl & Smsgs_dataFields_f1Sensor)
    {
       memcpy(&sensor.f1Sensor, &f1Sensor,
              sizeof(Smsgs_f1SensorField_t));
    }

    // f2 sensor
    if(sensor.frameControl & Smsgs_dataFields_f2Sensor)
    {
       memcpy(&sensor.f2Sensor, &f2Sensor,
              sizeof(Smsgs_f2SensorField_t));
    }

    // f3 sensor
    if(sensor.frameControl & Smsgs_dataFields_f3Sensor)
    {
       memcpy(&sensor.f3Sensor, &f3Sensor,
              sizeof(Smsgs_f3SensorField_t));
    }

    // f4 sensor
    if(sensor.frameControl & Smsgs_dataFields_f4Sensor)
    {
       memcpy(&sensor.f4Sensor, &f4Sensor,
              sizeof(Smsgs_f4SensorField_t));
    }

    // f5 sensor
    if(sensor.frameControl & Smsgs_dataFields_f5Sensor)
    {
       memcpy(&sensor.f5Sensor, &f5Sensor,
              sizeof(Smsgs_f5SensorField_t));
    }

    // f6 sensor
    if(sensor.frameControl & Smsgs_dataFields_f6Sensor)
    {
       memcpy(&sensor.f6Sensor, &f6Sensor,
              sizeof(Smsgs_f6SensorField_t));
    }

    // f7 sensor
    if(sensor.frameControl & Smsgs_dataFields_f7Sensor)
    {
       memcpy(&sensor.f7Sensor, &f7Sensor,
              sizeof(Smsgs_f7SensorField_t));
    }

    // f8 sensor
    if(sensor.frameControl & Smsgs_dataFields_f8Sensor)
    {
       memcpy(&sensor.f8Sensor, &f8Sensor,
              sizeof(Smsgs_f8SensorField_t));
    }

    // f9 sensor
    if(sensor.frameControl & Smsgs_dataFields_f9Sensor)
    {
       memcpy(&sensor.f9Sensor, &f9Sensor,
              sizeof(Smsgs_f9SensorField_t));
    }



    /* send the data to the collector */
    sendSensorMessage(&collectorAddr, &sensor);
}

/*!
 * @brief   Manually read the sensors
 */
static void readSensors(void)
{
#if defined(TEMP_SENSOR)
    /* Read the temp sensor values */
    tempSensor.ambienceTemp = Ssf_readTempSensor();
    tempSensor.objectTemp =  tempSensor.ambienceTemp;
#endif

    ///////////////////////////////////////////////////////
    ///////////////// Read sensor//////////////////////////
    ///////////////////////////////////////////////////////
    sensor_action = measure;
    Semaphore_post(semHandle); // now "SENSORTASK" is
    //enabled and the power meter is read

    ///////////////////////////////////////////////////////
    ///// Sensor data is copied into the message fields////
    ///////////////////////////////////////////////////////
    /*
     * For the sake of simplicity, the fields (the variables sent through
     * the message) are all uint16_t, which is enough for almost all the cases.
     *
     * in order to transmit a float number as an uint16_t, some transformation
     * must be done. In case its range contains negative values, an offset has
     * to be added. To get "n" decimal places, the float number can be multiplied
     * by 10^n.
     *
     * By applying these simple transformations, the float number can be converted
     * into an uint16_t variable, which can be easily transmitted through
     * radiofrequency to the collector. Then in the collector side, these
     * transformations will be reversed.
     *
     * in case the float number range does not fit into an uint16_t variable,
     * an uint32_t type is used. The uint32_t is splitted into two uint16_t
     * (two fields used) and they are transmitted through radiofrequency.
     * In the collector side the two employed field will be reconverted into an
     * uint32_t again.
     */
     // power sensor
    f0Sensor.f0RawData = powerMeterData.power;  // 0 decimal places

    // voltage sensor
    f1Sensor.f1RawData = powerMeterData.V_prim_rms*100; // 2 decimal places

    // current sensor
    f2Sensor.f2RawData = powerMeterData.I_prim_rms*1000; // 3 decimal places

    // freq sensor
    f3Sensor.f3RawData = powerMeterData.f*100; // 2 decimal places

    // angle sensor
    f4Sensor.f4RawData = (powerMeterData.angle2+90)*100; // 2 decimal places, offset = 90



    ///////////////////////////////////////////////////////
    ///////// Sensor data is printed through UART  ////////
    ///////////////////////////////////////////////////////
    System_printf("----------------------------------\r\n");
    System_printf("id:%04x%04x%04x%04x\r\n", id3Sensor.id3RawData, id2Sensor.id2RawData, id1Sensor.id1RawData, id0Sensor.id0RawData);
    System_printf("sensor type:%i (power meter sensor)\r\n", (int)typeSensor.typeRawData);
    System_printf("   power = %i [W]\r\n", (int)powerMeterData.power);
    System_printf("   voltage = %f [Vrms]\r\n", (double)powerMeterData.V_prim_rms);
    System_printf("   current = %f [Arms]\r\n", (double)powerMeterData.I_prim_rms);
    System_printf("   frequency = %f [Hz]\r\n", (double)powerMeterData.f);
    System_printf("   angle M1 = %f [deg]\r\n", (double)powerMeterData.angle1);
    System_printf("   angle M2 = %f [deg]\r\n", (double)powerMeterData.angle2);
    System_printf("----------------------------------\r\n");

}

/*!
 * @brief   Build and send sensor data message
 *
 * @param   pDstAddr - Where to send the message
 * @param   pMsg - pointer to the sensor data
 *
 * @return  true if message was sent, false if not
 */
static bool sendSensorMessage(ApiMac_sAddr_t *pDstAddr, Smsgs_sensorMsg_t *pMsg)
{
    bool ret = false;
    uint8_t *pMsgBuf;
    uint16_t len = SMSGS_BASIC_SENSOR_LEN;

    /* Figure out the length */
    if(pMsg->frameControl & Smsgs_dataFields_tempSensor)
    {
        len += SMSGS_SENSOR_TEMP_LEN;
    }
    if(pMsg->frameControl & Smsgs_dataFields_lightSensor)
    {
        len += SMSGS_SENSOR_LIGHT_LEN;
    }
    if(pMsg->frameControl & Smsgs_dataFields_humiditySensor)
    {
        len += SMSGS_SENSOR_HUMIDITY_LEN;
    }
    if(pMsg->frameControl & Smsgs_dataFields_msgStats)
    {
        //len += SMSGS_SENSOR_MSG_STATS_LEN;
        len += sizeof(Smsgs_msgStatsField_t);
    }
    if(pMsg->frameControl & Smsgs_dataFields_configSettings)
    {
        len += SMSGS_SENSOR_CONFIG_SETTINGS_LEN;
    }


    // id0 sensor
    if(pMsg->frameControl & Smsgs_dataFields_id0Sensor)
    {
       len += SMSGS_SENSOR_ID0_LEN;
    }
    // id1 sensor
    if(pMsg->frameControl & Smsgs_dataFields_id1Sensor)
    {
       len += SMSGS_SENSOR_ID1_LEN;
    }
    // id2 sensor
    if(pMsg->frameControl & Smsgs_dataFields_id2Sensor)
    {
       len += SMSGS_SENSOR_ID2_LEN;
    }
    // id3 sensor
    if(pMsg->frameControl & Smsgs_dataFields_id3Sensor)
    {
       len += SMSGS_SENSOR_ID3_LEN;
    }
    // type sensor
    if(pMsg->frameControl & Smsgs_dataFields_typeSensor)
    {
       len += SMSGS_SENSOR_TYPE_LEN;
    }
    // battery sensor
    if(pMsg->frameControl & Smsgs_dataFields_batterySensor)
    {
       len += SMSGS_SENSOR_BATTERY_LEN;
    }

    // f0 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f0Sensor)
    {
       len += SMSGS_SENSOR_F0_LEN;
    }
    // f1 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f1Sensor)
    {
       len += SMSGS_SENSOR_F1_LEN;
    }
    // f2 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f2Sensor)
    {
       len += SMSGS_SENSOR_F2_LEN;
    }
    // f3 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f3Sensor)
    {
       len += SMSGS_SENSOR_F3_LEN;
    }
    // f4 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f4Sensor)
    {
       len += SMSGS_SENSOR_F4_LEN;
    }
    // f5 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f5Sensor)
    {
       len += SMSGS_SENSOR_F5_LEN;
    }
    // f6 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f6Sensor)
    {
       len += SMSGS_SENSOR_F6_LEN;
    }
    // f7 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f7Sensor)
    {
       len += SMSGS_SENSOR_F7_LEN;
    }
    // f8 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f8Sensor)
    {
       len += SMSGS_SENSOR_F8_LEN;
    }
    // f9 sensor
    if(pMsg->frameControl & Smsgs_dataFields_f9Sensor)
    {
       len += SMSGS_SENSOR_F9_LEN;
    }


    pMsgBuf = (uint8_t *)Ssf_malloc(len);
    if(pMsgBuf)
    {
        uint8_t *pBuf = pMsgBuf;

        *pBuf++ = (uint8_t)Smsgs_cmdIds_sensorData;

        memcpy(pBuf, pMsg->extAddress, SMGS_SENSOR_EXTADDR_LEN);
        pBuf += SMGS_SENSOR_EXTADDR_LEN;

        pBuf  = Util_bufferUint16(pBuf,pMsg->frameControl);

        /* Buffer data in order of frameControl mask, starting with LSB */
        if(pMsg->frameControl & Smsgs_dataFields_tempSensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->tempSensor.ambienceTemp);
            pBuf = Util_bufferUint16(pBuf, pMsg->tempSensor.objectTemp);
        }
        if(pMsg->frameControl & Smsgs_dataFields_lightSensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->lightSensor.rawData);
        }
        if(pMsg->frameControl & Smsgs_dataFields_humiditySensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->humiditySensor.temp);
            pBuf = Util_bufferUint16(pBuf, pMsg->humiditySensor.humidity);
        }


        // id0 sensor
        if(pMsg->frameControl & Smsgs_dataFields_id0Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->id0Sensor.id0RawData);
        }
        // id1 sensor
        if(pMsg->frameControl & Smsgs_dataFields_id1Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->id1Sensor.id1RawData);
        }
        // id2 sensor
        if(pMsg->frameControl & Smsgs_dataFields_id2Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->id2Sensor.id2RawData);
        }
        // id3 sensor
        if(pMsg->frameControl & Smsgs_dataFields_id3Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->id3Sensor.id3RawData);
        }
        // type sensor
        if(pMsg->frameControl & Smsgs_dataFields_typeSensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->typeSensor.typeRawData);
        }
        // battery sensor
        if(pMsg->frameControl & Smsgs_dataFields_batterySensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->batterySensor.batteryRawData);
        }

        // f0 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f0Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f0Sensor.f0RawData);
        }
        // f1 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f1Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f1Sensor.f1RawData);
        }
        // f2 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f2Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f2Sensor.f2RawData);
        }
        // f3 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f3Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f3Sensor.f3RawData);
        }
        // f4 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f4Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f4Sensor.f4RawData);
        }
        // f5 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f5Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f5Sensor.f5RawData);
        }
        // f6 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f6Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f6Sensor.f6RawData);
        }
        // f7 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f7Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f7Sensor.f7RawData);
        }
        // f8 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f8Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f8Sensor.f8RawData);
        }
        // f9 sensor
        if(pMsg->frameControl & Smsgs_dataFields_f9Sensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->f9Sensor.f9RawData);
        }




        if(pMsg->frameControl & Smsgs_dataFields_msgStats)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.joinAttempts);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.joinFails);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.msgsAttempted);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.msgsSent);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.trackingRequests);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.trackingResponseAttempts);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.trackingResponseSent);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.configRequests);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.configResponseAttempts);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.configResponseSent);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.channelAccessFailures);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.macAckFailures);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.otherDataRequestFailures);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.syncLossIndications);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.rxDecryptFailures);
            pBuf = Util_bufferUint16(pBuf,  pMsg->msgStats.txEncryptFailures);
            pBuf = Util_bufferUint16(pBuf, Ssf_resetCount);
            pBuf = Util_bufferUint16(pBuf,  Ssf_resetReseason);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.joinTime);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.interimDelay);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.numBroadcastMsgRcvd);
            pBuf = Util_bufferUint16(pBuf,  pMsg->msgStats.numBroadcastMsglost);
            pBuf = Util_bufferUint16(pBuf,  pMsg->msgStats.avgE2EDelay);
            pBuf = Util_bufferUint16(pBuf,  pMsg->msgStats.worstCaseE2EDelay);
        }
        if(pMsg->frameControl & Smsgs_dataFields_configSettings)
        {
            pBuf = Util_bufferUint32(pBuf,
                                     pMsg->configSettings.reportingInterval);
            pBuf = Util_bufferUint32(pBuf,
                                     pMsg->configSettings.pollingInterval);

        }

        ret = Sensor_sendMsg(Smsgs_cmdIds_sensorData, pDstAddr, true, len, pMsgBuf);

        Ssf_free(pMsgBuf);
    }

    return (ret);
}

#endif // !defined(OAD_IMG_A)


/*!
 * @brief      Process the Config Request message.
 *
 * @param      pDataInd - pointer to the data indication information
 */
static void processConfigRequest(ApiMac_mcpsDataInd_t *pDataInd)
{
    Smsgs_statusValues_t stat = Smsgs_statusValues_invalid;
    Smsgs_configRspMsg_t configRsp;

    memset(&configRsp, 0, sizeof(Smsgs_configRspMsg_t));

    /* Make sure the message is the correct size */
    if(pDataInd->msdu.len == SMSGS_CONFIG_REQUEST_MSG_LENGTH)
    {
        uint8_t *pBuf = pDataInd->msdu.p;
        uint16_t frameControl;
        uint32_t reportingInterval;
        uint32_t pollingInterval;

        /* Parse the message */
        configSettings.cmdId = (Smsgs_cmdIds_t)*pBuf++;
        frameControl = Util_parseUint16(pBuf);
        pBuf += 2;
        reportingInterval = Util_parseUint32(pBuf);
        pBuf += 4;
        pollingInterval = Util_parseUint32(pBuf);

        stat = Smsgs_statusValues_success;
        collectorAddr.addrMode = pDataInd->srcAddr.addrMode;
        if(collectorAddr.addrMode == ApiMac_addrType_short)
        {
            collectorAddr.addr.shortAddr = pDataInd->srcAddr.addr.shortAddr;
        }
        else
        {
            memcpy(collectorAddr.addr.extAddr, pDataInd->srcAddr.addr.extAddr,
                   (APIMAC_SADDR_EXT_LEN));
        }

        configSettings.frameControl = validateFrameControl(frameControl);
        if(configSettings.frameControl != frameControl)
        {
            stat = Smsgs_statusValues_partialSuccess;
        }
        configRsp.frameControl = configSettings.frameControl;

        if((reportingInterval < MIN_REPORTING_INTERVAL)
           || (reportingInterval > MAX_REPORTING_INTERVAL))
        {
            stat = Smsgs_statusValues_partialSuccess;
        }
        else
        {
#ifndef POWER_MEAS
            configSettings.reportingInterval = reportingInterval;
#endif
            {
                uint32_t randomNum;
                randomNum = ((ApiMac_randomByte() << 16) +
                             (ApiMac_randomByte() << 8) + ApiMac_randomByte());
                randomNum = (randomNum % reportingInterval) +
                        SENSOR_MIN_POLL_TIME;
                Ssf_setReadingClock(randomNum);
            }

        }
        configRsp.reportingInterval = configSettings.reportingInterval;

        if((pollingInterval < MIN_POLLING_INTERVAL)
           || (pollingInterval > MAX_POLLING_INTERVAL))
        {
            stat = Smsgs_statusValues_partialSuccess;
        }
        else
        {
            configSettings.pollingInterval = pollingInterval;
            Jdllc_setPollRate(configSettings.pollingInterval);
        }
        configRsp.pollingInterval = configSettings.pollingInterval;
    }

    /* Send the response message */
    configRsp.cmdId = Smsgs_cmdIds_configRsp;
    configRsp.status = stat;

    /* Update the user */
    Ssf_configurationUpdate(&configRsp);

    /* Response the the source device */
    sendConfigRsp(&pDataInd->srcAddr, &configRsp);
}

/*!
 * @brief      Process the Broadcast Control Msg.
 *
 * @param      pDataInd - pointer to the data indication information
 */
static void processBroadcastCtrlMsg(ApiMac_mcpsDataInd_t *pDataInd)
{
    Smsgs_broadcastcmdmsg_t broadcastCmd;

    memset(&broadcastCmd, 0, sizeof(Smsgs_broadcastcmdmsg_t));

    /* Make sure the message is the correct size */
    if(pDataInd->msdu.len == SMSGS_BROADCAST_CMD_LENGTH)
    {
        uint8_t *pBuf = pDataInd->msdu.p;
        uint16_t broadcastMsgId;

        /* Parse the message */
        uint8_t cmdId = (Smsgs_cmdIds_t)*pBuf++;
        broadcastMsgId = Util_parseUint16(pBuf);

        /* Process Broadcast Command Message */
        Sensor_msgStats.numBroadcastMsgRcvd++;

        if(!initBroadcastMsg)
        {
            /* Not the first broadcast msg rcvdd after join or a rejoin*/
            if((broadcastMsgId - lastRcvdBroadcastMsgId) > 1)
            {
                Sensor_msgStats.numBroadcastMsglost +=
                                ((broadcastMsgId - lastRcvdBroadcastMsgId) -1);
            }
        }

        lastRcvdBroadcastMsgId = broadcastMsgId;
        /*To handle the very first broadcast msg rcvdd after join or a rejoin*/
        initBroadcastMsg = false;

    }
}

/*!
 * @brief   Build and send Config Response message
 *
 * @param   pDstAddr - Where to send the message
 * @param   pMsg - pointer to the Config Response
 *
 * @return  true if message was sent, false if not
 */
static bool sendConfigRsp(ApiMac_sAddr_t *pDstAddr, Smsgs_configRspMsg_t *pMsg)
{
    uint8_t msgBuf[SMSGS_CONFIG_RESPONSE_MSG_LENGTH];
    uint8_t *pBuf = msgBuf;

    *pBuf++ = (uint8_t) Smsgs_cmdIds_configRsp;
    pBuf = Util_bufferUint16(pBuf, pMsg->status);
    pBuf = Util_bufferUint16(pBuf, pMsg->frameControl);
    pBuf = Util_bufferUint32(pBuf, pMsg->reportingInterval);
    pBuf = Util_bufferUint32(pBuf, pMsg->pollingInterval);

    return (Sensor_sendMsg(Smsgs_cmdIds_configRsp, pDstAddr, true,
                    SMSGS_CONFIG_RESPONSE_MSG_LENGTH, msgBuf));
}

/*!
 * @brief   Filter the frameControl with readings supported by this device.
 *
 * @param   frameControl - suggested frameControl
 *
 * @return  new frame control settings supported
 */
static uint16_t validateFrameControl(uint16_t frameControl)
{
    uint16_t newFrameControl = 0;

#if defined(TEMP_SENSOR)
    if(frameControl & Smsgs_dataFields_tempSensor)
    {
        newFrameControl |= Smsgs_dataFields_tempSensor;
    }
#endif
#if defined(LIGHT_SENSOR)
    if(frameControl & Smsgs_dataFields_lightSensor)
    {
        newFrameControl |= Smsgs_dataFields_lightSensor;
    }
#endif
#if defined(HUMIDITY_SENSOR)
    if(frameControl & Smsgs_dataFields_humiditySensor)
    {
        newFrameControl |= Smsgs_dataFields_humiditySensor;
    }
#endif
    if(frameControl & Smsgs_dataFields_msgStats)
    {
        newFrameControl |= Smsgs_dataFields_msgStats;
    }
    if(frameControl & Smsgs_dataFields_configSettings)
    {
        newFrameControl |= Smsgs_dataFields_configSettings;
    }

    // id0 sensor
    if(frameControl & Smsgs_dataFields_id0Sensor)
    {
       newFrameControl |= Smsgs_dataFields_id0Sensor;
    }
    // id1 sensor
    if(frameControl & Smsgs_dataFields_id1Sensor)
    {
       newFrameControl |= Smsgs_dataFields_id1Sensor;
    }
    // id2 sensor
    if(frameControl & Smsgs_dataFields_id2Sensor)
    {
       newFrameControl |= Smsgs_dataFields_id2Sensor;
    }
    // id3 sensor
    if(frameControl & Smsgs_dataFields_id3Sensor)
    {
       newFrameControl |= Smsgs_dataFields_id3Sensor;
    }
    // type sensor
    if(frameControl & Smsgs_dataFields_typeSensor)
    {
       newFrameControl |= Smsgs_dataFields_typeSensor;
    }
    // battery sensor
    if(frameControl & Smsgs_dataFields_batterySensor)
    {
       newFrameControl |= Smsgs_dataFields_batterySensor;
    }

    // f0 sensor
    if(frameControl & Smsgs_dataFields_f0Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f0Sensor;
    }
    // f1 sensor
    if(frameControl & Smsgs_dataFields_f1Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f1Sensor;
    }
    // f2 sensor
    if(frameControl & Smsgs_dataFields_f2Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f2Sensor;
    }
    // f3 sensor
    if(frameControl & Smsgs_dataFields_f3Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f3Sensor;
    }
    // f4 sensor
    if(frameControl & Smsgs_dataFields_f4Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f4Sensor;
    }
    // f5 sensor
    if(frameControl & Smsgs_dataFields_f5Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f5Sensor;
    }
    // f6 sensor
    if(frameControl & Smsgs_dataFields_f6Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f6Sensor;
    }
    // f7 sensor
    if(frameControl & Smsgs_dataFields_f7Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f7Sensor;
    }
    // f8 sensor
    if(frameControl & Smsgs_dataFields_f8Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f8Sensor;
    }
    // f9 sensor
    if(frameControl & Smsgs_dataFields_f9Sensor)
    {
       newFrameControl |= Smsgs_dataFields_f9Sensor;
    }


    return (newFrameControl);
}


/*!
 * @brief   The device joined callback.
 *
 * @param   pDevInfo - This device's information
 * @param   pParentInfo - This is the parent's information
 */
static void jdllcJoinedCb(ApiMac_deviceDescriptor_t *pDevInfo,
                          Llc_netInfo_t *pParentInfo)
{
    uint32_t randomNum = 0;

    /* Copy the parent information */
    memcpy(&parentInfo, pParentInfo, sizeof(Llc_netInfo_t));

    /* Set the collector's address as the parent's address */
    if (pParentInfo->fh && CONFIG_RX_ON_IDLE)
    {
        collectorAddr.addrMode = ApiMac_addrType_extended;
        memcpy(collectorAddr.addr.extAddr, pParentInfo->devInfo.extAddress,
               (APIMAC_SADDR_EXT_LEN));
    }
    else
    {
        collectorAddr.addrMode = ApiMac_addrType_short;
        collectorAddr.addr.shortAddr = pParentInfo->devInfo.shortAddress;
    }

    /* Start the reporting timer */
    if(CONFIG_FH_ENABLE)
    {
        randomNum = ((ApiMac_randomByte() << 16) +
                     (ApiMac_randomByte() << 8) + ApiMac_randomByte());
        randomNum = (randomNum % configSettings.reportingInterval) +
                    SENSOR_MIN_POLL_TIME;
        Ssf_setReadingClock(randomNum);
    }
    else
    {
       uint32_t randomNum;
       randomNum = ((ApiMac_randomByte() << 16) +
                    (ApiMac_randomByte() << 8) + ApiMac_randomByte());
       randomNum = (randomNum % configSettings.reportingInterval ) +
                   SENSOR_MIN_POLL_TIME;
       Ssf_setReadingClock(randomNum);
    }

    /* Inform the user of the joined information */
    Ssf_networkUpdate(rejoining, pDevInfo, pParentInfo);

#ifdef FEATURE_SECURE_COMMISSIONING
        SM_Sensor_SAddress = pDevInfo->shortAddress;
#endif
    if((rejoining == false) && (pParentInfo->fh == false))
    {
#ifdef FEATURE_MAC_SECURITY
        ApiMac_status_t stat;
        /* Add the parent to the security device list */
        stat = Jdllc_addSecDevice(pParentInfo->devInfo.panID,
                                  pParentInfo->devInfo.shortAddress,
                                  &pParentInfo->devInfo.extAddress, 0);
        if(stat != ApiMac_status_success)
        {
            Ssf_displayError("Auth Error: 0x", (uint8_t)stat);
        }
#endif /* FEATURE_MAC_SECURITY */
    }

#if (CONFIG_MAC_SUPERFRAME_ORDER != 15) && defined(MAC_NO_AUTO_REQ)
    /*
     * Set MAC Auto Request to false to enable multiple poll requests
     * per beacon interval
     */
    ApiMac_mlmeSetReqBool(ApiMac_attribute_autoRequest, false);
#endif

#ifdef OSAL_PORT2TIRTOS
    /* Calculate Join Time */
    if(Clock_getTicks() < joinTimeTicks)
    {
        joinTimeTicks = Clock_getTicks() + (0xFFFFFFFF-joinTimeTicks);
    }
    else
    {
        joinTimeTicks = Clock_getTicks() - joinTimeTicks;
    }
#else
    /* Calculate Join Time */
    if(ICall_getTicks() < joinTimeTicks)
    {
        joinTimeTicks = ICall_getTicks() + (0xFFFFFFFF-joinTimeTicks);
    }
    else
    {
        joinTimeTicks = ICall_getTicks() - joinTimeTicks;
    }
#endif
    Sensor_msgStats.joinTime = joinTimeTicks / TICKPERIOD_MS_US;
#ifdef DISPLAY_PER_STATS
    /* clear the stats used for PER so that we start out at a
     * zeroed state
     */
    Sensor_msgStats.macAckFailures = 0;
    Sensor_msgStats.otherDataRequestFailures = 0;
    Sensor_msgStats.msgsSent = 0;
#endif
}

/*!
 * @brief   Disassociation indication callback.
 *
 * @param   pExtAddress - extended address
 * @param   reason - reason for disassociation
 */
static void jdllcDisassocIndCb(ApiMac_sAddrExt_t *pExtAddress,
                               ApiMac_disassocateReason_t reason)
{
    /* Stop the reporting timer */
    Ssf_setReadingClock(0);
    Ssf_clearNetworkInfo();
#ifdef FEATURE_SECURE_COMMISSIONING
    SM_removeEntryFromSeedKeyTable(pExtAddress);
    ApiMac_secDeleteDevice(pExtAddress);
    Ssf_clearDeviceKeyInfo();
#endif

#ifdef FEATURE_NATIVE_OAD
    /* OAD abort with no auto resume */
    OADClient_abort(false);
#endif //FEATURE_NATIVE_OAD
}

/*!
 * @brief   Disassociation confirm callback to an application intiated
 *          disassociation request.
 *
 * @param   pExtAddress - extended address
 * @param   status - status of disassociation
 */
static void jdllcDisassocCnfCb(ApiMac_sAddrExt_t *pExtAddress,
                               ApiMac_status_t status)
{
    /* Stop the reporting timer */
    Ssf_setReadingClock(0);
    Ssf_clearNetworkInfo();
#ifdef FEATURE_SECURE_COMMISSIONING
    SM_removeEntryFromSeedKeyTable(pExtAddress);
    ApiMac_secDeleteDevice(pExtAddress);
    Ssf_clearDeviceKeyInfo();
#endif
#ifdef FEATURE_NATIVE_OAD
    /* OAD abort with no auto resume */
    OADClient_abort(false);
#endif //FEATURE_NATIVE_OAD
}

/*!
 * @brief   JDLLC state change callback.
 *
 * @param   state - new state
 */
static void jdllcStateChangeCb(Jdllc_states_t state)
{
#ifdef FEATURE_NATIVE_OAD
    if( (state == Jdllc_states_joined) || (state == Jdllc_states_rejoined))
    {
#if (CONFIG_MAC_SUPERFRAME_ORDER == 15)
        /* resume an OAD that may have aborted */
        OADClient_resume(30000);
#else
        /* resume an OAD that may have aborted */
        OADClient_resume(60000);
#endif
    }
    else if(state == Jdllc_states_orphan)
    {
        /* OAD abort with no auto resume */
        OADClient_abort(false);
    }
#endif /* FEATURE_NATIVE_OAD */

    Ssf_stateChangeUpdate(state);

    switch(state){
    case Jdllc_states_orphan: // state 5
    case Jdllc_states_accessDenied: // state 6

        Board_Led_control(board_led_type_LED2, board_led_state_ON);//red led on
        Board_Led_control(board_led_type_LED1, board_led_state_OFF);//green led off

        break;

    case Jdllc_states_joined: // state 3
    case Jdllc_states_rejoined: // state 4


        Board_Led_control(board_led_type_LED2, board_led_state_OFF); //red led off
        Board_Led_control(board_led_type_LED1, board_led_state_ON);//green led on

        break;


    case Jdllc_states_initWaiting: // state 0
    case Jdllc_states_joining: //state 1
    case Jdllc_states_initRestoring: //state 2
    default:


        Board_Led_control(board_led_type_LED2, board_led_state_OFF);//red led off
        Board_Led_control(board_led_type_LED1, board_led_state_OFF);//green led off

        break;
    }


#ifdef OAD_IMG_A
    if( (state == Jdllc_states_joined) || (state == Jdllc_states_rejoined))
    {
        Util_setEvent(&Sensor_events, SENSOR_OAD_SEND_RESET_RSP_EVT);
    }
#endif
}


#ifdef FEATURE_SECURE_COMMISSIONING
/*!
 * @brief      Security manager failure processing function
 */
void smFailCMProcessCb(ApiMac_deviceDescriptor_t *devInfo, bool rxOnIdle, bool keyRefreshment)
{
    /* restore, write back current Pib value for auto request attribute */
    ApiMac_mlmeSetReqBool(ApiMac_attribute_autoRequest, currAutoReq);

    if (keyRefreshment == true)
    {
        LCD_WRITE_STRING_VALUE("Key Refresh Failed: 0x", SM_Sensor_SAddress, 16, 4);
    }
    else
    {
        LCD_WRITE_STRING_VALUE("Commissioning Failed: 0x", SM_Sensor_SAddress, 16, 4);
    }
}

/*!
 * @brief      Security manager success processing function
 */
void smSuccessCMProcessCb(ApiMac_deviceDescriptor_t *devInfo, bool keyRefreshment)
{
    /* restore, write back current Pib value for auto request attribute */
    ApiMac_mlmeSetReqBool(ApiMac_attribute_autoRequest, currAutoReq);

    if (keyRefreshment == true)
    {
        LCD_WRITE_STRING_VALUE("Key Refreshed: 0x", SM_Sensor_SAddress, 16, 4);
    }
    else
    {
        LCD_WRITE_STRING_VALUE("Commissioned: 0x", SM_Sensor_SAddress, 16, 4);
    }
}
#endif /* FEATURE_SECURE_COMMISSIONING */
