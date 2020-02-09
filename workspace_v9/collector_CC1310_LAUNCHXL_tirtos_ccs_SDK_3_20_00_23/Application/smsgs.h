/******************************************************************************

 @file smsgs.h

 @brief Data Structures for the sensor messages sent over the air.

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
#ifndef SMGSS_H
#define SMGSS_H

/******************************************************************************
 Includes
 *****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 \defgroup Sensor Over-the-air Messages
 <BR>
 This header file defines the over-the-air messages between the collector
 and the sensor applications.
 <BR>
 Each field in these message are formatted low byte first, so a 16 bit field
 with a value of 0x1516 will be sent as 0x16, 0x15.
 <BR>
 The first byte of each message (data portion) contains the command ID (@ref
 Smsgs_cmdIds).
 <BR>
 The <b>Configuration Request Message</b> is defined as:
     - Command ID - [Smsgs_cmdIds_configReq](@ref Smsgs_cmdIds) (1 byte)
     - Frame Control field - Smsgs_dataFields (16 bits) - tells the sensor
     what to report in the Sensor Data Message.
     - Reporting Interval - in millseconds (32 bits) - how often to report, 0
     means to turn off automated reporting, but will force the sensor device
     to send the Sensor Data message once.
     - Polling Interval - in millseconds (32 bits) - If the sensor device is
     a sleep device, this tells the device how often to poll its parent for
     data.
 <BR>
 The <b>Configuration Response Message</b> is defined as:
     - Command ID - [Smsgs_cmdIds_configRsp](@ref Smsgs_cmdIds) (1 byte)
     - Status field - Smsgs_statusValues (16 bits) - status of the
     configuration request.
     - Frame Control field - Smsgs_dataFields (16 bits) - tells the collector
     what fields are supported by this device (this only includes the bits set
     in the request message).
     - Reporting Interval - in millseconds (32 bits) - how often to report, 0
     means to turn off reporting.
     - Polling Interval - in millseconds (32 bits) - If the sensor device is
     a sleep device, this tells how often this device will poll its parent.
     A value of 0 means that the device doesn't sleep.
 <BR>
The <b>Sensor Ramp Data Message</b> is defined as:
     - Command ID - [Smsgs_cmdIds_rampdata](@ref Smsgs_cmdIds) (1 byte)     
     - Data Fields - Variable length ramp data of size SENSOR_TEST_RAMP_DATA_SIZE
 <BR>
 The <b>Sensor Data Message</b> is defined as:
     - Command ID - [Smsgs_cmdIds_sensorData](@ref Smsgs_cmdIds) (1 byte)
     - Frame Control field - Smsgs_dataFields (16 bits) - tells the collector
     what fields are included in this message.
     - Data Fields - The length of this field is determined by what data fields
     are included.  The order of the data fields are determined by the bit
     position of the Frame Control field (low bit first).  For example, if the
     frame control field has Smsgs_dataFields_tempSensor and
     Smsgs_dataFields_lightSensor set, then the Temp Sensor field is first,
     followed by the light sensor field.
 <BR>
 The <b>Temp Sensor Field</b> is defined as:
    - Ambience Chip Temperature - (int16_t) - each value represents signed
      integer part of temperature in Deg C (-256 .. +255)
    - Object Temperature - (int16_t) -  each value represents signed
      integer part of temperature in Deg C (-256 .. +255)
 <BR>
 The <b>Light Sensor Field</b> is defined as:
    - Raw Sensor Data - (uint16_6) raw data read out of the OPT2001 light
    sensor.
 <BR>
 The <b>Humidity Sensor Field</b> is defined as:
    - Raw Temp Sensor Data - (uint16_t) - raw temperature data from the
    Texas Instruments HCD1000 humidity sensor.
    - Raw Humidity Sensor Data - (uint16_t) - raw humidity data from the
    Texas Instruments HCD1000 humidity sensor.
 <BR>
 The <b>Message Statistics Field</b> is defined as:
     - joinAttempts - uint16_t - total number of join attempts (associate
     request sent)
     - joinFails - uint16_t - total number of join attempts failed
     - msgsAttempted - uint16_t - total number of sensor data messages
     attempted.
     - msgsSent - uint16_t - total number of sensor data messages successfully
     sent (OTA).
     - trackingRequests - uint16_t - total number of tracking requests received.
     - trackingResponseAttempts - uint16_t - total number of tracking response
     attempted
     - trackingResponseSent - uint16_t - total number of tracking response
     success
     - configRequests - uint16_t - total number of config requests received.
     - configResponseAttempts - uint16_t - total number of config response
     attempted
     - configResponseSent - uint16_t - total number of config response
     success
     - channelAccessFailures - uint16_t - total number of Channel Access
     Failures.  These are indicated in MAC data confirms for MAC data requests.
     - macAckFailures - uint16_t - Total number of MAC ACK failures. These are
     indicated in MAC data confirms for MAC data requests.
     - otherDataRequestFailures - uint16_t - Total number of MAC data request
     failures, other than channel access failure or MAC ACK failures.
     - syncLossIndications - uint16_t - Total number of sync loss failures
     received for sleepy devices.
     - rxDecryptFailues - uint16_t - Total number of RX Decrypt failures.
     - txEncryptFailues - uint16_t - Total number of TX Encrypt failures.
     - resetCount - uint16_t - Total number of resets.
     - lastResetReason - uint16_t - 0 - no reason, 2 - HAL/ICALL,
     3 - MAC, 4 - TIRTOS
 <BR>
 The <b>Config Settings Field</b> is defined as:
     - Reporting Interval - in millseconds (32 bits) - how often to report, 0
     means reporting is off.
     - Polling Interval - in millseconds (32 bits) - If the sensor device is
     a sleep device, this states how often the device polls its parent for
     data. This field is 0 if the device doesn't sleep.
 */

/******************************************************************************
 Constants and definitions
 *****************************************************************************/
/*! Sensor Message Extended Address Length */
#define SMGS_SENSOR_EXTADDR_LEN 8

/*! Config Request message length (over-the-air length) */
#define SMSGS_CONFIG_REQUEST_MSG_LENGTH 11
/*! Config Response message length (over-the-air length) */
#define SMSGS_CONFIG_RESPONSE_MSG_LENGTH 13
/*! Tracking Request message length (over-the-air length) */
#define SMSGS_TRACKING_REQUEST_MSG_LENGTH 1
/*! Tracking Response message length (over-the-air length) */
#define SMSGS_TRACKING_RESPONSE_MSG_LENGTH 1
/*! Broadcast Command message length (over-the-air-length) */
#define SMSGS_BROADCAST_CMD_LENGTH  3

/*! Length of a sensor data message with no configured data fields */
#define SMSGS_BASIC_SENSOR_LEN (3 + SMGS_SENSOR_EXTADDR_LEN)
/*! Length of the tempSensor portion of the sensor data message */
#define SMSGS_SENSOR_TEMP_LEN 4
/*! Length of the lightSensor portion of the sensor data message */
#define SMSGS_SENSOR_LIGHT_LEN 2
/*! Length of the humiditySensor portion of the sensor data message */
#define SMSGS_SENSOR_HUMIDITY_LEN 4
/*! Length of the messageStatistics portion of the sensor data message */
#define SMSGS_SENSOR_MSG_STATS_LEN 44
/*! Length of the configSettings portion of the sensor data message */
#define SMSGS_SENSOR_CONFIG_SETTINGS_LEN 8
/*! Toggle Led Request message length (over-the-air length) */
#define SMSGS_TOGGLE_LED_REQUEST_MSG_LEN 1
/*! Toggle Led Request message length (over-the-air length) */
#define SMSGS_TOGGLE_LED_RESPONSE_MSG_LEN 2

/*ID0 SENSOR*/
/*! Length of the IDSensor portion of the sensor data message */
#define SMSGS_SENSOR_ID0_LEN 2
/*! ID Request message length (over-the-air length) */
#define SMSGS_ID0_REQUEST_MSG_LEN 1
/*! ID Request message length (over-the-air length) */
#define SMSGS_ID0_RESPONSE_MSG_LEN 2

/*ID1 SENSOR*/
/*! Length of the IDSensor portion of the sensor data message */
#define SMSGS_SENSOR_ID1_LEN 2
/*! ID Request message length (over-the-air length) */
#define SMSGS_ID1_REQUEST_MSG_LEN 1
/*! ID Request message length (over-the-air length) */
#define SMSGS_ID1_RESPONSE_MSG_LEN 2

/*ID2 SENSOR*/
/*! Length of the IDSensor portion of the sensor data message */
#define SMSGS_SENSOR_ID2_LEN 2
/*! ID Request message length (over-the-air length) */
#define SMSGS_ID2_REQUEST_MSG_LEN 1
/*! ID Request message length (over-the-air length) */
#define SMSGS_ID2_RESPONSE_MSG_LEN 2

/*ID3 SENSOR*/
/*! Length of the IDSensor portion of the sensor data message */
#define SMSGS_SENSOR_ID3_LEN 2
/*! ID Request message length (over-the-air length) */
#define SMSGS_ID3_REQUEST_MSG_LEN 1
/*! ID Request message length (over-the-air length) */
#define SMSGS_ID3_RESPONSE_MSG_LEN 2

// TYPE sensor
/*! Length of the TYPESensor portion of the sensor data message */
#define SMSGS_SENSOR_TYPE_LEN 2
/*! TYPE Request message length (over-the-air length) */
#define SMSGS_TYPE_REQUEST_MSG_LEN 1
/*! TYPE Request message length (over-the-air length) */
#define SMSGS_TYPE_RESPONSE_MSG_LEN 2

// BATTERY sensor
/*! Length of the BATTERYSensor portion of the sensor data message */
#define SMSGS_SENSOR_BATTERY_LEN 2
/*! BATTERY Request message length (over-the-air length) */
#define SMSGS_BATTERY_REQUEST_MSG_LEN 1
/*! BATTERY Request message length (over-the-air length) */
#define SMSGS_BATTERY_RESPONSE_MSG_LEN 2

// F0 sensor
/*! Length of the F0Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F0_LEN 2
/*! F0 Request message length (over-the-air length) */
#define SMSGS_F0_REQUEST_MSG_LEN 1
/*! F0 Request message length (over-the-air length) */
#define SMSGS_F0_RESPONSE_MSG_LEN 2

// F1 sensor
/*! Length of the F1Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F1_LEN 2
/*! F1 Request message length (over-the-air length) */
#define SMSGS_F1_REQUEST_MSG_LEN 1
/*! F1 Request message length (over-the-air length) */
#define SMSGS_F1_RESPONSE_MSG_LEN 2

// F2 sensor
/*! Length of the F2Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F2_LEN 2
/*! F2 Request message length (over-the-air length) */
#define SMSGS_F2_REQUEST_MSG_LEN 1
/*! F2 Request message length (over-the-air length) */
#define SMSGS_F2_RESPONSE_MSG_LEN 2

// F3 sensor
/*! Length of the F3Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F3_LEN 2
/*! F3 Request message length (over-the-air length) */
#define SMSGS_F3_REQUEST_MSG_LEN 1
/*! F3 Request message length (over-the-air length) */
#define SMSGS_F3_RESPONSE_MSG_LEN 2

// F4 sensor
/*! Length of the F4Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F4_LEN 2
/*! F4 Request message length (over-the-air length) */
#define SMSGS_F4_REQUEST_MSG_LEN 1
/*! F4 Request message length (over-the-air length) */
#define SMSGS_F4_RESPONSE_MSG_LEN 2

// F5 sensor
/*! Length of the F5Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F5_LEN 2
/*! F5 Request message length (over-the-air length) */
#define SMSGS_F5_REQUEST_MSG_LEN 1
/*! F5 Request message length (over-the-air length) */
#define SMSGS_F5_RESPONSE_MSG_LEN 2

// F6 sensor
/*! Length of the F6Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F6_LEN 2
/*! F6 Request message length (over-the-air length) */
#define SMSGS_F6_REQUEST_MSG_LEN 1
/*! F6 Request message length (over-the-air length) */
#define SMSGS_F6_RESPONSE_MSG_LEN 2

// F7 sensor
/*! Length of the F7Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F7_LEN 2
/*! F7 Request message length (over-the-air length) */
#define SMSGS_F7_REQUEST_MSG_LEN 1
/*! F7 Request message length (over-the-air length) */
#define SMSGS_F7_RESPONSE_MSG_LEN 2

// F8 sensor
/*! Length of the F8Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F8_LEN 2
/*! F8 Request message length (over-the-air length) */
#define SMSGS_F8_REQUEST_MSG_LEN 1
/*! F8 Request message length (over-the-air length) */
#define SMSGS_F8_RESPONSE_MSG_LEN 2

// F9 sensor
/*! Length of the F9Sensor portion of the sensor data message */
#define SMSGS_SENSOR_F9_LEN 2
/*! F9 Request message length (over-the-air length) */
#define SMSGS_F9_REQUEST_MSG_LEN 1
/*! F9 Request message length (over-the-air length) */
#define SMSGS_F9_RESPONSE_MSG_LEN 2


/*! Identify Led Request message length (over-the-air length) */
#define SMSGS_INDENTIFY_LED_REQUEST_MSG_LEN 2
/*! Identify Led Response message length (over-the-air length) */
#define SMSGS_INDENTIFY_LED_RESPONSE_MSG_LEN 2

/*!
 Message IDs for Sensor data messages.  When sent over-the-air in a message,
 this field is one byte.
 */
 typedef enum
 {
    /*! Configuration message, sent from the collector to the sensor */
    Smsgs_cmdIds_configReq = 1,
    /*! Configuration Response message, sent from the sensor to the collector */
    Smsgs_cmdIds_configRsp = 2,
    /*! Tracking request message, sent from the the collector to the sensor */
    Smsgs_cmdIds_trackingReq = 3,
     /*! Tracking response message, sent from the sensor to the collector */
    Smsgs_cmdIds_trackingRsp = 4,
    /*! Sensor data message, sent from the sensor to the collector */
    Smsgs_cmdIds_sensorData = 5,
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_toggleLedReq = 6,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_toggleLedRsp = 7,
    /* new data type for ramp data */
    Smsgs_cmdIds_rampdata = 8,
    /*! OAD mesages, sent/received from both collector and sensor */
    Smsgs_cmdIds_oad = 9,
    /* Broadcast control msg, sent from the collector to the sensor */
    Smgs_cmdIds_broadcastCtrlMsg = 10,
    /* KEY Exchange msg, between collector and the sensor */
    Smgs_cmdIds_KeyExchangeMsg = 11,
    /* Identify LED request msg */
    Smsgs_cmdIds_IdentifyLedReq = 12,
    /* Identify LED response msg */
    Smsgs_cmdIds_IdentifyLedRsp = 13,
    /*! SM Commissioning start command sent from collector to the sensor */
    Smgs_cmdIds_CommissionStart  = 14,
    /*! SM Commissioning message sent bi-directionally between the collector and sensor */
    Smgs_cmdIds_CommissionMsg  = 15,

    /*id0 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_id0Req = 16,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_id0Rsp = 17,

    /*id1 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_id1Req = 18,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_id1Rsp = 19,

    /*id2 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_id2Req = 20,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_id2Rsp = 21,

    /*id3 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_id3Req = 22,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_id3Rsp = 23,

    /*type SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_typeReq = 24,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_typeRsp = 25,

    /*battery SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_batteryReq = 26,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_batteryRsp = 27,

    /*f0 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f0Req = 28,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f0Rsp = 29,

    /*f1 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f1Req = 30,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f1Rsp = 31,

    /*f2 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f2Req = 32,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f2Rsp = 33,

    /*f3 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f3Req = 34,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f3Rsp = 35,

    /*f4 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f4Req = 36,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f4Rsp = 37,

    /*f5 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f5Req = 38,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f5Rsp = 39,

    /*f6 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f6Req = 40,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f6Rsp = 41,

    /*f7 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f7Req = 42,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f7Rsp = 43,

    /*f8 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f8Req = 44,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f8Rsp = 45,

    /*f9 SENSOR*/
    /* Toggle LED message, sent from the collector to the sensor */
    Smsgs_cmdIds_f9Req = 46,
    /* Toggle LED response msg, sent from the sensor to the collector */
    Smsgs_cmdIds_f9Rsp = 47

 } Smsgs_cmdIds_t;

/*!
 Frame Control field states what data fields are included in reported
 sensor data, each value is a bit mask value so that they can be combined
 (OR'd together) in a control field.
 When sent over-the-air in a message this field is 2 bytes.
 */
typedef enum
{
    /*! Temperature Sensor */
    Smsgs_dataFields_tempSensor = 0x0001,
    /*! Light Sensor */
    Smsgs_dataFields_lightSensor = 0x0002,
    /*! Humidity Sensor */
    Smsgs_dataFields_humiditySensor = 0x0004,
    /*! Message Statistics */
    Smsgs_dataFields_msgStats = 0x0008,
    /*! Config Settings */
    Smsgs_dataFields_configSettings = 0x0010,

    /*! id0 Sensor */
    Smsgs_dataFields_id0Sensor = 0x0100,
    /*! id1 Sensor */
    Smsgs_dataFields_id1Sensor = 0x0120,
    /*! id2 Sensor */
    Smsgs_dataFields_id2Sensor = 0x0140,
    /*! id3 Sensor */
    Smsgs_dataFields_id3Sensor = 0x0160,
    /*! type Sensor */
    Smsgs_dataFields_typeSensor = 0x0180,
    /*! battery Sensor */
    Smsgs_dataFields_batterySensor = 0x01A0,

    /*! f0 Sensor */
    Smsgs_dataFields_f0Sensor = 0x01C0,
    /*! f1 Sensor */
    Smsgs_dataFields_f1Sensor = 0x01E0,
    /*! f2 Sensor */
    Smsgs_dataFields_f2Sensor = 0x0200,
    /*! f3 Sensor */
    Smsgs_dataFields_f3Sensor = 0x0220,
    /*! f4 Sensor */
    Smsgs_dataFields_f4Sensor = 0x0240,
    /*! f5 Sensor */
    Smsgs_dataFields_f5Sensor = 0x0260,
    /*! f6 Sensor */
    Smsgs_dataFields_f6Sensor = 0x0280,
    /*! f7 Sensor */
    Smsgs_dataFields_f7Sensor = 0x02A0,
    /*! f8 Sensor */
    Smsgs_dataFields_f8Sensor = 0x02C0,
    /*! f9 Sensor */
    Smsgs_dataFields_f9Sensor = 0x02E0

} Smsgs_dataFields_t;

/*!
 Status values for the over-the-air messages
 */
typedef enum
{
    /*! Success */
    Smsgs_statusValues_success = 0,
    /*! Message was invalid and ignored */
    Smsgs_statusValues_invalid = 1,
    /*!
     Config message was received but only some frame control fields
     can be sent or the reportingInterval or pollingInterval fail
     range checks.
     */
    Smsgs_statusValues_partialSuccess = 2,
} Smsgs_statusValues_t;

/******************************************************************************
 Structures - Building blocks for the over-the-air sensor messages
 *****************************************************************************/

/*!
 Configuration Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_configreqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
    /*! Frame Control field - bit mask of Smsgs_dataFields */
    uint16_t frameControl;
    /*! Reporting Interval */
    uint32_t reportingInterval;
    /*! Polling Interval */
    uint32_t pollingInterval;
} Smsgs_configReqMsg_t;

/*!
 Configuration Response message: sent from the sensor to the collector
 in response to the Configuration Request message.
 */
typedef struct _Smsgs_configrspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
    /*! Response Status - 2 bytes */
    Smsgs_statusValues_t status;
    /*! Frame Control field - 2 bytes - bit mask of Smsgs_dataFields */
    uint16_t frameControl;
    /*! Reporting Interval - 4 bytes */
    uint32_t reportingInterval;
    /*! Polling Interval - 4 bytes */
    uint32_t pollingInterval;
} Smsgs_configRspMsg_t;

//id0 SENSOR
/*!
 id0 Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_id0reqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_id0ReqMsg_t;

/*!
id0 Response message: sent from the sensor to the collector
 in response to the id Request message.
 */
typedef struct _Smsgs_id0rspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_id0RspMsg_t;

//id1 SENSOR
/*!
 id1 Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_id1reqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_id1ReqMsg_t;

/*!
id1 Response message: sent from the sensor to the collector
 in response to the id Request message.
 */
typedef struct _Smsgs_id1rspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_id1RspMsg_t;

//id2 SENSOR
/*!
 id2 Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_id2reqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_id2ReqMsg_t;

/*!
id2 Response message: sent from the sensor to the collector
 in response to the id Request message.
 */
typedef struct _Smsgs_id2rspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_id2RspMsg_t;

//id3 SENSOR
/*!
 id3 Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_id3reqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_id3ReqMsg_t;

/*!
id3 Response message: sent from the sensor to the collector
 in response to the id Request message.
 */
typedef struct _Smsgs_id3rspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_id3RspMsg_t;

//type SENSOR
/*!
 type Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_typereqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_typeReqMsg_t;

/*!
type Response message: sent from the sensor to the collector
 in response to the type Request message.
 */
typedef struct _Smsgs_typerspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_typeRspMsg_t;


//battery SENSOR
/*!
 battery Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_batteryreqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_batteryReqMsg_t;

/*!
battery Response message: sent from the sensor to the collector
 in response to the battery Request message.
 */
typedef struct _Smsgs_batteryrspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_batteryRspMsg_t;



//f0 sensor
typedef struct _Smsgs_f0reqmsg_t // f0 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f0ReqMsg_t;
typedef struct _Smsgs_f0rspmsg_t // f0 Response message: sent from the sensor to the collector in response to the f0 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f0RspMsg_t;

//f1 sensor
typedef struct _Smsgs_f1reqmsg_t // f1 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f1ReqMsg_t;
typedef struct _Smsgs_f1rspmsg_t // f1 Response message: sent from the sensor to the collector in response to the f1 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f1RspMsg_t;

//f2 sensor
typedef struct _Smsgs_f2reqmsg_t // f2 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f2ReqMsg_t;
typedef struct _Smsgs_f2rspmsg_t // f2 Response message: sent from the sensor to the collector in response to the f2 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f2RspMsg_t;

//f3 sensor
typedef struct _Smsgs_f3reqmsg_t // f3 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f3ReqMsg_t;
typedef struct _Smsgs_f3rspmsg_t // f3 Response message: sent from the sensor to the collector in response to the f3 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f3RspMsg_t;

//f4 sensor
typedef struct _Smsgs_f4reqmsg_t // f4 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f4ReqMsg_t;
typedef struct _Smsgs_f4rspmsg_t // f4 Response message: sent from the sensor to the collector in response to the f4 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f4RspMsg_t;

//f5 sensor
typedef struct _Smsgs_f5reqmsg_t // f5 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f5ReqMsg_t;
typedef struct _Smsgs_f5rspmsg_t // f5 Response message: sent from the sensor to the collector in response to the f5 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f5RspMsg_t;

//f6 sensor
typedef struct _Smsgs_f6reqmsg_t // f6 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f6ReqMsg_t;
typedef struct _Smsgs_f6rspmsg_t // f6 Response message: sent from the sensor to the collector in response to the f6 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f6RspMsg_t;

//f7 sensor
typedef struct _Smsgs_f7reqmsg_t // f7 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f7ReqMsg_t;
typedef struct _Smsgs_f7rspmsg_t // f7 Response message: sent from the sensor to the collector in response to the f7 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f7RspMsg_t;

//f8 sensor
typedef struct _Smsgs_f8reqmsg_t // f8 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f8ReqMsg_t;
typedef struct _Smsgs_f8rspmsg_t // f8 Response message: sent from the sensor to the collector in response to the f8 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f8RspMsg_t;

//f9 sensor
typedef struct _Smsgs_f9reqmsg_t // f9 Request message: sent from controller to the sensor.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f9ReqMsg_t;
typedef struct _Smsgs_f9rspmsg_t // f9 Response message: sent from the sensor to the collector in response to the f9 Request message.
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_f9RspMsg_t;



/*!
 Tracking Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_trackingreqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_trackingReqMsg_t;

/*!
 Tracking Response message: sent from the sensor to the collector
 in response to the Tracking Request message.
 */
typedef struct _Smsgs_trackingrspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_trackingRspMsg_t;

/*!
 Toggle LED Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_toggleledreqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
} Smsgs_toggleLedReqMsg_t;

/*!
 Toggle LED Response message: sent from the sensor to the collector
 in response to the Toggle LED Request message.
 */
typedef struct _Smsgs_toggleledrspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
    /*! LED State - 0 is off, 1 is on - 1 byte */
    uint8_t ledState;
} Smsgs_toggleLedRspMsg_t;

/*!
 Identify LED Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_identifyledreqmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
    /*! time to identify in ms */
    uint8_t identifyTime;
} Smsgs_identifyLedReqMsg_t;

typedef struct _Smsgs_identifyledrspmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
    /* status */
    uint8_t status;
} Smsgs_identifyLedRspMsg_t;

/*!
 Temp Sensor Field
 */
typedef struct _Smsgs_tempsensorfield_t
{
    /*!
     Ambience Chip Temperature - each value represents a 0.01 C
     degree, so a value of 2475 represents 24.75 C.
     */
    int16_t ambienceTemp;
    /*!
     Object Temperature - each value represents a 0.01 C
     degree, so a value of 2475 represents 24.75 C.
     */
    int16_t objectTemp;
} Smsgs_tempSensorField_t;

/*!
 Light Sensor Field
 */
typedef struct _Smsgs_lightsensorfield_t
{
    /*! Raw Sensor Data read out of the OPT2001 light sensor */
    uint16_t rawData;
} Smsgs_lightSensorField_t;

/*!
 Humidity Sensor Field
 */
typedef struct _Smsgs_humiditysensorfield_t
{
    /*! Raw Temp Sensor Data from the TI HCD1000 humidity sensor. */
    uint16_t temp;
    /*! Raw Humidity Sensor Data from the TI HCD1000 humidity sensor. */
    uint16_t humidity;
} Smsgs_humiditySensorField_t;


/*!
 id0 sensor Field
 */
typedef struct _Smsgs_id0sensorfield_t
{
    /*! Raw Sensor Data read out of the id sensor */
    uint16_t id0RawData;
} Smsgs_id0SensorField_t;

/*!
 id1 sensor Field
 */
typedef struct _Smsgs_id1sensorfield_t
{
    /*! Raw Sensor Data read out of the id sensor */
    uint16_t id1RawData;
} Smsgs_id1SensorField_t;


/*!
 id2 sensor Field
 */
typedef struct _Smsgs_id2sensorfield_t
{
    /*! Raw Sensor Data read out of the id sensor */
    uint16_t id2RawData;
} Smsgs_id2SensorField_t;


/*!
 id3 sensor Field
 */
typedef struct _Smsgs_id3sensorfield_t
{
    /*! Raw Sensor Data read out of the id sensor */
    uint16_t id3RawData;
} Smsgs_id3SensorField_t;


/*!
 type Sensor Field
 */
typedef struct _Smsgs_typesensorfield_t
{
    /*! Raw Sensor Data read out of the type sensor */
    uint16_t typeRawData;
} Smsgs_typeSensorField_t;

/*!
 battery Sensor Field
 */
typedef struct _Smsgs_batterysensorfield_t
{
    /*! Raw Sensor Data read out of the battery sensor */
    uint16_t batteryRawData;
} Smsgs_batterySensorField_t;


/*!
 f0 Sensor Field
 */
typedef struct _Smsgs_f0sensorfield_t
{
    /*! Raw Sensor Data read out of the f0 sensor */
    uint16_t f0RawData;
} Smsgs_f0SensorField_t;

/*!
 f1 Sensor Field
 */
typedef struct _Smsgs_f1sensorfield_t
{
    /*! Raw Sensor Data read out of the f1 sensor */
    uint16_t f1RawData;
} Smsgs_f1SensorField_t;

/*!
 f2 Sensor Field
 */
typedef struct _Smsgs_f2sensorfield_t
{
    /*! Raw Sensor Data read out of the f2 sensor */
    uint16_t f2RawData;
} Smsgs_f2SensorField_t;

/*!
 f3 Sensor Field
 */
typedef struct _Smsgs_f3sensorfield_t
{
    /*! Raw Sensor Data read out of the f3 sensor */
    uint16_t f3RawData;
} Smsgs_f3SensorField_t;

/*!
 f4 Sensor Field
 */
typedef struct _Smsgs_f4sensorfield_t
{
    /*! Raw Sensor Data read out of the f4 sensor */
    uint16_t f4RawData;
} Smsgs_f4SensorField_t;

/*!
 f5 Sensor Field
 */
typedef struct _Smsgs_f5sensorfield_t
{
    /*! Raw Sensor Data read out of the f5 sensor */
    uint16_t f5RawData;
} Smsgs_f5SensorField_t;

/*!
 f6 Sensor Field
 */
typedef struct _Smsgs_f6sensorfield_t
{
    /*! Raw Sensor Data read out of the f6 sensor */
    uint16_t f6RawData;
} Smsgs_f6SensorField_t;

/*!
 f7 Sensor Field
 */
typedef struct _Smsgs_f7sensorfield_t
{
    /*! Raw Sensor Data read out of the f7 sensor */
    uint16_t f7RawData;
} Smsgs_f7SensorField_t;

/*!
 f8 Sensor Field
 */
typedef struct _Smsgs_f8sensorfield_t
{
    /*! Raw Sensor Data read out of the f8 sensor */
    uint16_t f8RawData;
} Smsgs_f8SensorField_t;

/*!
 f9 Sensor Field
 */
typedef struct _Smsgs_f9sensorfield_t
{
    /*! Raw Sensor Data read out of the f9 sensor */
    uint16_t f9RawData;
} Smsgs_f9SensorField_t;


/*!
 Message Statistics Field
 */
typedef struct _Smsgs_msgstatsfield_t
{
    /*! total number of join attempts (associate request sent) */
    uint16_t joinAttempts;
    /*! total number of join attempts failed */
    uint16_t joinFails;
    /*! total number of sensor data messages attempted. */
    uint16_t msgsAttempted;
    /*! total number of sensor data messages sent. */
    uint16_t msgsSent;
    /*! total number of tracking requests received */
    uint16_t trackingRequests;
    /*! total number of tracking response attempted */
    uint16_t trackingResponseAttempts;
    /*! total number of tracking response success */
    uint16_t trackingResponseSent;
    /*! total number of config requests received */
    uint16_t configRequests;
    /*! total number of config response attempted */
    uint16_t configResponseAttempts;
    /*! total number of config response success */
    uint16_t configResponseSent;
    /*!
     Total number of Channel Access Failures.  These are indicated in MAC data
     confirms for MAC data requests.
     */
    uint16_t channelAccessFailures;
    /*!
     Total number of MAC ACK failures. These are indicated in MAC data
     confirms for MAC data requests.
     */
    uint16_t macAckFailures;
    /*!
     Total number of MAC data request failures, other than channel access
     failure or MAC ACK failures.
     */
    uint16_t otherDataRequestFailures;
    /*! Total number of sync loss failures received for sleepy devices. */
    uint16_t syncLossIndications;
    /*! Total number of RX Decrypt failures. */
    uint16_t rxDecryptFailures;
    /*! Total number of TX Encrypt failures. */
    uint16_t txEncryptFailures;
    /*! Total number of resets. */
    uint16_t resetCount;
    /*!
     Assert reason for the last reset -  0 - no reason, 2 - HAL/ICALL,
     3 - MAC, 4 - TIRTOS
     */
    uint16_t lastResetReason;
    /*! Amount of time taken for node to join */
    uint16_t joinTime;
    /*! Max re-join delay */
    uint16_t interimDelay;
    /*!
     Number of broadcast messages received from the collector
     */
    uint16_t numBroadcastMsgRcvd;
    /*!
    Number of broadcast messages missed from the collector
    */
    uint16_t numBroadcastMsglost;
    /*!
    Average end to end delay
    */
    uint16_t avgE2EDelay;
    /*!
    Worst Case end to end delay
    */
    uint16_t worstCaseE2EDelay;
} Smsgs_msgStatsField_t;

#ifdef POWER_MEAS
/*!
 Power Meas Statistics Field
 */
typedef struct _Smsgs_powerMeastatsField_t
{
    /*! total number of polls sent. Not applicable for beacon mode */
    uint16_t pollRequestsSent;
    /*! total number of collector ramp data received */
    uint16_t rampDataRcvd;
} Smsgs_powerMeastatsField_t;
#endif

/*!
 Message Statistics Field
 */
typedef struct _Smsgs_configsettingsfield_t
{
    /*!
     Reporting Interval - in millseconds, how often to report, 0
     means reporting is off
     */
    uint32_t reportingInterval;
    /*!
     Polling Interval - in millseconds (32 bits) - If the sensor device is
     a sleep device, this states how often the device polls its parent for
     data. This field is 0 if the device doesn't sleep.
     */
    uint32_t pollingInterval;
} Smsgs_configSettingsField_t;

/*!
 Sensor Data message: sent from the sensor to the collector
 */
typedef struct _Smsgs_sensormsg_t
{
    /*! Command ID */
    Smsgs_cmdIds_t cmdId;
    /*! Extended Address */
    uint8_t extAddress[SMGS_SENSOR_EXTADDR_LEN];
    /*! Frame Control field - bit mask of Smsgs_dataFields */
    uint16_t frameControl;
    /*!
     Temp Sensor field - valid only if Smsgs_dataFields_tempSensor
     is set in frameControl.
     */
    Smsgs_tempSensorField_t tempSensor;
    /*!
     Light Sensor field - valid only if Smsgs_dataFields_lightSensor
     is set in frameControl.
     */
    Smsgs_lightSensorField_t lightSensor;
    /*!
     Humidity Sensor field - valid only if Smsgs_dataFields_humiditySensor
     is set in frameControl.
     */
    Smsgs_humiditySensorField_t humiditySensor;
    /*!
     Message Statistics field - valid only if Smsgs_dataFields_msgStats
     is set in frameControl.
     */
    Smsgs_msgStatsField_t msgStats;
    /*!
     Configuration Settings field - valid only if
     Smsgs_dataFields_configSettings is set in frameControl.
     */
    Smsgs_configSettingsField_t configSettings;

    /*!
     id0 Sensor field - valid only if Smsgs_dataFields_id0Sensor
     is set in frameControl.
     */
    Smsgs_id0SensorField_t id0Sensor;

    /*!
     id1 Sensor field - valid only if Smsgs_dataFields_id1Sensor
     is set in frameControl.
     */
    Smsgs_id1SensorField_t id1Sensor;

    /*!
     id2 Sensor field - valid only if Smsgs_dataFields_id2Sensor
     is set in frameControl.
     */
    Smsgs_id2SensorField_t id2Sensor;

    /*!
     id3 Sensor field - valid only if Smsgs_dataFields_id3Sensor
     is set in frameControl.
     */
    Smsgs_id3SensorField_t id3Sensor;


   /*!
    type Sensor field - valid only if Smsgs_dataFields_typeSensor
    is set in frameControl.
    */
   Smsgs_typeSensorField_t typeSensor;

   /*!
    battery Sensor field - valid only if Smsgs_dataFields_batterySensor
    is set in frameControl.
    */
   Smsgs_batterySensorField_t batterySensor;



   /*!
    f0 Sensor field - valid only if Smsgs_dataFields_f0Sensor
    is set in frameControl.
    */
   Smsgs_f0SensorField_t f0Sensor;

   /*!
    f1 Sensor field - valid only if Smsgs_dataFields_f1Sensor
    is set in frameControl.
    */
   Smsgs_f1SensorField_t f1Sensor;

   /*!
    f2 Sensor field - valid only if Smsgs_dataFields_f2Sensor
    is set in frameControl.
    */
   Smsgs_f2SensorField_t f2Sensor;

   /*!
    f3 Sensor field - valid only if Smsgs_dataFields_f3Sensor
    is set in frameControl.
    */
   Smsgs_f3SensorField_t f3Sensor;

   /*!
    f4 Sensor field - valid only if Smsgs_dataFields_f4Sensor
    is set in frameControl.
    */
   Smsgs_f4SensorField_t f4Sensor;

   /*!
    f5 Sensor field - valid only if Smsgs_dataFields_f5Sensor
    is set in frameControl.
    */
   Smsgs_f5SensorField_t f5Sensor;

   /*!
    f6 Sensor field - valid only if Smsgs_dataFields_f6Sensor
    is set in frameControl.
    */
   Smsgs_f6SensorField_t f6Sensor;

   /*!
    f7 Sensor field - valid only if Smsgs_dataFields_f7Sensor
    is set in frameControl.
    */
   Smsgs_f7SensorField_t f7Sensor;

   /*!
    f8 Sensor field - valid only if Smsgs_dataFields_f8Sensor
    is set in frameControl.
    */
   Smsgs_f8SensorField_t f8Sensor;

   /*!
    f9 Sensor field - valid only if Smsgs_dataFields_f9Sensor
    is set in frameControl.
    */
   Smsgs_f9SensorField_t f9Sensor;



} Smsgs_sensorMsg_t;

/*!
 Broadcast Cmd Request message: sent from controller to the sensor.
 */
typedef struct _Smsgs_broadcastcmdmsg_t
{
    /*! Command ID - 1 byte */
    Smsgs_cmdIds_t cmdId;
    uint16_t broadcastMsgId;
}Smsgs_broadcastcmdmsg_t;


#ifdef __cplusplus
}
#endif

#endif /* SMGSS_H */

