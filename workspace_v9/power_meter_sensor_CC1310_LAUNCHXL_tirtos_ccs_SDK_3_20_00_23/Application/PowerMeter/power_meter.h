#ifndef __POWER_METER_H__
#define __POWER_METER_H__

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h> // For sleep()

/* Driver Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>
#include <string.h>
#include <stdlib.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include "uart_printf.h"
#include <sys_ctrl.h>
#include <ti/sysbios/knl/Task.h>
#include "nvintf.h"
#include "macconfig.h"
#include <ti/sysbios/knl/Clock.h>
#include "board_led.h"


#include "power_meter_config.h"

/* Example/Board Header files */
#include "Board.h"


typedef struct POWER_METER_VARIABLES{
    double power;
    double V_prim_rms, I_prim_rms;
    double angle1,angle2, f;
} power_meter_variables;

typedef struct CALIBRATION_COEFFICIENTS{
    double mV, nV;
    double mI, nI;
    double phi_e;
} calibration_coefficients;

enum SENSOR_ACTION{
    init,
    calibration,
    measure
};

void init_acPowerMeter();
void measureGridValues_acPowerMeter(power_meter_variables *val, uint8_t repetitions);
void measureRawValues_acPowerMeter(power_meter_variables *rawVal, uint8_t repetitions);

double rmsValue(uint32_t *microvolt_buffer, uint16_t size, double offset);
double meanValue(uint32_t *microvolt_buffer, uint16_t size);
double meanRawPower(double v_offset, double i_offset);

double getPeriod();
double getRawPhase_method1(double T);
double getRawPhase_method2(double V_rms_raw, double I_rms_raw, double V_offset, double I_offset, uint16_t sample_frequency);

double Q1_Q4_phase_reallocation(double angle);
double discreteDerivative(double X_n_minus_1, double X_n_plus_1, uint32_t sample_frequency);

void sqWaveCallbackFxn(PIN_Handle handle, PIN_Id pinId);
void timerCallback_powerMeter(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
void adcBufRead(ADCBuf_Handle handle, ADCBuf_Params handleParams, uint32_t frequency, uint_least8_t ADCchannel);

void calibration_acPowerMeter();
void dataReadingMode_calibration_acPowerMeter();
void calibrationCoefficientsMenu_acPowerMeter();
void calibrationCoefficients_acPowerMeter(uint8_t parameter);
char readAction(_Bool writeInput);
double readNumber(uint8_t L);
void NV_write(uint16_t itemID, double value);
double NV_read(uint16_t itemID);
void NV_write_uint8_t(uint16_t itemID, uint8_t value);
uint8_t NV_read_uint8_t(uint16_t itemID);
void timerCallback_calibrationTimeOut(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);

//char readConfirmation();
//double readNumber(uint8_t L, char *user_action);

#endif
