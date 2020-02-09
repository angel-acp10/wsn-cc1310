#ifndef __POWER_METER__CONFIG_H__
#define __POWER_METER__CONFIG_H__

/* Example/Board Header files */
#include "Board.h"

// Calibration parameters
#define DEFAULT_mV (0.9699)
#define DEFAULT_nV (22.2352)
#define DEFAULT_mI (1.0643)
#define DEFAULT_nI (0.0045)
#define DEFAULT_phaseE (17.9686)

#define V_SQ_WAVE_PIN   (Board_DIO21)
#define I_SQ_WAVE_PIN   (Board_DIO22)
#define V_SIGNAL_PIN    (Board_ADC0)
#define I_SIGNAL_PIN    (Board_ADC1)
#define OFFSET_SIGNAL_PIN (CC1310_LAUNCHXL_ADC2)

#define POINTS_PER_PERIOD (100)
#define N_PERIODS (2)
#define ADCBUFFERSIZE    (N_PERIODS*POINTS_PER_PERIOD)

//voltage conditioning circuit [kohm]
#define R8      (56000)//(39000)
#define R11     (4700)
#define TR_V    (0.04217391304)

//current conditioning circuit [kohm]
#define R12     (180000)//(120000)
#define R16     (27000)
#define TR_I    (0.001)
#define R15     (100)

#define MAXIMUM_TIMER_TIME (200) //ms
#define PERIODS_TO_MEASURE  (8)


#endif
