#include "power_meter.h"

#define V_FACTOR_TRANSF (double)(R8 + R11)/(R11 * TR_V)
#define I_FACTOR_TRANSF (double)(R12 + R16)/(R16 * TR_I * R15)

#define NV_NOT_FIRST_EXECUTION_ID (0x0010)
#define NV_mV_ID (0x0011)
#define NV_nV_ID (0x0012)
#define NV_mI_ID (0x0013)
#define NV_nI_ID (0x0014)
#define NV_phaseE_ID (0x0015)

uint_fast16_t ADCbuffer[ADCBUFFERSIZE]; //common buffer for the ADC measure
uint_fast32_t v_microvolt_buffer[ADCBUFFERSIZE]; //common microvolt buffer for the voltage and current signals
uint_fast32_t i_microvolt_buffer[ADCBUFFERSIZE]; //common microvolt buffer for the voltage and current signals

extern mac_Config_t Main_user1Cfg;

static PIN_Handle sqWavePinHandle;  /* Pin driver handles */
static PIN_State sqWavePinState;    /* Global memory storage for a PIN_Config table */

PIN_Config sqWavePinTable[] = {
    V_SQ_WAVE_PIN  | PIN_INPUT_EN | PIN_NOPULL,
    I_SQ_WAVE_PIN  | PIN_INPUT_EN | PIN_NOPULL,
    PIN_TERMINATE
};

enum INTERRUPT_ACTION_SWITCH{FREQUENCY,PHASE,ADC_TRIGGER};
enum INTERRUPT_ACTION_SWITCH action;

calibration_coefficients coef;

// ADC
ADCBuf_Handle adcBuf;
ADCBuf_Params adcBufParams;

// timer
GPTimerCC26XX_Handle hTimer_powerMeter;
GPTimerCC26XX_Handle hTimer_calibrationTimeOut;
volatile _Bool timer_completed = 0;
uint_fast16_t stop_watch;
uint_fast8_t period_counter = 0;

volatile _Bool FIRST_V_EDGE_DETECTED = 0;

void init_acPowerMeter(){

    // Call driver init functions
    ADCBuf_init();

    // Set up an ADCBuf peripheral in ADCBuf_RECURRENCE_MODE_ONE_SHOT
    ADCBuf_Params_init(&adcBufParams);

    // Set up a Timer peripheral in GPT_MODE_ONESHOT
    GPTimerCC26XX_Params timerParams_powerMeter;
    GPTimerCC26XX_Params_init(&timerParams_powerMeter);
    timerParams_powerMeter.width          = GPT_CONFIG_16BIT;
    timerParams_powerMeter.mode           = GPT_MODE_ONESHOT;
    timerParams_powerMeter.direction      = GPTimerCC26XX_DIRECTION_UP;
    timerParams_powerMeter.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    hTimer_powerMeter = GPTimerCC26XX_open(Board_GPTIMER0B, &timerParams_powerMeter);//ADC buffer uses GPTIMER0A
    xdc_runtime_Types_FreqHz  cpu_freq;
    BIOS_getCpuFreq(&cpu_freq);
    GPTimerCC26XX_Value loadVal = (cpu_freq.lo/1000)*MAXIMUM_TIMER_TIME - 1;
    GPTimerCC26XX_setLoadValue(hTimer_powerMeter, loadVal);
    GPTimerCC26XX_registerInterrupt(hTimer_powerMeter, timerCallback_powerMeter, GPT_INT_TIMEOUT);
    GPTimerCC26XX_enableInterrupt(hTimer_powerMeter,  GPT_INT_TIMEOUT);

    // Open LED pins
    sqWavePinHandle = PIN_open(&sqWavePinState, sqWavePinTable);
    PIN_registerIntCb(sqWavePinHandle, &sqWaveCallbackFxn);

    if( NV_read_uint8_t(NV_NOT_FIRST_EXECUTION_ID) != 1 ){ // if is the first execution, NV memory is written with default values

        NV_write_uint8_t(NV_NOT_FIRST_EXECUTION_ID, 1);

        NV_write(NV_mV_ID, DEFAULT_mV);
        NV_write(NV_nV_ID, DEFAULT_nV);
        NV_write(NV_mI_ID, DEFAULT_mI);
        NV_write(NV_nI_ID, DEFAULT_nI);
        NV_write(NV_phaseE_ID, DEFAULT_phaseE);
    }

    coef.mV = NV_read(NV_mV_ID);
    coef.nV = NV_read(NV_nV_ID);
    coef.mI = NV_read(NV_mI_ID);
    coef.nI = NV_read(NV_nI_ID);
    coef.phi_e = NV_read(NV_phaseE_ID);
}

void measureGridValues_acPowerMeter(power_meter_variables *val, uint8_t repetitions){

    power_meter_variables rawVal;

    measureRawValues_acPowerMeter(&rawVal, repetitions);

    val->V_prim_rms = rawVal.V_prim_rms * coef.mV + coef.nV;
    val->I_prim_rms = rawVal.I_prim_rms * coef.mI + coef.nI;

    val->angle1 = Q1_Q4_phase_reallocation( rawVal.angle1 - coef.phi_e );
    val->angle2 = Q1_Q4_phase_reallocation( rawVal.angle2 - coef.phi_e );

    if(val->I_prim_rms<0.7){
        val->angle1 = 0;
        val->angle2 = 0;
    }

    val->f = rawVal.f;

    val->power = val->V_prim_rms * val->I_prim_rms * cos(val->angle2*M_PI/180);


}

void measureRawValues_acPowerMeter(power_meter_variables *rawVal, uint8_t repetitions){

    uint8_t j;


    // frequency and phase method 1 measurement
    double T = 0, f = 0;
    double angle1_raw;
    double sin_angle1_raw = 0, cos_angle1_raw = 0;
    rawVal->f = 0;

    for(j = 0; j<repetitions; j++){

        T = getPeriod();
        f = 1/T;
        rawVal->f += f/repetitions;

        angle1_raw = getRawPhase_method1(T); //aqui falla
        sin_angle1_raw += sin(angle1_raw*M_PI/180);
        cos_angle1_raw += cos(angle1_raw*M_PI/180);
    }

    rawVal->angle1 = Q1_Q4_phase_reallocation( atan(sin_angle1_raw/cos_angle1_raw)*180/M_PI );



    ////////////////////////
    uint32_t sample_frequency = (uint32_t)rawVal->f*POINTS_PER_PERIOD;

    double V_offset, I_offset;
    double V_rms_raw, I_rms_raw;

    double angle2_raw;
    double sin_angle2_raw = 0, cos_angle2_raw = 0;
    rawVal->V_prim_rms = 0;
    rawVal->I_prim_rms = 0;

    for(j = 0; j<repetitions; j++){


        adcBufRead(adcBuf, adcBufParams, sample_frequency, V_SIGNAL_PIN);
        V_offset = meanValue(&v_microvolt_buffer[0], ADCBUFFERSIZE);
        V_rms_raw = rmsValue(&v_microvolt_buffer[0], ADCBUFFERSIZE, V_offset);
        rawVal->V_prim_rms += (V_FACTOR_TRANSF*V_rms_raw/repetitions);

        adcBufRead(adcBuf, adcBufParams, sample_frequency, I_SIGNAL_PIN);
        I_offset = meanValue(&i_microvolt_buffer[0], ADCBUFFERSIZE);
        I_rms_raw = rmsValue(&i_microvolt_buffer[0], ADCBUFFERSIZE, I_offset);
        rawVal->I_prim_rms += (I_FACTOR_TRANSF*I_rms_raw/repetitions);

        angle2_raw = getRawPhase_method2(V_rms_raw, I_rms_raw,
                                         V_offset, I_offset,
                                         sample_frequency);

        sin_angle2_raw += sin(angle2_raw*M_PI/180);
        cos_angle2_raw += cos(angle2_raw*M_PI/180);

    }

    rawVal->angle2 = Q1_Q4_phase_reallocation( atan(sin_angle2_raw/cos_angle2_raw)*180/M_PI );


}


double rmsValue(uint32_t *microvolt_buffer, uint16_t size, double offset){
    uint16_t k;
    double sum = 0;
    double pre_value;

    for(k=0; k<size; k++){
        pre_value = (double)microvolt_buffer[k]/1e6 - offset;
        sum = sum + (pre_value*pre_value/size);
    }

    return sqrt(sum);
}

double meanValue(uint32_t *microvolt_buffer, uint16_t size){
    uint16_t k;
    double sum = 0;
    double pre_value;

    for(k=0; k<size; k++){
        pre_value = (double)microvolt_buffer[k]/1e6;
        sum = sum + (pre_value/size);
    }

    return sum;
}

double meanRawPower(double v_offset, double i_offset){
    uint16_t k;
    double sum = 0;
    double pre_value;

    for(k=0; k<ADCBUFFERSIZE; k++){
        pre_value = ((double)v_microvolt_buffer[k]/1e6 - v_offset)*((double)i_microvolt_buffer[k]/1e6 - i_offset);
        sum = sum + (pre_value/ADCBUFFERSIZE);
    }

    if(sum<0)
        sum = -sum;

    return sum;

}


double getPeriod(){
    double T = 0, Tn = 0;

    xdc_runtime_Types_FreqHz  cpu_freq;
    BIOS_getCpuFreq(&cpu_freq);// get frequency

    FIRST_V_EDGE_DETECTED = 0;
    // period measurement
    timer_completed = 0;
    stop_watch = 0;
    period_counter = 0;
    action = FREQUENCY;

    PIN_setInterrupt(&sqWavePinState, I_SQ_WAVE_PIN | PIN_IRQ_DIS);
    PIN_setInterrupt(&sqWavePinState, V_SQ_WAVE_PIN | PIN_IRQ_POSEDGE);
    while(timer_completed==0){ //when the timer is completed, its value is reset to zero, so the next time used will start at zero
    }
    PIN_setInterrupt(&sqWavePinState, V_SQ_WAVE_PIN | PIN_IRQ_DIS);

    Tn = (double)(stop_watch)/cpu_freq.lo;

    if(period_counter >= PERIODS_TO_MEASURE){
        T = Tn/PERIODS_TO_MEASURE;
    }

    return T;
}

double getRawPhase_method1(double T){ //it returns the phase angle using the method 1
    double p = 0, pn = 0;
    double angle = 0;

    xdc_runtime_Types_FreqHz  cpu_freq;
    BIOS_getCpuFreq(&cpu_freq);// get frequency


    FIRST_V_EDGE_DETECTED = 0;
    timer_completed = 0;
    stop_watch = 0;
    period_counter = 0;
    action = PHASE;
    PIN_setInterrupt(&sqWavePinState, I_SQ_WAVE_PIN | PIN_IRQ_POSEDGE);//PIN_IRQ_POSEDGE
    PIN_setInterrupt(&sqWavePinState, V_SQ_WAVE_PIN | PIN_IRQ_POSEDGE);
    while(timer_completed==0){ //when the timer is completed, its value is reset to zero, so the next time used will start at zero
    }
    PIN_setInterrupt(&sqWavePinState, I_SQ_WAVE_PIN | PIN_IRQ_DIS);
    PIN_setInterrupt(&sqWavePinState, V_SQ_WAVE_PIN | PIN_IRQ_DIS);

    pn = (double)(stop_watch)/cpu_freq.lo;
    p = pn - (T*period_counter);

    angle = (p/T)*360;


    _Bool end = 0;
    while(!end){ // we make sure that the angle is expressed between 0 and 360 degrees
        if(angle>360)
            angle = angle - 360;
        else if(angle<0)
            angle = angle + 360;
        else
            end = 1;
    }

    return angle;
}


double getRawPhase_method2(double V_rms_raw, double I_rms_raw, double V_offset, double I_offset, uint16_t sample_frequency){

    double P_mean_raw = meanRawPower(V_offset, I_offset);
    double angle = acos((P_mean_raw)/(V_rms_raw*I_rms_raw))*180/M_PI;

    _Bool end = 0;
    while(!end){ // we make sure that the angle is expressed between 0 and 360 degrees
        if(angle>360)
            angle = angle - 360;
        else if(angle<0)
            angle = angle + 360;
        else
            end = 1;
    }

    double i0 = (double)i_microvolt_buffer[0]-I_offset;
    double di0 = discreteDerivative( (double)i_microvolt_buffer[0]-I_offset,
                                      (double)i_microvolt_buffer[2]-I_offset,
                                      sample_frequency);

    //first quadrant
    if(i0<=0 && di0>=0)
        angle = angle;

    //second quadrant
    else if(i0<=0 && di0<0)
        angle = 180 - angle;

    //third quadrant
    else if(i0>0 && di0>=0)
        angle = angle + 180;

    //fourth quadrant
    else
        angle = 360 - angle;

    return angle;

}


double Q1_Q4_phase_reallocation(double angle){

    _Bool end = 0;
    while(!end){ // we make sure that the angle is expressed between 0 and 360 degrees
        if(angle>360)
            angle = angle - 360;
        else if(angle<0)
            angle = angle + 360;
        else
            end = 1;
    }

     if( angle > 90 && angle <180 ){
         // if the angle is in the 2nd quadrant, it is translated to the 4th quadrant by adding 180
         angle = angle + 180;
     }else if( angle >= 180 && angle < 270){
         // if the angle is in the 3rd quadrant, it is translated to the 1st quadrant by subtracting 180
         angle = angle - 180;
     }

     if( angle >= 270 && angle < 360)
         angle = angle - 360;

     return angle;
}



double discreteDerivative(double X_n_minus_1, double X_n_plus_1, uint32_t sample_frequency){

    double derivative = (X_n_plus_1 - X_n_minus_1)/(2/(double)sample_frequency);

    return derivative;

}


void sqWaveCallbackFxn(PIN_Handle handle, PIN_Id pinId) {

    switch(action)
    {
        case FREQUENCY:
            switch(pinId)
            {
                case V_SQ_WAVE_PIN: //rising edge for the voltage square wave
                    if(PIN_getInputValue(V_SQ_WAVE_PIN)){
                        if(FIRST_V_EDGE_DETECTED == 0){//first voltage edge
                            GPTimerCC26XX_start(hTimer_powerMeter);
                            FIRST_V_EDGE_DETECTED = 1;

                        }else if( stop_watch == 0){

                            period_counter++;

                            if( period_counter >= PERIODS_TO_MEASURE )
                                stop_watch = GPTimerCC26XX_getFreeRunValue(hTimer_powerMeter);
                        }
                    }
                    break;

                case I_SQ_WAVE_PIN:
                default:
                    break;
            }

            break;

        case PHASE:
            switch(pinId)
            {
                case V_SQ_WAVE_PIN: //rising edge for the voltage square wave
                    if(PIN_getInputValue(V_SQ_WAVE_PIN)){
                        if(FIRST_V_EDGE_DETECTED == 0){ //first voltage edge
                            GPTimerCC26XX_start(hTimer_powerMeter);
                            FIRST_V_EDGE_DETECTED = 1;

                        }else if(stop_watch == 0)
                            period_counter++;
                    }
                    break;

                case I_SQ_WAVE_PIN: //rising edge for the current square wave
                    if(PIN_getInputValue(I_SQ_WAVE_PIN)){
                        if( (period_counter >= PERIODS_TO_MEASURE) && stop_watch == 0 && PIN_getInputValue(I_SQ_WAVE_PIN))
                            stop_watch = GPTimerCC26XX_getFreeRunValue(hTimer_powerMeter);
                    }
                    break;

                default:
                    break;

            }
            break;

        case ADC_TRIGGER:
            switch(pinId)
            {
                case V_SQ_WAVE_PIN:
                    if(PIN_getInputValue(V_SQ_WAVE_PIN)){
                        if(FIRST_V_EDGE_DETECTED == 0)
                            FIRST_V_EDGE_DETECTED = 1;

                    }
                    break;

                case I_SQ_WAVE_PIN:
                default:
                    break;

            }
            break;

        default:
            break;
    }


}

/*
 * Callback function for the timer
 */
void timerCallback_powerMeter(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {
    // interrupt callback code goes here. Minimize processing in interrupt.

    timer_completed = 1;
}

void timerCallback_calibrationTimeOut(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {
    // interrupt callback code goes here. Minimize processing in interrupt.
    SysCtrlSystemReset();
}



void adcBufRead(ADCBuf_Handle handle, ADCBuf_Params handleParams, uint32_t frequency, uint_least8_t ADCchannel){ //call to "ADCBuf_init()" before calling adcBufRead()
    handleParams.returnMode = ADCBuf_RETURN_MODE_BLOCKING;
    handleParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_ONE_SHOT;
    handleParams.samplingFrequency = frequency;
    // Open ADCBuf driver
    handle = ADCBuf_open(Board_ADCBUF0, &handleParams);

    /* Configure the conversion struct */
    ADCBuf_Conversion conversion = {0};
    conversion.arg = NULL;
    conversion.adcChannel = ADCchannel;
    conversion.sampleBuffer = ADCbuffer;
    conversion.samplesRequestedCount = ADCBUFFERSIZE;

    if (!handle){
        /* AdcBuf did not open correctly. */
        while(1);
    }

    FIRST_V_EDGE_DETECTED = 0;
    action = ADC_TRIGGER;
    PIN_setInterrupt(&sqWavePinState, V_SQ_WAVE_PIN | PIN_IRQ_POSEDGE);
    while(FIRST_V_EDGE_DETECTED == 0){
    }
    PIN_setInterrupt(&sqWavePinState, V_SQ_WAVE_PIN | PIN_IRQ_DIS);

    /* Start converting. */
    if (ADCBuf_convert(handle, &conversion, 1) !=
        ADCBuf_STATUS_SUCCESS) {
        /* Did not start conversion process correctly. */
        while(1);
    }

    ADCBuf_adjustRawValues(handle, ADCbuffer, ADCBUFFERSIZE, ADCchannel);
    if(ADCchannel == V_SIGNAL_PIN){
        ADCBuf_convertAdjustedToMicroVolts(handle, ADCchannel, ADCbuffer, v_microvolt_buffer, ADCBUFFERSIZE);
    }else{
        ADCBuf_convertAdjustedToMicroVolts(handle, ADCchannel, ADCbuffer, i_microvolt_buffer, ADCBUFFERSIZE);
    }

    ADCBuf_close(handle);
}


/*-----------------------
 *
 * Calibration functions
 *
 *-----------------------
 */

void calibration_acPowerMeter(){

    Board_Led_control(board_led_type_LED1, board_led_state_ON);
    Board_Led_control(board_led_type_LED2, board_led_state_ON);


    uint16_t timeOut = 30000;

    // Set up a Timer peripheral in GPT_MODE_ONESHOT
    GPTimerCC26XX_Params timerParams_calibrationTimeOut;
    GPTimerCC26XX_Params_init(&timerParams_calibrationTimeOut);
    timerParams_calibrationTimeOut.width          = GPT_CONFIG_32BIT;
    timerParams_calibrationTimeOut.mode           = GPT_MODE_ONESHOT;
    timerParams_calibrationTimeOut.direction      = GPTimerCC26XX_DIRECTION_UP;
    timerParams_calibrationTimeOut.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
    hTimer_calibrationTimeOut = GPTimerCC26XX_open(Board_GPTIMER1A, &timerParams_calibrationTimeOut);//ADC buffer uses GPTIMER0A
    xdc_runtime_Types_FreqHz  cpu_freq;
    BIOS_getCpuFreq(&cpu_freq);
    GPTimerCC26XX_Value loadVal = (cpu_freq.lo/1000)*timeOut - 1;
    GPTimerCC26XX_setLoadValue(hTimer_calibrationTimeOut, loadVal);
    GPTimerCC26XX_registerInterrupt(hTimer_calibrationTimeOut, timerCallback_calibrationTimeOut, GPT_INT_TIMEOUT);
    GPTimerCC26XX_enableInterrupt(hTimer_calibrationTimeOut,  GPT_INT_TIMEOUT);

    GPTimerCC26XX_start(hTimer_calibrationTimeOut);



    while(1){

        uint8_t user_action = '0';

        System_printf("\r\n\r\n");
        System_printf("=====================\r\n");
        System_printf("Calibration assistant\r\n");
        System_printf("=====================\r\n");

        System_printf("\r\nPress 'r' for the data reading mode.\r\n");
        System_printf("Press 'c' for the calibration coefficients mode.\r\n");
        System_printf("Press 'e' to exit.");

        while(     (user_action=='r' || user_action=='R'
                || user_action=='c' || user_action=='C'
                || user_action=='e' || user_action=='E') == 0)
        {

            System_printf("\r\n --> ");
            user_action = readAction(1);
        }


        GPTimerCC26XX_stop(hTimer_calibrationTimeOut);

        switch(user_action)
        {
        case 'r': //read
        case 'R':
            dataReadingMode_calibration_acPowerMeter();
            break;

        case 'c':
        case 'C':
            calibrationCoefficientsMenu_acPowerMeter();
            break;

        case 'e':
        case 'E':
        default:
            System_printf("\r\n\r\n");
            Task_sleep(100*(1000/Clock_tickPeriod));
            SysCtrlSystemReset();
            break;
        }
    }
}

void dataReadingMode_calibration_acPowerMeter(){

    uint8_t mode = '0';

    System_printf("\r\n\r\n");
    System_printf("Data reading mode\r\n");
    System_printf("-----------------\r\n");

    System_printf("\r\nPress 'n' for the non-calibrated data reading mode.\r\n");
    System_printf("Press 'c' for the calibrated data reading mode.\r\n");
    System_printf("Press 'e' to go back.");

    while(     (mode=='n' || mode=='N'
           || mode=='c' || mode=='C'
           || mode=='e' || mode=='E') == 0)
    {

       System_printf("\r\n --> ");
       mode = readAction(1);
    }

    if(mode == 'e' || mode == 'E')
        return;


    power_meter_variables val;
    uint8_t measure_counter = 0;

    System_printf("\r\n\r\nPress 'n' to get a new measure. \r\n");
    System_printf("Press 'e' to go back.\r\n\r\n");
    System_printf("Number | Voltage [V] | Current [A] | f [Hz] | Angle M1 [deg] | Angle M2 [deg]");

    uint8_t user_action = '0';

    while( (user_action=='e' || user_action=='E')==0 ){

        user_action = readAction(0);
        if(user_action == 'n' || user_action == 'N'){

            measure_counter++;
            System_printf("\r\n%u |", measure_counter);
            Task_sleep(100*(1000/Clock_tickPeriod));

            if(mode == 'n' || mode == 'N') //non calibrated mode
                measureRawValues_acPowerMeter(&val, 7);

            else if(mode == 'c' || mode == 'C')//calibrated mode
                measureGridValues_acPowerMeter(&val, 7);

            System_printf(" %f | %f | %f | %f | %f",
                          val.V_prim_rms,
                          val.I_prim_rms,
                          val.f,
                          val.angle1,
                          val.angle2);
        }
    }

}

void calibrationCoefficientsMenu_acPowerMeter(){

    uint8_t user_action = '0';

    while( (user_action=='e' || user_action=='E') == 0){

        user_action = '0';

        System_printf("\r\n\r\n");
        System_printf("Calibration coefficients mode\r\n");
        System_printf("-----------------------------\r\n\r\n");
        System_printf("Press 'v' to modify the voltage calibration.\r\n");
        System_printf("Press 'i' to modify the current calibration.\r\n");
        System_printf("Press 'p' to modify the phase calibration.\r\n");
        System_printf("Press 'e' to go back.");

        while(  (user_action=='v' || user_action=='V'
              || user_action=='i' || user_action=='I'
              || user_action=='p' || user_action=='P'
              || user_action=='e' || user_action=='E') == 0)
        {
            System_printf("\r\n --> ");
            user_action = readAction(1);
        }
        switch(user_action)
        {
        case 'v':
        case 'V':
            calibrationCoefficients_acPowerMeter('v');
            break;

        case 'i':
        case 'I':
            calibrationCoefficients_acPowerMeter('i');
            break;

        case 'p':
        case 'P':
            calibrationCoefficients_acPowerMeter('p');
            break;

        case 'e':
        case 'E':
            break;
        }
    }


}

void calibrationCoefficients_acPowerMeter(uint8_t parameter){

    uint8_t user_action = '0';

    System_printf("\r\n\r\n");

    if(parameter == 'v'){
        System_printf("[Voltage trendline]\r\n\r\n");
        System_printf("Vreal = Vsensor*m_V + n_V\r\n");
        System_printf("   m_V = %f\r\n", NV_read(NV_mV_ID));
        System_printf("   n_V = %f\r\n\r\n", NV_read(NV_nV_ID));

    }else if(parameter == 'i'){
        System_printf("[Current trendline]\r\n\r\n");
        System_printf("Ireal = Isensor*m_I + n_I\r\n");
        System_printf("   m_I = %f\r\n", NV_read(NV_mI_ID));
        System_printf("   n_I = %f\r\n\r\n", NV_read(NV_nI_ID));

    }else{ // phase
        System_printf("[Phase error]\r\n\r\n");
        System_printf("phaseE = %f\r\n\r\n", NV_read(NV_phaseE_ID));

    }

    System_printf("Press 'k' to keep present values.\r\n");
    System_printf("Press 'r' to restore factory values.\r\n");
    System_printf("Press 'm' to modify the values.\r\n");
    System_printf("Press 'e' to go back.");

    while(  (user_action=='k' || user_action=='K'
          || user_action=='r' || user_action=='R'
          || user_action=='m' || user_action=='M'
          || user_action=='e' || user_action=='E') == 0)
    {
        System_printf("\r\n --> ");
        user_action = readAction(1);
    }
    switch(user_action)
    {
    case 'k':
    case 'K': //keep
        if(parameter == 'v'){
            System_printf("\r\n\r\n   m_V = %f   KEPT\r\n",  NV_read(NV_mV_ID) );
            System_printf("   n_V = %f   KEPT\r\n",  NV_read(NV_nV_ID) );

        }else if(parameter == 'i'){
            System_printf("\r\n\r\n   m_I = %f   KEPT\r\n",  NV_read(NV_mI_ID) );
            System_printf("   n_I = %f   KEPT\r\n",  NV_read(NV_nI_ID) );

        }else{ //phase
            System_printf("\r\n\r\n   phaseE = %f   KEPT\r\n",  NV_read(NV_phaseE_ID) );

        }
        break;

    case 'r':
    case 'R': //restore default
        if(parameter == 'v'){
            NV_write(NV_mV_ID, DEFAULT_mV );
            System_printf("\r\n\r\n   m_V = %f   RESTORED\r\n",  DEFAULT_mV );

            NV_write(NV_nV_ID, DEFAULT_nV );
            System_printf("   n_V = %f   RESTORED\r\n",  DEFAULT_nV );

        }else if(parameter == 'i'){
            NV_write(NV_mI_ID, DEFAULT_mI );
            System_printf("\r\n\r\n   m_I = %f   RESTORED\r\n",  DEFAULT_mI );

            NV_write(NV_nI_ID, DEFAULT_nI );
            System_printf("   n_I = %f   RESTORED\r\n",  DEFAULT_nI );

        }else{ // phase
            NV_write(NV_phaseE_ID, DEFAULT_phaseE );
            System_printf("\r\n\r\n   phaseE = %f   RESTORED\r\n",  DEFAULT_phaseE );

        }

        break;

    case 'm':
    case 'M': //modify
        System_printf("\r\n\r\nEnter the new values with 6 digits:\r\n\r\n");

        if(parameter == 'v'){
            System_printf("   m_V = ");
            NV_write(NV_mV_ID, readNumber(6+1) );
            System_printf("   SAVED\r\n");

            System_printf("   n_V = ");
            NV_write(NV_nV_ID, readNumber(6+1) );
            System_printf("   SAVED\r\n");

        }else if(parameter == 'i'){
            System_printf("   m_I = ");
            NV_write(NV_mI_ID, readNumber(6+1) );
            System_printf("   SAVED\r\n");

            System_printf("   n_I = ");
            NV_write(NV_nI_ID, readNumber(6+1) );
            System_printf("   SAVED\r\n");

        }else{ // phase
            System_printf("   phaseE = ");
            NV_write(NV_phaseE_ID, readNumber(6+1) );
            System_printf("   SAVED\r\n");

        }
        break;

    case 'e':
    case 'E':
        break;
    }

}

void NV_write(uint16_t itemID, double value){
    NVINTF_itemID_t id;

    /* Setup NV ID for the number of entries in the black list */
    id.systemID = NVINTF_SYSID_APP;
    id.itemID = itemID;
    id.subID = 0;

    // write to NV
    Main_user1Cfg.nvFps.writeItem(id, sizeof(value), &value);

    Task_sleep(200*(1000/Clock_tickPeriod));

}

double NV_read(uint16_t itemID){
    double value = 0;
    NVINTF_itemID_t id;

    /* Setup NV ID for the number of entries in the black list */
    id.systemID = NVINTF_SYSID_APP;
    id.itemID = itemID;
    id.subID = 0;

    // read from NV
    Main_user1Cfg.nvFps.readItem(id, 0, sizeof(value), &value);

    Task_sleep(200*(1000/Clock_tickPeriod));

    return value;
}

void NV_write_uint8_t(uint16_t itemID, uint8_t value){
    NVINTF_itemID_t id;

    /* Setup NV ID for the number of entries in the black list */
    id.systemID = NVINTF_SYSID_APP;
    id.itemID = itemID;
    id.subID = 0;

    // write to NV
    Main_user1Cfg.nvFps.writeItem(id, sizeof(value), &value);

    Task_sleep(200*(1000/Clock_tickPeriod));
}

uint8_t NV_read_uint8_t(uint16_t itemID){
    uint8_t value = 0;
    NVINTF_itemID_t id;

    /* Setup NV ID for the number of entries in the black list */
    id.systemID = NVINTF_SYSID_APP;
    id.itemID = itemID;
    id.subID = 0;

    // read from NV
    Main_user1Cfg.nvFps.readItem(id, 0, sizeof(value), &value);

    Task_sleep(200*(1000/Clock_tickPeriod));

    return value;
}

char readAction(_Bool writeInput){
    char input[1]={'0'};

    UART_read(hUart, &input, sizeof(input));

    if(writeInput)
        UART_write(hUart, input, sizeof(input));

    return input[0];
}

double readNumber(uint8_t L){ // L is the number of characters of the string to be read.

     char input[1];
     int counter = 0;
     int decPosition = -1;
     double num = 0;
     _Bool negative = 0;


     while(counter<L){
         UART_read(hUart, &input, sizeof(input));
         if( input[0]=='-' ){
             UART_write(hUart, input, sizeof(input));
             negative = 1;
         }else if( input[0]=='.' || input[0]==',' ){   // decimal separator detected
             UART_write(hUart, input, sizeof(input));
             counter++;
             decPosition = 1;

         }else if( (input[0]>='0' && input[0]<='9') ){   //number detected
             UART_write(hUart, input, sizeof(input));

             if(decPosition <= 0)    //integer part
                 num = num*10.0 + (double)(input[0]-48);
             else{   //fractional part
                 num = num + (double)((input[0]-48)*pow(10.0,-decPosition));
                 decPosition++;
             }
             counter++;
         }
     }

     if(negative)
         num = -num;

     return num;
 }

