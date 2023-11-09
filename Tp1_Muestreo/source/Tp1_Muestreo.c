/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Tp1_Muestreo.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "arm_math.h"
/* TODO: insert other include files here. */
#define CAN_FREC 5
#define CAN_SAMPLES 512

#define NTAPS8LP 102
#define NTAPS8BP 102
#define NTAPS8HP 102
#define NTAPS8BSF 102

#define NTAPS16LP 102
#define NTAPS16BP 102
#define NTAPS16HP 102
#define NTAPS16BSF 102

#define NTAPS22LP 102
#define NTAPS22BP 102
#define NTAPS22HP 102
#define NTAPS22BSF 102

#define NTAPS44LP 102
#define NTAPS44BP 102
#define NTAPS44HP 102
#define NTAPS44BSF 102

#define NTAPS48LP 102
#define NTAPS48BP 102
#define NTAPS48HP 102
#define NTAPS48BSF 102


uint16_t frequencyBuff[CAN_FREC];
uint8_t currentFrequency;
uint8_t sampleMode;
uint8_t currentFilter;
q15_t circular_buffer[CAN_SAMPLES];
volatile uint16_t rbuff_index = 0;
volatile uint16_t f_rbuff_index = 0;
volatile uint16_t wbuff_index = 0;



const q15_t coeffs8LP[48] = {
	     1028,    -89,     43,     36,   -141,    263,   -389,    503,   -590,
	      629,   -607,    512,   -336,     79,    254,   -651,   1093,  -1555,
	     2012,  -2436,   2799,  -3078,   3253,  29455,   3253,  -3078,   2799,
	    -2436,   2012,  -1555,   1093,   -651,    254,     79,   -336,    512,
	     -607,    629,   -590,    503,   -389,    263,   -141,     36,     43,
	      -89,   1028, 0
	};

const q15_t coeffs8HP[48];
const q15_t coeffs8BP[48];
const q15_t coeffs8BSF[48];

const q15_t coeffs16LP[48];
const q15_t coeffs16HP[48];
const q15_t coeffs16BP[48];
const q15_t coeffs16BSF[48];

const q15_t coeffs22LP[48];
const q15_t coeffs22HP[48];
const q15_t coeffs22BP[48];
const q15_t coeffs22BSF[48];

const q15_t coeffs44LP[48];
const q15_t coeffs44HP[48];
const q15_t coeffs44BP[48];
const q15_t coeffs44BSF[48];


const int16_t coeffs48LP[102] = {
      -17,    -15,    -11,     -3,      7,     17,     25,     30,     28,
       18,      0,    -23,    -45,    -63,    -68,    -57,    -28,     16,
       66,    111,    138,    136,     99,     29,    -63,   -159,   -235,
     -268,   -240,   -148,      0,    177,    345,    462,    488,    399,
      192,   -107,   -447,   -756,   -952,   -959,   -723,   -223,    520,
     1443,   2445,   3408,   4206,   4733,   4917,   4733,   4206,   3408,
     2445,   1443,    520,   -223,   -723,   -959,   -952,   -756,   -447,
     -107,    192,    399,    488,    462,    345,    177,      0,   -148,
     -240,   -268,   -235,   -159,    -63,     29,     99,    136,    138,
      111,     66,     16,    -28,    -57,    -68,    -63,    -45,    -23,
        0,     18,     28,     30,     25,     17,      7,     -3,    -11,
      -15,    -17, 0
};
const q15_t coeffs48HP[48];
const q15_t coeffs48BP[48];
const q15_t coeffs48BSF[48];


arm_status init_status;



q15_t state8LP[NTAPS8LP];
q15_t state8BP[NTAPS8BP];
q15_t state8HP[NTAPS8HP];
q15_t state8BSF[NTAPS8BSF];

q15_t state16LP[NTAPS16LP];
q15_t state16BP[NTAPS16BP];
q15_t state16HP[NTAPS16HP];
q15_t state16BSF[NTAPS16BSF];

q15_t state22LP[NTAPS22LP];
q15_t state22BP[NTAPS22BP];
q15_t state22HP[NTAPS22HP];
q15_t state22BSF[NTAPS22BSF];

q15_t state44LP[NTAPS44LP];
q15_t state44BP[NTAPS44BP];
q15_t state44HP[NTAPS44HP];
q15_t state44BSF[NTAPS44BSF];

q15_t state48LP[NTAPS48LP];
q15_t state48BP[NTAPS48BP];
q15_t state48HP[NTAPS48HP];
q15_t state48BSF[NTAPS48BSF];




enum SampleModes{
    BY_PASS,
    BUFFER,

    SM_LAST
};

enum Frecuency{
	FR_8KH,
	FR_16KH,
	FR_22KH,
	FR_44KH,
	FR_48KH,

	FR_LAST
};

enum Filters{
	FI_NOFILTER,
    FI_LOWPASS,
	FI_HIGHTPASS,
	FI_BANDPASS,
	FI_DELETEBAND,

	FI_LAST
};

arm_fir_instance_q15 filters_matrix[FR_LAST][FI_LAST];

enum Colors{
    BLUE ,
    RED,
    GREEN,
    CYAN,
    YELLOW,
	MAGENTA,
    WHITE,

    C_LAST
};

void setLedColor(int color);


void initBuffers(){
    currentFrequency = 0;
    setLedColor(BLUE);
    sampleMode = BY_PASS;

//TODO SETEAR ESTE NUMERO CON LOS TICKS NECESARIOS PARA EL ADC
// Ec. de ticks: LDVALL = (period/clock period) - 1
    frequencyBuff[0] = 7499;
    frequencyBuff[1] = 3749;
    frequencyBuff[2] = 2726;
    frequencyBuff[3] = 1363;
    frequencyBuff[4] = 1249;
}


void initFilter(){
    init_status = arm_fir_init_q15(&filters_matrix[FR_8KH][FI_LOWPASS],     NTAPS8LP,  &coeffs8LP[0],  &state8LP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_8KH][FI_HIGHTPASS],   NTAPS8HP, &coeffs8HP[0],  &state8HP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_8KH][FI_BANDPASS],    NTAPS8BP,  &coeffs8BP[0],  &state8BP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_8KH][FI_DELETEBAND],  NTAPS8BSF, &coeffs8BSF[0], &state8BSF[0], 1);

    init_status = arm_fir_init_q15(&filters_matrix[FR_16KH][FI_LOWPASS],    NTAPS16LP,  &coeffs16LP[0],  &state16LP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_16KH][FI_HIGHTPASS],  NTAPS16HP,  &coeffs16HP[0],  &state16HP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_16KH][FI_BANDPASS],   NTAPS16BP,  &coeffs16BP[0],  &state16BP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_16KH][FI_DELETEBAND], NTAPS16BSF, &coeffs16BSF[0], &state16BSF[0], 1);

    init_status = arm_fir_init_q15(&filters_matrix[FR_22KH][FI_LOWPASS],    NTAPS22LP,  &coeffs22LP[0],  &state22LP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_22KH][FI_HIGHTPASS],  NTAPS22HP,  &coeffs22HP[0],  &state22HP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_22KH][FI_BANDPASS],   NTAPS22BP,  &coeffs22BP[0],  &state22BP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_22KH][FI_DELETEBAND], NTAPS22BSF, &coeffs22BSF[0], &state22BSF[0], 1);

    init_status = arm_fir_init_q15(&filters_matrix[FR_44KH][FI_LOWPASS],    NTAPS44LP,  &coeffs44LP[0],  &state44LP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_44KH][FI_HIGHTPASS],  NTAPS44HP,  &coeffs44HP[0],  &state44HP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_44KH][FI_BANDPASS],   NTAPS44BP,  &coeffs44BP[0],  &state44BP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_44KH][FI_DELETEBAND], NTAPS44BSF, &coeffs44BSF[0], &state44BSF[0], 1);

    init_status = arm_fir_init_q15(&filters_matrix[FR_48KH][FI_LOWPASS],    NTAPS48LP,  &coeffs48LP[0],  &state48LP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_48KH][FI_HIGHTPASS],  NTAPS48HP,  &coeffs48HP[0],  &state48HP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_48KH][FI_BANDPASS],   NTAPS48BP,  &coeffs48BP[0],  &state48BP[0],  1);
    init_status = arm_fir_init_q15(&filters_matrix[FR_48KH][FI_DELETEBAND], NTAPS48BSF, &coeffs48BSF[0], &state48BSF[0], 1);

}

q15_t BufferRead(){
    q15_t temp = 0;

    temp = circular_buffer[rbuff_index];
    rbuff_index++;
    if(rbuff_index>=CAN_SAMPLES){
        rbuff_index = 0;
    }


	return temp;
}

void BufferWrite(q15_t value){

	circular_buffer[wbuff_index] = value;

	wbuff_index++;
	if(wbuff_index>=CAN_SAMPLES){
		wbuff_index = 0;
	}

}
void setNextMode(uint8_t mode){

	sampleMode = mode;

	if(sampleMode== BY_PASS){
		setLedColor(currentFrequency);
	}else{
		setLedColor(WHITE);
	}
}

void setNextFrec(uint8_t f_indx){


    setLedColor(f_indx);
    currentFrequency = f_indx;
    //TODO ACA NACHO DEBERIA CAMBIAR LA FRECUENCIA DEL ADC
    //Usando el array at current frec
    PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, frequencyBuff[f_indx]);
}

void setFilter(uint8_t f_indx){


    //setLedColor(); ?????
    currentFilter = f_indx;

}

void ConfigLeds(){
    gpio_pin_config_t led_config = { kGPIO_DigitalOutput,1};

    GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);
}

void ConfigSW(){
    gpio_pin_config_t sw_config = { kGPIO_DigitalInput,0};

    GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);
    GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);

    PORT_SetPinInterruptConfig(BOARD_SW2_PORT, BOARD_SW2_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_SW2_PORT, BOARD_SW2_GPIO_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(BOARD_SW2_IRQ);
    EnableIRQ(BOARD_SW3_IRQ);

}

void ConfigDAC(){
    dac_config_t dacConfigStruct;

    DAC_GetDefaultConfig(&dacConfigStruct);
    DAC_Init(DAC0, &dacConfigStruct);
    DAC_Enable(DAC0, true);             /* Enable output. */
    DAC_SetBufferReadPointer(DAC0, 0U);
}

void setLedColor(int color){

    switch(color){
        case YELLOW:
        GPIO_PortSet(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PIN_MASK);
        GPIO_PortClear(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PIN_MASK);
        GPIO_PortClear(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN_MASK);
        break;
        case CYAN:
        GPIO_PortClear(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PIN_MASK);
        GPIO_PortSet(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PIN_MASK);
        GPIO_PortClear(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN_MASK);
        break;
        case MAGENTA:
        GPIO_PortClear(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PIN_MASK);
        GPIO_PortClear(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PIN_MASK);
        GPIO_PortSet(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN_MASK);
        break;
        case GREEN:
        GPIO_PortSet(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PIN_MASK);
        GPIO_PortSet(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PIN_MASK);
        GPIO_PortClear(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN_MASK);
        break;
        case RED:
        GPIO_PortSet(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PIN_MASK);
        GPIO_PortClear(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PIN_MASK);
        GPIO_PortSet(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN_MASK);
        break;
        case BLUE:
        GPIO_PortClear(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PIN_MASK);
        GPIO_PortSet(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PIN_MASK);
        GPIO_PortSet(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN_MASK);
        break;
        case WHITE:
        GPIO_PortClear(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PIN_MASK);
        GPIO_PortClear(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PIN_MASK);
        GPIO_PortClear(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PIN_MASK);
        break;
        default:
        break;
    }
}



/* Rutinas de interrupcion */

/* PIT0_IRQn interrupt handler
 * Le el valor del ADC, lo escribe en el buffer
 * y lo convierte por el DAC
*/
void PIT_CHANNEL_0_IRQHANDLER(void) {
  uint32_t intStatus;
  /* Reading all interrupt flags of status register */
  intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
  PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);
  volatile uint16_t g_Adc16ConversionValue = 0;
  q15_t q_filterOuput = 0;
  q15_t q_filterInput = 0;

  switch(sampleMode){
  case BY_PASS:

        /* Place your code here */

        g_Adc16ConversionValue = (uint16_t)(ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, 0U))>>4;
        //DAC_SetBufferValue(DAC0_PERIPHERAL, 0, g_Adc16ConversionValue);
        if(currentFilter == FI_NOFILTER){
            DAC_SetBufferValue(DAC0_PERIPHERAL, 0, g_Adc16ConversionValue);
        }else{
            q_filterInput = (q15_t)(g_Adc16ConversionValue);
            arm_fir_q15(&filters_matrix[currentFrequency][currentFilter],&q_filterInput, &q_filterOuput, 1);
            DAC_SetBufferValue(DAC0_PERIPHERAL, 0, (uint16_t)q_filterOuput);
        }
        BufferWrite((q15_t)(g_Adc16ConversionValue));
        break;
  case BUFFER:
        if(currentFilter == FI_NOFILTER){
            DAC_SetBufferValue(DAC0_PERIPHERAL, 0, (uint16_t)BufferRead());
        }else{
            q_filterInput = BufferRead();
            arm_fir_q15(&filters_matrix[currentFrequency][currentFilter],&q_filterInput, &q_filterOuput, 1);
            DAC_SetBufferValue(DAC0_PERIPHERAL, 0, (uint16_t)q_filterOuput());
        }
        break;
  }

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


/* UART0_RX_TX_IRQn interrupt handler */
void UART0_SERIAL_RX_TX_IRQHANDLER(void) {
  uint32_t intStatus;
  uint8_t data;
  uint8_t data_filter;
  uint8_t data_freq;
  uint8_t data_mode;
  /* Reading all interrupt flags of status registers */
  intStatus = UART_GetStatusFlags(UART0_PERIPHERAL);

  /* Flags can be cleared by reading the status register and reading/writing data registers.
    See the reference manual for details of each flag.
    The UART_ClearStatusFlags() function can be also used for clearing of flags in case the content of data regsiter is not used.
    For example:
        status_t status;
        intStatus &= ~(kUART_RxOverrunFlag | kUART_NoiseErrorFlag | kUART_FramingErrorFlag | kUART_ParityErrorFlag);
        status = UART_ClearStatusFlags(UART0_PERIPHERAL, intStatus);
  */

  /* Place your code here */
  /* If new data arrived. */
  if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & intStatus)
  {
	  data = UART_ReadByte(UART0);
	  data_filter = (data>>3) & 7;
	  data_freq = (data) & 7;
	  data_mode = (data>>6) & 1;



	 setNextFrec(data_freq);
     setNextMode(data_mode);
     setFilter(data_filter);


  }

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}




/*
 * @brief   Application entry point.
 */
int main(void) {

	
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
    ConfigLeds();
    ConfigSW();
    ConfigDAC();

    initBuffers();
    initFilter();

    PRINTF("Hello World\n");

    //q15_t buffer_data =0;

    /* Force the counter to be placed into memory. */

    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
    	/*
        if(buffer_flag){
        	if((buffer_data = BufferRead(rbuff_index))>0){
        		 DAC_SetBufferValue(DAC0_PERIPHERAL, 0, ((uint16_t)buffer_data&0x0fff));
        	}
        }
        delay();
        */
    }
    return 0 ;
}


/* Cambia la frecuencia con la cual interrumpe el PIT */
void BOARD_SW2_IRQ_HANDLER(){
    GPIO_PortClearInterruptFlags(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN_MASK);
    if(sampleMode == BY_PASS){
    	//setNextFrec();
    }

}


/* Desactiva y activa la interrupcion del PIT de manera alternativa con las presiones del boton,
 * Deteniendo y continuando el procesamiento de la señal
 * */
void BOARD_SW3_IRQ_HANDLER(){
    GPIO_PortClearInterruptFlags(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN_MASK);
	//setNextMode();

}

/*
    TODO IRQ DEL PIT QUE CON UN CASE EN sampleMode ponga en la funcion del dac
DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, dacValue);
el valor del adc o el del buffer
tambien falta el buffer con los q15 y que el pit cambie el puntero del buffer
*/
