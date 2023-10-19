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
#define NTAPS8LP 241

uint16_t frequencyBuff[CAN_FREC];
uint8_t currentFrequency;
uint8_t sampleMode;
q15_t circular_buffer[CAN_SAMPLES];
q15_t circular_buffer_filtered[CAN_SAMPLES];
volatile uint16_t rbuff_index = 0;
volatile uint16_t f_rbuff_index = 0;
volatile uint16_t wbuff_index = 0;



const float32_t coeffs8LP[241] = {
               -0,-5.100030265e-08,3.916120761e-07,-1.22464246e-06,2.585895118e-06,
  -4.294790415e-06,5.949324532e-06,-6.97117548e-06,6.698498055e-06,-4.515455657e-06,
  -2.33052399e-19,6.933951227e-06,-1.592601438e-05,2.611505806e-05,-3.616024696e-05,
  4.434746734e-05,-4.87795769e-05,4.763582183e-05,-3.947316873e-05, 2.35317475e-05,
  -3.364308987e-19,-2.980618228e-05,6.339113315e-05,-9.718345973e-05,0.0001268090418,
  -0.0001475244062,0.0001547789725,-0.0001448519179,0.0001154919737,-6.647586997e-05,
  4.835918305e-18,7.917221956e-05,-0.0001638645917,0.0002449713938,-0.000312259217,
   0.000355435157,-0.0003653887834,0.0003354792134,-0.0002627163194,0.0001486758993,
  -4.560860899e-18,-0.0001716368715,0.0003501654719,-0.0005163699971,0.0006496809074,
  -0.0007303715101,0.0007419537869,-0.0006735153147,0.0005217175349,-0.0002921783307,
  -2.961560331e-18,0.0003307237348,-0.0006684910622,0.0009770152392,-0.001218712307,
    0.00135875633,-0.001369307516,    0.001233452,-0.000948375673,0.0005273252027,
  -2.474486318e-17,-0.0005888420856, 0.001182607608,-0.001717756386, 0.002129993867,
  -0.002361217048, 0.002366528148,-0.002120546531, 0.001622254262,-0.0008976933896,
  5.985398262e-18,0.0009934961563,-0.001987084979, 0.002875062171,-0.003552033799,
   0.003924217541,-0.003920623101, 0.003502904437,-0.002672703704,  0.00147546723,
  -2.210263324e-17,-0.001626598998, 0.003248513909,-0.004694719799,  0.00579536939,
  -0.006399621721, 0.006393202581,-0.005713852588, 0.004362905864,-0.002411453519,
   5.62984485e-17, 0.002668945584,-0.005345216487, 0.007751463912,-0.009608275257,
    0.01066188142, -0.01071199775, 0.009637139738,-0.007414808962, 0.004134247545,
  -3.133005134e-17,-0.004674733616, 0.009485092945, -0.01396109071,  0.01760195754,
   -0.01991576515,  0.02046117373, -0.01888781227,  0.01497172005,-0.008642576635,
  3.410980172e-17,  0.01068295538,  -0.0229699444,  0.03628481179, -0.04994696379,
       0.06321612, -0.07534306496,   0.0856225118, -0.09344382584,  0.09833552688,
     0.8999999762,  0.09833552688, -0.09344382584,   0.0856225118, -0.07534306496,
       0.06321612, -0.04994696379,  0.03628481179,  -0.0229699444,  0.01068295538,
  3.410980172e-17,-0.008642576635,  0.01497172005, -0.01888781227,  0.02046117373,
   -0.01991576515,  0.01760195754, -0.01396109071, 0.009485092945,-0.004674733616,
  -3.133005134e-17, 0.004134247545,-0.007414808962, 0.009637139738, -0.01071199775,
    0.01066188142,-0.009608275257, 0.007751463912,-0.005345216487, 0.002668945584,
   5.62984485e-17,-0.002411453519, 0.004362905864,-0.005713852588, 0.006393202581,
  -0.006399621721,  0.00579536939,-0.004694719799, 0.003248513909,-0.001626598998,
  -2.210263324e-17,  0.00147546723,-0.002672703704, 0.003502904437,-0.003920623101,
   0.003924217541,-0.003552033799, 0.002875062171,-0.001987084979,0.0009934961563,
  5.985398262e-18,-0.0008976933896, 0.001622254262,-0.002120546531, 0.002366528148,
  -0.002361217048, 0.002129993867,-0.001717756386, 0.001182607608,-0.0005888420856,
  -2.474486318e-17,0.0005273252027,-0.000948375673,    0.001233452,-0.001369307516,
    0.00135875633,-0.001218712307,0.0009770152392,-0.0006684910622,0.0003307237348,
  -2.961560331e-18,-0.0002921783307,0.0005217175349,-0.0006735153147,0.0007419537869,
  -0.0007303715101,0.0006496809074,-0.0005163699971,0.0003501654719,-0.0001716368715,
  -4.560860899e-18,0.0001486758993,-0.0002627163194,0.0003354792134,-0.0003653887834,
   0.000355435157,-0.000312259217,0.0002449713938,-0.0001638645917,7.917221956e-05,
  4.835918305e-18,-6.647586997e-05,0.0001154919737,-0.0001448519179,0.0001547789725,
  -0.0001475244062,0.0001268090418,-9.718345973e-05,6.339113315e-05,-2.980618228e-05,
  -3.364308987e-19, 2.35317475e-05,-3.947316873e-05,4.763582183e-05,-4.87795769e-05,
  4.434746734e-05,-3.616024696e-05,2.611505806e-05,-1.592601438e-05,6.933951227e-06,
  -2.33052399e-19,-4.515455657e-06,6.698498055e-06,-6.97117548e-06,5.949324532e-06,
  -4.294790415e-06,2.585895118e-06,-1.22464246e-06,3.916120761e-07,-5.100030265e-08,
               -0
};


q15_t coeffsq15_8[NTAPS8LP];



int filters_matrix[5][4];

arm_fir_instance_q15 filter8LP;
q15_t state8LP[512+NTAPS8LP];

arm_fir_instance_q15 filter8BP;
arm_fir_instance_q15 filter8HP;
arm_fir_instance_q15 filter8BSF;

arm_fir_instance_q15 filter16LP;
arm_fir_instance_q15 filter16BP;
arm_fir_instance_q15 filter16HP;
arm_fir_instance_q15 filter16BSF;

arm_fir_instance_q15 filter22LP;
arm_fir_instance_q15 filter22BP;
arm_fir_instance_q15 filter22HP;
arm_fir_instance_q15 filter22BSF;

arm_fir_instance_q15 filter44LP;
arm_fir_instance_q15 filter44BP;
arm_fir_instance_q15 filter44HP;
arm_fir_instance_q15 filter44BSP;

arm_fir_instance_q15 filter48LP;
arm_fir_instance_q15 filter48BP;
arm_fir_instance_q15 filter48HP;
arm_fir_instance_q15 filter48BSP;




enum SampleModes{
    BY_PASS,
    BUFFER,

    SM_LAST
};

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
	arm_float_to_q15(&coeffs8LP[0],&coeffsq15_8[0], NTAPS8LP);
	arm_fir_init_q15(&filter8LP, NTAPS8LP, &coeffsq15_8[0], &state8LP[0], 512);
}

q15_t BufferRead(int filter){
	q15_t temp = 0;
	if(!filter){
		temp = circular_buffer[rbuff_index];
		rbuff_index++;
		if(rbuff_index>=CAN_SAMPLES){
			rbuff_index = 0;
		}
	}
	if(filter){
		temp = circular_buffer_filtered[f_rbuff_index];
		f_rbuff_index++;
		if(f_rbuff_index>=CAN_SAMPLES){
			f_rbuff_index = 0;
		}
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
	if(mode){
		sampleMode = BY_PASS;
	}
	if(!mode){
		sampleMode = BUFFER;
	}


	if(sampleMode== BY_PASS){
		setLedColor(currentFrequency);
	}else{
		setLedColor(WHITE);
	}
}

void setNextFrec(uint8_t f_indx){


    setLedColor(f_indx);

    //TODO ACA NACHO DEBERIA CAMBIAR LA FRECUENCIA DEL ADC
    //Usando el array at current frec
    PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, frequencyBuff[f_indx]);
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
/* TODO: insert other definitions and declarations here. */
void delay(){
	for(int i =0 ;i<800000; i++ ){
        __asm volatile ("nop");
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

  switch(sampleMode){
    case BY_PASS:

    /* Place your code here */


        g_Adc16ConversionValue = (uint16_t)(ADC16_GetChannelConversionValue(ADC1_PERIPHERAL, 0U))>>4;
        //DAC_SetBufferValue(DAC0_PERIPHERAL, 0, g_Adc16ConversionValue);
        BufferWrite((q15_t)(g_Adc16ConversionValue));
        arm_fir_q15(&coeffsq15_8[0], &circular_buffer[0], &circular_buffer_filtered[0], 512);
        DAC_SetBufferValue(DAC0_PERIPHERAL, 0, (uint16_t)BufferRead(1));

    break;
    case BUFFER: 
        DAC_SetBufferValue(DAC0_PERIPHERAL, 0, (uint16_t)BufferRead(0));
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
	  data_filter = (data>>3) & 5;
	  data_freq = (data) & 5;
	  data_mode = (data>>6) & 1;



	 setNextFrec(data_freq);
     setNextMode(data_mode);
     //setFilter(data_filter);


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
