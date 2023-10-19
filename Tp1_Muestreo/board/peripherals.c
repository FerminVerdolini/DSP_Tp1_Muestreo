/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v11.0
processor: MK64FN1M0xxx12
package_id: MK64FN1M0VLL12
mcu_data: ksdk2_0
processor_version: 12.0.0
board: FRDM-K64F
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: ba93d951-eaa6-447f-8dec-a87581dad20f
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'gpio_adapter_common'
- type_id: 'gpio_adapter_common_57579b9ac814fe26bf95df0a384c36b6'
- global_gpio_adapter_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common_9cb8e302497aa696fdbb5a4fd622c2a8'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table:
      - 0: []
      - 1: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * PIT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'PIT'
- type: 'pit'
- mode: 'LPTMR_GENERAL'
- custom_name_enabled: 'false'
- type_id: 'pit_ab54f91356454adb874dafbb69e655fd'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'PIT'
- config_sets:
  - fsl_pit:
    - enableRunInDebug: 'false'
    - timingConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'BOARD_BootClockRUN'
    - channels:
      - 0:
        - channel_id: 'CHANNEL_0'
        - channelNumber: '0'
        - enableChain: 'false'
        - timerPeriod: '8khz'
        - startTimer: 'true'
        - enableInterrupt: 'true'
        - interrupt:
          - IRQn: 'PIT0_IRQn'
          - enable_interrrupt: 'enabled'
          - enable_priority: 'false'
          - priority: '0'
          - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const pit_config_t PIT_config = {
  .enableRunInDebug = false
};

static void PIT_init(void) {
  /* Initialize the PIT. */
  PIT_Init(PIT_PERIPHERAL, &PIT_config);
  /* Set channel 0 period to 125 µs (7500 ticks). */
  PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, PIT_CHANNEL_0_TICKS);
  /* Enable interrupts from channel 0. */
  PIT_EnableInterrupts(PIT_PERIPHERAL, PIT_CHANNEL_0, kPIT_TimerInterruptEnable);
  /* Enable interrupt PIT_CHANNEL_0_IRQN request in the NVIC */
  EnableIRQ(PIT_CHANNEL_0_IRQN);
  /* Start channel 0. */
  PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
}

/***********************************************************************************************************************
 * DAC0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'DAC0'
- type: 'dac'
- mode: 'basic'
- custom_name_enabled: 'false'
- type_id: 'dac_a54f338a6fa6fd273bc89d61f5a3b85e'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'DAC0'
- config_sets:
  - fsl_dac:
    - dac_config:
      - referenceVoltageSource: 'kDAC_ReferenceVoltageSourceVref2'
      - enableLowPowerMode: 'false'
    - dac_enable: 'true'
    - dac_value: '0'
    - quick_selection: 'QS_DAC_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const dac_config_t DAC0_config = {
  .referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2,
  .enableLowPowerMode = false
};

static void DAC0_init(void) {
  /* Initialize DAC converter */
  DAC_Init(DAC0_PERIPHERAL, &DAC0_config);
  /* Output value of DAC. */
  DAC_SetBufferValue(DAC0_PERIPHERAL, 0U, 0U);
  /* Make sure the read pointer is set to the start */
  DAC_SetBufferReadPointer(DAC0_PERIPHERAL, 0U);
  /* Enable DAC output */
  DAC_Enable(DAC0_PERIPHERAL, true);
}

/***********************************************************************************************************************
 * ADC1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC1'
- type: 'adc16'
- mode: 'ADC'
- custom_name_enabled: 'false'
- type_id: 'adc16_897558f9b7366ed198de18c33097d7d2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'ADC1'
- config_sets:
  - fsl_adc16:
    - adc16_config:
      - referenceVoltageSource: 'kADC16_ReferenceVoltageSourceVref'
      - clockSource: 'kADC16_ClockSourceAsynchronousClock'
      - enableAsynchronousClock: 'true'
      - clockDivider: 'kADC16_ClockDivider8'
      - resolution: 'kADC16_ResolutionSE16Bit'
      - longSampleMode: 'kADC16_LongSampleDisabled'
      - hardwareAverageMode: 'kADC16_HardwareAverageDisabled'
      - enableHighSpeed: 'false'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'true'
    - adc16_channel_mux_mode: 'kADC16_ChannelMuxA'
    - adc16_hardware_compare_config:
      - hardwareCompareModeEnable: 'false'
    - doAutoCalibration: 'false'
    - offset: '0'
    - trigger: 'false'
    - enable_dma: 'false'
    - enable_irq: 'false'
    - adc_interrupt:
      - IRQn: 'ADC0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - adc16_channels_config:
      - 0:
        - channelName: ''
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.18'
        - enableInterruptOnConversionCompleted: 'false'
        - channelGroup: '0'
        - initializeChannel: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
adc16_channel_config_t ADC1_channelsConfig[1] = {
  {
    .channelNumber = 18U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false,
  }
};
const adc16_config_t ADC1_config = {
  .referenceVoltageSource = kADC16_ReferenceVoltageSourceVref,
  .clockSource = kADC16_ClockSourceAsynchronousClock,
  .enableAsynchronousClock = true,
  .clockDivider = kADC16_ClockDivider8,
  .resolution = kADC16_ResolutionSE16Bit,
  .longSampleMode = kADC16_LongSampleDisabled,
  .hardwareAverageMode = kADC16_HardwareAverageDisabled,
  .enableHighSpeed = false,
  .enableLowPower = false,
  .enableContinuousConversion = true
};
const adc16_channel_mux_mode_t ADC1_muxMode = kADC16_ChannelMuxA;

static void ADC1_init(void) {
  /* Initialize ADC16 converter */
  ADC16_Init(ADC1_PERIPHERAL, &ADC1_config);
  /* Make sure, that software trigger is used */
  ADC16_EnableHardwareTrigger(ADC1_PERIPHERAL, false);
  /* Configure channel multiplexing mode */
  ADC16_SetChannelMuxMode(ADC1_PERIPHERAL, ADC1_muxMode);
  /* Initialize channel */
  ADC16_SetChannelConfig(ADC1_PERIPHERAL, ADC1_CH0_CONTROL_GROUP, &ADC1_channelsConfig[0]);
}

/***********************************************************************************************************************
 * UART0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART0'
- type: 'uart'
- mode: 'interrupts'
- custom_name_enabled: 'false'
- type_id: 'uart_9b45c456566d03f79ecfe90751c10bb4'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART0'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '115200'
      - parityMode: 'kUART_ParityDisabled'
      - dataBitsCount: 'kUART_EightDataBits'
      - stopBitCount: 'kUART_OneStopBit'
      - enableMatchAddress1: 'false'
      - matchAddress1: '0'
      - enableMatchAddress2: 'false'
      - matchAddress2: '0'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
    - quick_selection: 'QuickSelection1'
  - interruptsCfg:
    - interrupts: 'kUART_TransmissionCompleteInterruptEnable kUART_RxDataRegFullInterruptEnable kUART_RxOverrunInterruptEnable'
    - interrupt_vectors:
      - enable_rx_tx_irq: 'true'
      - interrupt_rx_tx:
        - IRQn: 'UART0_RX_TX_IRQn'
        - enable_interrrupt: 'enabled'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
      - enable_err_irq: 'false'
      - interrupt_err:
        - IRQn: 'UART0_ERR_IRQn'
        - enable_interrrupt: 'enabled'
        - enable_priority: 'false'
        - priority: '0'
        - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART0_config = {
  .baudRate_Bps = 115200UL,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0U,
  .rxFifoWatermark = 1U,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

static void UART0_init(void) {
  UART_Init(UART0_PERIPHERAL, &UART0_config, UART0_CLOCK_SOURCE);
  UART_EnableInterrupts(UART0_PERIPHERAL, kUART_TransmissionCompleteInterruptEnable | kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
  /* Enable interrupt UART0_RX_TX_IRQn request in the NVIC. */
  EnableIRQ(UART0_SERIAL_RX_TX_IRQN);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  PIT_init();
  DAC0_init();
  ADC1_init();
  UART0_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
