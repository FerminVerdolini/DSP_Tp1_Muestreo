################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/Tp1_Muestreo.c \
../source/semihost_hardfault.c 

C_DEPS += \
./source/Tp1_Muestreo.d \
./source/semihost_hardfault.d 

OBJS += \
./source/Tp1_Muestreo.o \
./source/semihost_hardfault.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MK64FN1M0VLL12 -DCPU_MK64FN1M0VLL12_cm4 -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -DSERIAL_PORT_TYPE_UART=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\drivers" -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\device" -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\CMSIS" -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\utilities" -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\component\serial_manager" -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\component\uart" -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\component\lists" -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\board" -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\CMSIS\DSP\Include" -I"C:\Users\Usuario\Documents\GitHub\DSP_Tp1_Muestreo\Tp1_Muestreo\source" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/Tp1_Muestreo.d ./source/Tp1_Muestreo.o ./source/semihost_hardfault.d ./source/semihost_hardfault.o

.PHONY: clean-source

