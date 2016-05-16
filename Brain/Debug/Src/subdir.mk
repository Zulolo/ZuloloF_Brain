################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/ahrs.c \
../Src/crc.c \
../Src/dma.c \
../Src/fatfs.c \
../Src/freertos.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/iwdg.c \
../Src/main.c \
../Src/motor.c \
../Src/rf.c \
../Src/routine.c \
../Src/sdio.c \
../Src/spi.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_TIM.c \
../Src/stm32f4xx_it.c \
../Src/tim.c \
../Src/usart.c \
../Src/usb_device.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c \
../Src/user_diskio.c 

OBJS += \
./Src/adc.o \
./Src/ahrs.o \
./Src/crc.o \
./Src/dma.o \
./Src/fatfs.o \
./Src/freertos.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/iwdg.o \
./Src/main.o \
./Src/motor.o \
./Src/rf.o \
./Src/routine.o \
./Src/sdio.o \
./Src/spi.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_TIM.o \
./Src/stm32f4xx_it.o \
./Src/tim.o \
./Src/usart.o \
./Src/usb_device.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o \
./Src/user_diskio.o 

C_DEPS += \
./Src/adc.d \
./Src/ahrs.d \
./Src/crc.d \
./Src/dma.d \
./Src/fatfs.d \
./Src/freertos.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/iwdg.d \
./Src/main.d \
./Src/motor.d \
./Src/rf.d \
./Src/routine.d \
./Src/sdio.d \
./Src/spi.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_TIM.d \
./Src/stm32f4xx_it.d \
./Src/tim.d \
./Src/usart.d \
./Src/usb_device.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d \
./Src/user_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -I"D:\Keil_v5\ARM\ARMCC\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


