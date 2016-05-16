################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.c \
../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.c 

OBJS += \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.o \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.o 

C_DEPS += \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_f32.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_cos_q31.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_f32.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sin_q31.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q15.d \
./Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/arm_sqrt_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/%.o: ../Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -I"D:\Keil_v5\ARM\ARMCC\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


