################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/option/cc932.c \
../Middlewares/Third_Party/FatFs/src/option/cc936.c \
../Middlewares/Third_Party/FatFs/src/option/cc949.c \
../Middlewares/Third_Party/FatFs/src/option/cc950.c \
../Middlewares/Third_Party/FatFs/src/option/ccsbcs.c \
../Middlewares/Third_Party/FatFs/src/option/syscall.c \
../Middlewares/Third_Party/FatFs/src/option/unicode.c 

OBJS += \
./Middlewares/Third_Party/FatFs/src/option/cc932.o \
./Middlewares/Third_Party/FatFs/src/option/cc936.o \
./Middlewares/Third_Party/FatFs/src/option/cc949.o \
./Middlewares/Third_Party/FatFs/src/option/cc950.o \
./Middlewares/Third_Party/FatFs/src/option/ccsbcs.o \
./Middlewares/Third_Party/FatFs/src/option/syscall.o \
./Middlewares/Third_Party/FatFs/src/option/unicode.o 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/option/cc932.d \
./Middlewares/Third_Party/FatFs/src/option/cc936.d \
./Middlewares/Third_Party/FatFs/src/option/cc949.d \
./Middlewares/Third_Party/FatFs/src/option/cc950.d \
./Middlewares/Third_Party/FatFs/src/option/ccsbcs.d \
./Middlewares/Third_Party/FatFs/src/option/syscall.d \
./Middlewares/Third_Party/FatFs/src/option/unicode.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/option/%.o: ../Middlewares/Third_Party/FatFs/src/option/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -I"D:\Keil_v5\ARM\ARMCC\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


