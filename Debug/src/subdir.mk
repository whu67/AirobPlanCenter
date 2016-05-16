################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CommHelper.cpp \
../src/TCPClient.cpp \
../src/main.cpp \
../src/unit.cpp 

OBJS += \
./src/CommHelper.o \
./src/TCPClient.o \
./src/main.o \
./src/unit.o 

CPP_DEPS += \
./src/CommHelper.d \
./src/TCPClient.d \
./src/main.d \
./src/unit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


