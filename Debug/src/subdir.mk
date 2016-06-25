################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CommHelper.cpp \
../src/EstRXY.cpp \
../src/MapDealer.cpp \
../src/ParameterAdjust.cpp \
../src/TCPClient.cpp \
../src/astar_algoirthm.cpp \
../src/main.cpp \
../src/unit.cpp 

C_SRCS += \
../src/inifile.c 

OBJS += \
./src/CommHelper.o \
./src/EstRXY.o \
./src/MapDealer.o \
./src/ParameterAdjust.o \
./src/TCPClient.o \
./src/astar_algoirthm.o \
./src/inifile.o \
./src/main.o \
./src/unit.o 

C_DEPS += \
./src/inifile.d 

CPP_DEPS += \
./src/CommHelper.d \
./src/EstRXY.d \
./src/MapDealer.d \
./src/ParameterAdjust.d \
./src/TCPClient.d \
./src/astar_algoirthm.d \
./src/main.d \
./src/unit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


