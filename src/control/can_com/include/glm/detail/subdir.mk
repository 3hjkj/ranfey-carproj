################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../linuxOdrManager/include/glm/detail/dummy.cpp \
../linuxOdrManager/include/glm/detail/glm.cpp 

OBJS += \
./linuxOdrManager/include/glm/detail/dummy.o \
./linuxOdrManager/include/glm/detail/glm.o 

CPP_DEPS += \
./linuxOdrManager/include/glm/detail/dummy.d \
./linuxOdrManager/include/glm/detail/glm.d 


# Each subdirectory must supply rules for building sources it contributes
linuxOdrManager/include/glm/detail/%.o: ../linuxOdrManager/include/glm/detail/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -fPIC -std=c++0x -I"/home/chtgeo/eclipse-workspace/OdrManager/ThirdPart/glm" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


