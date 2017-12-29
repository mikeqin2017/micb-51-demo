################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/peripheral_driver/src/spi_0.c \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/peripheral_driver/src/uart_0.c \
C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/peripheral_driver/src/uart_1.c 

OBJS += \
./lib/efm8ub2/peripheralDrivers/src/spi_0.OBJ \
./lib/efm8ub2/peripheralDrivers/src/uart_0.OBJ \
./lib/efm8ub2/peripheralDrivers/src/uart_1.OBJ 


# Each subdirectory must supply rules for building sources it contributes
lib/efm8ub2/peripheralDrivers/src/spi_0.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/peripheral_driver/src/spi_0.c
	@echo 'Building file: $<'
	@echo 'Invoking: Keil 8051 Compiler'
	C51 "@$(patsubst %.OBJ,%.__i,$@)" || $(RC)
	@echo 'Finished building: $<'
	@echo ' '

lib/efm8ub2/peripheralDrivers/src/spi_0.OBJ: C:/Users/shuwen.ou/SimplicityStudio/v4_workspace/MICB-51/inc/config/efm8_config.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/inc/SI_EFM8UB2_Register_Enums.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/peripheral_driver/inc/spi_0.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/inc/SI_EFM8UB2_Defs.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/shared/si8051Base/si_toolchain.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/shared/si8051Base/stdint.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/shared/si8051Base/stdbool.h

lib/efm8ub2/peripheralDrivers/src/uart_0.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/peripheral_driver/src/uart_0.c
	@echo 'Building file: $<'
	@echo 'Invoking: Keil 8051 Compiler'
	C51 "@$(patsubst %.OBJ,%.__i,$@)" || $(RC)
	@echo 'Finished building: $<'
	@echo ' '

lib/efm8ub2/peripheralDrivers/src/uart_0.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/peripheral_driver/inc/uart_0.h C:/Users/shuwen.ou/SimplicityStudio/v4_workspace/MICB-51/inc/config/efm8_config.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/inc/SI_EFM8UB2_Register_Enums.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/inc/SI_EFM8UB2_Defs.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/shared/si8051Base/si_toolchain.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/shared/si8051Base/stdint.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/shared/si8051Base/stdbool.h

lib/efm8ub2/peripheralDrivers/src/uart_1.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/peripheral_driver/src/uart_1.c
	@echo 'Building file: $<'
	@echo 'Invoking: Keil 8051 Compiler'
	C51 "@$(patsubst %.OBJ,%.__i,$@)" || $(RC)
	@echo 'Finished building: $<'
	@echo ' '

lib/efm8ub2/peripheralDrivers/src/uart_1.OBJ: C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/peripheral_driver/inc/uart_1.h C:/Users/shuwen.ou/SimplicityStudio/v4_workspace/MICB-51/inc/config/efm8_config.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/inc/SI_EFM8UB2_Register_Enums.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/EFM8UB2/inc/SI_EFM8UB2_Defs.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/shared/si8051Base/si_toolchain.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/shared/si8051Base/stdint.h C:/SiliconLabs/SimplicityStudio/v4/developer/sdks/8051/v4.1.1/Device/shared/si8051Base/stdbool.h


