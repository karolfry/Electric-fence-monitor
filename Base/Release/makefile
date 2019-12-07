################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include 1Wire/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
RC_AC_V1.0_Reciver.lss \

FLASH_IMAGE += \
RC_AC_V1.0_Reciver.hex \

EEPROM_IMAGE += \
RC_AC_V1.0_Reciver.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: RC_AC_V1.0_Reciver.elf secondary-outputs

# Tool invocations
RC_AC_V1.0_Reciver.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,RC_AC_V1.0_Reciver.map -mmcu=atmega8 -o "RC_AC_V1.0_Reciver.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

RC_AC_V1.0_Reciver.lss: RC_AC_V1.0_Reciver.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S RC_AC_V1.0_Reciver.elf  >"RC_AC_V1.0_Reciver.lss"
	@echo 'Finished building: $@'
	@echo ' '

RC_AC_V1.0_Reciver.hex: RC_AC_V1.0_Reciver.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex RC_AC_V1.0_Reciver.elf  "RC_AC_V1.0_Reciver.hex"
	@echo 'Finished building: $@'
	@echo ' '

RC_AC_V1.0_Reciver.eep: RC_AC_V1.0_Reciver.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex RC_AC_V1.0_Reciver.elf  "RC_AC_V1.0_Reciver.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: RC_AC_V1.0_Reciver.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega8 RC_AC_V1.0_Reciver.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) RC_AC_V1.0_Reciver.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents

-include ../makefile.targets
