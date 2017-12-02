################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: UltrasonicCorrelationMeasure.elf

dependents:
	-cd G:/Documents\ and\ stuff/Project\ Avalanche/Workspace\ 4/nucleo-l152re_stdperiph_lib/Debug && $(MAKE) all

# Tool invocations
UltrasonicCorrelationMeasure.elf: $(OBJS) $(USER_OBJS) G:/Documents\ and\ stuff/Project\ Avalanche/Workspace\ 4/nucleo-l152re_stdperiph_lib/Debug/libnucleo-l152re_stdperiph_lib.a G:\Documents\ and\ stuff\Project\ Avalanche\Workspace\ 4\UltrasonicCorrelationMeasure\LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -L"G:\Documents and stuff\Project Avalanche\Workspace 4\nucleo-l152re_stdperiph_lib\Debug" -T"G:\Documents and stuff\Project Avalanche\Workspace 4\UltrasonicCorrelationMeasure\LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "UltrasonicCorrelationMeasure.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "UltrasonicCorrelationMeasure.elf" "UltrasonicCorrelationMeasure.bin"
	arm-none-eabi-size "UltrasonicCorrelationMeasure.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
G:/Documents\ and\ stuff/Project\ Avalanche/Workspace\ 4/nucleo-l152re_stdperiph_lib/Debug/libnucleo-l152re_stdperiph_lib.a:

-include ../makefile.targets
