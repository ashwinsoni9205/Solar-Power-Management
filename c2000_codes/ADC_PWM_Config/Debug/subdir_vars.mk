################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../2837x_FLASH_lnk_cpu1.cmd \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/headers/cmd/F2837xD_Headers_nonBIOS_cpu1.cmd 

ASM_SRCS += \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_CodeStartBranch.asm \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_usDelay.asm 

C_SRCS += \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_Adc.c \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_CpuTimers.c \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_DefaultISR.c \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_EPwm.c \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/headers/source/F2837xD_GlobalVariableDefs.c \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_Gpio.c \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_Ipc.c \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_PieCtrl.c \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_PieVect.c \
C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_SysCtrl.c \
../main.c 

C_DEPS += \
./F2837xD_Adc.d \
./F2837xD_CpuTimers.d \
./F2837xD_DefaultISR.d \
./F2837xD_EPwm.d \
./F2837xD_GlobalVariableDefs.d \
./F2837xD_Gpio.d \
./F2837xD_Ipc.d \
./F2837xD_PieCtrl.d \
./F2837xD_PieVect.d \
./F2837xD_SysCtrl.d \
./main.d 

OBJS += \
./F2837xD_Adc.obj \
./F2837xD_CodeStartBranch.obj \
./F2837xD_CpuTimers.obj \
./F2837xD_DefaultISR.obj \
./F2837xD_EPwm.obj \
./F2837xD_GlobalVariableDefs.obj \
./F2837xD_Gpio.obj \
./F2837xD_Ipc.obj \
./F2837xD_PieCtrl.obj \
./F2837xD_PieVect.obj \
./F2837xD_SysCtrl.obj \
./F2837xD_usDelay.obj \
./main.obj 

ASM_DEPS += \
./F2837xD_CodeStartBranch.d \
./F2837xD_usDelay.d 

OBJS__QUOTED += \
"F2837xD_Adc.obj" \
"F2837xD_CodeStartBranch.obj" \
"F2837xD_CpuTimers.obj" \
"F2837xD_DefaultISR.obj" \
"F2837xD_EPwm.obj" \
"F2837xD_GlobalVariableDefs.obj" \
"F2837xD_Gpio.obj" \
"F2837xD_Ipc.obj" \
"F2837xD_PieCtrl.obj" \
"F2837xD_PieVect.obj" \
"F2837xD_SysCtrl.obj" \
"F2837xD_usDelay.obj" \
"main.obj" 

C_DEPS__QUOTED += \
"F2837xD_Adc.d" \
"F2837xD_CpuTimers.d" \
"F2837xD_DefaultISR.d" \
"F2837xD_EPwm.d" \
"F2837xD_GlobalVariableDefs.d" \
"F2837xD_Gpio.d" \
"F2837xD_Ipc.d" \
"F2837xD_PieCtrl.d" \
"F2837xD_PieVect.d" \
"F2837xD_SysCtrl.d" \
"main.d" 

ASM_DEPS__QUOTED += \
"F2837xD_CodeStartBranch.d" \
"F2837xD_usDelay.d" 

C_SRCS__QUOTED += \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_Adc.c" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_CpuTimers.c" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_DefaultISR.c" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_EPwm.c" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/headers/source/F2837xD_GlobalVariableDefs.c" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_Gpio.c" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_Ipc.c" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_PieCtrl.c" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_PieVect.c" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_SysCtrl.c" \
"../main.c" 

ASM_SRCS__QUOTED += \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_CodeStartBranch.asm" \
"C:/ti/c2000/C2000Ware_5_05_00_00/device_support/f2837xd/common/source/F2837xD_usDelay.asm" 


