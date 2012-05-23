# MPLAB IDE generated this makefile for use with GNU make.
# Project: SACT_FW_Grip.mcp
# Date: Tue May 15 19:00:32 2012

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

./build/SACT_FW_Grip.hex : ./build/SACT_FW_Grip.cof
	$(HX) ".\build\SACT_FW_Grip.cof"

./build/SACT_FW_Grip.cof : build/main.o build/globals.o build/lib_crc.o build/Controls.o build/Trajectories.o build/Comms.o build/SACT_Protocol.o build/PID.o build/ADC.o build/PWM.o build/QEI.o build/Timers.o build/traps.o build/FxSqrtAbs.o build/Q16wrappers.o build/EEPROM_params.o
	$(CC) -mcpu=30F6015 "build\main.o" "build\globals.o" "build\lib_crc.o" "build\Controls.o" "build\Trajectories.o" "build\Comms.o" "build\SACT_Protocol.o" "build\PID.o" "build\ADC.o" "build\PWM.o" "build\QEI.o" "build\Timers.o" "build\traps.o" "build\FxSqrtAbs.o" "build\Q16wrappers.o" "build\EEPROM_params.o" -o".\build\SACT_FW_Grip.cof" -Wl,--script="gld\linkerscript.gld",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,-Map=".\build\SACT_FW_Grip.map",--report-mem

build/main.o : include/EEPROM_params.h include/SACT_protocol.h include/Controls.h include/Timers.h include/QEI.h include/ADC.h include/PWM.h include/sys_hw.h include/Comms.h include/generic_defs.h include/Trajectories.h include/PID.h include/generic_defs.h include/extern_globals.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/main.c
	$(CC) -mcpu=30F6015 -x c -c "src\main.c" -o".\build\main.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/globals.o : include/Trajectories.h include/PID.h include/generic_defs.h include/extern_globals.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/globals.c
	$(CC) -mcpu=30F6015 -x c -c "src\globals.c" -o".\build\globals.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/lib_crc.o : lib_crc/lib_crc.h lib_crc/lib_crc.c
	$(CC) -mcpu=30F6015 -x c -c "lib_crc\lib_crc.c" -o".\build\lib_crc.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/Controls.o : ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/generic/h/libq.h include/generic_defs.h lib_math/my_fractmath.h include/sys_hw.h include/PWM.h include/PID.h include/Trajectories.h include/Timers.h include/Controls.h include/Trajectories.h include/PID.h include/generic_defs.h include/extern_globals.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/Controls.c
	$(CC) -mcpu=30F6015 -x c -c "src\Controls.c" -o".\build\Controls.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/Trajectories.o : ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/generic/h/libq.h include/generic_defs.h lib_math/my_fractmath.h include/generic_defs.h include/Trajectories.h src/Trajectories.c
	$(CC) -mcpu=30F6015 -x c -c "src\Trajectories.c" -o".\build\Trajectories.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/Comms.o : include/sys_hw.h include/Comms.h include/generic_defs.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/Comms.c
	$(CC) -mcpu=30F6015 -x c -c "src\Comms.c" -o".\build\Comms.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/SACT_Protocol.o : ../../../../../../../programmi/microchip/mplabc30/v3.30c/include/limits.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/include/stdlib.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/include/stddef.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/include/string.h lib_crc/lib_crc.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/generic/h/libq.h include/generic_defs.h lib_math/my_fractmath.h include/SACT_Protocol.h include/sys_hw.h include/Comms.h include/Trajectories.h include/PID.h include/generic_defs.h include/extern_globals.h include/generic_defs.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/SACT_Protocol.c
	$(CC) -mcpu=30F6015 -x c -c "src\SACT_Protocol.c" -o".\build\SACT_Protocol.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/PID.o : include/generic_defs.h include/PID.h src/PID.c
	$(CC) -mcpu=30F6015 -x c -c "src\PID.c" -o".\build\PID.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/ADC.o : include/Controls.h include/sys_hw.h include/PWM.h include/ADC.h include/generic_defs.h include/Trajectories.h include/PID.h include/generic_defs.h include/extern_globals.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/ADC.c
	$(CC) -mcpu=30F6015 -x c -c "src\ADC.c" -o".\build\ADC.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/PWM.o : include/sys_hw.h include/PWM.h include/Trajectories.h include/PID.h include/generic_defs.h include/extern_globals.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/PWM.c
	$(CC) -mcpu=30F6015 -x c -c "src\PWM.c" -o".\build\PWM.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/QEI.o : include/QEI.h include/Trajectories.h include/PID.h include/generic_defs.h include/extern_globals.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/QEI.c
	$(CC) -mcpu=30F6015 -x c -c "src\QEI.c" -o".\build\QEI.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/Timers.o : include/Controls.h include/Timers.h include/Trajectories.h include/PID.h include/generic_defs.h include/extern_globals.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/Timers.c
	$(CC) -mcpu=30F6015 -x c -c "src\Timers.c" -o".\build\Timers.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/traps.o : ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/traps.c
	$(CC) -mcpu=30F6015 -x c -c "src\traps.c" -o".\build\traps.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/FxSqrtAbs.o : ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/generic/h/libq.h include/generic_defs.h lib_math/my_fractmath.h lib_math/FxSqrtAbs.c
	$(CC) -mcpu=30F6015 -x c -c "lib_math\FxSqrtAbs.c" -o".\build\FxSqrtAbs.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/Q16wrappers.o : ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/generic/h/libq.h include/generic_defs.h lib_math/my_fractmath.h lib_math/Q16wrappers.c
	$(CC) -mcpu=30F6015 -x c -c "lib_math\Q16wrappers.c" -o".\build\Q16wrappers.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

build/EEPROM_params.o : ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/generic/h/libpic30.h include/SACT_protocol.h include/EEPROM_params.h include/Trajectories.h include/PID.h include/generic_defs.h include/extern_globals.h include/generic_defs.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30F6015.h ../../../../../../../programmi/microchip/mplabc30/v3.30c/support/dsPIC30F/h/p30fxxxx.h include/sys_hw.h src/EEPROM_params.c
	$(CC) -mcpu=30F6015 -x c -c "src\EEPROM_params.c" -o".\build\EEPROM_params.o" -I".\include" -I".\lib_math" -I".\lib_crc" -D__DEBUG -g -Wall

clean : 
	$(RM) "build\main.o" "build\globals.o" "build\lib_crc.o" "build\Controls.o" "build\Trajectories.o" "build\Comms.o" "build\SACT_Protocol.o" "build\PID.o" "build\ADC.o" "build\PWM.o" "build\QEI.o" "build\Timers.o" "build\traps.o" "build\FxSqrtAbs.o" "build\Q16wrappers.o" "build\EEPROM_params.o" ".\build\SACT_FW_Grip.cof" ".\build\SACT_FW_Grip.hex"

