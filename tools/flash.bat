@ECHO OFF
SETLOCAL ENABLEDELAYEDEXPANSION 

openocd -f ../tools/stlink.cfg -f ../tools/stm32f3x.cfg -c "program ../build/quadcopter-flightcontroller.elf verify reset exit"
