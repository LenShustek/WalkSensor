@echo off
prompt $S
echo.
echo *** press any key to display current fuses
pause >nul
echo on
avrdude -c stk500v2 -P COM7 -p m328p 
@echo off
echo *** press any key to set the low fuse for 8 Mhz
pause >nul
echo on
avrdude -c stk500v2 -P COM7 -p m328p -U lfuse:w:0xfe:m
@echo off
echo *** press any key to exit
pause >nul
exit