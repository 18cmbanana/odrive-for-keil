@echo off
REM ======================================================
rem Sets path for cygwin, toolchain

set ROOT_DRIVE=D:
set APPSROOT=%ROOT_DRIVE%\cygwin\opt\ecos
set UTILROOT=%ROOT_DRIVE%\cygwin

set cygwin_=%UTILROOT%\bin
set toolchain_=%APPSROOT%\gnutools\arm-none-eabi\bin\
rem
rem Path for cygwin, toolchain
rem
set path=.;%cygwin_%;%toolchain_%;%path%
REM ======================================================
chdir %~dp0
echo Congratulations! Work is done.
pause
mintty.exe -i /Cygwin-Terminal.ico
bash
exit

