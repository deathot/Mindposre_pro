@echo off

rem
rem For scripting: call ascend-imager.exe and wait until it finished before continuing
rem This is necessary because it is compiled as GUI application, and Windows
rem normalling does not wait until those exit
rem

start /WAIT ascend-ai-devkit-imager.exe --cli %*
