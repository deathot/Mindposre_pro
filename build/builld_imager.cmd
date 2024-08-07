set DEVKIT_BUILD_DIR=%~dp0
C:\Windows\System32\cmd.exe /A /Q /K C:\Qt\5.15.2\mingw81_32\bin\qtenv2.bat <"
@echo off
echo "build dir:%DEVKIT_BUILD_DIR%"
cd %DEVKIT_BUILD_DIR%

echo "start build devkit."

if exist output (
echo "Delete output dir!"
rd /S /Q output
)
md output

echo "start build rpi-imager"
if exist rpi-imager (
echo "Delete rpi-imager dir!"
rd /S /Q rpi-imager
)
robocopy ../src/rpi-imager rpi-imager/ /S /NFL
cd rpi-imager/src
md build
cd build
@REM C:/Qt/Tools/CMake_64/bin/cmake -G "MinGW Makefiles" ..
C:/Qt/Tools/CMake_64/bin/cmake.exe -G "MinGW Makefiles" -D CMAKE_C_COMPILER=C:/Qt/Tools/mingw810_32/bin/gcc.exe -D CMAKE_CXX_COMPILER=C:/Qt/Tools/mingw810_32/bin/g++.exe -D CMAKE_MAKE_PROGRAM=C:/Qt/Tools/mingw810_32/bin/mingw32-make.exe -D CMAKE_RC_COMPILER=C:/Qt/Tools/mingw810_32/bin/windres.exe ..
C:/Qt/Tools/mingw810_32/bin/make.exe -j8
if %errorlevel% == 0 (
echo "buiud rpi-imager succeeded!"
) else (
echo "buiud rpi-imager failed!"
exit -1
)

makensis rpi-imager.nsi
if %errorlevel% == 0 (
echo "makensis rpi-imager.nsi succeeded!"
) else (
echo "makensis rpi-imager.nsi failed!"
exit -1
)

robocopy .\ ..\..\..\output\ Ascend-devkit-imager_*_win-x86_64.exe
if %errorlevel% == 1 (
echo "copy rpi-imager to output dir succeeded!"
) else (
echo "copy rpi-imager to output dir failed!"
exit -1
)

makensis update.nsi
if %errorlevel% == 0 (
echo "makensis update.nsi succeeded!"
) else (
echo "makensis update.nsi failed!"
exit -1
)

robocopy .\ ..\..\..\output\ Ascend-devkit-imager-update_*_win-x86_64.exe
if %errorlevel% == 1 (
echo "copy update to output dir succeeded!"
) else (
echo "copy update to output dir failed!"
exit -1
)

cd ..\..\..\
rd /S /Q rpi-imager
echo "end build rpi-imager"

echo "build devkit succeeded."
@echo on"