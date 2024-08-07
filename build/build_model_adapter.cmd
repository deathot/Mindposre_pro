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

echo "start build labelme"
if exist labelme (
echo "Delete labelme dir!"
rd /S /Q labelme
)

cd C:\ProgramData\anaconda3\envs\model-adapter-tool\Scripts
call activate.bat
cd %DEVKIT_BUILD_DIR%

CALL pyrcc5 ..\src\labelme\labelme\uicommponet\image.qrc -o ..\src\labelme\labelme\uicommponet\image_rc.py
robocopy  ..\src\labelme labelme\ /S
cd labelme
if exist dist (
echo "Delete build dir!"
rd /S /Q dist
)

CALL activate model-adapter-tool
pyinstaller labelme.spec
if %errorlevel% == 0 (
echo "buiud labelme succeeded!"
) else (
echo "buiud labelme failed!"
exit -1
)

robocopy .\ "dist\Ascend AI Devkit Model Adapter" install.nsi
robocopy .\ "dist\Ascend AI Devkit Model Adapter" update.nsi
cd "dist\Ascend AI Devkit Model Adapter"
makensis install.nsi
cd ..\..\
move "dist\Ascend AI Devkit Model Adapter\Ascend-devkit-model-adapter_*_win-x86_64.exe" ..\output\

cd "dist\Ascend AI Devkit Model Adapter"
makensis update.nsi
cd ..\..\
move "dist\Ascend AI Devkit Model Adapter\Ascend-devkit-model-adapter_*_win-x86_64_update.exe" ..\output\

cd ..
rd /S /Q labelme
echo "end build labelme"

echo "build devkit succeeded."
@echo on"
