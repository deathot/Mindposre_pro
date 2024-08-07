@echo off
title Transform
echo "Begin activate model-adapter-tool."
CALL conda init cmd.exe
CALL conda activate model-adapter-tool
CALL cd ..\..\..\..\models_adaption\segmentation\unetplusplus\labelme2segmentic
CALL python labelme2segmentic.py
CALL conda deactivate