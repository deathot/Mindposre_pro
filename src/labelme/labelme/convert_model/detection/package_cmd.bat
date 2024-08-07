@echo off
title ConvertModel
echo "Begin activate model-adapter-tool."
CALL conda init cmd.exe
CALL conda activate model-adapter-tool
CALL cd ..\..\..\..\models_adaption\detection\yolov5_deepsort\yolov5_v7\train
CALL python package.py
CALL conda deactivate