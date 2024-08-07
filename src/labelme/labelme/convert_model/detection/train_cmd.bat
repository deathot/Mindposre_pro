@echo off
title ModelTrain
echo "Begin activate model-adapter-tool."
CALL conda init cmd.exe
CALL conda activate model-adapter-tool
CALL cd ..\..\..\..\models_adaption\detection\yolov5_deepsort\yolov5_v7\train
CALL python train.py
CALL conda deactivate