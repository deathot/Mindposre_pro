@echo off
title Transform
echo "Begin activate model-adapter-tool."
CALL conda init cmd.exe
CALL conda activate model-adapter-tool
CALL cd ..\..\..\..\models_adaption\detection\yolov5_deepsort\yolov5_v7\train\
CALL python labelme2yolov5.py
CALL conda deactivate