@echo off
title ModelTrain
echo "Begin activate model-adapter-tool."

CALL conda init cmd.exe
CALL conda activate model-adapter-tool
CALL cd ..\..\..\..\models_adaption\segmentation\unetplusplus\train
CALL python train.py
CALL conda deactivate

