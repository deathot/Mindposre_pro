@echo off
title ModelTrain
echo "Begin activate model-adapter-tool."
CALL conda init cmd.exe
CALL conda activate model-adapter-tool
CALL cd ..\..\..\..\models_adaption\classification\mobilenetv3\train_code
CALL python train.py
CALL conda deactivate