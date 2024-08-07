@echo off
title ConvertModel
echo "Begin activate model-adapter-tool."
CALL conda init cmd.exe
CALL conda activate model-adapter-tool
CALL cd ..\..\..\..\models_adaption\segmentation\unetplusplus
CALL python model_convert.py
CALL conda deactivate