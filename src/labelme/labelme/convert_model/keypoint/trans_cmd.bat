@echo off
title Transform
echo "Begin activate model-adapter-tool."
CALL conda init cmd.exe
CALL conda activate model-adapter-tool
CALL cd ..\..\..\..\models_adaption\keypoint\AlphaPose_ORB\train
CALL python .\scripts\labelme2coco_keypoints.py
CALL conda deactivate