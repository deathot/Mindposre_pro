@echo off
title Create conda env model-adapter-tool
echo "Begin create conda env model-adapter-tool."
CALL conda init cmd.exe
echo "create nev_name=model-adapter-tool  python=3.8.13"
CALL conda create -y -n model-adapter-tool python=3.8.13
CALL conda activate model-adapter-tool
CALL cd src/labelme/envs
pip install -r requirements.txt -U -i http://pypi.mirrors.ustc.edu.cn/simple/ --trusted-host pypi.mirrors.ustc.edu.cn
CALL conda deactivate