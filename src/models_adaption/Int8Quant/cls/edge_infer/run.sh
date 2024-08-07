#!/bin/bash

# set acl env
. /usr/local/Ascend/ascend-toolkit/set_env.sh
export LD_PRELOAD=$LD_PRELOAD:/usr/local/python3.9.7/lib/python3.9/site-packages/torch/lib/libgomp-d22c30c5.so.1

echo "set env successfully!!"
echo "start exec atc"

# start infer
python3.9 infer.py --model mobilenetv3_100_bs1.om  --label_path class_indices.json --output_path ./
