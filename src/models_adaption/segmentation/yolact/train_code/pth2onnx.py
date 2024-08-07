import torch
from yolact2 import Yolact

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

weight_path = "weights/yolact_base_transfer_6_20.pth"
onnx_model_path = "weights\yolact_base_transfer_6_20.onnx"

net = Yolact()
net.load_weights(weight_path)
net.eval()
net.to(device)

inputs = torch.randn(1,3,550,550).to(device)
print("convert net to ", onnx_model_path,"...")
torch.onnx.export(net,
                  (inputs,),
                  onnx_model_path,
                  verbose=False,
                #   input_names=["img"],
                #   output_names=["loc","conf","mask","proto"],
                  opset_version=11,
                  do_constant_folding=True

)
print("converted successed")