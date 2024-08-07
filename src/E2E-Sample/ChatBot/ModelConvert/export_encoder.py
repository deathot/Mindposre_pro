import torch
from transformers import T5Tokenizer, T5ForConditionalGeneration
from argparse import ArgumentParser

def read_args():
    args = ArgumentParser()
    args.add_argument('--version', type=str, default='v1',choices=['v1','v2'])
    return args.parse_args()

def preprocess(t):
    return t.replace("\n", "\\n").replace("\t", "\\t")


if __name__ == '__main__':
    opts = read_args()
    model = T5ForConditionalGeneration.from_pretrained(f"ClueAI/ChatYuan-large-{opts.version}")
    tokenizer = T5Tokenizer.from_pretrained(f"ClueAI/ChatYuan-large-{opts.version}")
    encoder_part = model.encoder
    input_text = "帮我写一个快速排序的Python代码"
    input_text = "用户：" + input_text + "\n小元："
    inputs = tokenizer(text=[preprocess(input_text)], truncation=True, padding='max_length', max_length=768,
                       return_tensors="pt")
    print('export encoder start')
    # Export the model
    input_names = ['input_ids', 'attention_mask']
    output_names = ['last_hidden_state']
    torch.onnx.export(encoder_part,  # model being run
                      (inputs['input_ids'], inputs['attention_mask']),  # model input (or a tuple for multiple inputs)
                      "encoder.onnx",  # where to save the model (can be a file or file-like object)
                      opset_version=13,  # the ONNX version to export the model to
                      input_names=input_names,  # the model's input names
                      output_names=output_names,  # the model's output names
                      )
