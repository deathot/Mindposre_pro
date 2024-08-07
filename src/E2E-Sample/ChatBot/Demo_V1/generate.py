#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import os
import time

from filelock import FileLock
from ais_bench.infer.interface import InferSession

from utils import preprocess, postprocess, content_generate, generate_onnx_input


# 创建临时文件夹
temp_path = os.path.join(os.getcwd(), 'temp')
os.makedirs(temp_path, exist_ok=True)

input_filepath = os.path.join(temp_path, 'input.txt')
output_filepath = os.path.join(temp_path, 'output.json')
lock_filepath = os.path.join(temp_path, 'lock.txt')

# 创建文件锁
lock = FileLock(lock_filepath, timeout=5)


# 检查是否有待处理的输入文件
def check_input():
    with lock:
        try:
            with open(input_filepath, 'r', encoding='utf-8') as f:
                text = f.read()
            # 删除或清空文件内容
            os.remove(input_filepath)
            return text
        except FileNotFoundError:
            return None


# 将输出结果写入文件
def write_output_json(data):
    with lock:
        while os.path.exists(output_filepath):
            time.sleep(0.1)
        with open(output_filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)


# 预先加载om模型确保GE能申请到足够的内存
encoder = InferSession(0, './models/encoder.om')

if __name__ == '__main__':
    # 确保模型加载完成后再导入其他包
    import numpy as np
    import onnxruntime
    import torch
    from transformers import T5Tokenizer
    from transformers.generation import LogitsProcessorList, NoRepeatNGramLogitsProcessor, TemperatureLogitsWarper, \
        TopKLogitsWarper, StoppingCriteriaList, MaxLengthCriteria

    print('[INFO]The encoder has been initialized. Initializing the first decoder in progress.')
    first_decoder = onnxruntime.InferenceSession('./models/decoder_first_sim_quant.onnx')
    print('[INFO]The first decoder has been initialized. Initializing the second decoder in progress.')
    decoder_iter = onnxruntime.InferenceSession('./models/decoder_iter_sim_quant.onnx')
    tokenizer = T5Tokenizer.from_pretrained("./tokenizer_file")
    dummy_decoder_input_ids = np.array([[0]], dtype=np.int64)
    logits_processor = LogitsProcessorList([NoRepeatNGramLogitsProcessor(3)])
    logits_warper = LogitsProcessorList(
        [TemperatureLogitsWarper(0.7), TopKLogitsWarper(filter_value=float('-inf'), top_k=50)])
    stopping_criteria = StoppingCriteriaList([MaxLengthCriteria(512)])
    eos_token_id = [1]
    record = []
    print('[INFO]init finished')


    def generate_ss_mode(input_text, output=False):
        if input_text == 'clear' and output:
            print('record clear')
            record.clear()
            data = {
                "code": 200,
                "data": {
                    "isEnd": False,
                    "message": '聊天记录已清空'
                }
            }
            write_output_json(data)
            return

        # 生成附带上下文的模型输入
        content = content_generate(record, input_text)

        # 对输入文本进行预处理，生成token和attention_mask
        inputs = tokenizer(text=[preprocess(content)], truncation=True, padding='max_length', max_length=768,
                           return_tensors="np")

        encoder_input_ids = inputs['input_ids']
        attention_mask = inputs['attention_mask']

        # 使用encoder模型生成encoder_hidden_states
        encoder_hidden_states = encoder.infer([encoder_input_ids, attention_mask])[0]

        print('autoregression start')
        first_loop = True
        decoder_input_ids = dummy_decoder_input_ids

        input_ids = torch.tensor(dummy_decoder_input_ids)
        unfinished_sequences = torch.tensor([1])
        while True:
            if first_loop:
                outputs = first_decoder.run(None, {'decoder_input_ids': decoder_input_ids,
                                                   'hidden_states': encoder_hidden_states,
                                                   'attention_mask': attention_mask})
                first_loop = False
            else:
                onnx_input = generate_onnx_input(decoder_input_ids, attention_mask, past_key_values)
                outputs = decoder_iter.run(None, onnx_input)
            logits = torch.tensor(outputs[0])
            past_key_values = outputs[1:]
            next_token_logits = logits[:, -1, :]

            next_token_scores = logits_processor(input_ids, next_token_logits)
            next_token_scores = logits_warper(input_ids, next_token_scores)

            probs = torch.nn.functional.softmax(next_token_scores, dim=-1)
            next_tokens = torch.multinomial(probs, num_samples=1).squeeze(1)
            message = postprocess(tokenizer.batch_decode(next_tokens, skip_special_tokens=True)[0])
            if message == ' ' or message == '':
                message = '&nbsp'
            elif message == '\n':
                message = '<br />'

            data = {
                "code": 200,
                "data": {
                    "isEnd": False,
                    "message": message
                }
            }

            input_ids = torch.cat([input_ids, next_tokens[:, None]], dim=-1)
            decoder_input_ids = input_ids[:, -1:].numpy()

            # 判断是否结束
            unfinished_sequences = unfinished_sequences.mul((sum(next_tokens != i for i in eos_token_id)).long())
            if unfinished_sequences.max() == 0 or stopping_criteria(input_ids, None):
                data['data']['isEnd'] = True
                if output:
                    write_output_json(data)

                break
            else:
                if output:
                    write_output_json(data)

        out_text = tokenizer.batch_decode(input_ids, skip_special_tokens=True)[0]
        out_text = postprocess(out_text)

        record.append([input_text, out_text])
        if tokenizer(text=[preprocess(content_generate(record, ''))], truncation=True, padding=True,
                     max_length=768,
                     return_tensors="np")['attention_mask'].shape[1] > 256:
            print('record clear')
            record.clear()


    print('[INFO]ChatBot Ready')
    while True:
        msg = check_input()
        if msg is None:
            continue
        print('input text: ', msg)
        generate_ss_mode(msg, True)
