#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time

import numpy as np
import onnxruntime
import torch
import torch.nn as nn
from ais_bench.infer.interface import InferSession
from tqdm import tqdm
from transformers import T5Tokenizer
from transformers.generation import LogitsProcessorList, NoRepeatNGramLogitsProcessor, TemperatureLogitsWarper, \
    TopKLogitsWarper, StoppingCriteriaList, MaxLengthCriteria

from utils import preprocess, postprocess, generate_onnx_input


class T5Model:
    def __init__(self):
        print('[INFO]start init encoder. It will cost a few minutes')
        self.encoder = InferSession(0, './models/encoder.om')
        time.sleep(10)
        print('[INFO]The encoder has been initialized. Initializing the first decoder in progress.')
        self.first_decoder = onnxruntime.InferenceSession('./models/decoder_first_sim_quant.onnx')
        print('[INFO]The first decoder has been initialized. Initializing the second decoder in progress.')
        self.decoder_iter = onnxruntime.InferenceSession('./models/decoder_iter_sim_quant.onnx')
        print('[INFO]The second decoder has been initialized.')

        self.tokenizer = T5Tokenizer.from_pretrained("./tokenizer_file")
        self.dummy_decoder_input_ids = np.array([[0]], dtype=np.int64)
        self.logits_processor = LogitsProcessorList([NoRepeatNGramLogitsProcessor(3)])
        self.logits_warper = LogitsProcessorList(
            [TemperatureLogitsWarper(0.7), TopKLogitsWarper(filter_value=float('-inf'), top_k=50)])
        self.stopping_criteria = StoppingCriteriaList([MaxLengthCriteria(512)])
        self.eos_token_id = [1]
        print('[INFO]init finished. The next step is warm up')

        print('[INFO]warm up start, it will cost a few minutes')
        self.warmup()

    def warmup(self):
        for _ in tqdm(range(3)):
            self.generate_ss_mode('test', '你好', None)
        print('[INFO]warm up finished')

    def generate_ss_mode(self, web_id, input_text, out_mq):
        """
        使用流式模式生成对话
        @param web_id: 前端页面的id
        @param input_text: 已经处理好上下文的输入文本
        @param out_mq: 输出的消息队列
        """
        print(f'[INFO]generate start,{web_id},{input_text}')
        pre_text = ''
        inputs = self.tokenizer(text=[preprocess(input_text)], truncation=True, padding='max_length', max_length=768,
                                return_tensors="np")

        encoder_input_ids = inputs['input_ids'].astype(np.int64)
        attention_mask = inputs['attention_mask'].astype(np.int64)
        encoder_hidden_states = self.encoder.infer([encoder_input_ids, attention_mask])[0]

        print('autoregression start')
        first_loop = True
        decoder_input_ids = self.dummy_decoder_input_ids

        input_ids = torch.tensor(self.dummy_decoder_input_ids)
        unfinished_sequences = torch.tensor([1])
        while True:
            if first_loop:
                outputs = self.first_decoder.run(None, {'decoder_input_ids': decoder_input_ids,
                                                        'hidden_states': encoder_hidden_states,
                                                        'attention_mask': attention_mask})
                first_loop = False
            else:
                onnx_input = generate_onnx_input(decoder_input_ids, attention_mask, past_key_values)
                outputs = self.decoder_iter.run(None, onnx_input)
            logits = torch.tensor(outputs[0])
            past_key_values = outputs[1:]
            next_token_logits = logits[:, -1, :]

            next_token_scores = self.logits_processor(input_ids, next_token_logits)
            next_token_scores = self.logits_warper(input_ids, next_token_scores)

            probs = nn.functional.softmax(next_token_scores, dim=-1)
            next_tokens = torch.multinomial(probs, num_samples=1).squeeze(1)
            message = postprocess(self.tokenizer.batch_decode(next_tokens, skip_special_tokens=True)[0])

            if message == '%' or message == '20%' or message == '%20':
                pre_text += message
                message = None
            else:
                if pre_text:
                    message = pre_text + message
                    pre_text = ''

            if message == '':
                message = '&nbsp'

            if message is None:
                message = ''
            elif message == ' ':
                message = '&nbsp'
            elif message == '\n':
                message = '<br />'
            elif len(message) > 1:
                message = message.replace('%20', '&nbsp&nbsp').replace('\n', '<br />')

            data = [web_id, False, message, '']

            input_ids = torch.cat([input_ids, next_tokens[:, None]], dim=-1)
            decoder_input_ids = input_ids[:, -1:].numpy()

            unfinished_sequences = unfinished_sequences.mul((sum(next_tokens != i for i in self.eos_token_id)).long())

            if unfinished_sequences.max() == 0 or self.stopping_criteria(input_ids, None):
                data[1] = True
                out_text = self.tokenizer.batch_decode(input_ids, skip_special_tokens=True)[0]
                out_text = postprocess(out_text)
                data[3] = out_text
                if out_mq is not None:
                    out_mq.put(data)

                break
            else:
                if out_mq is not None:
                    out_mq.put(data)
