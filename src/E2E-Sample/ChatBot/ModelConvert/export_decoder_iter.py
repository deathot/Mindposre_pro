import numpy as np
import torch
import torch.nn as nn
from transformers import T5ForConditionalGeneration
from argparse import ArgumentParser

def read_args():
    args = ArgumentParser()
    args.add_argument('--version', type=str, default='v1',choices=['v1','v2'])
    return args.parse_args()

class Decoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.decoder = nn.Identity()
        self.lm_head = nn.Identity()

    def forward(self, decoder_input_ids, hidden_states, encoder_attention_mask, kv_in_0_0, kv_in_0_1, kv_in_0_2,
                kv_in_0_3,
                kv_in_1_0, kv_in_1_1, kv_in_1_2, kv_in_1_3,
                kv_in_2_0, kv_in_2_1, kv_in_2_2, kv_in_2_3,
                kv_in_3_0, kv_in_3_1, kv_in_3_2, kv_in_3_3,
                kv_in_4_0, kv_in_4_1, kv_in_4_2, kv_in_4_3,
                kv_in_5_0, kv_in_5_1, kv_in_5_2, kv_in_5_3,
                kv_in_6_0, kv_in_6_1, kv_in_6_2, kv_in_6_3,
                kv_in_7_0, kv_in_7_1, kv_in_7_2, kv_in_7_3,
                kv_in_8_0, kv_in_8_1, kv_in_8_2, kv_in_8_3,
                kv_in_9_0, kv_in_9_1, kv_in_9_2, kv_in_9_3,
                kv_in_10_0, kv_in_10_1, kv_in_10_2, kv_in_10_3,
                kv_in_11_0, kv_in_11_1, kv_in_11_2, kv_in_11_3,
                kv_in_12_0, kv_in_12_1, kv_in_12_2, kv_in_12_3,
                kv_in_13_0, kv_in_13_1, kv_in_13_2, kv_in_13_3,
                kv_in_14_0, kv_in_14_1, kv_in_14_2, kv_in_14_3,
                kv_in_15_0, kv_in_15_1, kv_in_15_2, kv_in_15_3,
                kv_in_16_0, kv_in_16_1, kv_in_16_2, kv_in_16_3,
                kv_in_17_0, kv_in_17_1, kv_in_17_2, kv_in_17_3,
                kv_in_18_0, kv_in_18_1, kv_in_18_2, kv_in_18_3,
                kv_in_19_0, kv_in_19_1, kv_in_19_2, kv_in_19_3,
                kv_in_20_0, kv_in_20_1, kv_in_20_2, kv_in_20_3,
                kv_in_21_0, kv_in_21_1, kv_in_21_2, kv_in_21_3,
                kv_in_22_0, kv_in_22_1, kv_in_22_2, kv_in_22_3,
                kv_in_23_0, kv_in_23_1, kv_in_23_2, kv_in_23_3):
        past_key_values = ((kv_in_0_0, kv_in_0_1, kv_in_0_2, kv_in_0_3),
                           (kv_in_1_0, kv_in_1_1, kv_in_1_2, kv_in_1_3),
                           (kv_in_2_0, kv_in_2_1, kv_in_2_2, kv_in_2_3),
                           (kv_in_3_0, kv_in_3_1, kv_in_3_2, kv_in_3_3),
                           (kv_in_4_0, kv_in_4_1, kv_in_4_2, kv_in_4_3),
                           (kv_in_5_0, kv_in_5_1, kv_in_5_2, kv_in_5_3),
                           (kv_in_6_0, kv_in_6_1, kv_in_6_2, kv_in_6_3),
                           (kv_in_7_0, kv_in_7_1, kv_in_7_2, kv_in_7_3),
                           (kv_in_8_0, kv_in_8_1, kv_in_8_2, kv_in_8_3),
                           (kv_in_9_0, kv_in_9_1, kv_in_9_2, kv_in_9_3),
                           (kv_in_10_0, kv_in_10_1, kv_in_10_2, kv_in_10_3),
                           (kv_in_11_0, kv_in_11_1, kv_in_11_2, kv_in_11_3),
                           (kv_in_12_0, kv_in_12_1, kv_in_12_2, kv_in_12_3),
                           (kv_in_13_0, kv_in_13_1, kv_in_13_2, kv_in_13_3),
                           (kv_in_14_0, kv_in_14_1, kv_in_14_2, kv_in_14_3),
                           (kv_in_15_0, kv_in_15_1, kv_in_15_2, kv_in_15_3),
                           (kv_in_16_0, kv_in_16_1, kv_in_16_2, kv_in_16_3),
                           (kv_in_17_0, kv_in_17_1, kv_in_17_2, kv_in_17_3),
                           (kv_in_18_0, kv_in_18_1, kv_in_18_2, kv_in_18_3),
                           (kv_in_19_0, kv_in_19_1, kv_in_19_2, kv_in_19_3),
                           (kv_in_20_0, kv_in_20_1, kv_in_20_2, kv_in_20_3),
                           (kv_in_21_0, kv_in_21_1, kv_in_21_2, kv_in_21_3),
                           (kv_in_22_0, kv_in_22_1, kv_in_22_2, kv_in_22_3),
                           (kv_in_23_0, kv_in_23_1, kv_in_23_2, kv_in_23_3))

        decoder_outputs = self.decoder(input_ids=decoder_input_ids,
                                       attention_mask=None,
                                       inputs_embeds=None,
                                       past_key_values=past_key_values,
                                       encoder_hidden_states=hidden_states,
                                       encoder_attention_mask=encoder_attention_mask,
                                       head_mask=None,
                                       cross_attn_head_mask=None,
                                       use_cache=True,
                                       output_attentions=None,
                                       output_hidden_states=None,
                                       return_dict=True, )

        sequence_output = decoder_outputs[0]
        lm_logits = self.lm_head(sequence_output)

        logits = lm_logits

        past_key_values = decoder_outputs.past_key_values
        kv_out_0_0, kv_out_0_1, kv_out_0_2, kv_out_0_3 = past_key_values[0]
        kv_out_1_0, kv_out_1_1, kv_out_1_2, kv_out_1_3 = past_key_values[1]
        kv_out_2_0, kv_out_2_1, kv_out_2_2, kv_out_2_3 = past_key_values[2]
        kv_out_3_0, kv_out_3_1, kv_out_3_2, kv_out_3_3 = past_key_values[3]
        kv_out_4_0, kv_out_4_1, kv_out_4_2, kv_out_4_3 = past_key_values[4]
        kv_out_5_0, kv_out_5_1, kv_out_5_2, kv_out_5_3 = past_key_values[5]
        kv_out_6_0, kv_out_6_1, kv_out_6_2, kv_out_6_3 = past_key_values[6]
        kv_out_7_0, kv_out_7_1, kv_out_7_2, kv_out_7_3 = past_key_values[7]
        kv_out_8_0, kv_out_8_1, kv_out_8_2, kv_out_8_3 = past_key_values[8]
        kv_out_9_0, kv_out_9_1, kv_out_9_2, kv_out_9_3 = past_key_values[9]
        kv_out_10_0, kv_out_10_1, kv_out_10_2, kv_out_10_3 = past_key_values[10]
        kv_out_11_0, kv_out_11_1, kv_out_11_2, kv_out_11_3 = past_key_values[11]
        kv_out_12_0, kv_out_12_1, kv_out_12_2, kv_out_12_3 = past_key_values[12]
        kv_out_13_0, kv_out_13_1, kv_out_13_2, kv_out_13_3 = past_key_values[13]
        kv_out_14_0, kv_out_14_1, kv_out_14_2, kv_out_14_3 = past_key_values[14]
        kv_out_15_0, kv_out_15_1, kv_out_15_2, kv_out_15_3 = past_key_values[15]
        kv_out_16_0, kv_out_16_1, kv_out_16_2, kv_out_16_3 = past_key_values[16]
        kv_out_17_0, kv_out_17_1, kv_out_17_2, kv_out_17_3 = past_key_values[17]
        kv_out_18_0, kv_out_18_1, kv_out_18_2, kv_out_18_3 = past_key_values[18]
        kv_out_19_0, kv_out_19_1, kv_out_19_2, kv_out_19_3 = past_key_values[19]
        kv_out_20_0, kv_out_20_1, kv_out_20_2, kv_out_20_3 = past_key_values[20]
        kv_out_21_0, kv_out_21_1, kv_out_21_2, kv_out_21_3 = past_key_values[21]
        kv_out_22_0, kv_out_22_1, kv_out_22_2, kv_out_22_3 = past_key_values[22]
        kv_out_23_0, kv_out_23_1, kv_out_23_2, kv_out_23_3 = past_key_values[23]

        return logits, kv_out_0_0, kv_out_0_1, kv_out_0_2, kv_out_0_3, kv_out_1_0, kv_out_1_1, kv_out_1_2, kv_out_1_3, kv_out_2_0, kv_out_2_1, kv_out_2_2, kv_out_2_3, kv_out_3_0, kv_out_3_1, kv_out_3_2, kv_out_3_3, kv_out_4_0, kv_out_4_1, kv_out_4_2, kv_out_4_3, kv_out_5_0, kv_out_5_1, kv_out_5_2, kv_out_5_3, kv_out_6_0, kv_out_6_1, kv_out_6_2, kv_out_6_3, kv_out_7_0, kv_out_7_1, kv_out_7_2, kv_out_7_3, kv_out_8_0, kv_out_8_1, kv_out_8_2, kv_out_8_3, kv_out_9_0, kv_out_9_1, kv_out_9_2, kv_out_9_3, kv_out_10_0, kv_out_10_1, kv_out_10_2, kv_out_10_3, kv_out_11_0, kv_out_11_1, kv_out_11_2, kv_out_11_3, kv_out_12_0, kv_out_12_1, kv_out_12_2, kv_out_12_3, kv_out_13_0, kv_out_13_1, kv_out_13_2, kv_out_13_3, kv_out_14_0, kv_out_14_1, kv_out_14_2, kv_out_14_3, kv_out_15_0, kv_out_15_1, kv_out_15_2, kv_out_15_3, kv_out_16_0, kv_out_16_1, kv_out_16_2, kv_out_16_3, kv_out_17_0, kv_out_17_1, kv_out_17_2, kv_out_17_3, kv_out_18_0, kv_out_18_1, kv_out_18_2, kv_out_18_3, kv_out_19_0, kv_out_19_1, kv_out_19_2, kv_out_19_3, kv_out_20_0, kv_out_20_1, kv_out_20_2, kv_out_20_3, kv_out_21_0, kv_out_21_1, kv_out_21_2, kv_out_21_3, kv_out_22_0, kv_out_22_1, kv_out_22_2, kv_out_22_3, kv_out_23_0, kv_out_23_1, kv_out_23_2, kv_out_23_3


if __name__ == '__main__':
    opts = read_args()
    model = T5ForConditionalGeneration.from_pretrained(f"ClueAI/ChatYuan-large-{opts.version}")

    decoder_part = model.decoder
    attention_mask = torch.ones(1, 768, dtype=torch.int64)

    hidden_states = torch.randn(1, 768, 1024, dtype=torch.float32)
    decoder_input_ids = torch.tensor(np.array([[22678]]), dtype=torch.int64)

    decoder = Decoder()
    decoder.decoder = decoder_part
    decoder.lm_head = model.lm_head

    past_key_values = tuple([(torch.randn(1, 16, 0, 64), torch.randn(1, 16, 0, 64), torch.randn(1, 16, 768, 64),
                              torch.randn(1, 16, 768, 64)) for _ in range(24)])
    kv_in_0_0, kv_in_0_1, kv_in_0_2, kv_in_0_3 = past_key_values[0]
    kv_in_1_0, kv_in_1_1, kv_in_1_2, kv_in_1_3 = past_key_values[1]
    kv_in_2_0, kv_in_2_1, kv_in_2_2, kv_in_2_3 = past_key_values[2]
    kv_in_3_0, kv_in_3_1, kv_in_3_2, kv_in_3_3 = past_key_values[3]
    kv_in_4_0, kv_in_4_1, kv_in_4_2, kv_in_4_3 = past_key_values[4]
    kv_in_5_0, kv_in_5_1, kv_in_5_2, kv_in_5_3 = past_key_values[5]
    kv_in_6_0, kv_in_6_1, kv_in_6_2, kv_in_6_3 = past_key_values[6]
    kv_in_7_0, kv_in_7_1, kv_in_7_2, kv_in_7_3 = past_key_values[7]
    kv_in_8_0, kv_in_8_1, kv_in_8_2, kv_in_8_3 = past_key_values[8]
    kv_in_9_0, kv_in_9_1, kv_in_9_2, kv_in_9_3 = past_key_values[9]
    kv_in_10_0, kv_in_10_1, kv_in_10_2, kv_in_10_3 = past_key_values[10]
    kv_in_11_0, kv_in_11_1, kv_in_11_2, kv_in_11_3 = past_key_values[11]
    kv_in_12_0, kv_in_12_1, kv_in_12_2, kv_in_12_3 = past_key_values[12]
    kv_in_13_0, kv_in_13_1, kv_in_13_2, kv_in_13_3 = past_key_values[13]
    kv_in_14_0, kv_in_14_1, kv_in_14_2, kv_in_14_3 = past_key_values[14]
    kv_in_15_0, kv_in_15_1, kv_in_15_2, kv_in_15_3 = past_key_values[15]
    kv_in_16_0, kv_in_16_1, kv_in_16_2, kv_in_16_3 = past_key_values[16]
    kv_in_17_0, kv_in_17_1, kv_in_17_2, kv_in_17_3 = past_key_values[17]
    kv_in_18_0, kv_in_18_1, kv_in_18_2, kv_in_18_3 = past_key_values[18]
    kv_in_19_0, kv_in_19_1, kv_in_19_2, kv_in_19_3 = past_key_values[19]
    kv_in_20_0, kv_in_20_1, kv_in_20_2, kv_in_20_3 = past_key_values[20]
    kv_in_21_0, kv_in_21_1, kv_in_21_2, kv_in_21_3 = past_key_values[21]
    kv_in_22_0, kv_in_22_1, kv_in_22_2, kv_in_22_3 = past_key_values[22]
    kv_in_23_0, kv_in_23_1, kv_in_23_2, kv_in_23_3 = past_key_values[23]

    # Export the model
    input_names = ['decoder_input_ids', 'hidden_states', 'encoder_attention_mask', 'kv_in_0_0', 'kv_in_0_1',
                   'kv_in_0_2', 'kv_in_0_3',
                   'kv_in_1_0', 'kv_in_1_1', 'kv_in_1_2', 'kv_in_1_3',
                   'kv_in_2_0', 'kv_in_2_1', 'kv_in_2_2', 'kv_in_2_3',
                   'kv_in_3_0', 'kv_in_3_1', 'kv_in_3_2', 'kv_in_3_3',
                   'kv_in_4_0', 'kv_in_4_1', 'kv_in_4_2', 'kv_in_4_3',
                   'kv_in_5_0', 'kv_in_5_1', 'kv_in_5_2', 'kv_in_5_3',
                   'kv_in_6_0', 'kv_in_6_1', 'kv_in_6_2', 'kv_in_6_3',
                   'kv_in_7_0', 'kv_in_7_1', 'kv_in_7_2', 'kv_in_7_3',
                   'kv_in_8_0', 'kv_in_8_1', 'kv_in_8_2', 'kv_in_8_3',
                   'kv_in_9_0', 'kv_in_9_1', 'kv_in_9_2', 'kv_in_9_3',
                   'kv_in_10_0', 'kv_in_10_1', 'kv_in_10_2', 'kv_in_10_3',
                   'kv_in_11_0', 'kv_in_11_1', 'kv_in_11_2', 'kv_in_11_3',
                   'kv_in_12_0', 'kv_in_12_1', 'kv_in_12_2', 'kv_in_12_3',
                   'kv_in_13_0', 'kv_in_13_1', 'kv_in_13_2', 'kv_in_13_3',
                   'kv_in_14_0', 'kv_in_14_1', 'kv_in_14_2', 'kv_in_14_3',
                   'kv_in_15_0', 'kv_in_15_1', 'kv_in_15_2', 'kv_in_15_3',
                   'kv_in_16_0', 'kv_in_16_1', 'kv_in_16_2', 'kv_in_16_3',
                   'kv_in_17_0', 'kv_in_17_1', 'kv_in_17_2', 'kv_in_17_3',
                   'kv_in_18_0', 'kv_in_18_1', 'kv_in_18_2', 'kv_in_18_3',
                   'kv_in_19_0', 'kv_in_19_1', 'kv_in_19_2', 'kv_in_19_3',
                   'kv_in_20_0', 'kv_in_20_1', 'kv_in_20_2', 'kv_in_20_3',
                   'kv_in_21_0', 'kv_in_21_1', 'kv_in_21_2', 'kv_in_21_3',
                   'kv_in_22_0', 'kv_in_22_1', 'kv_in_22_2', 'kv_in_22_3',
                   'kv_in_23_0', 'kv_in_23_1', 'kv_in_23_2', 'kv_in_23_3']
    output_names = ['logits', 'kv_out_0_0', 'kv_out_0_1', 'kv_out_0_2', 'kv_out_0_3', 'kv_out_1_0', 'kv_out_1_1',
                    'kv_out_1_2', 'kv_out_1_3', 'kv_out_2_0', 'kv_out_2_1', 'kv_out_2_2', 'kv_out_2_3', 'kv_out_3_0',
                    'kv_out_3_1', 'kv_out_3_2', 'kv_out_3_3', 'kv_out_4_0', 'kv_out_4_1', 'kv_out_4_2', 'kv_out_4_3',
                    'kv_out_5_0', 'kv_out_5_1', 'kv_out_5_2', 'kv_out_5_3', 'kv_out_6_0', 'kv_out_6_1', 'kv_out_6_2',
                    'kv_out_6_3', 'kv_out_7_0', 'kv_out_7_1', 'kv_out_7_2', 'kv_out_7_3', 'kv_out_8_0', 'kv_out_8_1',
                    'kv_out_8_2', 'kv_out_8_3', 'kv_out_9_0', 'kv_out_9_1', 'kv_out_9_2', 'kv_out_9_3', 'kv_out_10_0',
                    'kv_out_10_1', 'kv_out_10_2', 'kv_out_10_3', 'kv_out_11_0', 'kv_out_11_1', 'kv_out_11_2',
                    'kv_out_11_3', 'kv_out_12_0', 'kv_out_12_1', 'kv_out_12_2', 'kv_out_12_3', 'kv_out_13_0',
                    'kv_out_13_1', 'kv_out_13_2', 'kv_out_13_3', 'kv_out_14_0', 'kv_out_14_1', 'kv_out_14_2',
                    'kv_out_14_3', 'kv_out_15_0', 'kv_out_15_1', 'kv_out_15_2', 'kv_out_15_3', 'kv_out_16_0',
                    'kv_out_16_1', 'kv_out_16_2', 'kv_out_16_3', 'kv_out_17_0', 'kv_out_17_1', 'kv_out_17_2',
                    'kv_out_17_3', 'kv_out_18_0', 'kv_out_18_1', 'kv_out_18_2', 'kv_out_18_3', 'kv_out_19_0',
                    'kv_out_19_1', 'kv_out_19_2', 'kv_out_19_3', 'kv_out_20_0', 'kv_out_20_1', 'kv_out_20_2',
                    'kv_out_20_3', 'kv_out_21_0', 'kv_out_21_1', 'kv_out_21_2', 'kv_out_21_3', 'kv_out_22_0',
                    'kv_out_22_1', 'kv_out_22_2', 'kv_out_22_3', 'kv_out_23_0', 'kv_out_23_1', 'kv_out_23_2',
                    'kv_out_23_3']

    dynamic_axes = {
        'kv_in_0_0': {2: 'in_length'}, 'kv_in_0_1': {2: 'in_length'},
        'kv_in_1_0': {2: 'in_length'}, 'kv_in_1_1': {2: 'in_length'},
        'kv_in_2_0': {2: 'in_length'}, 'kv_in_2_1': {2: 'in_length'},
        'kv_in_3_0': {2: 'in_length'}, 'kv_in_3_1': {2: 'in_length'},
        'kv_in_4_0': {2: 'in_length'}, 'kv_in_4_1': {2: 'in_length'},
        'kv_in_5_0': {2: 'in_length'}, 'kv_in_5_1': {2: 'in_length'},
        'kv_in_6_0': {2: 'in_length'}, 'kv_in_6_1': {2: 'in_length'},
        'kv_in_7_0': {2: 'in_length'}, 'kv_in_7_1': {2: 'in_length'},
        'kv_in_8_0': {2: 'in_length'}, 'kv_in_8_1': {2: 'in_length'},
        'kv_in_9_0': {2: 'in_length'}, 'kv_in_9_1': {2: 'in_length'},
        'kv_in_10_0': {2: 'in_length'}, 'kv_in_10_1': {2: 'in_length'},
        'kv_in_11_0': {2: 'in_length'}, 'kv_in_11_1': {2: 'in_length'},
        'kv_in_12_0': {2: 'in_length'}, 'kv_in_12_1': {2: 'in_length'},
        'kv_in_13_0': {2: 'in_length'}, 'kv_in_13_1': {2: 'in_length'},
        'kv_in_14_0': {2: 'in_length'}, 'kv_in_14_1': {2: 'in_length'},
        'kv_in_15_0': {2: 'in_length'}, 'kv_in_15_1': {2: 'in_length'},
        'kv_in_16_0': {2: 'in_length'}, 'kv_in_16_1': {2: 'in_length'},
        'kv_in_17_0': {2: 'in_length'}, 'kv_in_17_1': {2: 'in_length'},
        'kv_in_18_0': {2: 'in_length'}, 'kv_in_18_1': {2: 'in_length'},
        'kv_in_19_0': {2: 'in_length'}, 'kv_in_19_1': {2: 'in_length'},
        'kv_in_20_0': {2: 'in_length'}, 'kv_in_20_1': {2: 'in_length'},
        'kv_in_21_0': {2: 'in_length'}, 'kv_in_21_1': {2: 'in_length'},
        'kv_in_22_0': {2: 'in_length'}, 'kv_in_22_1': {2: 'in_length'},
        'kv_in_23_0': {2: 'in_length'}, 'kv_in_23_1': {2: 'in_length'},
        'kv_out_0_0': {2: 'out_length'}, 'kv_out_0_1': {2: 'out_length'},
        'kv_out_1_0': {2: 'out_length'}, 'kv_out_1_1': {2: 'out_length'},
        'kv_out_2_0': {2: 'out_length'}, 'kv_out_2_1': {2: 'out_length'},
        'kv_out_3_0': {2: 'out_length'}, 'kv_out_3_1': {2: 'out_length'},
        'kv_out_4_0': {2: 'out_length'}, 'kv_out_4_1': {2: 'out_length'},
        'kv_out_5_0': {2: 'out_length'}, 'kv_out_5_1': {2: 'out_length'},
        'kv_out_6_0': {2: 'out_length'}, 'kv_out_6_1': {2: 'out_length'},
        'kv_out_7_0': {2: 'out_length'}, 'kv_out_7_1': {2: 'out_length'},
        'kv_out_8_0': {2: 'out_length'}, 'kv_out_8_1': {2: 'out_length'},
        'kv_out_9_0': {2: 'out_length'}, 'kv_out_9_1': {2: 'out_length'},
        'kv_out_10_0': {2: 'out_length'}, 'kv_out_10_1': {2: 'out_length'},
        'kv_out_11_0': {2: 'out_length'}, 'kv_out_11_1': {2: 'out_length'},
        'kv_out_12_0': {2: 'out_length'}, 'kv_out_12_1': {2: 'out_length'},
        'kv_out_13_0': {2: 'out_length'}, 'kv_out_13_1': {2: 'out_length'},
        'kv_out_14_0': {2: 'out_length'}, 'kv_out_14_1': {2: 'out_length'},
        'kv_out_15_0': {2: 'out_length'}, 'kv_out_15_1': {2: 'out_length'},
        'kv_out_16_0': {2: 'out_length'}, 'kv_out_16_1': {2: 'out_length'},
        'kv_out_17_0': {2: 'out_length'}, 'kv_out_17_1': {2: 'out_length'},
        'kv_out_18_0': {2: 'out_length'}, 'kv_out_18_1': {2: 'out_length'},
        'kv_out_19_0': {2: 'out_length'}, 'kv_out_19_1': {2: 'out_length'},
        'kv_out_20_0': {2: 'out_length'}, 'kv_out_20_1': {2: 'out_length'},
        'kv_out_21_0': {2: 'out_length'}, 'kv_out_21_1': {2: 'out_length'},
        'kv_out_22_0': {2: 'out_length'}, 'kv_out_22_1': {2: 'out_length'},
        'kv_out_23_0': {2: 'out_length'}, 'kv_out_23_1': {2: 'out_length'},
    }

    print('export decoder iter start')
    torch.onnx.export(decoder,  # model being run
                      (decoder_input_ids, hidden_states, attention_mask, kv_in_0_0, kv_in_0_1, kv_in_0_2,
                       kv_in_0_3, kv_in_1_0, kv_in_1_1, kv_in_1_2, kv_in_1_3, kv_in_2_0, kv_in_2_1, kv_in_2_2,
                       kv_in_2_3, kv_in_3_0, kv_in_3_1, kv_in_3_2, kv_in_3_3, kv_in_4_0, kv_in_4_1, kv_in_4_2,
                       kv_in_4_3, kv_in_5_0, kv_in_5_1, kv_in_5_2, kv_in_5_3,
                       kv_in_6_0, kv_in_6_1, kv_in_6_2, kv_in_6_3,
                       kv_in_7_0, kv_in_7_1, kv_in_7_2, kv_in_7_3,
                       kv_in_8_0, kv_in_8_1, kv_in_8_2, kv_in_8_3,
                       kv_in_9_0, kv_in_9_1, kv_in_9_2, kv_in_9_3,
                       kv_in_10_0, kv_in_10_1, kv_in_10_2, kv_in_10_3,
                       kv_in_11_0, kv_in_11_1, kv_in_11_2, kv_in_11_3,
                       kv_in_12_0, kv_in_12_1, kv_in_12_2, kv_in_12_3,
                       kv_in_13_0, kv_in_13_1, kv_in_13_2, kv_in_13_3,
                       kv_in_14_0, kv_in_14_1, kv_in_14_2, kv_in_14_3,
                       kv_in_15_0, kv_in_15_1, kv_in_15_2, kv_in_15_3,
                       kv_in_16_0, kv_in_16_1, kv_in_16_2, kv_in_16_3,
                       kv_in_17_0, kv_in_17_1, kv_in_17_2, kv_in_17_3,
                       kv_in_18_0, kv_in_18_1, kv_in_18_2, kv_in_18_3,
                       kv_in_19_0, kv_in_19_1, kv_in_19_2, kv_in_19_3,
                       kv_in_20_0, kv_in_20_1, kv_in_20_2, kv_in_20_3,
                       kv_in_21_0, kv_in_21_1, kv_in_21_2, kv_in_21_3,
                       kv_in_22_0, kv_in_22_1, kv_in_22_2, kv_in_22_3,
                       kv_in_23_0, kv_in_23_1, kv_in_23_2, kv_in_23_3),
                      "decoder_iter.onnx",  # where to save the model (can be a file or file-like object)
                      opset_version=13,  # the ONNX version to export the model to
                      input_names=input_names,  # the model's input names
                      output_names=output_names,  # the model's output names
                      dynamic_axes=dynamic_axes
                      )
