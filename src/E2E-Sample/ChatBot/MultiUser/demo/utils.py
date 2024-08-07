#!/usr/bin/env python3
# -*- coding: utf-8 -*-

def content_generate(records, inputs):
    if not records:
        context = ''
    else:
        context = "\n".join([f"用户：{input_text}\n小元：{ans_text}" for input_text, ans_text in records]) + "\n"
    return context + "用户：" + inputs + "\n小元："


def preprocess(t):
    return t.replace("\n", "\\n").replace("\t", "\\t")


def postprocess(text):
    return text.replace("\\n", "\n").replace("\\t", "\t")


def generate_onnx_input(decode_input_ids, attention_mask, past_key_values):
    input_list = [decode_input_ids, None, attention_mask]
    for tensor in past_key_values:
        input_list.append(tensor)

    input_dict = {'decoder_input_ids': input_list[0], 'encoder_attention_mask': input_list[2],
                  'kv_in_0_0': input_list[3], 'kv_in_0_1': input_list[4], 'kv_in_0_2': input_list[5],
                  'kv_in_0_3': input_list[6],
                  'kv_in_1_0': input_list[7], 'kv_in_1_1': input_list[8], 'kv_in_1_2': input_list[9],
                  'kv_in_1_3': input_list[10],
                  'kv_in_2_0': input_list[11], 'kv_in_2_1': input_list[12], 'kv_in_2_2': input_list[13],
                  'kv_in_2_3': input_list[14],
                  'kv_in_3_0': input_list[15], 'kv_in_3_1': input_list[16], 'kv_in_3_2': input_list[17],
                  'kv_in_3_3': input_list[18],
                  'kv_in_4_0': input_list[19], 'kv_in_4_1': input_list[20], 'kv_in_4_2': input_list[21],
                  'kv_in_4_3': input_list[22],
                  'kv_in_5_0': input_list[23], 'kv_in_5_1': input_list[24], 'kv_in_5_2': input_list[25],
                  'kv_in_5_3': input_list[26],
                  'kv_in_6_0': input_list[27], 'kv_in_6_1': input_list[28], 'kv_in_6_2': input_list[29],
                  'kv_in_6_3': input_list[30],
                  'kv_in_7_0': input_list[31], 'kv_in_7_1': input_list[32], 'kv_in_7_2': input_list[33],
                  'kv_in_7_3': input_list[34],
                  'kv_in_8_0': input_list[35], 'kv_in_8_1': input_list[36], 'kv_in_8_2': input_list[37],
                  'kv_in_8_3': input_list[38],
                  'kv_in_9_0': input_list[39], 'kv_in_9_1': input_list[40], 'kv_in_9_2': input_list[41],
                  'kv_in_9_3': input_list[42],
                  'kv_in_10_0': input_list[43], 'kv_in_10_1': input_list[44], 'kv_in_10_2': input_list[45],
                  'kv_in_10_3': input_list[46],
                  'kv_in_11_0': input_list[47], 'kv_in_11_1': input_list[48], 'kv_in_11_2': input_list[49],
                  'kv_in_11_3': input_list[50],
                  'kv_in_12_0': input_list[51], 'kv_in_12_1': input_list[52], 'kv_in_12_2': input_list[53],
                  'kv_in_12_3': input_list[54],
                  'kv_in_13_0': input_list[55], 'kv_in_13_1': input_list[56], 'kv_in_13_2': input_list[57],
                  'kv_in_13_3': input_list[58],
                  'kv_in_14_0': input_list[59], 'kv_in_14_1': input_list[60], 'kv_in_14_2': input_list[61],
                  'kv_in_14_3': input_list[62],
                  'kv_in_15_0': input_list[63], 'kv_in_15_1': input_list[64], 'kv_in_15_2': input_list[65],
                  'kv_in_15_3': input_list[66],
                  'kv_in_16_0': input_list[67], 'kv_in_16_1': input_list[68], 'kv_in_16_2': input_list[69],
                  'kv_in_16_3': input_list[70],
                  'kv_in_17_0': input_list[71], 'kv_in_17_1': input_list[72], 'kv_in_17_2': input_list[73],
                  'kv_in_17_3': input_list[74],
                  'kv_in_18_0': input_list[75], 'kv_in_18_1': input_list[76], 'kv_in_18_2': input_list[77],
                  'kv_in_18_3': input_list[78],
                  'kv_in_19_0': input_list[79], 'kv_in_19_1': input_list[80], 'kv_in_19_2': input_list[81],
                  'kv_in_19_3': input_list[82],
                  'kv_in_20_0': input_list[83], 'kv_in_20_1': input_list[84], 'kv_in_20_2': input_list[85],
                  'kv_in_20_3': input_list[86],
                  'kv_in_21_0': input_list[87], 'kv_in_21_1': input_list[88], 'kv_in_21_2': input_list[89],
                  'kv_in_21_3': input_list[90],
                  'kv_in_22_0': input_list[91], 'kv_in_22_1': input_list[92], 'kv_in_22_2': input_list[93],
                  'kv_in_22_3': input_list[94],
                  'kv_in_23_0': input_list[95], 'kv_in_23_1': input_list[96], 'kv_in_23_2': input_list[97],
                  'kv_in_23_3': input_list[98]}
    return input_dict
