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

    def forward(self, decoder_input_ids, hidden_states, encoder_attention_mask):
        decoder_outputs = self.decoder(input_ids=decoder_input_ids,
                                       attention_mask=None,
                                       inputs_embeds=None,
                                       past_key_values=None,
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
        return logits, past_key_values


if __name__ == '__main__':
    opts = read_args()
    model = T5ForConditionalGeneration.from_pretrained(f"ClueAI/ChatYuan-large-{opts.version}")
    decoder_part = model.decoder
    attention_mask = torch.ones(1, 768, dtype=torch.int64)

    hidden_states = torch.randn(1, 768, 1024, dtype=torch.float32)
    decoder_input_ids = torch.tensor(np.array([[0]]), dtype=torch.int64)
    decoder = Decoder()
    decoder.decoder = decoder_part
    decoder.lm_head = model.lm_head

    # Export the model
    input_names = ['decoder_input_ids', 'hidden_states', 'attention_mask']
    output_names = ['logits', 'past_key_values', ]

    print('export first decoder start')
    torch.onnx.export(decoder,  # model being run
                      (decoder_input_ids, hidden_states, attention_mask),
                      # model input (or a tuple for multiple inputs)
                      "decoder_first.onnx",  # where to save the model (can be a file or file-like object)
                      opset_version=13,  # the ONNX version to export the model to
                      input_names=input_names,  # the model's input names
                      output_names=output_names,  # the model's output names
                      )
