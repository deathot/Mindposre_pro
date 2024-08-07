import argparse
import os

import onnxruntime
import torch
import yaml

from wenet.transformer.asr_model import init_asr_model


def test_equality(xlist, blist, rtol=1e-3, atol=1e-5, tolerate_small_mismatch=True):
    for a, b in zip(xlist, blist):
        try:
            torch.testing.assert_allclose(a, b, rtol=rtol, atol=atol)
        except AssertionError as error:
            if tolerate_small_mismatch:
                print(error)
            else:
                raise


def to_numpy(tensors):
    out = []
    if type(tensors) == torch.tensor:
        tensors = [tensors]
    for tensor in tensors:
        if tensor.requires_grad:
            tensor = tensor.detach().cpu().numpy()
        else:
            tensor = tensor.cpu().numpy()
        out.append(tensor)
    return out


def load_checkpoint(model, ckpt_path):
    checkpoint = torch.load(ckpt_path, map_location='cpu')
    model.load_state_dict(checkpoint, strict=False)
    print(f"Loaded checkpoint from {ckpt_path}")


def export_offline_encoder(model, configs, encoder_onnx_path):
    bz = 1
    seq_len = 100
    beam_size = 10
    feature_size = configs["input_dim"]

    speech = torch.randn(bz, seq_len, feature_size, dtype=torch.float32)
    speech_lens = torch.randint(low=10, high=seq_len, size=(bz,), dtype=torch.int32)
    encoder = Encoder(model.encoder, model.ctc, beam_size)
    encoder.eval()

    torch.onnx.export(encoder,
                      (speech, speech_lens),
                      encoder_onnx_path,
                      export_params=True,
                      opset_version=11,
                      do_constant_folding=True,
                      input_names=['speech', 'speech_lengths'],
                      output_names=['encoder_out', 'encoder_out_lens',
                                    'ctc_log_probs',
                                    'beam_log_probs', 'beam_log_probs_idx'],
                      dynamic_axes={
                          'speech': {0: 'B', 1: 'T'},
                          'speech_lengths': {0: 'B'},
                          'encoder_out': {0: 'B', 1: 'T_OUT'},
                          'encoder_out_lens': {0: 'B'},
                          'ctc_log_probs': {0: 'B', 1: 'T_OUT'},
                          'beam_log_probs': {0: 'B', 1: 'T_OUT'},
                          'beam_log_probs_idx': {0: 'B', 1: 'T_OUT'},
                      },
                      verbose=False
                      )

    with torch.no_grad():
        o0, o1, o2, o3, o4 = encoder(speech, speech_lens)

    providers = ["CUDAExecutionProvider"]
    ort_session = onnxruntime.InferenceSession(encoder_onnx_path,
                                               providers=providers)
    ort_inputs = {'speech': to_numpy(speech),
                  'speech_lengths': to_numpy(speech_lens)}
    ort_outs = ort_session.run(None, ort_inputs)

    # check encoder output
    test_equality(to_numpy([o0, o1, o2, o3, o4]), ort_outs)
    print("export offline onnx encoder succeed!")


class Encoder(torch.nn.Module):
    def __init__(self,
                 encoder,
                 ctc,
                 beam_size=10):
        super().__init__()
        self.encoder = encoder
        self.ctc = ctc
        self.beam_size = beam_size

    def forward(self, speech: torch.Tensor,
                speech_lengths: torch.Tensor, ):
        """Encoder
        Args:
            speech: (Batch, Length, ...)
            speech_lengths: (Batch, )
        Returns:
            encoder_out: B x T x F
            encoder_out_lens: B
            ctc_log_probs: B x T x V
            beam_log_probs: B x T x beam_size
            beam_log_probs_idx: B x T x beam_size
        """
        encoder_out, encoder_mask = self.encoder(speech,
                                                 speech_lengths,
                                                 -1, -1)
        encoder_out_lens = encoder_mask.squeeze(1).sum(1)
        ctc_log_probs = self.ctc.log_softmax(encoder_out)
        encoder_out_lens = encoder_out_lens.int()
        beam_log_probs, beam_log_probs_idx = torch.topk(
            ctc_log_probs, self.beam_size, dim=2)
        return encoder_out, encoder_out_lens, ctc_log_probs, \
            beam_log_probs, beam_log_probs_idx


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, help='config file')
    parser.add_argument('--checkpoint', required=True, help='checkpoint model')
    parser.add_argument('--cmvn_file', required=False, default='', type=str,
                        help='global_cmvn file which stores the mean and variance statistics')

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    with open(args.config, 'r') as fin:
        configs = yaml.load(fin, Loader=yaml.FullLoader)

    configs["encoder_conf"]["use_dynamic_chunk"] = False
    if args.cmvn_file and os.path.exists(args.cmvn_file):
        print(f"load cmvn from {args.cmvn_file}")
        configs['cmvn_file'] = args.cmvn_file

    model = init_asr_model(configs)
    load_checkpoint(model, args.checkpoint)
    model.eval()

    save_path = './offline_encoder.onnx'
    export_offline_encoder(model, configs, save_path)


if __name__ == '__main__':
    main()
