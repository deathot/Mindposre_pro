import unittest

import torch

from v2.wenet.model import WeNetASR, pad_sequence, resample, load_vocab, compute_fbank


class TestWeNetASR(unittest.TestCase):
    def setUp(self):
        self.model_path = "offline_encoder.om"
        self.vocab_path = "vocab.txt"
        self.wav_file = "sample.wav"
        self.max_len = 966

    def test_transcribe(self):
        model = WeNetASR(self.model_path, self.vocab_path)
        txt = model.transcribe(self.wav_file)
        self.assertIsInstance(txt, str)
        self.assertNotEqual(txt, "")

    def test_preprocess(self):
        model = WeNetASR(self.model_path, self.vocab_path)
        feats_pad, feats_lengths = model.preprocess(self.wav_file)

        self.assertIsInstance(feats_pad, torch.Tensor)
        self.assertIsInstance(feats_lengths, torch.Tensor)
        self.assertEqual(feats_pad.ndim, 3)
        self.assertEqual(feats_lengths.ndim, 1)
        self.assertEqual(feats_pad.size(0), 1)  # Batch size should be 1
        self.assertLessEqual(feats_pad.size(1), self.max_len)

    def test_post_process(self):
        model = WeNetASR(self.model_path, self.vocab_path)
        output = (torch.rand(2, 10, 256), torch.tensor([8, 6]), torch.rand(2, 10, 30),
                  torch.rand(2, 10, 100), torch.rand(2, 10, 3))
        txt = model.post_process(output)
        self.assertIsInstance(txt, str)
        self.assertNotEqual(txt, "")

    def test_pad_sequence(self):
        seq_feature = torch.randn(100, 40)  # Input sequence with shape (100, 40)
        padded_seq = pad_sequence(seq_feature)
        self.assertIsInstance(padded_seq, torch.Tensor)
        self.assertEqual(padded_seq.size(0), 1)  # Batch size should be 1
        self.assertEqual(padded_seq.size(1), self.max_len)

    def test_resample(self):
        waveform = torch.randn(1, 16000)  # Input waveform with shape (1, 16000)
        sample_rate = 16000
        resample_rate = 8000
        resampled_waveform, resampled_sample_rate = resample(waveform, sample_rate, resample_rate)
        self.assertIsInstance(resampled_waveform, torch.Tensor)
        self.assertEqual(resampled_waveform.size(1), resample_rate)
        self.assertEqual(resampled_sample_rate, resample_rate)

    def test_compute_fbank(self, num_mel_bins=80):
        waveform = torch.randn(1, 8000)  # Input waveform with shape (1, 8000)
        sample_rate = 8000
        mat = compute_fbank(waveform, sample_rate)
        self.assertIsInstance(mat, torch.Tensor)
        self.assertEqual(mat.size(1), num_mel_bins)

    def test_load_vocab(self):
        vocab = load_vocab(self.vocab_path)
        self.assertIsInstance(vocab, list)
        self.assertNotEqual(len(vocab), 0)


if __name__ == '__main__':
    unittest.main()
