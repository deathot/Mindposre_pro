from onnxruntime.quantization import quantize_dynamic

if __name__ == '__main__':
    decoder_iter_fp32 = "./decoder_iter_sim.onnx"
    decoder_iter_quant = "./decoder_iter_sim_quant.onnx"

    quantize_dynamic(decoder_iter_fp32,decoder_iter_quant)

    first_decoder_fp32 = "./decoder_first_sim.onnx"
    first_decoder_quant = "./decoder_first_sim_quant.onnx"

    quantize_dynamic(first_decoder_fp32, first_decoder_quant)