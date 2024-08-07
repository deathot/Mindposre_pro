import os

from ais_bench.infer.interface import InferSession

if __name__ == '__main__':
    cwd_path = os.getcwd()
    encoder = InferSession(0, os.path.join(cwd_path, 'models', 'encoder.om'))
    print(encoder)
