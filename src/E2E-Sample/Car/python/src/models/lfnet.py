import cv2
import numpy as np

from src.models.bsae_model import Model


class LFNet(Model):
    def __init__(self, model_path, acl_init=True):
        super().__init__(model_path, acl_init)

    def infer(self, inputs):
        inputs = inputs[:, :, [2, 1, 0]]
        inputs = cv2.resize(inputs, (320, 180))
        inputs = inputs.astype(np.float32) / 255
        batched_img = np.expand_dims(inputs, axis=0)
        result = self.execute([np.ascontiguousarray(batched_img)])
        return result
