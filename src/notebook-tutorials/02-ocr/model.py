import math
from abc import abstractmethod, ABC

import acl
import cv2
import numpy as np

from utils import detect, dtype_map

ACL_MEM_MALLOC_HUGE_FIRST = 0
ACL_MEMCPY_HOST_TO_DEVICE = 1
ACL_MEMCPY_DEVICE_TO_HOST = 2


class Model(ABC):
    def __init__(self, model_path, device_id=0):
        self.model_id = None
        self.output_data = None
        self.load_output_dataset = None
        self.input_data = None
        self.load_input_dataset = None
        self.device_id = device_id
        self.context = None
        self.model_desc = None

        self.model_height = None
        self.model_width = None
        self.model_channel = None
        self.output_shapes = []
        self.output_dtypes = []

        self.init_acl(model_path)

    def init_acl(self, model_path):
        self.model_id, ret = acl.mdl.load_from_file(model_path)
        if ret:
            raise RuntimeError(ret)

        model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(model_desc, self.model_id)
        if ret:
            raise RuntimeError(ret)

        dims, ret = acl.mdl.get_input_dims_v2(model_desc, 0)
        if ret:
            raise RuntimeError(ret)
        dims = dims['dims']

        self.model_channel = dims[1]
        self.model_height = dims[2]
        self.model_width = dims[3]

        self.model_desc = model_desc
        # prepare input data resources for model infer
        self.load_input_dataset = acl.mdl.create_dataset()
        input_size = acl.mdl.get_num_inputs(self.model_desc)
        self.input_data = []
        for i in range(input_size):
            buffer_size = acl.mdl.get_input_size_by_index(self.model_desc, i)
            buffer, ret = acl.rt.malloc(buffer_size, ACL_MEM_MALLOC_HUGE_FIRST)
            data = acl.create_data_buffer(buffer, buffer_size)
            _, ret = acl.mdl.add_dataset_buffer(self.load_input_dataset, data)
            self.input_data.append({"buffer": buffer, "size": buffer_size})

        # prepare output data resources for model infer
        self.load_output_dataset = acl.mdl.create_dataset()
        output_size = acl.mdl.get_num_outputs(self.model_desc)
        self.output_data = []
        for i in range(output_size):
            dims, ret = acl.mdl.get_output_dims(model_desc, i)
            self.output_shapes.append(tuple(dims['dims']))
            data_type = acl.mdl.get_output_data_type(model_desc, i)
            self.output_dtypes.append(dtype_map[data_type])
            buffer_size = acl.mdl.get_output_size_by_index(self.model_desc, i)
            buffer, ret = acl.rt.malloc(buffer_size, ACL_MEM_MALLOC_HUGE_FIRST)
            data = acl.create_data_buffer(buffer, buffer_size)
            _, ret = acl.mdl.add_dataset_buffer(self.load_output_dataset, data)
            self.output_data.append({"buffer": buffer, "size": buffer_size})

        print(f'{self.__class__.__name__} Resources init successfully.')

    @abstractmethod
    def preprocess(self, img):
        pass

    def infer(self, tensor):
        np_ptr = acl.util.bytes_to_ptr(tensor.tobytes())
        # copy input data from host to device
        ret = acl.rt.memcpy(self.input_data[0]["buffer"], self.input_data[0]["size"], np_ptr,
                            self.input_data[0]["size"], ACL_MEMCPY_HOST_TO_DEVICE)

        # infer exec
        ret = acl.mdl.execute(self.model_id, self.load_input_dataset, self.load_output_dataset)

        inference_result = []

        for i, item in enumerate(self.output_data):
            buffer_host, ret = acl.rt.malloc_host(self.output_data[i]["size"])
            # copy output data from device to host
            ret = acl.rt.memcpy(buffer_host, self.output_data[i]["size"], self.output_data[i]["buffer"],
                                self.output_data[i]["size"], ACL_MEMCPY_DEVICE_TO_HOST)

            data = acl.util.ptr_to_bytes(buffer_host, self.output_data[i]['size'])
            data = np.frombuffer(data, dtype=self.output_dtypes[i]).reshape(self.output_shapes[i])
            inference_result.append(data)

        return inference_result

    @abstractmethod
    def postprocess(self, output):
        pass

    def deinit(self):
        ret = acl.mdl.unload(self.model_id)
        if ret:
            raise RuntimeError(ret)
        if self.model_desc:
            acl.mdl.destroy_desc(self.model_desc)
            self.model_desc = None

        while self.input_data:
            item = self.input_data.pop()
            ret = acl.rt.free(item["buffer"])
            if ret:
                raise RuntimeError(ret)

        while self.output_data:
            item = self.output_data.pop()
            ret = acl.rt.free(item["buffer"])
            if ret:
                raise RuntimeError(ret)
        print(f'{self.__class__.__name__} Resources released successfully.')


class CTPN(Model):
    def __init__(self, model_path, device_id=0):
        super().__init__(model_path, device_id)
        self.mean = np.array([123.675, 116.28, 103.53]).reshape((1, 1, 3)).astype(np.float32)
        self.std = np.array([58.395, 57.12, 57.375]).reshape((1, 1, 3)).astype(np.float32)

    def preprocess(self, img):
        # resize image and convert dtype to fp32
        dst_img = cv2.resize(img, (int(self.model_width), int(self.model_height))).astype(np.float32)

        # normalization
        dst_img -= self.mean
        dst_img /= self.std

        # hwc to chw
        dst_img = dst_img.transpose((2, 0, 1))

        # chw to nchw
        dst_img = np.expand_dims(dst_img, axis=0)
        dst_img = np.ascontiguousarray(dst_img).astype(np.float32)
        return dst_img

    def postprocess(self, output):
        proposal = output[0]
        proposal_mask = output[1]
        all_box_tmp = proposal
        all_mask_tmp = np.expand_dims(proposal_mask, axis=1)
        using_boxes_mask = all_box_tmp * all_mask_tmp
        textsegs = using_boxes_mask[:, 0:4].astype(np.float32)
        scores = using_boxes_mask[:, 4].astype(np.float32)
        bboxes = detect(textsegs, scores[:, np.newaxis], (self.model_height, self.model_width))
        return bboxes


class SVTR(Model):
    def __init__(self, model_path, dict_path, device_id=0):
        super().__init__(model_path, device_id)
        self.labels = ['']
        with open(dict_path, 'r') as f:
            labels = f.readlines()
            for char in labels:
                self.labels.append(char.strip())
        self.labels.append(' ')
        self.scale = np.float32(1 / 255)
        self.mean = 0.5
        self.std = 0.5

    def preprocess(self, img):
        h, w, _ = img.shape
        ratio = w / h
        if math.ceil(ratio * self.model_height) > self.model_width:
            resize_w = self.model_width
        else:
            resize_w = math.ceil(ratio * self.model_height)

        img = cv2.resize(img, (resize_w, self.model_height))

        _, w, _ = img.shape
        padding_w = self.model_width - w
        img = cv2.copyMakeBorder(img, 0, 0, 0, padding_w, cv2.BORDER_CONSTANT, value=0.).astype(np.float32)
        img *= self.scale
        img -= self.mean
        img /= self.std

        # hwc to chw
        dst_img = img.transpose((2, 0, 1))

        # chw to nchw
        dst_img = np.expand_dims(dst_img, axis=0)
        dst_img = np.ascontiguousarray(dst_img).astype(np.float32)

        return dst_img

    def postprocess(self, output):
        output = np.argmax(output[0], axis=2).reshape(-1)
        ans = []
        last_char = ''
        for i, char in enumerate(output):
            if char and self.labels[char] != last_char:
                ans.append(self.labels[char])
            last_char = self.labels[char]
        return ''.join(ans)
