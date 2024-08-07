"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-6-04 20:12:13
MODIFIED: 2020-6-28 14:04:45
"""
import atexit
import time
import os
from abc import ABC, abstractmethod

import acl

import numpy as np

from src.utils import copy_data_device_to_device, check_ret, SUCCESS, ACL_MEMCPY_DEVICE_TO_DEVICE, \
    ACL_MEM_MALLOC_NORMAL_ONLY, ACL_FLOAT16, ACL_FLOAT, ACL_INT32, ACL_UINT32, FAILED, init_acl, deinit_acl, SUCCESS
from src.utils import log
from src.utils.constant import ACL_INT8,ACL_BOOL,ACL_INT32,ACL_UINT32,ACL_DOUBLE,ACL_FLOAT,ACL_FLOAT16,ACL_UINT8,ACL_INT16,ACL_UINT16,ACL_INT64,ACL_UINT64


class Model(ABC):
    def __init__(self, model_path, init_acl=True):
        self._run_mode, ret = acl.rt.get_run_mode()
        check_ret("acl.rt.get_run_mode", ret)
        self._copy_policy = ACL_MEMCPY_DEVICE_TO_DEVICE

        self._model_path = model_path    # string
        self._model_id = None            # pointer
        self._input_num = 0
        self._input_buffer = []
        self._input_dataset = None
        self._output_dataset = None
        self._model_desc = None          # pointer when using
        self._output_size = 0
        self._init_resource()
        self._is_destroyed = False

    def _init_resource(self):
        log.info("Init model resource start...")
        if not os.path.isfile(self._model_path):
            log.error(
                "model_path failed, please check. model_path=%s" %
                self._model_path)
            return FAILED


        self._model_id, ret = acl.mdl.load_from_file(self._model_path)
        check_ret("acl.mdl.load_from_file", ret)

        self._model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self._model_desc, self._model_id)
        check_ret("acl.mdl.get_desc", ret)
        # get outputs num of model
        self._output_size = acl.mdl.get_num_outputs(self._model_desc)
        # create output dataset
        self._gen_output_dataset(self._output_size)
        # recode input data address,if need malloc memory,the memory will be
        # reuseable
        self._init_input_buffer()
        log.info("Init model resource success")

        return SUCCESS

    def _gen_output_dataset(self, ouput_num):
        log.info("[AclLiteModel] create model output dataset:")
        dataset = acl.mdl.create_dataset()
        for i in range(ouput_num):
            # malloc device memory for output
            size = acl.mdl.get_output_size_by_index(self._model_desc, i)
            buf, ret = acl.rt.malloc(size, ACL_MEM_MALLOC_NORMAL_ONLY)
            check_ret("acl.rt.malloc", ret)
            # crate oputput data buffer
            dataset_buffer = acl.create_data_buffer(buf, size)
            _, ret = acl.mdl.add_dataset_buffer(dataset, dataset_buffer)
            log.info("malloc output %d, size %d" % (i, size))
            if ret:
                acl.rt.free(buf)
                acl.destroy_data_buffer(dataset_buffer)
                check_ret("acl.destroy_data_buffer", ret)
        self._output_dataset = dataset
        log.info("Create model output dataset success")

    def _init_input_buffer(self):
        self._input_num = acl.mdl.get_num_inputs(self._model_desc)
        for i in range(self._input_num):
            item = {"addr": None, "size": 0}
            self._input_buffer.append(item)

    def _gen_input_dataset(self, input_list):
        dynamicIdx, ret = acl.mdl.get_input_index_by_name(self._model_desc, "ascend_mbatch_shape_data")
        if ret == SUCCESS:
            dataLen = acl.mdl.get_input_size_by_index(self._model_desc, dynamicIdx)
            buf, ret = acl.rt.malloc(dataLen, ACL_MEM_MALLOC_NORMAL_ONLY)
            check_ret("acl.rt.malloc", ret)
            batch_buffer = {'data': buf, 'size':dataLen}
            input_list.append(batch_buffer)

        ret = SUCCESS
        if len(input_list) != self._input_num:
            log.error("Current input data num %d unequal to model "
                      "input num %d" % (len(input_list), self._input_num))
            return FAILED

        self._input_dataset = acl.mdl.create_dataset()
        for i in range(self._input_num):
            item = input_list[i]
            data, size = self._parse_input_data(item, i)
            if (data is None) or (size == 0):
                ret = FAILED
                log.error("The %d input is invalid" % (i))
                break

            model_size = acl.mdl.get_input_size_by_index(self._model_desc, i)
            if size != model_size:
                log.warn(" Input[%d] size: %d not equal om size: %d" % (i, size, model_size) +\
                        ", may cause inference result error, please check model input")


            dataset_buffer = acl.create_data_buffer(data, size)
            _, ret = acl.mdl.add_dataset_buffer(self._input_dataset,
                                                dataset_buffer)
            if ret:
                log.error("Add input dataset buffer failed, ret = ", ret)
                acl.destroy_data_buffer(self._input_dataset)
                ret = FAILED
                break
        if ret == FAILED:
            self._release_dataset(self._input_dataset)
            self._input_dataset = None

        return ret

    def _parse_input_data(self, input_data, index):
        data = None
        size = 0
        if isinstance(input_data, np.ndarray):
            size = input_data.size * input_data.itemsize
            if "bytes_to_ptr" in dir(acl.util):
                bytes_data=input_data.tobytes()
                ptr=acl.util.bytes_to_ptr(bytes_data)
            else:
                ptr = acl.util.numpy_to_ptr(input_data)
            data = self._copy_input_to_device(ptr, size, index)
            if data is None:
                size = 0
                log.error("Copy input to device failed")
        elif (isinstance(input_data, dict) and
              ('data' in input_data.keys()) and ('size' in input_data.keys())):
            size = input_data['size']
            data = input_data['data']
        else:
            log.error("Unsupport input")

        return data, size

    def _copy_input_to_device(self, input_ptr, size, index):
        buffer_item = self._input_buffer[index]
        data = None
        if buffer_item['addr'] is None:

            data = copy_data_device_to_device(input_ptr, size)
            if data is None:
                log.error("Malloc memory and copy model %dth "
                          "input to device failed" % (index))
                return None
            buffer_item['addr'] = data
            buffer_item['size'] = size
        elif size == buffer_item['size']:

            ret = acl.rt.memcpy(buffer_item['addr'], size,
                                    input_ptr, size,
                                    ACL_MEMCPY_DEVICE_TO_DEVICE)                
            if ret != SUCCESS:
                log.error("Copy model %dth input to device failed, ret = %d" % (index, ret))
                return None
            data = buffer_item['addr']
        else:
            log.error("The model %dth input size %d is change,"
                      " before is %d" % (index, size, buffer_item['size']))
            return None

        return data

    def _set_dynamic_batch_size(self, batch):
        dynamicIdx, ret = acl.mdl.get_input_index_by_name(self._model_desc, "ascend_mbatch_shape_data")
        if ret != SUCCESS:
            log.error("get_input_index_by_name failed, ret = ", ret)
            return FAILED
        batch_dic, ret = acl.mdl.get_dynamic_batch(self._model_desc)
        if ret != SUCCESS:
            log.error("get_dynamic_batch failed, ret = ", ret)
            return FAILED
        log.info("[INFO] get dynamic_batch = ", batch_dic)
        ret = acl.mdl.set_dynamic_batch_size(self._model_id, self._input_dataset, dynamicIdx, batch)
        if ret != SUCCESS:
            log.error("set_dynamic_batch_size failed, ret = ", ret)
            return FAILED
        if batch in batch_dic["batch"]:
            return SUCCESS
        else:
            log.error("[INFO] [dynamic batch] {} is not in {}".format(batch, batch_dic["batch"]))
            return FAILED

    def _execute_with_dynamic_batch_size(self, input_list, batch):
        ret = self._gen_input_dataset(input_list)
        if ret == FAILED:
            log.error("Gen model input dataset failed")
            return None

        ret = self._set_dynamic_batch_size(batch)
        if ret == FAILED:
            log.error("Set dynamic batch failed")
            return None

        ret = acl.mdl.execute(self._model_id,
                              self._input_dataset,
                              self._output_dataset)
        if ret != SUCCESS:
            log.error("Execute model failed for acl.mdl.execute error, ret = ", ret)
            return None

        self._release_dataset(self._input_dataset)
        self._input_dataset = None

        return self._output_dataset_to_numpy()

    def execute(self, input_list):
        """
        inference input data
        Args:
            input_list: input data list, support AclLiteImage,
            numpy array and {'data': ,'size':} dict
        returns:
            inference result data, which is a numpy array list,
            each corresponse to a model output
        """
        ret = self._gen_input_dataset(input_list)
        if ret == FAILED:
            log.error("Gen model input dataset failed")
            return None

        ret = acl.mdl.execute(self._model_id,
                              self._input_dataset,
                              self._output_dataset)
        if ret != SUCCESS:
            log.error("Execute model failed for acl.mdl.execute error, ret = ", ret)
            return None

        self._release_dataset(self._input_dataset)
        self._input_dataset = None

        return self._output_dataset_to_numpy()

    def _output_dataset_to_numpy(self):
        dataset = []
        output_tensor_list = self._gen_output_tensor()
        num = acl.mdl.get_dataset_num_buffers(self._output_dataset)

        for i in range(num):
            buf = acl.mdl.get_dataset_buffer(self._output_dataset, i)
            data = acl.get_data_buffer_addr(buf)
            size = int(acl.get_data_buffer_size(buf))
            output_ptr = output_tensor_list[i]["ptr"]
            output_data = output_tensor_list[i]["tensor"]
            if isinstance (output_data,bytes):
                data_size = len(output_data)
            else:
                data_size = output_data.size * output_data.itemsize
            ret = acl.rt.memcpy(output_ptr,
                                data_size,
                                data, size, self._copy_policy)
            if ret != SUCCESS:
                log.error("Memcpy inference output to local failed, ret = ", ret)
                return None

            if isinstance (output_data,bytes):
                output_data = np.frombuffer(output_data, dtype=output_tensor_list[i]["dtype"]).reshape(output_tensor_list[i]["shape"])
                output_tensor = output_data.copy()
            else:
                output_tensor = output_data
            dataset.append(output_tensor)

        return dataset

    def _gen_output_tensor(self):
        output_tensor_list = []
        for i in range(self._output_size):
            dims = acl.mdl.get_output_dims(self._model_desc, i)
            shape = tuple(dims[0]["dims"])
            datatype = acl.mdl.get_output_data_type(self._model_desc, i)
            size = acl.mdl.get_output_size_by_index(self._model_desc, i)

            if datatype == ACL_FLOAT:
                np_type = np.float32
                output_tensor = np.zeros(
                    size // 4, dtype=np_type).reshape(shape)
            elif datatype == ACL_DOUBLE:
                np_type = np.float64
                output_tensor = np.zeros(
                    size // 8, dtype=np_type).reshape(shape)
            elif datatype == ACL_INT64:
                np_type = np.int64
                output_tensor = np.zeros(
                    size // 8, dtype=np_type).reshape(shape)
            elif datatype == ACL_UINT64:
                np_type = np.uint64
                output_tensor = np.zeros(
                    size // 8, dtype=np_type).reshape(shape)
            elif datatype == ACL_INT32:
                np_type = np.int32
                output_tensor = np.zeros(
                    size // 4, dtype=np_type).reshape(shape)
            elif datatype == ACL_UINT32:
                np_type = np.uint32
                output_tensor = np.zeros(
                    size // 4, dtype=np_type).reshape(shape)
            elif datatype == ACL_FLOAT16:
                np_type = np.float16
                output_tensor = np.zeros(
                    size // 2, dtype=np_type).reshape(shape)
            elif datatype == ACL_INT16:
                np_type = np.int16
                output_tensor = np.zeros(
                    size // 2, dtype=np_type).reshape(shape)
            elif datatype == ACL_UINT16:
                np_type = np.uint16
                output_tensor = np.zeros(
                    size // 2, dtype=np_type).reshape(shape)
            elif datatype == ACL_INT8:
                np_type = np.int8
                output_tensor = np.zeros(
                    size, dtype=np_type).reshape(shape)
            elif datatype == ACL_BOOL or datatype == ACL_UINT8:
                np_type = np.uint8
                output_tensor = np.zeros(
                    size, dtype=np_type).reshape(shape) 
            else:
                log.error("Unspport model output datatype ", datatype)
                return None

            if not output_tensor.flags['C_CONTIGUOUS']:
                output_tensor = np.ascontiguousarray(output_tensor)

            if "bytes_to_ptr" in dir(acl.util):
                bytes_data = output_tensor.tobytes()
                tensor_ptr = acl.util.bytes_to_ptr(bytes_data)
                output_tensor_list.append({"ptr": tensor_ptr,
                                        "tensor": bytes_data,
                                        "shape":output_tensor.shape,
                                        "dtype":output_tensor.dtype},)
            else:
                tensor_ptr = acl.util.numpy_to_ptr(output_tensor)
                output_tensor_list.append({"ptr": tensor_ptr,
                                       "tensor": output_tensor})

        return output_tensor_list

    def _release_dataset(self, dataset, free_memory=False):
        if not dataset:
            return

        num = acl.mdl.get_dataset_num_buffers(dataset)
        for i in range(num):
            data_buf = acl.mdl.get_dataset_buffer(dataset, i)
            if data_buf:
                self._release_databuffer(data_buf, free_memory)

        ret = acl.mdl.destroy_dataset(dataset)
        if ret != SUCCESS:
            log.error("Destroy data buffer error, ret = ", ret)

    def _release_databuffer(self, data_buffer, free_memory=False):
        if free_memory:
            data_addr = acl.get_data_buffer_addr(data_buffer)
            if data_addr:
                acl.rt.free(data_addr)

        ret = acl.destroy_data_buffer(data_buffer)
        if ret != SUCCESS:
            log.error("Destroy data buffer error, ret = ", ret)

    def destroy(self):
        """
        release resource of model inference
        Args:
            null
        Returns:
            null
        """
        if self._is_destroyed:
            return

        self._release_dataset(self._output_dataset, free_memory=True)
        if self._model_id:
            ret = acl.mdl.unload(self._model_id)
            if ret != SUCCESS:
                log.info("acl.mdl.unload error, ret = ", ret)

        if self._model_desc:
            ret = acl.mdl.destroy_desc(self._model_desc)
            if ret != SUCCESS:
                log.info("acl.mdl.destroy_desc error, ret = ", ret)

        self._is_destroyed = True
        log.info("AclLiteModel release source success")

    def __del__(self):
        self.destroy()

    @abstractmethod
    def infer(self, inputs):
        pass
