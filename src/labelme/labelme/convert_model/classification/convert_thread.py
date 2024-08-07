import os
from PyQt5.QtCore import QThread, pyqtSignal
from src.labelme.labelme.run_cmd_utils import cmd_run

import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig

here = os.path.dirname(__file__).replace('\\', '/') + '/'


class ConvertThread(QThread):
    thread_end = pyqtSignal()
    process_value = pyqtSignal(int)
    ret_value = pyqtSignal(str)
    status_value = pyqtSignal(str)

    def __init__(self, ds_data, ratio_data, epoch_data, batch_data, output_data,
                 earlystopping_enabled: bool,
                 earlystopping_threshold: float,
                 earlystopping_tolerance: int):
        super(ConvertThread, self).__init__()
        self.ds_data = ds_data
        self.ratio_data = ratio_data
        self.epoch_data = epoch_data
        self.batch_data = batch_data
        self.output_data = output_data

        self.earlystopping_enabled = earlystopping_enabled
        self.earlystopping_threshold = earlystopping_threshold
        self.earlystopping_tolerance = earlystopping_tolerance

        yaml_file = os.path.join(here, '../../../../models_adaption/config/config.yaml')
        config = ModelConfig(yaml_file)
        config.set_para('classification', 'labelme_dataset_path', self.ds_data)
        config.set_para('classification', 'class_txt_path', os.path.join(os.path.dirname(self.ds_data), 'flags.txt'))
        with open(os.path.join(os.path.dirname(self.ds_data), 'flags.txt'), 'r') as f:
            class_num = len(f.readlines())
        config.set_para('classification', 'class_num', class_num)
        config.set_para('classification', 'split_ratio', float(self.ratio_data))
        config.set_para('classification', 'epochs', self.epoch_data)
        config.set_para('classification', 'train_batch_size', self.batch_data)
        config.set_para('classification', 'output_path', self.output_data)
        config.set_para('classification', 'earlystopping_enabled', self.earlystopping_enabled)
        config.set_para('classification', 'earlystopping_threshold', self.earlystopping_threshold)
        config.set_para('classification', 'earlystopping_tolerance', self.earlystopping_tolerance)
        config.yaml_dump(yaml_file)

    def run(self):
        # step1 --> trans
        self.status_value.emit(self.tr("Data transforming..."))
        trans_bat_file = os.path.join(here, "trans_cmd.bat")
        if cmd_run(trans_bat_file, self.process_value, self.ret_value):
            self.status_value.emit(self.tr("Data transfer failed!"))
            self.thread_end.emit()
            return
        else:
            self.status_value.emit(self.tr("Data training..."))
        # step2 --> train
        train_bat_file = os.path.join(here, "train_cmd.bat")
        if cmd_run(train_bat_file, self.process_value, self.ret_value):
            self.status_value.emit(self.tr("Data transfer failed!"))
            self.thread_end.emit()
            return
        else:
            self.status_value.emit(self.tr("Data converting..."))
        # step3 --> convert
        convert_bat_file = os.path.join(here, "convert_cmd.bat")
        if cmd_run(convert_bat_file, self.process_value, self.ret_value):
            self.status_value.emit(self.tr("Data transfer failed!"))
            self.thread_end.emit()
            return
        else:
            self.status_value.emit(self.tr("Data transfer success!"))
        self.thread_end.emit()
