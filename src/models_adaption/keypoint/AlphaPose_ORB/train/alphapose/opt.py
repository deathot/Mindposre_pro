# -----------------------------------------------------
# Copyright (c) Shanghai Jiao Tong University. All rights reserved.
# Written by Jiefeng Li (jeff.lee.sjtu@gmail.com)
# -----------------------------------------------------
import argparse
import logging
import os
import sys
from types import MethodType

import torch
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig
from .utils.config import update_config

parser = argparse.ArgumentParser(description='AlphaPose Training')

"----------------------------- Experiment options -----------------------------"
parser.add_argument('--cfg', default='./configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml', type=str,
                    help='experiment configure file name')
parser.add_argument('--exp-id', default='default', type=str,
                    help='Experiment ID')

"----------------------------- General options -----------------------------"
parser.add_argument('--nThreads', default=4, type=int,
                    help='Number of data loading threads')
parser.add_argument('--snapshot', default=2, type=int,
                    help='How often to take a snapshot of the model (0 = never)')

parser.add_argument('--rank', default=-1, type=int,
                    help='node rank for distributed training')
parser.add_argument('--dist-url', default='tcp://192.168.1.214:23345', type=str,
                    help='url used to set up distributed training')
parser.add_argument('--dist-backend', default='nccl', type=str,
                    help='distributed backend')
parser.add_argument('--launcher', choices=['none', 'pytorch', 'slurm', 'mpi'], default='none',
                    help='job launcher')

"----------------------------- Training options -----------------------------"
parser.add_argument('--sync', default=False, dest='sync',
                    help='Use Sync Batchnorm', action='store_true')
parser.add_argument('--detector', dest='detector',
                    help='detector name', default="yolo")

"----------------------------- Log options -----------------------------"
parser.add_argument('--board', default=True, dest='board',
                    help='Logging with tensorboard', action='store_true')
parser.add_argument('--debug', default=False, dest='debug',
                    help='Visualization debug', action='store_true')
parser.add_argument('--map', default=True, dest='map',
                    help='Evaluate mAP per epoch', action='store_true')

"----------------------------- GUI parameters options -----------------------------"
parser.add_argument('--input_path',type=str,
                    default='', help='input path')
parser.add_argument('--output_path', type=str,
                    default='', help='output path')
parser.add_argument('--batch_size', type=int,
                    default=2, help='train batch size')
parser.add_argument('--end_epoch', type=int,
                    default=2, help='train end epoch')
parser.add_argument('--pretrained', type=str,
                    default='', help='pretrained model')
parser.add_argument('--try_load', type=str,
                    default='', help='try_load model')
parser.add_argument('--weights', type=str,
                    default='', help='model weights')                    
parser.add_argument('--input_pth', type=str,
                    default='', help='input pth model')
parser.add_argument('--output_onnx', type=str,
                    default='', help='output onnx model')
parser.add_argument('--output_exist', type=bool,
                    default=True, help='output directory')

opt = parser.parse_args()
cfg_file_name = os.path.basename(opt.cfg)
cfg = update_config(opt.cfg)

cfg['FILE_NAME'] = cfg_file_name
cfg.TRAIN.DPG_STEP = [i - cfg.TRAIN.DPG_MILESTONE for i in cfg.TRAIN.DPG_STEP]
opt.world_size = cfg.TRAIN.WORLD_SIZE
opt.work_dir = '{}/'.format(opt.exp_id)
opt.gpus = [i for i in range(torch.cuda.device_count())]
#opt.device = torch.device("cuda:" + str(opt.gpus[0]) if opt.gpus[0] >= 0 else "cpu")
opt.device = "cpu"

yaml_file = '../../../config/config.yaml'
config = ModelConfig(yaml_file)
yaml_data = config.get_yaml_data().get('keypoint')
opt.output_path = os.path.join(yaml_data['output_path'], 'train_output')
config.set_para('keypoint', 'train_output', opt.output_path)
config.yaml_dump(yaml_file)
if not os.path.exists("{}/data.yaml".format(opt.output_path)):
    os.makedirs("{}/data.yaml".format(opt.output_path))

filehandler = logging.FileHandler(
    '{}/data.yaml/training.log'.format(opt.output_path))
streamhandler = logging.StreamHandler()

logger = logging.getLogger('')
logger.setLevel(logging.INFO)
logger.addHandler(filehandler)
logger.addHandler(streamhandler)


def epochInfo(self, set, idx, loss, acc):
    self.info('{set}-{idx:d} epoch | loss:{loss:.8f} | acc:{acc:.4f}'.format(
        set=set,
        idx=idx,
        loss=loss,
        acc=acc
    ))


logger.epochInfo = MethodType(epochInfo, logger)
