import argparse
import glob
import math
import os
import time
from pathlib import Path
import random

import numpy as np
import torch.distributed as dist
import torch.nn.functional as F
import torch.optim as optim
import torch.optim.lr_scheduler as lr_scheduler
import torch.utils.data
import yaml
# from torch.cuda import amp
from torch.nn.parallel import DistributedDataParallel as DDP
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

import test  # import test.py to get mAP after each epoch
from models.yolo import Model
from utils.datasets import create_dataloader
from utils.general import (
    check_img_size, torch_distributed_zero_first, labels_to_class_weights, plot_labels, check_anchors,
    labels_to_image_weights, compute_loss, plot_images, fitness, strip_optimizer, plot_results,
    get_latest_run, check_file, increment_dir, print_mutation, plot_evolution)
from utils.torch_utils import init_torch_seeds, ModelEMA, select_device
from utils.prune_utils import parse_module_defs,parse_module_defs2,gather_bn_weights,get_sr_flag,BNOptimizer

from modelsori import *

# Hyperparameters
hyp = {'lr0': 0.001,  # initial learning rate (SGD=1E-2, Adam=1E-3)
       'momentum': 0.937,  # SGD momentum/Adam beta1
       'weight_decay': 5e-4,  # optimizer weight decay
       'giou': 0.05,  # GIoU loss gain
       'cls': 0.5,  # cls loss gain
       'cls_pw': 1.0,  # cls BCELoss positive_weight
       'obj': 1.0,  # obj loss gain (scale with pixels)
       'obj_pw': 1.0,  # obj BCELoss positive_weight
       'iou_t': 0.20,  # IoU training threshold
       'anchor_t': 4.0,  # anchor-multiple threshold
       'fl_gamma': 0.0,  # focal loss gamma (efficientDet default gamma=1.5)
       'hsv_h': 0.015,  # image HSV-Hue augmentation (fraction)
       'hsv_s': 0.7,  # image HSV-Saturation augmentation (fraction)
       'hsv_v': 0.4,  # image HSV-Value augmentation (fraction)
       'degrees': 0.5,  # image rotation (+/- deg)
       'translate': 0.5,  # image translation (+/- fraction)
       'scale': 0.5,  # image scale (+/- gain)
       'shear': 0.5,  # image shear (+/- deg)
       'perspective': 0.0005,  # image perspective (+/- fraction), range 0-0.001
       'flipud': 0.5,  # image flip up-down (probability)
       'fliplr': 0.5,  # image flip left-right (probability)
       'mixup': 0.5
       }  # image mixup (probability)

def copy_weight(modelyolov5,model):
    focus = list(modelyolov5.model.children())[0]
    model.module_list[1][0] = focus.conv.conv
    model.module_list[1][1] = focus.conv.bn
    model.module_list[1][2] = focus.conv.act
    conv1 = list(modelyolov5.model.children())[1]
    model.module_list[2][0] = conv1.conv
    model.module_list[2][1] = conv1.bn
    model.module_list[2][2] = conv1.act
    cspnet1 = list(modelyolov5.model.children())[2]
    model.module_list[3][0] = cspnet1.cv2
    model.module_list[5][0] = cspnet1.cv1.conv
    model.module_list[5][1] = cspnet1.cv1.bn
    model.module_list[5][2] = cspnet1.cv1.act
    model.module_list[9][0] = cspnet1.cv3
    model.module_list[11][0] = cspnet1.bn
    model.module_list[11][1] = cspnet1.act
    model.module_list[6][0] = cspnet1.m[0].cv1.conv
    model.module_list[6][1] = cspnet1.m[0].cv1.bn
    model.module_list[6][2] = cspnet1.m[0].cv1.act
    model.module_list[7][0] = cspnet1.m[0].cv2.conv
    model.module_list[7][1] = cspnet1.m[0].cv2.bn
    model.module_list[7][2] = cspnet1.m[0].cv2.act
    model.module_list[12][0] = cspnet1.cv4.conv
    model.module_list[12][1] = cspnet1.cv4.bn
    model.module_list[12][2] = cspnet1.cv4.act
    conv2 = list(modelyolov5.model.children())[3]
    model.module_list[13][0] = conv2.conv
    model.module_list[13][1] = conv2.bn
    model.module_list[13][2] = conv2.act
    cspnet2 = list(modelyolov5.model.children())[4]
    model.module_list[14][0] = cspnet2.cv2
    model.module_list[16][0] = cspnet2.cv1.conv
    model.module_list[16][1] = cspnet2.cv1.bn
    model.module_list[16][2] = cspnet2.cv1.act
    model.module_list[26][0] = cspnet2.cv3
    model.module_list[28][0] = cspnet2.bn
    model.module_list[28][1] = cspnet2.act
    model.module_list[29][0] = cspnet2.cv4.conv
    model.module_list[29][1] = cspnet2.cv4.bn
    model.module_list[29][2] = cspnet2.cv4.act
    model.module_list[17][0] = cspnet2.m[0].cv1.conv
    model.module_list[17][1] = cspnet2.m[0].cv1.bn
    model.module_list[17][2] = cspnet2.m[0].cv1.act
    model.module_list[18][0] = cspnet2.m[0].cv2.conv
    model.module_list[18][1] = cspnet2.m[0].cv2.bn
    model.module_list[18][2] = cspnet2.m[0].cv2.act
    model.module_list[20][0] = cspnet2.m[1].cv1.conv
    model.module_list[20][1] = cspnet2.m[1].cv1.bn
    model.module_list[20][2] = cspnet2.m[1].cv1.act
    model.module_list[21][0] = cspnet2.m[1].cv2.conv
    model.module_list[21][1] = cspnet2.m[1].cv2.bn
    model.module_list[21][2] = cspnet2.m[1].cv2.act
    model.module_list[23][0] = cspnet2.m[2].cv1.conv
    model.module_list[23][1] = cspnet2.m[2].cv1.bn
    model.module_list[23][2] = cspnet2.m[2].cv1.act
    model.module_list[24][0] = cspnet2.m[2].cv2.conv
    model.module_list[24][1] = cspnet2.m[2].cv2.bn
    model.module_list[24][2] = cspnet2.m[2].cv2.act
    conv3 = list(modelyolov5.model.children())[5]
    model.module_list[30][0] = conv3.conv
    model.module_list[30][1] = conv3.bn
    model.module_list[30][2] = conv3.act
    cspnet3 = list(modelyolov5.model.children())[6]
    model.module_list[31][0] = cspnet3.cv2
    model.module_list[33][0] = cspnet3.cv1.conv
    model.module_list[33][1] = cspnet3.cv1.bn
    model.module_list[33][2] = cspnet3.cv1.act
    model.module_list[43][0] = cspnet3.cv3
    model.module_list[45][0] = cspnet3.bn
    model.module_list[45][1] = cspnet3.act
    model.module_list[46][0] = cspnet3.cv4.conv
    model.module_list[46][1] = cspnet3.cv4.bn
    model.module_list[46][2] = cspnet3.cv4.act
    model.module_list[34][0] = cspnet3.m[0].cv1.conv
    model.module_list[34][1] = cspnet3.m[0].cv1.bn
    model.module_list[34][2] = cspnet3.m[0].cv1.act
    model.module_list[35][0] = cspnet3.m[0].cv2.conv
    model.module_list[35][1] = cspnet3.m[0].cv2.bn
    model.module_list[35][2] = cspnet3.m[0].cv2.act
    model.module_list[37][0] = cspnet3.m[1].cv1.conv
    model.module_list[37][1] = cspnet3.m[1].cv1.bn
    model.module_list[37][2] = cspnet3.m[1].cv1.act
    model.module_list[38][0] = cspnet3.m[1].cv2.conv
    model.module_list[38][1] = cspnet3.m[1].cv2.bn
    model.module_list[38][2] = cspnet3.m[1].cv2.act
    model.module_list[40][0] = cspnet3.m[2].cv1.conv
    model.module_list[40][1] = cspnet3.m[2].cv1.bn
    model.module_list[40][2] = cspnet3.m[2].cv1.act
    model.module_list[41][0] = cspnet3.m[2].cv2.conv
    model.module_list[41][1] = cspnet3.m[2].cv2.bn
    model.module_list[41][2] = cspnet3.m[2].cv2.act
    conv4 = list(modelyolov5.model.children())[7]
    model.module_list[47][0] = conv4.conv
    model.module_list[47][1] = conv4.bn
    model.module_list[47][2] = conv4.act
    spp = list(modelyolov5.model.children())[8]
    model.module_list[48][0] = spp.cv1.conv
    model.module_list[48][1] = spp.cv1.bn
    model.module_list[48][2] = spp.cv1.act
    model.module_list[49] = spp.m[0]
    model.module_list[51] = spp.m[1]
    model.module_list[53] = spp.m[2]
    model.module_list[55][0] = spp.cv2.conv
    model.module_list[55][1] = spp.cv2.bn
    model.module_list[55][2] = spp.cv2.act
    cspnet4 = list(modelyolov5.model.children())[9]
    model.module_list[56][0] = cspnet4.cv2
    model.module_list[58][0] = cspnet4.cv1.conv
    model.module_list[58][1] = cspnet4.cv1.bn
    model.module_list[58][2] = cspnet4.cv1.act
    model.module_list[61][0] = cspnet4.cv3
    model.module_list[63][0] = cspnet4.bn
    model.module_list[63][1] = cspnet4.act
    model.module_list[64][0] = cspnet4.cv4.conv
    model.module_list[64][1] = cspnet4.cv4.bn
    model.module_list[64][2] = cspnet4.cv4.act
    model.module_list[59][0] = cspnet4.m[0].cv1.conv
    model.module_list[59][1] = cspnet4.m[0].cv1.bn
    model.module_list[59][2] = cspnet4.m[0].cv1.act
    model.module_list[60][0] = cspnet4.m[0].cv2.conv
    model.module_list[60][1] = cspnet4.m[0].cv2.bn
    model.module_list[60][2] = cspnet4.m[0].cv2.act
    conv5 = list(modelyolov5.model.children())[10]
    model.module_list[65][0] = conv5.conv
    model.module_list[65][1] = conv5.bn
    model.module_list[65][2] = conv5.act
    upsample1 = list(modelyolov5.model.children())[11]
    model.module_list[66] = upsample1
    cspnet5 = list(modelyolov5.model.children())[13]
    model.module_list[68][0] = cspnet5.cv2
    model.module_list[70][0] = cspnet5.cv1.conv
    model.module_list[70][1] = cspnet5.cv1.bn
    model.module_list[70][2] = cspnet5.cv1.act
    model.module_list[73][0] = cspnet5.cv3
    model.module_list[75][0] = cspnet5.bn
    model.module_list[75][1] = cspnet5.act
    model.module_list[76][0] = cspnet5.cv4.conv
    model.module_list[76][1] = cspnet5.cv4.bn
    model.module_list[76][2] = cspnet5.cv4.act
    model.module_list[71][0] = cspnet5.m[0].cv1.conv
    model.module_list[71][1] = cspnet5.m[0].cv1.bn
    model.module_list[71][2] = cspnet5.m[0].cv1.act
    model.module_list[72][0] = cspnet5.m[0].cv2.conv
    model.module_list[72][1] = cspnet5.m[0].cv2.bn
    model.module_list[72][2] = cspnet5.m[0].cv2.act
    conv6 = list(modelyolov5.model.children())[14]
    model.module_list[77][0] = conv6.conv
    model.module_list[77][1] = conv6.bn
    model.module_list[77][2] = conv6.act
    upsample2 = list(modelyolov5.model.children())[15]
    model.module_list[78] = upsample2
    cspnet6 = list(modelyolov5.model.children())[17]
    model.module_list[80][0] = cspnet6.cv2
    model.module_list[82][0] = cspnet6.cv1.conv
    model.module_list[82][1] = cspnet6.cv1.bn
    model.module_list[82][2] = cspnet6.cv1.act
    model.module_list[85][0] = cspnet6.cv3
    model.module_list[87][0] = cspnet6.bn
    model.module_list[87][1] = cspnet6.act
    model.module_list[88][0] = cspnet6.cv4.conv
    model.module_list[88][1] = cspnet6.cv4.bn
    model.module_list[88][2] = cspnet6.cv4.act
    model.module_list[83][0] = cspnet6.m[0].cv1.conv
    model.module_list[83][1] = cspnet6.m[0].cv1.bn
    model.module_list[83][2] = cspnet6.m[0].cv1.act
    model.module_list[84][0] = cspnet6.m[0].cv2.conv
    model.module_list[84][1] = cspnet6.m[0].cv2.bn
    model.module_list[84][2] = cspnet6.m[0].cv2.act
    conv7 = list(modelyolov5.model.children())[18]
    model.module_list[92][0] = conv7.conv
    model.module_list[92][1] = conv7.bn
    model.module_list[92][2] = conv7.act
    cspnet7 = list(modelyolov5.model.children())[20]
    model.module_list[94][0] = cspnet7.cv2
    model.module_list[96][0] = cspnet7.cv1.conv
    model.module_list[96][1] = cspnet7.cv1.bn
    model.module_list[96][2] = cspnet7.cv1.act
    model.module_list[99][0] = cspnet7.cv3
    model.module_list[101][0] = cspnet7.bn
    model.module_list[101][1] = cspnet7.act
    model.module_list[102][0] = cspnet7.cv4.conv
    model.module_list[102][1] = cspnet7.cv4.bn
    model.module_list[102][2] = cspnet7.cv4.act
    model.module_list[97][0] = cspnet7.m[0].cv1.conv
    model.module_list[97][1] = cspnet7.m[0].cv1.bn
    model.module_list[97][2] = cspnet7.m[0].cv1.act
    model.module_list[98][0] = cspnet7.m[0].cv2.conv
    model.module_list[98][1] = cspnet7.m[0].cv2.bn
    model.module_list[98][2] = cspnet7.m[0].cv2.act
    conv8 = list(modelyolov5.model.children())[21]
    model.module_list[106][0] = conv8.conv
    model.module_list[106][1] = conv8.bn
    model.module_list[106][2] = conv8.act
    cspnet8 = list(modelyolov5.model.children())[23]
    model.module_list[108][0] = cspnet8.cv2
    model.module_list[110][0] = cspnet8.cv1.conv
    model.module_list[110][1] = cspnet8.cv1.bn
    model.module_list[110][2] = cspnet8.cv1.act
    model.module_list[113][0] = cspnet8.cv3
    model.module_list[115][0] = cspnet8.bn
    model.module_list[115][1] = cspnet8.act
    model.module_list[116][0] = cspnet8.cv4.conv
    model.module_list[116][1] = cspnet8.cv4.bn
    model.module_list[116][2] = cspnet8.cv4.act
    model.module_list[111][0] = cspnet8.m[0].cv1.conv
    model.module_list[111][1] = cspnet8.m[0].cv1.bn
    model.module_list[111][2] = cspnet8.m[0].cv1.act
    model.module_list[112][0] = cspnet8.m[0].cv2.conv
    model.module_list[112][1] = cspnet8.m[0].cv2.bn
    model.module_list[112][2] = cspnet8.m[0].cv2.act
    detect = list(modelyolov5.model.children())[24]
    model.module_list[89][0] = detect.m[0]
    model.module_list[103][0] = detect.m[1]
    model.module_list[117][0] = detect.m[2]

def train(hyp, opt, device, tb_writer=None):
    print(f'Hyperparameters {hyp}')
    log_dir = tb_writer.log_dir if tb_writer else 'runs/evolve'  # run directory
    wdir = str(Path(log_dir) / 'weights') + os.sep  # weights directory
    os.makedirs(wdir, exist_ok=True)
    last = wdir + 'last.pt'
    best = wdir + 'best.pt'
    results_file = log_dir + os.sep + 'results.txt'
    epochs, batch_size, total_batch_size, weights, rank = \
        opt.epochs, opt.batch_size, opt.total_batch_size, opt.weights, opt.local_rank
    # TODO: Use DDP logging. Only the first process is allowed to log.

    # Save run settings
    with open(Path(log_dir) / 'hyp.yaml', 'w') as f:
        yaml.dump(hyp, f, sort_keys=False)
    with open(Path(log_dir) / 'opt.yaml', 'w') as f:
        yaml.dump(vars(opt), f, sort_keys=False)

    # Configure
    cuda = device.type != 'cpu'
    init_torch_seeds(2 + rank)
    with open(opt.data, encoding='UTF-8') as f:
        data_dict = yaml.load(f, Loader=yaml.FullLoader)  # model dict
    train_path = data_dict['train']
    test_path = data_dict['val']
    nc, names = (1, ['item']) if opt.single_cls else (int(data_dict['nc']), data_dict['names'])  # number classes, names
    assert len(names) == nc, '%g names found for nc=%g dataset in %s' % (len(names), nc, opt.data)  # check

    # Remove previous results
    if rank in [-1, 0]:
        for f in glob.glob('*_batch*.jpg') + glob.glob(results_file):
            os.remove(f)

    # Create model
    model = Model(opt.cfg, nc=nc).to(device)
    #TODO 将cfg添加到配置变量中
    cfg_model = Darknet('cfg/yolov5s_v2.cfg', (opt.img_size[0], opt.img_size[0])).to(device)

    # Image sizes
    gs = int(max(model.stride))  # grid size (max stride)
    imgsz, imgsz_test = [check_img_size(x, gs) for x in opt.img_size]  # verify imgsz are gs-multiples

    # Optimizer
    nbs = 64  # nominal batch size
    # default DDP implementation is slow for accumulation according to: https://pytorch.org/docs/stable/notes/ddp.html
    # all-reduce operation is carried out during loss.backward().
    # Thus, there would be redundant all-reduce communications in a accumulation procedure,
    # which means, the result is still right but the training speed gets slower.
    # TODO: If acceleration is needed, there is an implementation of allreduce_post_accumulation
    # in https://github.com/NVIDIA/DeepLearningExamples/blob/master/PyTorch/LanguageModeling/BERT/run_pretraining.py
    accumulate = max(round(nbs / total_batch_size), 1)  # accumulate loss before optimizing
    hyp['weight_decay'] *= total_batch_size * accumulate / nbs  # scale weight_decay

    pg0, pg1, pg2 = [], [], []  # optimizer parameter groups
    for k, v in model.named_parameters():
        if v.requires_grad:
            if '.bias' in k:
                pg2.append(v)  # biases
            elif '.weight' in k and '.bn' not in k:
                pg1.append(v)  # apply weight decay
            else:
                pg0.append(v)  # all else

    if opt.adam:
        optimizer = optim.Adam(pg0, lr=hyp['lr0'], betas=(hyp['momentum'], 0.999))  # adjust beta1 to momentum
    else:
        optimizer = optim.SGD(pg0, lr=hyp['lr0'], momentum=hyp['momentum'], nesterov=True)

    optimizer.add_param_group({'params': pg1, 'weight_decay': hyp['weight_decay']})  # add pg1 with weight_decay
    optimizer.add_param_group({'params': pg2})  # add pg2 (biases)
    print('Optimizer groups: %g .bias, %g conv.weight, %g other' % (len(pg2), len(pg1), len(pg0)))
    del pg0, pg1, pg2

    # Scheduler https://arxiv.org/pdf/1812.01187.pdf
    # https://pytorch.org/docs/stable/_modules/torch/optim/lr_scheduler.html#OneCycleLR
    lf = lambda x: (((1 + math.cos(x * math.pi / epochs)) / 2) ** 1.0) * 0.8 + 0.2  # cosine
    scheduler = lr_scheduler.LambdaLR(optimizer, lr_lambda=lf)
    # plot_lr_scheduler(optimizer, scheduler, epochs)

    # Load Model
    with torch_distributed_zero_first(rank):
        attempt_download(weights)
    start_epoch, best_fitness = 0, 0.0
    if weights.endswith('.pt'):  # pytorch format
        ckpt = torch.load(weights, map_location=device)  # load checkpoint

        # load model
        try:
            exclude = ['anchor']  # exclude keys
            ckpt['model'] = {k: v for k, v in ckpt['model'].float().state_dict().items()
                             if k in model.state_dict() and not any(x in k for x in exclude)
                             and model.state_dict()[k].shape == v.shape}
            model.load_state_dict(ckpt['model'], strict=False)
            print('Transferred %g/%g items from %s' % (len(ckpt['model']), len(model.state_dict()), weights))
        except KeyError as e:
            s = "%s is not compatible with %s. This may be due to model differences or %s may be out of date. " \
                "Please delete or update %s and try again, or use --weights '' to train from scratch." \
                % (weights, opt.cfg, weights, weights)
            raise KeyError(s) from e

        # load optimizer
        if ckpt['optimizer'] is not None:
            optimizer.load_state_dict(ckpt['optimizer'])
            best_fitness = ckpt['best_fitness']

        # load results
        if ckpt.get('training_results') is not None:
            with open(results_file, 'w') as file:
                file.write(ckpt['training_results'])  # write results.txt

        # epochs
        start_epoch = ckpt['epoch'] + 1
        if epochs < start_epoch:
            print('%s has been trained for %g epochs. Fine-tuning for %g additional epochs.' %
                  (weights, ckpt['epoch'], epochs))
            epochs += ckpt['epoch']  # finetune additional epochs

        del ckpt

    #复制模型权重 目前只支持yolov5s
    copy_weight(model, cfg_model)
    # 剪枝操作  sr开启稀疏训练  prune 不同的剪枝策略
    # 剪枝操作
    if opt.prune == 1:
        CBL_idx, _, prune_idx, shortcut_idx, _ = parse_module_defs2(cfg_model.module_defs)
        if opt.sr:
            print('shortcut sparse training')
    elif opt.prune == 0:
        CBL_idx, _, prune_idx = parse_module_defs(cfg_model.module_defs)
        if opt.sr:
            print('normal sparse training ')

    # DP mode
    if cuda and rank == -1 and torch.cuda.device_count() > 1:
        model = torch.nn.DataParallel(model)

    # SyncBatchNorm
    if opt.sync_bn and cuda and rank != -1:
        model = torch.nn.SyncBatchNorm.convert_sync_batchnorm(model).to(device)
        print('Using SyncBatchNorm()')

    # Exponential moving average
    ema = ModelEMA(model) if rank in [-1, 0] else None

    # DDP mode
    if cuda and rank != -1:
        model = DDP(model, device_ids=[rank], output_device=rank)

    # Trainloader
    dataloader, dataset = create_dataloader(train_path, imgsz, batch_size, gs, opt, hyp=hyp, augment=True,
                                            cache=opt.cache_images, rect=opt.rect, local_rank=rank,
                                            world_size=opt.world_size)
    mlc = np.concatenate(dataset.labels, 0)[:, 0].max()  # max label class
    nb = len(dataloader)  # number of batches
    assert mlc < nc, 'Label class %g exceeds nc=%g in %s. Possible class labels are 0-%g' % (mlc, nc, opt.data, nc - 1)

    # Testloader
    if rank in [-1, 0]:
        # local_rank is set to -1. Because only the first process is expected to do evaluation.
        testloader = create_dataloader(test_path, imgsz_test, total_batch_size, gs, opt, hyp=hyp, augment=False,
                                       cache=opt.cache_images, rect=True, local_rank=-1, world_size=opt.world_size)[0]

    # Model parameters
    hyp['cls'] *= nc / 80.  # scale coco-tuned hyp['cls'] to current dataset
    model.nc = nc  # attach number of classes to model
    model.hyp = hyp  # attach hyperparameters to model
    model.gr = 1.0  # giou loss ratio (obj_loss = 1.0 or giou)
    model.class_weights = labels_to_class_weights(dataset.labels, nc).to(device)  # attach class weights
    model.names = names

    # Class frequency
    if rank in [-1, 0]:
        labels = np.concatenate(dataset.labels, 0)
        c = torch.tensor(labels[:, 0])  # classes
        # cf = torch.bincount(c.long(), minlength=nc) + 1.
        # model._initialize_biases(cf.to(device))
        plot_labels(labels, save_dir=log_dir)
        if tb_writer:
            # tb_writer.add_hparams(hyp, {})  # causes duplicate https://github.com/ultralytics/yolov5/pull/384
            tb_writer.add_histogram('classes', c, 0)

        # Check anchors
        if not opt.noautoanchor:
            check_anchors(dataset, model=model, thr=hyp['anchor_t'], imgsz=imgsz)

    #剪枝前bn层权重
    for idx in prune_idx:
        bn_weights = gather_bn_weights(cfg_model.module_list, [idx])
        tb_writer.add_histogram('before_train_perlayer_bn_weights/hist', bn_weights.numpy(), idx, bins='doane')

    # Start training
    t0 = time.time()
    nw = max(3 * nb, 1e3)  # number of warmup iterations, max(3 epochs, 1k iterations)
    # nw = min(nw, (epochs - start_epoch) / 2 * nb)  # limit warmup to < 1/2 of training
    maps = np.zeros(nc)  # mAP per class
    results = (0, 0, 0, 0, 0, 0, 0)  # 'P', 'R', 'mAP', 'F1', 'val GIoU', 'val Objectness', 'val Classification'
    scheduler.last_epoch = start_epoch - 1  # do not move
    # scaler = amp.GradScaler(enabled=cuda)
    if rank in [0, -1]:
        print('Image sizes %g train, %g test' % (imgsz, imgsz_test))
        print('Using %g dataloader workers' % dataloader.num_workers)
        print('Starting training for %g epochs...' % epochs)
    # torch.autograd.set_detect_anomaly(True)
    for epoch in range(start_epoch, epochs):  # epoch ------------------------------------------------------------------
        model.train()

        # Update image weights (optional)
        if dataset.image_weights:
            # Generate indices
            if rank in [-1, 0]:
                w = model.class_weights.cpu().numpy() * (1 - maps) ** 2  # class weights
                image_weights = labels_to_image_weights(dataset.labels, nc=nc, class_weights=w)
                dataset.indices = random.choices(range(dataset.n), weights=image_weights,
                                                 k=dataset.n)  # rand weighted idx
            # Broadcast if DDP
            if rank != -1:
                indices = torch.zeros([dataset.n], dtype=torch.int)
                if rank == 0:
                    indices[:] = torch.from_tensor(dataset.indices, dtype=torch.int)
                dist.broadcast(indices, 0)
                if rank != 0:
                    dataset.indices = indices.cpu().numpy()

        # Update mosaic border
        # b = int(random.uniform(0.25 * imgsz, 0.75 * imgsz + gs) // gs * gs)
        # dataset.mosaic_border = [b - imgsz, -b]  # height, width borders

        mloss = torch.zeros(4, device=device)  # mean losses
        if rank != -1:
            dataloader.sampler.set_epoch(epoch)
        pbar = enumerate(dataloader)
        if rank in [-1, 0]:
            print(('\n' + '%10s' * 8) % ('Epoch', 'gpu_mem', 'GIoU', 'obj', 'cls', 'total', 'targets', 'img_size'))
            pbar = tqdm(pbar, total=nb)  # progress bar
        optimizer.zero_grad()
        sr_flag = get_sr_flag(epoch, opt.sr)
        for i, (imgs, targets, paths, _) in pbar:  # batch -------------------------------------------------------------
            ni = i + nb * epoch  # number integrated batches (since train start)
            imgs = imgs.to(device, non_blocking=True).float() / 255.0  # uint8 to float32, 0-255 to 0.0-1.0
            # Warmup
            if ni <= nw:
                xi = [0, nw]  # x interp
                # model.gr = np.interp(ni, xi, [0.0, 1.0])  # giou loss ratio (obj_loss = 1.0 or giou)
                accumulate = max(1, np.interp(ni, xi, [1, nbs / total_batch_size]).round())
                for j, x in enumerate(optimizer.param_groups):
                    # bias lr falls from 0.1 to lr0, all other lrs rise from 0.0 to lr0
                    x['lr'] = np.interp(ni, xi, [0.1 if j == 2 else 0.0, x['initial_lr'] * lf(epoch)])
                    if 'momentum' in x:
                        x['momentum'] = np.interp(ni, xi, [0.9, hyp['momentum']])

            # Multi-scale
            if opt.multi_scale:
                sz = random.randrange(imgsz * 0.5, imgsz * 1.5 + gs) // gs * gs  # size
                sf = sz / max(imgs.shape[2:])  # scale factor
                if sf != 1:
                    ns = [math.ceil(x * sf / gs) * gs for x in imgs.shape[2:]]  # new shape (stretched to gs-multiple)
                    imgs = F.interpolate(imgs, size=ns, mode='bilinear', align_corners=False)

            # Forward
            pred = model(imgs)

            # Loss
            loss, loss_items = compute_loss(pred, targets.to(device), model)  # scaled by batch_size
            if rank != -1:
                loss *= opt.world_size  # gradient averaged between devices in DDP mode
            if not torch.isfinite(loss):
                print('WARNING: non-finite loss, ending training ', loss_items)
                return results

            # Backward
            loss.backward()

            idx2mask = None
            # if opt.sr and opt.prune==1 and epoch > opt.epochs * 0.5:
            #     idx2mask = get_mask2(model, prune_idx, 0.85)
            # copy_weight(model,cfg_model)
            BNOptimizer.updateBN(sr_flag, cfg_model.module_list, opt.s, prune_idx, epoch, idx2mask, opt)

            # Optimize
            if ni % accumulate == 0:
                # scaler.step(optimizer)  # optimizer.step
                # scaler.update()
                optimizer.step()
                optimizer.zero_grad()
                if ema is not None:
                    ema.update(model)

            # Print
            if rank in [-1, 0]:
                mloss = (mloss * i + loss_items) / (i + 1)  # update mean losses
                mem = '%.3gG' % (torch.cuda.memory_reserved() / 1E9 if torch.cuda.is_available() else 0)  # (GB)
                s = ('%10s' * 2 + '%10.4g' * 6) % (
                    '%g/%g' % (epoch, epochs - 1), mem, *mloss, targets.shape[0], imgs.shape[-1])
                pbar.set_description(s)

                # Plot
                if ni < 3:
                    f = str(Path(log_dir) / ('train_batch%g.jpg' % ni))  # filename
                    result = plot_images(images=imgs, targets=targets, paths=paths, fname=f)
                    if tb_writer and result is not None:
                        tb_writer.add_image(f, result, dataformats='HWC', global_step=epoch)
                        # tb_writer.add_graph(model, imgs)  # add model to tensorboard

            # end batch ------------------------------------------------------------------------------------------------

        # Scheduler
        scheduler.step()

        # DDP process 0 or single-GPU
        if rank in [-1, 0]:
            # mAP
            if ema is not None:
                ema.update_attr(model, include=['yaml', 'nc', 'hyp', 'gr', 'names', 'stride'])
            final_epoch = epoch + 1 == epochs
            if not opt.notest or final_epoch:  # Calculate mAP
                results, maps, times = test.test(opt.data,
                                                 batch_size=total_batch_size,
                                                 imgsz=imgsz_test,
                                                 save_json=final_epoch and opt.data.endswith(os.sep + 'coco.yaml'),
                                                 model=ema.ema.module if hasattr(ema.ema, 'module') else ema.ema,
                                                 single_cls=opt.single_cls,
                                                 dataloader=testloader,
                                                 save_dir=log_dir)

            # Write
            with open(results_file, 'a') as f:
                f.write(s + '%10.4g' * 7 % results + '\n')  # P, R, mAP, F1, test_losses=(GIoU, obj, cls)
            if len(opt.name) and opt.bucket:
                os.system('gsutil cp %s gs://%s/results/results%s.txt' % (results_file, opt.bucket, opt.name))

            # Tensorboard
            if tb_writer:
                tags = ['train/giou_loss', 'train/obj_loss', 'train/cls_loss',
                        'metrics/precision', 'metrics/recall', 'metrics/mAP_0.5', 'metrics/mAP_0.5:0.95',
                        'val/giou_loss', 'val/obj_loss', 'val/cls_loss']
                for x, tag in zip(list(mloss[:-1]) + list(results), tags):
                    tb_writer.add_scalar(tag, x, epoch)
                #剪枝后bn层权重
                bn_weights = gather_bn_weights(cfg_model.module_list, prune_idx)
                tb_writer.add_histogram('bn_weights/hist', bn_weights.numpy(), epoch, bins='doane')

            # Update best mAP
            fi = fitness(np.array(results).reshape(1, -1))  # fitness_i = weighted combination of [P, R, mAP, F1]
            if fi > best_fitness:
                best_fitness = fi

            # Save model
            save = (not opt.nosave) or (final_epoch and not opt.evolve)
            if save:
                with open(results_file, 'r') as f:  # create checkpoint
                    ckpt = {'epoch': epoch,
                            'best_fitness': best_fitness,
                            'training_results': f.read(),
                            'model': ema.ema.module if hasattr(ema, 'module') else ema.ema,
                            'optimizer': None if final_epoch else optimizer.state_dict()}

                # Save last, best and delete
                torch.save(ckpt, last)
                if best_fitness == fi:
                    torch.save(ckpt, best)
                del ckpt
        # end epoch ----------------------------------------------------------------------------------------------------
    # end training

    if rank in [-1, 0]:
        # Strip optimizers
        n = ('_' if len(opt.name) and not opt.name.isnumeric() else '') + opt.name
        fresults, flast, fbest = 'results%s.txt' % n, wdir + 'last%s.pt' % n, wdir + 'best%s.pt' % n
        for f1, f2 in zip([wdir + 'last.pt', wdir + 'best.pt', 'results.txt'], [flast, fbest, fresults]):
            if os.path.exists(f1):
                os.rename(f1, f2)  # rename
                ispt = f2.endswith('.pt')  # is *.pt
                strip_optimizer(f2) if ispt else None  # strip optimizer
                os.system('gsutil cp %s gs://%s/weights' % (f2, opt.bucket)) if opt.bucket and ispt else None  # upload
        # Finish
        if not opt.evolve:
            plot_results(save_dir=log_dir)  # save as results.png
        print('%g epochs completed in %.3f hours.\n' % (epoch - start_epoch + 1, (time.time() - t0) / 3600))

    dist.destroy_process_group() if rank not in [-1, 0] else None
    torch.cuda.empty_cache()
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', type=str, default='models/yolov5s.yaml', help='model.yaml path')
    parser.add_argument('--data', type=str, default='data/coco128.yaml', help='data.yaml path')
    parser.add_argument('--hyp', type=str, default='', help='hyp.yaml path (optional)')
    parser.add_argument('--epochs', type=int, default=300)
    parser.add_argument('--batch-size', type=int, default=12, help='total batch size for all GPUs')
    parser.add_argument('--img-size', nargs='+', type=int, default=[640, 640], help='train,test sizes')
    parser.add_argument('--rect', action='store_true', help='rectangular training')
    parser.add_argument('--resume', nargs='?', const='get_last', default=False,
                        help='resume from given path/last.pt, or most recent run if blank')
    parser.add_argument('--nosave', action='store_true', help='only save final checkpoint')
    parser.add_argument('--notest', action='store_true', help='only test final epoch')
    parser.add_argument('--noautoanchor', action='store_true', help='disable autoanchor check')
    parser.add_argument('--evolve', action='store_true', help='evolve hyperparameters')
    parser.add_argument('--bucket', type=str, default='', help='gsutil bucket')
    parser.add_argument('--cache-images', action='store_true', help='cache images for faster training')
    parser.add_argument('--weights', type=str, default='', help='initial weights path')
    parser.add_argument('--name', default='', help='renames results.txt to results_name.txt if supplied')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--multi-scale', action='store_true', help='vary img-size +/- 50%%')
    parser.add_argument('--single-cls', action='store_true', help='train as single-class dataset')
    parser.add_argument('--adam', action='store_true', help='use torch.optim.Adam() optimizer')
    parser.add_argument('--sync-bn', action='store_true', help='use SyncBatchNorm, only available in DDP mode')
    parser.add_argument('--local_rank', type=int, default=-1, help='DDP parameter, do not modify')
    parser.add_argument('--sparsity-regularization', '-sr', dest='sr', action='store_true',
                        help='train with channel sparsity regularization')
    parser.add_argument('--s', type=float, default=0.001, help='scale sparse rate')
    parser.add_argument('--prune', type=int, default=1, help='0:nomal prune 1:other prune ')
    opt = parser.parse_args()

    # Resume
    last = get_latest_run() if opt.resume == 'get_last' else opt.resume  # resume from most recent run
    if last and not opt.weights:
        print(f'Resuming training from {last}')
    opt.weights = last if opt.resume and not opt.weights else opt.weights

    # if opt.local_rank in [-1, 0]:
    #     check_git_status()
    opt.cfg = check_file(opt.cfg)  # check file
    opt.data = check_file(opt.data)  # check file
    if opt.hyp:  # update hyps
        opt.hyp = check_file(opt.hyp)  # check file
        with open(opt.hyp) as f:
            hyp.update(yaml.load(f, Loader=yaml.FullLoader))  # update hyps
    opt.img_size.extend([opt.img_size[-1]] * (2 - len(opt.img_size)))  # extend to 2 sizes (train, test)
    device = select_device(opt.device, batch_size=opt.batch_size)
    opt.total_batch_size = opt.batch_size
    opt.world_size = 1

    # DDP mode
    if opt.local_rank != -1:
        assert torch.cuda.device_count() > opt.local_rank
        torch.cuda.set_device(opt.local_rank)
        device = torch.device('cuda', opt.local_rank)
        dist.init_process_group(backend='nccl', init_method='env://')  # distributed backend
        opt.world_size = dist.get_world_size()
        assert opt.batch_size % opt.world_size == 0, '--batch-size must be multiple of CUDA device count'
        opt.batch_size = opt.total_batch_size // opt.world_size

    print(opt)

    # Train
    if not opt.evolve:
        tb_writer = None
        if opt.local_rank in [-1, 0]:
            print('Start Tensorboard with "tensorboard --logdir=runs", view at http://localhost:6006/')
            tb_writer = SummaryWriter(log_dir=increment_dir('runs/exp', opt.name))

        train(hyp, opt, device, tb_writer)

    # Evolve hyperparameters (optional)
    else:
        # Hyperparameter evolution metadata (mutation scale 0-1, lower_limit, upper_limit)
        meta = {'lr0': (1, 1e-5, 1e-1),  # initial learning rate (SGD=1E-2, Adam=1E-3)
                'momentum': (0.1, 0.6, 0.98),  # SGD momentum/Adam beta1
                'weight_decay': (1, 0.0, 0.001),  # optimizer weight decay
                'giou': (1, 0.02, 0.2),  # GIoU loss gain
                'cls': (1, 0.2, 4.0),  # cls loss gain
                'cls_pw': (1, 0.5, 2.0),  # cls BCELoss positive_weight
                'obj': (1, 0.2, 4.0),  # obj loss gain (scale with pixels)
                'obj_pw': (1, 0.5, 2.0),  # obj BCELoss positive_weight
                'iou_t': (0, 0.1, 0.7),  # IoU training threshold
                'anchor_t': (1, 2.0, 8.0),  # anchor-multiple threshold
                'fl_gamma': (0, 0.0, 2.0),  # focal loss gamma (efficientDet default gamma=1.5)
                'hsv_h': (1, 0.0, 0.1),  # image HSV-Hue augmentation (fraction)
                'hsv_s': (1, 0.0, 0.9),  # image HSV-Saturation augmentation (fraction)
                'hsv_v': (1, 0.0, 0.9),  # image HSV-Value augmentation (fraction)
                'degrees': (1, 0.0, 45.0),  # image rotation (+/- deg)
                'translate': (1, 0.0, 0.9),  # image translation (+/- fraction)
                'scale': (1, 0.0, 0.9),  # image scale (+/- gain)
                'shear': (1, 0.0, 10.0),  # image shear (+/- deg)
                'perspective': (1, 0.0, 0.001),  # image perspective (+/- fraction), range 0-0.001
                'flipud': (0, 0.0, 1.0),  # image flip up-down (probability)
                'fliplr': (1, 0.0, 1.0),  # image flip left-right (probability)
                'mixup': (1, 0.0, 1.0)}  # image mixup (probability)

        assert opt.local_rank == -1, 'DDP mode not implemented for --evolve'
        opt.notest, opt.nosave = True, True  # only test/save final epoch
        # ei = [isinstance(x, (int, float)) for x in hyp.values()]  # evolvable indices
        yaml_file = Path('runs/evolve/hyp_evolved.yaml')  # save best result here
        if opt.bucket:
            os.system('gsutil cp gs://%s/evolve.txt .' % opt.bucket)  # download evolve.txt if exists

        for _ in range(100):  # generations to evolve
            if os.path.exists('evolve.txt'):  # if evolve.txt exists: select best hyps and mutate
                # Select parent(s)
                parent = 'single'  # parent selection method: 'single' or 'weighted'
                x = np.loadtxt('evolve.txt', ndmin=2)
                n = min(5, len(x))  # number of previous results to consider
                x = x[np.argsort(-fitness(x))][:n]  # top n mutations
                w = fitness(x) - fitness(x).min()  # weights
                if parent == 'single' or len(x) == 1:
                    # x = x[random.randint(0, n - 1)]  # random selection
                    x = x[random.choices(range(n), weights=w)[0]]  # weighted selection
                elif parent == 'weighted':
                    x = (x * w.reshape(n, 1)).sum(0) / w.sum()  # weighted combination

                # Mutate
                mp, s = 0.9, 0.2  # mutation probability, sigma
                npr = np.random
                npr.seed(int(time.time()))
                g = np.array([x[0] for x in meta.values()])  # gains 0-1
                ng = len(meta)
                v = np.ones(ng)
                while all(v == 1):  # mutate until a change occurs (prevent duplicates)
                    v = (g * (npr.random(ng) < mp) * npr.randn(ng) * npr.random() * s + 1).clip(0.3, 3.0)
                for i, k in enumerate(hyp.keys()):  # plt.hist(v.ravel(), 300)
                    hyp[k] = float(x[i + 7] * v[i])  # mutate

            # Constrain to limits
            for k, v in meta.items():
                hyp[k] = max(hyp[k], v[1])  # lower limit
                hyp[k] = min(hyp[k], v[2])  # upper limit
                hyp[k] = round(hyp[k], 5)  # significant digits

            # Train mutation
            results = train(hyp.copy(), opt, device)

            # Write mutation results
            print_mutation(hyp.copy(), results, yaml_file, opt.bucket)

        # Plot results
        plot_evolution(yaml_file)
        print('Hyperparameter evolution complete. Best results saved as: %s\nCommand to train a new model with these '
              'hyperparameters: $ python train.py --hyp %s' % (yaml_file, yaml_file))