import argparse
import os
from collections import OrderedDict
import yaml
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig
import json
import pandas as pd
from tqdm import tqdm
import urllib.request
import torch
import torch.backends.cudnn as cudnn
import torch.nn as nn
import torch.optim as optim
from torch.optim import lr_scheduler
import albumentations as albu
from albumentations.augmentations import transforms
from albumentations.core.composition import Compose, OneOf

import archs
import losses
from dataset import Dataset
from metrics import iou_score
from utils import AverageMeter, str2bool

ARCH_NAMES = archs.__all__
LOSS_NAMES = losses.__all__
LOSS_NAMES.append('BCEWithLogitsLoss')


def parse_args():
    parser = argparse.ArgumentParser()

    # open params
    parser.add_argument('--epochs', default=50, type=int,
                        metavar='N', help='number of total epochs to run')
    parser.add_argument('-b', '--batch_size', default=8, type=int,
                        metavar='N', help='mini-batch size (default: 16)')
    parser.add_argument('--input_dir', default='C:/Users/TDTECH/Desktop/out/output/trans_output',
                        type=str, help='input directory, including images and masks')
    parser.add_argument('--out_dir', default='C:/Users/TDTECH/Desktop/out/output/train_output', 
                        type=str, help='output directory')

    parser.add_argument('--transfer_training', default=True)
    parser.add_argument('--trained_model', default='nested_unet.pth',
                        type=str, help='output directory, including images and masks')

    # model
    parser.add_argument('--name', default=None,
                        help='model name: (default: arch+timestamp)')
    parser.add_argument('--arch', '-a', metavar='ARCH', default='NestedUNet',
                        choices=ARCH_NAMES, help='model architecture: ' + ' | '.join(ARCH_NAMES) + ' (default: NestedUNet)')
    parser.add_argument('--deep_supervision', default=False, type=str2bool)
    parser.add_argument('--input_channels', default=3,
                        type=int, help='input channels')
    parser.add_argument('--num_classes', default=1,
                        type=int, help='number of classes')
    parser.add_argument('--input_w', default=96, type=int, help='image width')
    parser.add_argument('--input_h', default=96, type=int, help='image height')

    # loss
    parser.add_argument('--loss', default='LovaszHingeLoss',
                        choices=LOSS_NAMES, help='loss: ' + ' | '.join(LOSS_NAMES) + ' (default: BCEDiceLoss)')  # LovaszHingeLoss BCEDiceLoss

    # dataset
    parser.add_argument(
        '--dataset', default='customer_dataset', help='dataset name')
    parser.add_argument('--img_ext', default='.png',
                        help='image file extension')
    parser.add_argument('--mask_ext', default='.png',
                        help='mask file extension')

    # optimizer
    parser.add_argument('--optimizer', default='SGD',
                        choices=['Adam', 'SGD'], help='loss: ' + ' | '.join(['Adam', 'SGD']) + ' (default: Adam)')
    parser.add_argument('--lr', '--learning_rate', default=1e-3,
                        type=float, metavar='LR', help='initial learning rate')
    parser.add_argument('--momentum', default=0.9, type=float, help='momentum')
    parser.add_argument('--weight_decay', default=1e-4,
                        type=float, help='weight decay')
    parser.add_argument('--nesterov', default=False,
                        type=str2bool, help='nesterov')

    # scheduler
    parser.add_argument('--scheduler', default='CosineAnnealingLR',
                        choices=['CosineAnnealingLR', 'ReduceLROnPlateau', 'MultiStepLR', 'ConstantLR'])
    parser.add_argument('--min_lr', default=1e-5,
                        type=float, help='minimum learning rate')
    parser.add_argument('--factor', default=0.1, type=float)
    parser.add_argument('--patience', default=2, type=int)
    parser.add_argument('--milestones', default='1,2', type=str)
    parser.add_argument('--gamma', default=2/3, type=float)
    parser.add_argument('--early_stopping', default=-1, type=int,
                        metavar='N', help='early stopping (default: -1)')

    parser.add_argument('--num_workers', default=4, type=int)

    config = parser.parse_args()

    return config


def train(config, train_loader, model, criterion, optimizer):
    avg_meters = {'loss': AverageMeter(),
                  'iou': AverageMeter()}

    model.train()

    pbar = tqdm(total=len(train_loader))
    for input, target, _ in train_loader:

        # compute output
        if config['deep_supervision']:
            outputs = model(input)
            loss = 0
            for output in outputs:
                loss += criterion(output, target)
            loss /= len(outputs)
            iou = iou_score(outputs[-1], target)
        else:
            output = model(input)
            loss = criterion(output, target)
            iou = iou_score(output, target)

        # compute gradient and do optimizing step
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        avg_meters['loss'].update(loss.item(), input.size(0))
        avg_meters['iou'].update(iou, input.size(0))

        postfix = OrderedDict([
            ('loss', avg_meters['loss'].avg),
            ('iou', avg_meters['iou'].avg),
        ])
        pbar.set_postfix(postfix)
        pbar.update(1)
    pbar.close()

    return OrderedDict([('loss', avg_meters['loss'].avg),
                        ('iou', avg_meters['iou'].avg)])


def validate(config, val_loader, model, criterion):
    avg_meters = {'loss': AverageMeter(),
                  'iou': AverageMeter()}

    # switch to evaluate mode
    model.eval()

    with torch.no_grad():
        pbar = tqdm(total=len(val_loader))
        for input, target, _ in val_loader:
            # input = input.cuda()
            # target = target.cuda()

            # compute output
            if config['deep_supervision']:
                outputs = model(input)
                loss = 0
                for output in outputs:
                    loss += criterion(output, target)
                loss /= len(outputs)
                iou = iou_score(outputs[-1], target)
            else:
                output = model(input)
                # this criterion first cal sigmoid of output, then cal the loss with cross entropy
                loss = criterion(output, target)
                iou = iou_score(output, target)

            avg_meters['loss'].update(loss.item(), input.size(0))
            avg_meters['iou'].update(iou, input.size(0))

            postfix = OrderedDict([
                ('loss', avg_meters['loss'].avg),
                ('iou', avg_meters['iou'].avg),
            ])
            pbar.set_postfix(postfix)
            pbar.update(1)
        pbar.close()

    return OrderedDict([('loss', avg_meters['loss'].avg),
                        ('iou', avg_meters['iou'].avg)])


def main(args):
    config = vars(args)

    # get num_classes
    mask_dir = os.path.join(config['input_dir'], 'train', 'masks')
    config['num_classes'] = len(os.listdir(mask_dir))

    if config['name'] is None:
        if config['deep_supervision']:
            config['name'] = '%s_%s_wDS' % (config['dataset'], config['arch'])
        else:
            config['name'] = '%s_%s_woDS' % (config['dataset'], config['arch'])
    os.makedirs(config['out_dir'], exist_ok=True)

    print('-' * 20)
    for key in config:
        print('%s: %s' % (key, config[key]))
    print('-' * 20)

    with open(os.path.join(config['out_dir'], 'config.yml'), 'w') as f:
        yaml.dump(config, f)

    # define loss function (criterion)
    if config['loss'] == 'BCEWithLogitsLoss':
        criterion = nn.BCEWithLogitsLoss()  # .cuda()
    else:
        criterion = losses.__dict__[config['loss']]()  # .cuda()

    cudnn.benchmark = True

    # create model
    print("=> creating model %s" % config['arch'])
    model = archs.__dict__[config['arch']](config['num_classes'],
                                           config['input_channels'],
                                           config['deep_supervision'])
    if not os.path.exists(config['trained_model']):
        print('downloading pretrained models from obs ...')
        with open("../../../../../version.json", "r") as download_urls:
            try:
                url = json.load(download_urls)["model_urls"]['unetpp']
                urllib.request.urlretrieve(url, config['trained_model'])
            except Exception as e:
                print(e)
                
    if config['transfer_training']:
        new_dict = model.state_dict()
        old_dict = torch.load(config['trained_model'], map_location='cpu')
        for k in new_dict.keys():
            if k in old_dict.keys() and k not in new_dict.keys():
                del old_dict[k]
                print('--delete weights: {}'.format(k))
            if k in old_dict.keys() and k in new_dict.keys() and old_dict[k].shape != new_dict[k].shape:
                old_dict[k] = new_dict[k]
                print('--change weights: {}'.format(k))
        model.load_state_dict(old_dict, strict=False)
        # model.load_state_dict(torch.load(config['trained_model'], map_location='cpu'), strict=False)

    params = filter(lambda p: p.requires_grad, model.parameters())
    if config['optimizer'] == 'Adam':
        optimizer = optim.Adam(
            params, lr=config['lr'], weight_decay=config['weight_decay'])
    elif config['optimizer'] == 'SGD':
        optimizer = optim.SGD(params, lr=config['lr'], momentum=config['momentum'],
                              nesterov=config['nesterov'], weight_decay=config['weight_decay'])
    else:
        raise NotImplementedError

    if config['scheduler'] == 'CosineAnnealingLR':
        scheduler = lr_scheduler.CosineAnnealingLR(
            optimizer, T_max=config['epochs'], eta_min=config['min_lr'])
    elif config['scheduler'] == 'ReduceLROnPlateau':
        scheduler = lr_scheduler.ReduceLROnPlateau(optimizer, factor=config['factor'], patience=config['patience'],
                                                   verbose=1, min_lr=config['min_lr'])
    elif config['scheduler'] == 'MultiStepLR':
        scheduler = lr_scheduler.MultiStepLR(optimizer, milestones=[int(
            e) for e in config['milestones'].split(',')], gamma=config['gamma'])
    elif config['scheduler'] == 'ConstantLR':
        scheduler = None
    else:
        raise NotImplementedError

    # get train and val ids
    train_img_ids = [os.path.splitext(os.path.basename(p))[0] for p in os.listdir(
        os.path.join(config['input_dir'], 'train', 'images'))]
    val_img_ids = [os.path.splitext(os.path.basename(p))[0] for p in os.listdir(
        os.path.join(config['input_dir'], 'val', 'images'))]
    print('\n--------------------')
    print('using {} training samples'.format(len(train_img_ids)))
    print('using {} validate samples'.format(len(val_img_ids)))

    # get extension type
    examp_train_img = os.listdir(os.path.join(
        config['input_dir'], 'train', 'images'))[0]
    config['img_ext'] = os.path.splitext(examp_train_img)[1]
    examp_train_msk = os.listdir(os.path.join(
        config['input_dir'], 'train', 'masks', '0'))[0]
    config['mask_ext'] = os.path.splitext(examp_train_msk)[1]

    train_transform = Compose([
        albu.RandomRotate90(),
        albu.Flip(),
        OneOf([
            transforms.HueSaturationValue(),
            transforms.RandomBrightness(),
            transforms.RandomContrast(),
        ], p=1),
        albu.Resize(config['input_h'], config['input_w']),
        transforms.Normalize(),
    ])

    val_transform = Compose([
        albu.Resize(config['input_h'], config['input_w']),
        transforms.Normalize(),
    ])

    train_dataset = Dataset(
        img_ids=train_img_ids,
        img_dir=os.path.join(config['input_dir'], 'train', 'images'),
        mask_dir=os.path.join(config['input_dir'], 'train', 'masks'),
        img_ext=config['img_ext'],
        mask_ext=config['mask_ext'],
        num_classes=config['num_classes'],
        transform=train_transform)
    val_dataset = Dataset(
        img_ids=val_img_ids,
        img_dir=os.path.join(config['input_dir'], 'val', 'images'),
        mask_dir=os.path.join(config['input_dir'], 'val', 'masks'),
        img_ext=config['img_ext'],
        mask_ext=config['mask_ext'],
        num_classes=config['num_classes'],
        transform=val_transform)

    train_loader = torch.utils.data.DataLoader(
        train_dataset,
        batch_size=config['batch_size'],
        shuffle=True,
        num_workers=config['num_workers'],
        drop_last=True)
    val_loader = torch.utils.data.DataLoader(
        val_dataset,
        batch_size=config['batch_size'],
        shuffle=False,
        num_workers=config['num_workers'],
        drop_last=False)

    log = OrderedDict([
        ('epoch', []),
        ('lr', []),
        ('loss', []),
        ('iou', []),
        ('val_loss', []),
        ('val_iou', []),
    ])

    best_iou = 0
    trigger = 0
    best_epoch = 0
    best_fitness = 0

    for epoch in range(config['epochs']):
        print(f"process_value={int(epoch / config['epochs'] * 90 + 6)}")

        # train for one epoch
        train_log = train(config, train_loader, model, criterion, optimizer)
        # evaluate on validation set
        val_log = validate(config, val_loader, model, criterion)

        if config['scheduler'] == 'CosineAnnealingLR':
            scheduler.step()
        elif config['scheduler'] == 'ReduceLROnPlateau':
            scheduler.step(val_log['loss'])

        print('epoch %d - loss %.4f - iou %.4f - val_loss %.4f - val_iou %.4f' % (epoch, train_log['loss'], train_log['iou'], val_log['loss'], val_log['iou']))

        log['epoch'].append(epoch)
        log['lr'].append(config['lr'])
        log['loss'].append(train_log['loss'])
        log['iou'].append(train_log['iou'])
        log['val_loss'].append(val_log['loss'])
        log['val_iou'].append(val_log['iou'])

        pd.DataFrame(log).to_csv(os.path.join(
            config['out_dir'], 'log.csv'), index=False)

        trigger += 1

        if val_log['iou'] > best_iou:
            torch.save(model.state_dict(),  os.path.join(
                config['out_dir'], 'model.pth'))
            best_iou = val_log['iou']
            print("=> saved best model")
            trigger = 0

        # early stopping

        if args.early_stopping:
            if val_log['iou'] > best_fitness:
                best_fitness = val_log['iou']
                best_epoch = epoch

            if epoch - best_epoch > args.early_stopping_tolerance:
                print(f"INFO: Early stopping after {args.early_stopping_tolerance} epochs without improvement.")
                break

            if best_iou > args.early_stopping_threshold:
                print(
                    f"INFO: Early stopping as IOU ({best_iou:.3f}) exceeded threshold ({args.early_stopping_threshold:.3f}).")
                break
    
    return "INFO:Train success!"


if __name__ == '__main__':
    args = parse_args()
    
    yaml_file = '../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('segmentation')

    args.input_dir = yaml_data['trans_output']
    args.epochs = yaml_data['epochs']
    args.batch_size = yaml_data['train_batch_size']
    train_output_path = os.path.join(yaml_data['output_path'], 'train_output')
    args.out_dir = train_output_path

    args.early_stopping = yaml_data.get('earlystopping_enabled', False)
    args.early_stopping_threshold = yaml_data.get('earlystopping_threshold', 0.99)
    args.early_stopping_tolerance = yaml_data.get('earlystopping_tolerance', 10)

    config.set_para('segmentation', 'train_output', train_output_path)
    config.yaml_dump(yaml_file)

    try:
        ret = main(args)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")
