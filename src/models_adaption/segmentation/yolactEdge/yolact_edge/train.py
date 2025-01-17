import yaml

from yolact_edge.utils import timer
from yolact_edge.data import *
from yolact_edge.utils.augmentations import SSDAugmentation, SSDAugmentationVideo, BaseTransform, BaseTransformVideo
from yolact_edge.utils.functions import MovingAverage, SavePath
from yolact_edge.layers.modules import MultiBoxLoss
from yolact_edge.layers.modules.optical_flow_loss import OpticalFlowLoss
from yolact_edge.yolact import Yolact
import os
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig
import time
import math
from pathlib import Path
import torch
from torch.autograd import Variable
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data as data
import numpy as np
import argparse
import datetime
import json
from yolact_edge.utils.tensorboard_helper import SummaryHelper
import yolact_edge.utils.misc as misc
import torch.distributed as dist
import torch.multiprocessing as mp
from yolact_edge.utils.logging_helper import setup_logger
import logging
import random

# Oof
import eval as eval_script


def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")


pth_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yolact_edge_resnet50_54_800000.pth')
if not os.path.exists(pth_path):
    with open("../../../../../version.json", "r") as download_urls:
        try:
            pth_url =json.load(download_urls)["model_urls"]['yolact_edge']
            torch.hub.download_url_to_file(pth_url, pth_path)
            print(f"Download {pth_url} to {pth_path}.")
            if not os.path.isfile(pth_path):
                print("ERROR: Download file failed.")
                raise Exception("ERROR: Download file failed.")
        except Exception as e:
            print(e)
            
parser = argparse.ArgumentParser(description='Yolact Training Script')

# 开放参数
parser.add_argument('--resume', default='./yolact_edge_resnet50_54_800000.pth', type=str,
                    help='Checkpoint state_dict file to resume training from. If this is "interrupt" , the model will resume training from the interrupt file.')
parser.add_argument('--max_iter', default=20, type=int,  # 800000
                    help='The number of training iteration.')
parser.add_argument('--print_every_iternum', default=1, type=int,  # 10
                    help='Print info frenquency.')
parser.add_argument('--save_folder', default='weights/',
                    help='Directory for saving checkpoint models')

# 用 自己构造地数据集 时，必须用的
parser.add_argument('--dataset', default='my_custom_dataset', type=str,  # coco2017_dataset
                    help='If specified, override the dataset specified in the config with this one (example: coco2017_dataset).')
parser.add_argument('--train_images', default='data/coco_small_data/train/images', type=str)
parser.add_argument('--train_info', default='data/coco_small_data/train/train_annotation.json', type=str)
parser.add_argument('--class_names', default="('bear','person','car')", type=str)
# parser.add_argument('--valid_images', default='data/coco_small_data/valid/images', type=str)
# parser.add_argument('--valid_info', default='data/coco_small_data/valid/val_annotation.json', type=str)


# 暂时不用
parser.add_argument('--validation_epoch', default=-1, type=int,
                    help='Output validation information every n iterations. If -1, do no validation.')
parser.add_argument('--validation_size', default=3, type=int,  # 5000
                    help='The number of images to use for validation.')
# 不开放参数
parser.add_argument('--config', default='yolact_resnet50_config',
                    help='The config object to use.')
parser.add_argument('--batch_size', default=8, type=int,
                    help='Batch size for training')
parser.add_argument('--start_iter', default=0, type=int,
                    help='Resume training at this iter. If this is -1, the iteration will be' \
                         'determined from the file name.')
parser.add_argument('--random_seed', default=42, type=int,
                    help='Random seed used across all workers')
parser.add_argument('--num_workers', default=0, type=int,
                    help='Number of workers used in dataloading')
parser.add_argument('--num_gpus', default=1, type=int,
                    help='Number of GPUs used in training')
parser.add_argument('--cuda', default=False, type=str2bool,
                    help='Use CUDA to train model')
parser.add_argument('--lr', '--learning_rate', default=None, type=float,
                    help='Initial learning rate. Leave as None to read this from the config.')
parser.add_argument('--momentum', default=None, type=float,
                    help='Momentum for SGD. Leave as None to read this from the config.')
parser.add_argument('--decay', '--weight_decay', default=None, type=float,
                    help='Weight decay for SGD. Leave as None to read this from the config.')
parser.add_argument('--gamma', default=None, type=float,
                    help='For each lr step, what to multiply the lr by. Leave as None to read this from the config.')
parser.add_argument('--log_folder', default='../../logs/',
                    help='Directory for saving Tensorboard logs')
parser.add_argument('--save_interval', default=10000, type=int,
                    help='The number of iterations between saving the model.')
parser.add_argument('--keep_latest', dest='keep_latest', action='store_true',
                    help='Only keep the latest checkpoint instead of each one.')
parser.add_argument('--keep_latest_interval', default=100000, type=int,
                    help='When --keep_latest is on, don\'t delete the latest file at these intervals. This should be a multiple of save_interval or 0.')

parser.add_argument('--yolact_transfer', dest='yolact_transfer', action='store_true',
                    help='Split pretrained FPN weights to two phase FPN (for models trained by YOLACT).')
parser.add_argument('--coco_transfer', dest='coco_transfer', action='store_true',
                    help='[Deprecated] Split pretrained FPN weights to two phase FPN (for models trained by YOLACT).')
parser.add_argument('--drop_weights', default=None, type=str,
                    help='Drop specified weights (split by comma) from existing model.')
parser.add_argument('--interrupt_no_save', dest='interrupt_no_save', action='store_true',
                    help='Just exit when keyboard interrupt occurs for testing.')
parser.add_argument('--no_warmup_rescale', dest='warmup_rescale', action='store_false',
                    help='Do not rescale warmup coefficients on multiple GPU training.')

parser.set_defaults(keep_latest=False)
args = parser.parse_args()

yaml_file = '../../../config/config.yaml'
config = ModelConfig(yaml_file)
yaml_data = config.get_yaml_data().get('segmentation')
args.resume = pth_path
args.train_images = os.path.join(yaml_data['trans_output'], 'train', 'images')
args.train_info = os.path.join(yaml_data['trans_output'], 'train', 'train_annotation.json')
class_names = list()
with open(args.train_info, errors='ignore') as f:
    anno = yaml.safe_load(f)
for i in anno['categories']:
    class_names.append(i['name'])
args.class_names = tuple(class_names)
args.max_iter = yaml_data['epochs']
train_output_path = os.path.join(yaml_data['output_path'], 'train_output')
args.save_folder = train_output_path

if args.config is not None:
    set_cfg(args.config)

if args.dataset is not None:
    set_dataset(args.dataset)
    if args.dataset == 'my_custom_dataset':
        cfg.dataset.train_images = args.train_images
        cfg.dataset.train_info = args.train_info
        cfg.dataset.class_names = args.class_names
        # cfg.dataset.valid_images = args.valid_images
        # cfg.dataset.valid_info = args.valid_info
    cfg.num_classes = len(cfg.dataset.class_names) + 1  # FIXME: this could be better handled

cfg.max_iter = args.max_iter


# Update training parameters from the config if necessary
def replace(name):
    if getattr(args, name) == None: setattr(args, name, getattr(cfg, name))


replace('lr')
replace('decay')
replace('gamma')
replace('momentum')

lr = args.lr
loss_types = ['B', 'C', 'M', 'P', 'D', 'E', 'S', 'F', 'R', 'W']

if torch.cuda.is_available():
    if args.cuda:
        torch.set_default_tensor_type('torch.cuda.FloatTensor')
    if not args.cuda:
        print("WARNING: It looks like you have a CUDA device, but aren't " +
              "using CUDA.\nRun with --cuda for optimal training speed.")
        torch.set_default_tensor_type('torch.FloatTensor')
else:
    torch.set_default_tensor_type('torch.FloatTensor')


def train(rank, args):
    cfg.max_iter = args.max_iter

    if rank == 0:
        if not os.path.exists(args.save_folder):
            os.mkdir(args.save_folder)

    # fix the seed for reproducibility
    seed = args.random_seed + rank
    torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)

    # set up logger
    setup_logger(output=os.path.join(args.log_folder, cfg.name), distributed_rank=rank)
    logger = logging.getLogger("yolact.train")

    w = SummaryHelper(distributed_rank=rank, log_dir=os.path.join(args.log_folder, cfg.name))
    w.add_text("argv", " ".join(sys.argv))
    logger.info("Args: {}".format(" ".join(sys.argv)))
    # import git
    # with git.Repo(search_parent_directories=True) as repo:
    #     w.add_text("git_hash", repo.head.object.hexsha)
    #     logger.info("git hash: {}".format(repo.head.object.hexsha))

    if args.num_gpus > 1:
        try:
            logger.info("Initializing torch.distributed backend...")
            dist.init_process_group(
                backend='nccl',
                init_method=args.dist_url,
                world_size=args.num_gpus,
                rank=rank
            )
        except Exception as e:
            logger.error("Process group URL: {}".format(args.dist_url))
            raise e

    misc.barrier()

    if torch.cuda.device_count() > 1:
        logger.info('Multiple GPUs detected! Turning off JIT.')

    collate_fn = detection_collate
    if cfg.dataset.name == 'YouTube VIS':
        dataset = YoutubeVIS(image_path=cfg.dataset.train_images,
                             info_file=cfg.dataset.train_info,
                             configs=cfg.dataset,
                             transform=SSDAugmentationVideo(MEANS))

        if cfg.dataset.joint == 'coco':
            joint_dataset = COCODetection(image_path=cfg.joint_dataset.train_images,
                                          info_file=cfg.joint_dataset.train_info,
                                          transform=SSDAugmentation(MEANS))
            joint_collate_fn = detection_collate

        if args.validation_epoch > 0:
            setup_eval()
            val_dataset = YoutubeVIS(image_path=cfg.dataset.valid_images,
                                     info_file=cfg.dataset.valid_info,
                                     configs=cfg.dataset,
                                     transform=BaseTransformVideo(MEANS))
        collate_fn = collate_fn_youtube_vis

    elif cfg.dataset.name == 'FlyingChairs':
        dataset = FlyingChairs(image_path=cfg.dataset.trainval_images,
                               info_file=cfg.dataset.trainval_info)

        collate_fn = collate_fn_flying_chairs

    else:
        dataset = COCODetection(image_path=cfg.dataset.train_images,
                                info_file=cfg.dataset.train_info,
                                transform=SSDAugmentation(MEANS))

        if args.validation_epoch > 0:
            setup_eval()
            val_dataset = COCODetection(image_path=cfg.dataset.valid_images,
                                        info_file=cfg.dataset.valid_info,
                                        transform=BaseTransform(MEANS))

    # Set cuda device early to avoid duplicate model in master GPU
    if args.cuda:
        torch.cuda.set_device(rank)

    # Parallel wraps the underlying module, but when saving and loading we don't want that
    yolact_net = Yolact()
    net = yolact_net
    net.train()

    # I don't use the timer during training (I use a different timing method).
    # Apparently there's a race condition with multiple GPUs.

    # use timer for experiments
    timer.disable_all()

    # Both of these can set args.resume to None, so do them before the check
    if args.resume == 'interrupt':
        args.resume = SavePath.get_interrupt(args.save_folder)
    elif args.resume == 'latest':
        args.resume = SavePath.get_latest(args.save_folder, cfg.name)

    if args.resume is not None:
        logger.info('Resuming training, loading {}...'.format(args.resume))
        yolact_net.load_weights(args.resume, args=args)

        if args.start_iter == -1:
            args.start_iter = SavePath.from_str(args.resume).iteration
    else:
        logger.info('Initializing weights...')
        yolact_net.init_weights(backbone_path=args.save_folder + cfg.backbone.path)

    if cfg.flow.train_flow:
        criterion = OpticalFlowLoss()

    else:
        criterion = MultiBoxLoss(num_classes=cfg.num_classes,
                                 pos_threshold=cfg.positive_iou_threshold,
                                 neg_threshold=cfg.negative_iou_threshold,
                                 negpos_ratio=3)

    if args.cuda:
        net.cuda(rank)

        if misc.is_distributed_initialized():
            net = nn.parallel.DistributedDataParallel(net, device_ids=[rank], output_device=rank,
                                                      broadcast_buffers=False,
                                                      find_unused_parameters=True)

    optimizer = optim.SGD(filter(lambda x: x.requires_grad, net.parameters()),
                          lr=args.lr, momentum=args.momentum,
                          weight_decay=args.decay)

    # loss counters
    iteration = max(args.start_iter, 0)
    w.set_step(iteration)
    last_time = time.time()

    epoch_size = len(dataset) // args.batch_size // args.num_gpus
    if not epoch_size:
        epoch_size = 1
    num_epochs = math.ceil(cfg.max_iter / epoch_size)

    # Which learning rate adjustment step are we on? lr' = lr * gamma ^ step_index
    step_index = 0

    from yolact_edge.data.sampler_utils import InfiniteSampler, build_batch_data_sampler

    infinite_sampler = InfiniteSampler(dataset, seed=args.random_seed, num_replicas=args.num_gpus,
                                       rank=rank, shuffle=True)
    train_sampler = build_batch_data_sampler(infinite_sampler, images_per_batch=args.batch_size)

    data_loader = data.DataLoader(dataset,
                                  num_workers=args.num_workers,
                                  collate_fn=collate_fn,
                                  multiprocessing_context="fork" if args.num_workers > 1 else None,
                                  batch_sampler=train_sampler)
    data_loader_iter = iter(data_loader)

    if cfg.dataset.joint:
        joint_infinite_sampler = InfiniteSampler(joint_dataset, seed=args.random_seed, num_replicas=args.num_gpus,
                                                 rank=rank, shuffle=True)
        joint_train_sampler = build_batch_data_sampler(joint_infinite_sampler, images_per_batch=args.batch_size)
        joint_data_loader = data.DataLoader(joint_dataset,
                                            num_workers=args.num_workers,
                                            collate_fn=joint_collate_fn,
                                            multiprocessing_context="fork" if args.num_workers > 1 else None,
                                            batch_sampler=joint_train_sampler)
        joint_data_loader_iter = iter(joint_data_loader)

    save_path = lambda epoch, iteration: SavePath(cfg.name, epoch, iteration).get_path(root=args.save_folder)
    time_avg = MovingAverage()
    data_time_avg = MovingAverage(10)

    global loss_types  # Forms the print order
    loss_avgs = {k: MovingAverage(100) for k in loss_types}

    def backward_and_log(prefix, net_outs, targets, masks, num_crowds, extra_loss=None):
        optimizer.zero_grad()

        out = net_outs["pred_outs"]
        losses = criterion(out, targets, masks, num_crowds)

        losses = {k: v.mean() for k, v in losses.items()}  # Mean here because Dataparallel

        if extra_loss is not None:
            assert type(extra_loss) == dict
            losses.update(extra_loss)

        loss = sum([losses[k] for k in losses])

        # Backprop
        loss.backward()  # Do this to free up vram even if loss is not finite
        if torch.isfinite(loss).item():
            optimizer.step()

        # Add the loss to the moving average for bookkeeping
        for k in losses:
            loss_avgs[k].add(losses[k].item())
            w.add_scalar('{prefix}/{key}'.format(prefix=prefix, key=k), losses[k].item())

        return losses

    logger.info('Begin training!')
    # try-except so you can use ctrl+c to save early and stop training
    try:
        for epoch in range(num_epochs):

            # Resume from start_iter
            if (epoch + 1) * epoch_size < iteration:
                continue

            while True:
                data_start_time = time.perf_counter()
                datum = next(data_loader_iter)
                data_end_time = time.perf_counter()
                data_time = data_end_time - data_start_time
                if iteration != args.start_iter:
                    data_time_avg.add(data_time)
                # Stop if we've reached an epoch if we're resuming from start_iter
                if iteration == (epoch + 1) * epoch_size:
                    break

                # Stop at the configured number of iterations even if mid-epoch
                if iteration == cfg.max_iter:
                    break

                # Change a config setting if we've reached the specified iteration
                changed = False
                for change in cfg.delayed_settings:
                    if iteration >= change[0]:
                        changed = True
                        cfg.replace(change[1])

                        # Reset the loss averages because things might have changed
                        for avg in loss_avgs:
                            avg.reset()

                # If a config setting was changed, remove it from the list so we don't keep checking
                if changed:
                    cfg.delayed_settings = [x for x in cfg.delayed_settings if x[0] > iteration]

                # Warm up by linearly interpolating the learning rate from some smaller value
                if cfg.lr_warmup_until > 0 and iteration <= cfg.lr_warmup_until and cfg.lr_warmup_init < args.lr:
                    set_lr(optimizer,
                           (args.lr - cfg.lr_warmup_init) * (iteration / cfg.lr_warmup_until) + cfg.lr_warmup_init)

                elif cfg.lr_schedule == 'cosine':
                    set_lr(optimizer, args.lr * ((math.cos(math.pi * iteration / cfg.max_iter) + 1.) * .5))

                # Adjust the learning rate at the given iterations, but also if we resume from past that iteration
                while cfg.lr_schedule == 'step' and step_index < len(cfg.lr_steps) and iteration >= cfg.lr_steps[
                    step_index]:
                    step_index += 1
                    set_lr(optimizer, args.lr * (args.gamma ** step_index))

                global lr
                w.add_scalar('meta/lr', lr)

                if cfg.dataset.name == "FlyingChairs":
                    imgs_1, imgs_2, flows = prepare_flow_data(datum)
                    net_outs = net(None, extras=(imgs_1, imgs_2))
                    # Compute Loss
                    optimizer.zero_grad()

                    losses = criterion(net_outs, flows)

                    losses = {k: v.mean() for k, v in losses.items()}  # Mean here because Dataparallel
                    loss = sum([losses[k] for k in losses])

                    # Backprop
                    loss.backward()  # Do this to free up vram even if loss is not finite
                    if torch.isfinite(loss).item():
                        optimizer.step()

                    # Add the loss to the moving average for bookkeeping
                    for k in losses:
                        loss_avgs[k].add(losses[k].item())
                        w.add_scalar('loss/%s' % k, losses[k].item())

                elif cfg.dataset.joint or not cfg.dataset.is_video:
                    if cfg.dataset.joint:
                        joint_datum = next(joint_data_loader_iter)
                        # Load training data
                        # Note, for training on multiple gpus this will use the custom replicate and gather I wrote up there
                        images, targets, masks, num_crowds = prepare_data(joint_datum)
                    else:
                        images, targets, masks, num_crowds = prepare_data(datum)
                    extras = {"backbone": "full", "interrupt": False,
                              "moving_statistics": {"aligned_feats": []}}
                    net_outs = net(images, extras=extras)
                    run_name = "joint" if cfg.dataset.joint else "compute"
                    losses = backward_and_log(run_name, net_outs, targets, masks, num_crowds)

                # Forward Pass
                if cfg.dataset.is_video:
                    # reference frames
                    references = []
                    moving_statistics = {"aligned_feats": [], "conf_hist": []}
                    for idx, frame in enumerate(datum[:0:-1]):
                        images, annots = frame

                        extras = {"backbone": "full", "interrupt": True, "keep_statistics": True,
                                  "moving_statistics": moving_statistics}

                        with torch.no_grad():
                            net_outs = net(images, extras=extras)

                        moving_statistics["feats"] = net_outs["feats"]
                        moving_statistics["lateral"] = net_outs["lateral"]

                        keys_to_save = ("outs_phase_1", "outs_phase_2")
                        for key in set(net_outs.keys()) - set(keys_to_save):
                            del net_outs[key]
                        references.append(net_outs)

                    # key frame with annotation, but not compute full backbone
                    frame = datum[0]
                    images, annots = frame
                    frame = (images, annots,)
                    images, targets, masks, num_crowds = prepare_data(frame)

                    extras = {"backbone": "full", "interrupt": not cfg.flow.base_backward,
                              "moving_statistics": moving_statistics}
                    gt_net_outs = net(images, extras=extras)
                    if cfg.flow.base_backward:
                        losses = backward_and_log("compute", gt_net_outs, targets, masks, num_crowds)

                    keys_to_save = ("outs_phase_1", "outs_phase_2")
                    for key in set(gt_net_outs.keys()) - set(keys_to_save):
                        del gt_net_outs[key]

                    # now do the warp
                    if len(references) > 0:
                        reference_frame = references[0]
                        extras = {"backbone": "partial", "moving_statistics": moving_statistics}

                        net_outs = net(images, extras=extras)
                        extra_loss = yolact_net.extra_loss(net_outs, gt_net_outs)

                        losses = backward_and_log("warp", net_outs, targets, masks, num_crowds, extra_loss=extra_loss)

                cur_time = time.time()
                elapsed = cur_time - last_time
                last_time = cur_time
                w.add_scalar('meta/data_time', data_time)
                w.add_scalar('meta/iter_time', elapsed)

                # Exclude graph setup from the timing information
                if iteration != args.start_iter:
                    time_avg.add(elapsed)
                print(f"process_value={int((iteration / cfg.max_iter) * 90 + 6)}")

                if iteration % args.print_every_iternum == 0:
                    eta_str = \
                    str(datetime.timedelta(seconds=(cfg.max_iter - iteration) * time_avg.get_avg())).split('.')[0]
                    if torch.cuda.is_available():
                        max_mem_mb = torch.cuda.max_memory_allocated() / 1024.0 / 1024.0
                        # torch.cuda.reset_max_memory_allocated()
                    else:
                        max_mem_mb = 0

                    logger.info(
                        """eta: {eta}  epoch: {epoch}  iter: {iter}  {losses}  {loss_total}  time: {time}  data_time: {data_time}  lr: {lr}  {memory}""".format(
                            eta=eta_str, epoch=epoch, iter=iteration,
                            losses="  ".join(
                                ["{}: {:.3f}".format(k, loss_avgs[k].get_avg()) for k in losses]
                            ),
                            loss_total="T: {:.3f}".format(sum([loss_avgs[k].get_avg() for k in losses])),
                            data_time="{:.3f}".format(data_time_avg.get_avg()),
                            time="{:.3f}".format(elapsed),
                            lr="{:.6f}".format(lr), memory="max_mem: {:.0f}M".format(max_mem_mb)
                        ))

                if rank == 0 and iteration % 100 == 0:

                    if cfg.flow.train_flow:
                        import flowiz as fz
                        from yolact_edge.layers.warp_utils import deform_op
                        tgt_size = (64, 64)
                        flow_size = flows.size()[2:]
                        vis_data = []
                        for pred_flow in net_outs:
                            vis_data.append(pred_flow)

                        deform_gt = deform_op(imgs_2, flows)
                        flows_pred = [F.interpolate(x, size=flow_size, mode='bilinear', align_corners=False) for x in
                                      net_outs]
                        deform_preds = [deform_op(imgs_2, x) for x in flows_pred]

                        vis_data.append(F.interpolate(flows, size=tgt_size, mode='area'))

                        vis_data = [F.interpolate(flow[:1], size=tgt_size) for flow in vis_data]
                        vis_data = [fz.convert_from_flow(flow[0].data.cpu().numpy().transpose(1, 2, 0))
                                    .transpose(2, 0, 1).astype('float32') / 255
                                    for flow in vis_data]

                        def convert_image(image):
                            image = F.interpolate(image, size=tgt_size, mode='area')
                            image = image[0]
                            image = image.data.cpu().numpy()
                            image = image[::-1]
                            image = image.transpose(1, 2, 0)
                            image = image * np.array(STD) + np.array(MEANS)
                            image = image.transpose(2, 0, 1)
                            image = image / 255
                            image = np.clip(image, -1, 1)
                            image = image[::-1]
                            return image

                        vis_data.append(convert_image(imgs_1))
                        vis_data.append(convert_image(imgs_2))
                        vis_data.append(convert_image(deform_gt))
                        vis_data.extend([convert_image(x) for x in deform_preds])

                        vis_data_stack = np.stack(vis_data, axis=0)
                        w.add_images("preds_flow", vis_data_stack)

                    elif cfg.flow.warp_mode == "flow":
                        import flowiz as fz
                        tgt_size = (64, 64)
                        vis_data = []
                        for pred_flow, _, _ in net_outs["preds_flow"]:
                            vis_data.append(pred_flow)

                        vis_data = [F.interpolate(flow[:1], size=tgt_size) for flow in vis_data]
                        vis_data = [fz.convert_from_flow(flow[0].data.cpu().numpy().transpose(1, 2, 0))
                                    .transpose(2, 0, 1).astype('float32') / 255
                                    for flow in vis_data]
                        input_image = F.interpolate(images, size=tgt_size, mode='area')
                        input_image = input_image[0]
                        input_image = input_image.data.cpu().numpy()
                        input_image = input_image.transpose(1, 2, 0)
                        input_image = input_image * np.array(STD[::-1]) + np.array(MEANS[::-1])
                        input_image = input_image.transpose(2, 0, 1)
                        input_image = input_image / 255
                        input_image = np.clip(input_image, -1, 1)
                        vis_data.append(input_image)

                        vis_data_stack = np.stack(vis_data, axis=0)
                        w.add_images("preds_flow", vis_data_stack)

                iteration += 1
                w.set_step(iteration)

                if rank == 0 and iteration % args.save_interval == 0 and iteration != args.start_iter:
                    if args.keep_latest:
                        latest = SavePath.get_latest(args.save_folder, cfg.name)

                    logger.info('Saving state, iter: {}'.format(iteration))
                    yolact_net.save_weights(save_path(epoch, iteration))

                    if args.keep_latest and latest is not None:
                        if args.keep_latest_interval <= 0 or iteration % args.keep_latest_interval != args.save_interval:
                            logger.info('Deleting old save...')
                            os.remove(latest)

                # # This is done per iter
                # print('the map of {} iter: '.format(iteration))
                # compute_validation_map(yolact_net, val_dataset)
                # misc.barrier()

            misc.barrier()

            # # This is done per epoch 因为这里会涉及到 eval 文件里面的参数配置，两者对不上时会出错，暂不使用
            # if args.validation_epoch > 0:
            #     if epoch % args.validation_epoch == 0 and epoch > 0:
            #         if rank == 0:
            #             compute_validation_map(yolact_net, val_dataset)
            #         misc.barrier()


    except KeyboardInterrupt:
        misc.barrier()
        if args.interrupt_no_save:
            logger.info('No save on interrupt, just exiting...')
        elif rank == 0:
            print('Stopping early. Saving network...')
            # Delete previous copy of the interrupted network so we don't spam the weights folder
            SavePath.remove_interrupt(args.save_folder)

            yolact_net.save_weights(save_path(epoch, repr(iteration) + '_interrupt'))
        return

    if rank == 0:
        yolact_net.save_weights(save_path(epoch, iteration))
    config.set_para('segmentation', 'train_output', save_path(epoch, iteration))
    config.set_para('segmentation', 'class_names', class_names)
    config.yaml_dump(yaml_file)
    return 'INFO:Traning success!'


def set_lr(optimizer, new_lr):
    global lr
    lr = new_lr
    for param_group in optimizer.param_groups:
        param_group['lr'] = new_lr


def prepare_flow_data(datum):
    imgs_1, imgs_2, flows = datum

    if args.cuda:
        imgs_1 = Variable(imgs_1.cuda(non_blocking=True), requires_grad=False)
        imgs_2 = Variable(imgs_2.cuda(non_blocking=True), requires_grad=False)
        flows = Variable(flows.cuda(non_blocking=True), requires_grad=False)
    else:
        imgs_1 = Variable(imgs_1, requires_grad=False)
        imgs_2 = Variable(imgs_2, requires_grad=False)
        flows = Variable(flows, requires_grad=False)

    return imgs_1, imgs_2, flows


def prepare_data(datum):
    images, (targets, masks, num_crowds) = datum

    if args.cuda:
        images = Variable(images.cuda(non_blocking=True), requires_grad=False)
        targets = [Variable(ann.cuda(non_blocking=True), requires_grad=False) if ann is not None else ann for ann in
                   targets]
        masks = [Variable(mask.cuda(non_blocking=True), requires_grad=False) if mask is not None else mask for mask in
                 masks]
    else:
        images = Variable(images, requires_grad=False)
        targets = [Variable(ann, requires_grad=False) for ann in targets]
        masks = [Variable(mask, requires_grad=False) for mask in masks]

    return images, targets, masks, num_crowds


def compute_validation_loss(net, data_loader, criterion):
    global loss_types

    with torch.no_grad():
        losses = {}

        # Don't switch to eval mode because we want to get losses
        iterations = 0
        for datum in data_loader:
            images, targets, masks, num_crowds = prepare_data(datum)
            out = net(images)

            _losses = criterion(out, targets, masks, num_crowds)

            for k, v in _losses.items():
                v = v.mean().item()
                if k in losses:
                    losses[k] += v
                else:
                    losses[k] = v

            iterations += 1
            if args.validation_size <= iterations * args.batch_size:
                break

        for k in losses:
            losses[k] /= iterations

        loss_labels = sum([[k, losses[k]] for k in loss_types if k in losses], [])
        print(('Validation ||' + (' %s: %.3f |' * len(losses)) + ')') % tuple(loss_labels), flush=True)


def compute_validation_map(yolact_net, dataset):
    with torch.no_grad():
        yolact_net.eval()
        logger = logging.getLogger("yolact.eval")
        logger.info("Computing validation mAP (this may take a while)...")
        eval_script.evaluate(yolact_net, dataset, train_mode=True, train_cfg=cfg)
        yolact_net.train()


def setup_eval():
    eval_script.parse_args(['--no_bar', '--fast_eval', '--max_images=' + str(args.validation_size)])


if __name__ == '__main__':
    if args.num_gpus is None:
        args.num_gpus = torch.cuda.device_count()
    if args.num_gpus > 1:
        mp.spawn(train, nprocs=args.num_gpus, args=(args,), daemon=False)
    else:
        try:
            ret = train(0, args=args)
            print(ret)
        except Exception as e:
            print(f"ERROR:{e}")
