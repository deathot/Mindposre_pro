"""Script for multi-gpu training."""
import json
import os
import sys
import shutil

import numpy as np
import torch
import torch.nn as nn
import torch.utils.data
from tensorboardX import SummaryWriter
from tqdm import tqdm
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '../'))
from alphapose.models import builder
from alphapose.opt import cfg, logger, opt
from alphapose.utils.logger import board_writing, debug_writing
from alphapose.utils.metrics import DataLogger, calc_accuracy, calc_integral_accuracy, evaluate_mAP
from alphapose.utils.transforms import get_func_heatmap_to_coord

valid_batch = 1
if opt.sync:
    norm_layer = nn.SyncBatchNorm
else:
    norm_layer = nn.BatchNorm2d


def train(opt, train_loader, m, criterion, optimizer, writer):
    loss_logger = DataLogger()
    acc_logger = DataLogger()

    combined_loss = (cfg.LOSS.get('TYPE') == 'Combined')

    m.train()
    norm_type = cfg.LOSS.get('NORM_TYPE', None)

    train_loader = tqdm(train_loader, dynamic_ncols=True)

    for i, (inps, labels, label_masks, _, bboxes) in enumerate(train_loader):
        if isinstance(inps, list):
            inps = [inp.cpu().requires_grad_() for inp in inps]
        else:
            inps = inps.cpu().requires_grad_()
        if isinstance(labels, list):
            labels = [label.cpu() for label in labels]
            label_masks = [label_mask.cpu() for label_mask in label_masks]
        else:
            labels = labels.cpu()
            label_masks = label_masks.cpu()

        output = m(inps)

        if cfg.LOSS.get('TYPE') == 'MSELoss':
            loss = 0.5 * criterion(output.mul(label_masks), labels.mul(label_masks))
            acc = calc_accuracy(output.mul(label_masks), labels.mul(label_masks))
        elif cfg.LOSS.get('TYPE') == 'Combined':
            if output.size()[1] == 68:
                face_hand_num = 42
            else:
                face_hand_num = 110

            output_body_foot = output[:, :-face_hand_num, :, :]
            output_face_hand = output[:, -face_hand_num:, :, :]
            num_body_foot = output_body_foot.shape[1]
            num_face_hand = output_face_hand.shape[1]

            label_masks_body_foot = label_masks[0]
            label_masks_face_hand = label_masks[1]

            labels_body_foot = labels[0]
            labels_face_hand = labels[1]

            loss_body_foot = 0.5 * criterion[0](output_body_foot.mul(label_masks_body_foot), labels_body_foot.mul(label_masks_body_foot))
            acc_body_foot = calc_accuracy(output_body_foot.mul(label_masks_body_foot), labels_body_foot.mul(label_masks_body_foot))

            loss_face_hand = criterion[1](output_face_hand, labels_face_hand, label_masks_face_hand)
            acc_face_hand = calc_integral_accuracy(output_face_hand, labels_face_hand, label_masks_face_hand, output_3d=False, norm_type=norm_type)

            loss_body_foot *= 100
            loss_face_hand *= 0.01

            loss = loss_body_foot + loss_face_hand
            acc = acc_body_foot * num_body_foot / (num_body_foot + num_face_hand) + acc_face_hand * num_face_hand / (num_body_foot + num_face_hand)
        else:
            loss = criterion(output, labels, label_masks)
            acc = calc_integral_accuracy(output, labels, label_masks, output_3d=False, norm_type=norm_type)

        if isinstance(inps, list):
            batch_size = inps[0].size(0)
        else:
            batch_size = inps.size(0)

        loss_logger.update(loss.item(), batch_size)
        acc_logger.update(acc, batch_size)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        opt.trainIters += 1
        # Tensorboard
        if opt.board:
            board_writing(writer, loss_logger.avg, acc_logger.avg, opt.trainIters, 'Train')

        # Debug
        if opt.debug and not i % 10:
            debug_writing(writer, output, labels, inps, opt.trainIters)

        # TQDM
        train_loader.set_description(
            'loss: {loss:.8f} | acc: {acc:.4f}'.format(
                loss=loss_logger.avg,
                acc=acc_logger.avg)
        )

    train_loader.close()

    return loss_logger.avg, acc_logger.avg


def validate(output_path, m, opt, heatmap_to_coord, batch_size=20):
    cfg.DATASET.TEST.DET_FILE = os.path.join(output_path[:-10], 'json/test_det_yolo.json')
    det_dataset = builder.build_dataset(cfg.DATASET.TEST, preset_cfg=cfg.DATA_PRESET, train=False, opt=opt)
    det_loader = torch.utils.data.DataLoader(
        det_dataset, batch_size=batch_size, shuffle=False, num_workers=0, drop_last=False)
    kpt_json = []
    eval_joints = det_dataset.EVAL_JOINTS

    m.eval()

    norm_type = cfg.LOSS.get('NORM_TYPE', None)
    hm_size = cfg.DATA_PRESET.HEATMAP_SIZE
    combined_loss = (cfg.LOSS.get('TYPE') == 'Combined')

    halpe = (cfg.DATA_PRESET.NUM_JOINTS == 133) or (cfg.DATA_PRESET.NUM_JOINTS == 136)

    for inps, crop_bboxes, bboxes, img_ids, scores, imghts, imgwds in tqdm(det_loader, dynamic_ncols=True):
        if isinstance(inps, list):
            inps = [inp.cpu() for inp in inps]
        else:
            inps = inps.cpu()
        output = m(inps)

        pred = output
        assert pred.dim() == 4
        pred = pred[:, eval_joints, :, :]

        if output.size()[1] == 68:
            face_hand_num = 42
        else:
            face_hand_num = 110

        for i in range(output.shape[0]):
            bbox = crop_bboxes[i].tolist()
            if combined_loss:
                pose_coords_body_foot, pose_scores_body_foot = heatmap_to_coord[0](
                    pred[i][det_dataset.EVAL_JOINTS[:-face_hand_num]], bbox, hm_shape=hm_size, norm_type=norm_type)
                pose_coords_face_hand, pose_scores_face_hand = heatmap_to_coord[1](
                    pred[i][det_dataset.EVAL_JOINTS[-face_hand_num:]], bbox, hm_shape=hm_size, norm_type=norm_type)
                pose_coords = np.concatenate((pose_coords_body_foot, pose_coords_face_hand), axis=0)
                pose_scores = np.concatenate((pose_scores_body_foot, pose_scores_face_hand), axis=0)
            else:
                pose_coords, pose_scores = heatmap_to_coord(
                    pred[i][det_dataset.EVAL_JOINTS], bbox, hm_shape=hm_size, norm_type=norm_type)

            keypoints = np.concatenate((pose_coords, pose_scores), axis=1)
            keypoints = keypoints.reshape(-1).tolist()

            data = dict()
            data['bbox'] = bboxes[i, 0].tolist()
            data['image_id'] = int(img_ids[i])
            data['score'] = float(scores[i] + np.mean(pose_scores) + 1.25 * np.max(pose_scores))
            data['category_id'] = 1
            data['keypoints'] = keypoints

            kpt_json.append(data)

    sysout = sys.stdout
    json_path = os.path.join(output_path, 'test_kpt.json')
    with open(json_path, 'w') as fid:
        json.dump(kpt_json, fid)
    res = evaluate_mAP(json_path, ann_type='keypoints', ann_file=os.path.join(cfg.DATASET.VAL.ROOT, cfg.DATASET.VAL.ANN), halpe=halpe)
    sys.stdout = sysout
    return res


def validate_gt(output_path, m, opt, cfg, heatmap_to_coord, batch_size=20):
    gt_val_dataset = builder.build_dataset(cfg.DATASET.VAL, preset_cfg=cfg.DATA_PRESET, train=False)
    eval_joints = gt_val_dataset.EVAL_JOINTS

    gt_val_loader = torch.utils.data.DataLoader(
        gt_val_dataset, batch_size=batch_size, shuffle=False, num_workers=0, drop_last=False)
    kpt_json = []
    m.eval()

    norm_type = cfg.LOSS.get('NORM_TYPE', None)
    hm_size = cfg.DATA_PRESET.HEATMAP_SIZE
    combined_loss = (cfg.LOSS.get('TYPE') == 'Combined')

    halpe = (cfg.DATA_PRESET.NUM_JOINTS == 133) or (cfg.DATA_PRESET.NUM_JOINTS == 136)
    for inps, labels, label_masks, img_ids, bboxes in tqdm(gt_val_loader, dynamic_ncols=True):
        if isinstance(inps, list):
            inps = [inp.cpu() for inp in inps]
        else:
            inps = inps.cpu()
        output = m(inps)

        pred = output
        assert pred.dim() == 4
        pred = pred[:, eval_joints, :, :]

        if output.size()[1] == 68:
            face_hand_num = 42
        else:
            face_hand_num = 110

        for i in range(output.shape[0]):
            bbox = bboxes[i].tolist()
            if combined_loss:
                pose_coords_body_foot, pose_scores_body_foot = heatmap_to_coord[0](
                    pred[i][gt_val_dataset.EVAL_JOINTS[:-face_hand_num]], bbox, hm_shape=hm_size, norm_type=norm_type)
                pose_coords_face_hand, pose_scores_face_hand = heatmap_to_coord[1](
                    pred[i][gt_val_dataset.EVAL_JOINTS[-face_hand_num:]], bbox, hm_shape=hm_size, norm_type=norm_type)
                pose_coords = np.concatenate((pose_coords_body_foot, pose_coords_face_hand), axis=0)
                pose_scores = np.concatenate((pose_scores_body_foot, pose_scores_face_hand), axis=0)
            else:
                pose_coords, pose_scores = heatmap_to_coord(
                    pred[i][gt_val_dataset.EVAL_JOINTS], bbox, hm_shape=hm_size, norm_type=norm_type)

            keypoints = np.concatenate((pose_coords, pose_scores), axis=1)
            keypoints = keypoints.reshape(-1).tolist()

            data = dict()
            data['bbox'] = bboxes[i].tolist()
            data['image_id'] = int(img_ids[i])
            data['score'] = float(np.mean(pose_scores) + 1.25 * np.max(pose_scores))
            data['category_id'] = 1
            data['keypoints'] = keypoints

            kpt_json.append(data)

    sysout = sys.stdout
    json_path = os.path.join(output_path, 'test_gt_kpt.json')
    with open(json_path, 'w') as fid:
        json.dump(kpt_json, fid)
    res = evaluate_mAP(json_path, ann_type='keypoints', ann_file=os.path.join(cfg.DATASET.VAL.ROOT, cfg.DATASET.VAL.ANN), halpe=halpe)
    sys.stdout = sysout
    return res


def main(output_path, batch_size, end_epoch, pretrained, try_load):
    cfg.DATASET.TRAIN.ROOT = opt.input_path
    cfg.DATASET.VAL.ROOT = opt.input_path
    cfg.DATASET.TEST.ROOT = opt.input_path
    cfg.DETECTOR.WEIGHTS = opt.weights
    
    logger.info('******************************')
    logger.info(opt)
    logger.info('******************************')
    logger.info(cfg)
    logger.info('******************************')

    if not os.path.isfile(opt.cfg):
        return 'ERROR: not found dataset yaml file.'
    if opt.batch_size <= 0:
        return 'ERROR: batch size should bigger than 0.'
    if opt.end_epoch <= 0:
        return 'ERROR: epochs should bigger than 0.'
    
    # Output Directory
    output_default = f"{output_path}\data.yaml".replace('\\\\', '\\')
    # Model Initialize
    m = preset_model(cfg, pretrained, try_load)
    m = nn.DataParallel(m).cpu()

    combined_loss = (cfg.LOSS.get('TYPE') == 'Combined')
    if combined_loss:
        criterion1 = builder.build_loss(cfg.LOSS.LOSS_1).cpu()
        criterion2 = builder.build_loss(cfg.LOSS.LOSS_2).cpu()
        criterion = [criterion1, criterion2]
    else:
        criterion = builder.build_loss(cfg.LOSS).cpu()

    if cfg.TRAIN.OPTIMIZER == 'adam':
        optimizer = torch.optim.Adam(m.parameters(), lr=cfg.TRAIN.LR)
    elif cfg.TRAIN.OPTIMIZER == 'rmsprop':
        optimizer = torch.optim.RMSprop(m.parameters(), lr=cfg.TRAIN.LR)

    lr_scheduler = torch.optim.lr_scheduler.MultiStepLR(
        optimizer, milestones=cfg.TRAIN.LR_STEP, gamma=cfg.TRAIN.LR_FACTOR)

    writer = SummaryWriter('.tensorboard/{}-{}'.format(opt.exp_id, cfg.FILE_NAME))

    train_dataset = builder.build_dataset(cfg.DATASET.TRAIN, preset_cfg=cfg.DATA_PRESET, train=True)
    train_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True, num_workers=0)

    heatmap_to_coord = get_func_heatmap_to_coord(cfg)

    opt.trainIters = 0
    best_epoch = 0
    best_fitness = 0
    for i in range(cfg.TRAIN.BEGIN_EPOCH, end_epoch):
        print(f"process_value={int(i / (end_epoch - cfg.TRAIN.BEGIN_EPOCH) * 90 + 6)}")
        opt.epoch = i
        current_lr = optimizer.state_dict()['param_groups'][0]['lr']

        logger.info(f'############# Starting Epoch {opt.epoch} | LR: {current_lr} #############')

        # Training
        loss, miou = train(opt, train_loader, m, criterion, optimizer, writer)
        logger.epochInfo('Train', opt.epoch, loss, miou)

        if opt.early_stopping:
            if miou > best_fitness:
                best_fitness = miou
                best_epoch = i

            if i - best_epoch > opt.early_stopping_tolerance:
                print(f"INFO: Early stopping after {opt.early_stopping_tolerance} epochs without improvement.")
                break

            if miou > opt.early_stopping_threshold:
                print(
                    f"INFO: Early stopping as mIOU ({miou:.3f}) exceeded threshold ({opt.early_stopping_threshold:.3f}).")
                break


        lr_scheduler.step()

        if (i + 1) % opt.snapshot == 0:
            # Save checkpoint
            torch.save(m.module.state_dict(), '{}/model_{}.pth'.format(output_default, opt.epoch))
            # Prediction Test
            with torch.no_grad():
                gt_AP = validate_gt(output_default, m.module, opt, cfg, heatmap_to_coord)
                rcnn_AP = validate(output_default, m.module, opt, heatmap_to_coord)
                logger.info(f'##### Epoch {opt.epoch} | gt mAP: {gt_AP} | rcnn mAP: {rcnn_AP} #####')

        # Time to add DPG
        if i == cfg.TRAIN.DPG_MILESTONE:
            torch.save(m.module.state_dict(), '{}/final.pth'.format(output_default))
            # Adjust learning rate
            for param_group in optimizer.param_groups:
                param_group['lr'] = cfg.TRAIN.LR
            lr_scheduler = torch.optim.lr_scheduler.MultiStepLR(optimizer, milestones=cfg.TRAIN.DPG_STEP, gamma=0.1)
            # Reset dataset
            train_dataset = builder.build_dataset(cfg.DATASET.TRAIN, preset_cfg=cfg.DATA_PRESET, train=True, dpg=True)
            train_loader = torch.utils.data.DataLoader(
                train_dataset, batch_size=batch_size, shuffle=True, num_workers=0)

    torch.save(m.module.state_dict(), '{}/final_DPG.pth'.format(output_default))
    return "INFO:Train success!"


def preset_model(cfg, pretrained, try_load):
    model = builder.build_sppe(cfg.MODEL, preset_cfg=cfg.DATA_PRESET)

    if pretrained:
        logger.info(f'Loading model from {pretrained}...')
        model.load_state_dict(torch.load(pretrained, map_location=torch.device('cpu')))
    elif try_load:
        logger.info(f'Loading model from {try_load}...')
        pretrained_state = torch.load(try_load, map_location=torch.device('cpu'))
        model_state = model.state_dict()
        pretrained_state = {k: v for k, v in pretrained_state.items()
                            if k in model_state and v.size() == model_state[k].size()}

        model_state.update(pretrained_state)
        model.load_state_dict(model_state)
    else:
        logger.info('Create new model')
        logger.info('=> init weights')
        model._initialize()

    return model


if __name__ == "__main__":
    resnet50_path = os.path.join(torch.hub.get_dir(), "checkpoints", 'resnet50-19c8e357.pth')
    if not os.path.exists(os.path.join(torch.hub.get_dir(), "checkpoints")):
        os.makedirs(os.path.join(torch.hub.get_dir(), "checkpoints"))
    if not os.path.exists(resnet50_path):
        with open("../../../../../version.json", "r") as download_urls:
            try:
                resnet50_url = json.load(download_urls)["model_urls"]['resnet50']
                torch.hub.download_url_to_file(resnet50_url, resnet50_path)
                print(f"Download {resnet50_url} to {resnet50_path}.")
                if not os.path.isfile(resnet50_path):
                    print("ERROR: Download file failed.")
                    raise Exception("ERROR: Download file failed.")
            except Exception as e:
                print(e)

    weight_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yolov3-spp.weights')
    if not os.path.exists(weight_path):
        with open("../../../../../version.json", "r") as download_urls:
            try:
                weight_url = json.load(download_urls)["model_urls"]["yolov3_weights"]
                torch.hub.download_url_to_file(weight_url, weight_path)
                print(f"Download {weight_url} to {weight_path}.")
                if not os.path.isfile(weight_path):
                    print("ERROR: Download file failed.")
                    raise Exception("ERROR: Download file failed.")
            except Exception as e:
                print(e)

    pth_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fast_res50_256x192.pth')
    if not os.path.exists(pth_path):
        with open("../../../../../version.json", "r") as download_urls:
            try:
                pth_url = json.load(download_urls)["model_urls"]["keypoint"]
                torch.hub.download_url_to_file(pth_url, pth_path)
                print(f"Download {pth_url} to {pth_path}.")
                if not os.path.isfile(pth_path):
                    print("ERROR: Download file failed.")
                    raise Exception("ERROR: Download file failed.")
            except Exception as e:
                print(e)

    yaml_file = '../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('keypoint')
    opt.input_path = yaml_data['trans_output']
    opt.cfg = os.path.join(yaml_data['trans_output'], 'data.yaml')
    opt.batch_size = yaml_data['train_batch_size']
    opt.end_epoch = yaml_data['epochs']
    opt.weights = weight_path
    opt.try_load = pth_path

    opt.early_stopping = yaml_data.get('earlystopping_enabled', False)
    opt.early_stopping_threshold = yaml_data.get('earlystopping_threshold', 0.99)
    opt.early_stopping_tolerance = yaml_data.get('earlystopping_tolerance', 10)

    try:
        ret = main(opt.output_path, opt.batch_size, opt.end_epoch, opt.pretrained, opt.try_load)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")
