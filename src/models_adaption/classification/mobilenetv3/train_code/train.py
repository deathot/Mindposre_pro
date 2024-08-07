# -*- coding: utf-8 -*-
import os
import sys
import json
import argparse

import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import transforms, datasets
from tqdm import tqdm
import urllib

from geffnet.mobilenetv3 import mobilenetv3_large_100
import time
import sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))
sys.path.append(BASE_DIR)
from src.models_adaption.config.config import ModelConfig


def main(model, num_class, data_path, batch_size, epochs, output_path,early_stop_setting):
    # check input params.
    if not os.path.exists(model):
        with open("../../../../../version.json") as download_urls:
            try:
                url = json.load(download_urls)["model_urls"]['mobilenetv3_large']
                urllib.request.urlretrieve(url, './weights/mobilenetv3_large_100.pth')
            except Exception as e:
                print(e)
                return 'ERROR: model not exists.'
    if num_class <= 0 and num_class >= 200:
        return 'ERROR: num_class out of range (0, 200)'
    if not os.path.exists(data_path):
        return 'ERROR: data_path not exists.'
    if batch_size <= 0:
        return 'ERROR: batch_size should bigger than 0.'
    if epochs <= 0:
        return 'ERROR: epochs should bigger than 0.'
    if os.path.exists(output_path):
        return 'ERROR: output_path already exists.'
    os.mkdir(output_path)
    t1 = time.time()
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print("using {} device.".format(device))

    data_transform = {
        "train": transforms.Compose([transforms.RandomResizedCrop(224),
                                     transforms.RandomHorizontalFlip(),
                                     transforms.ToTensor(),
                                     transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])]),
        "val": transforms.Compose([transforms.Resize(256),
                                   transforms.CenterCrop(224),
                                   transforms.ToTensor(),
                                   transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])}

    assert os.path.exists(data_path), "{} path does not exist.".format(data_path)
    train_dataset = datasets.ImageFolder(root=os.path.join(data_path, "train"), transform=data_transform["train"])
    train_num = len(train_dataset)

    class_list = train_dataset.class_to_idx
    class_dict = dict((val, key) for key, val in class_list.items())
    # write dict into json file

    json_str = json.dumps(class_dict, indent=4)
    with open(f'{output_path}/class_indices.json', 'w') as json_file:
        json_file.write(json_str)

    nw = min([os.cpu_count(), batch_size if batch_size > 1 else 0, 8])  # number of workers
    print('Using {} dataloader workers every process'.format(nw))

    train_loader = torch.utils.data.DataLoader(train_dataset,
                                               batch_size=batch_size, shuffle=True,
                                               num_workers=nw)

    validate_dataset = datasets.ImageFolder(root=os.path.join(data_path, "val"),
                                            transform=data_transform["val"])
    val_num = len(validate_dataset)
    validate_loader = torch.utils.data.DataLoader(validate_dataset,
                                                  batch_size=batch_size, shuffle=False,
                                                  num_workers=nw)

    print("using {} images for training, {} images for validation.".format(train_num,
                                                                           val_num))

    # create model
    net = mobilenetv3_large_100(pretrained=False, num_classes=num_class)

    # load pretrain weights
    # download url: https://github.com/rwightman/pytorch-image-models/releases/download/v0.1-weights/mobilenetv3_large_100_ra-f55367f5.pth

    assert os.path.exists(model), "file {} dose not exist.".format(model)
    pre_weights = torch.load(model, map_location='cpu')

    # delete classifier weights
    pre_dict = {k: v for k, v in pre_weights.items() if net.state_dict()[k].numel() == v.numel()}
    missing_keys, unexpected_keys = net.load_state_dict(pre_dict, strict=False)

    # freeze features weights
    for param in net.parameters():
        param.requires_grad = False
    for param in net.classifier.parameters():
        param.requires_grad = True

    net.to(device)

    # define loss function
    loss_function = nn.CrossEntropyLoss()

    # construct an optimizer
    params = [p for p in net.parameters() if p.requires_grad]
    optimizer = optim.Adam(params, lr=0.0001)
    best_acc = 0.0
    train_steps = len(train_loader)
    best_epoch = 0
    best_fitness = 0

    for epoch in range(epochs):
        print(f"process_value={int((epoch) / (epochs) * 90 + 6)}")
        # train
        net.train()
        running_loss = 0.0
        train_bar = tqdm(train_loader, file=sys.stdout)
        for step, data in enumerate(train_bar):
            images, labels = data
            optimizer.zero_grad()
            logits = net(images.to(device))
            loss = loss_function(logits, labels.to(device))
            loss.backward()
            optimizer.step()

            # print statistics
            running_loss += loss.item()
            train_bar.desc = "train epoch[{}/{}] loss:{:.3f}".format(epoch + 1, epochs, loss)

        # validate
        net.eval()
        acc = 0.0  # accumulate accurate number / epoch
        with torch.no_grad():
            val_bar = tqdm(validate_loader, file=sys.stdout)
            for val_data in val_bar:
                val_images, val_labels = val_data
                outputs = net(val_images.to(device))

                predict_y = torch.max(outputs, dim=1)[1]
                acc += torch.eq(predict_y, val_labels.to(device)).sum().item()

                val_bar.desc = "valid epoch[{}/{}]".format(epoch + 1,
                                                           epochs)
        val_accurate = acc / val_num
        print('[epoch %d] train_loss: %.3f  val_accuracy: %.3f' %
              (epoch + 1, running_loss / train_steps, val_accurate))

        if val_accurate > best_acc:
            best_acc = val_accurate
            torch.save(net.state_dict(), f'{output_path}/MobileNetV3_large.pth')

        if early_stop_setting['enable']:
            if val_accurate > best_fitness:
                best_fitness = val_accurate
                best_epoch = epoch

            if epoch-best_epoch > early_stop_setting['tolerance']:
                print(f"INFO: Early stopping after {early_stop_setting['tolerance']} epochs without improvement.")
                break

            if val_accurate > early_stop_setting['threshold']:
                print(f"INFO: Early stopping as validation accuracy ({val_accurate:.3f}) exceeded threshold ({early_stop_setting['threshold']:.3f}).")
                break

    t2 = time.time()
    print('Total Time: {:.1f}ms'.format((t2 - t1) * 1000))
    return 'INFO:Training Success!'


def parse_args():
    parser = argparse.ArgumentParser("Convert '.pt' model into '.onnx'.")
    parser.add_argument("--model", required=False, type=str, help="Input om model path.",
                        default=r"./weights/mobilenetv3_large_100.pth")
    parser.add_argument("--num_class", required=True, type=int, help="class_num.")
    parser.add_argument("--batch_size", required=False, type=int, help="batch_size.",
                        default=16)
    parser.add_argument("--epochs", required=False, type=int, help="epochs.",
                        default=100)
    parser.add_argument("--data_path", required=True, type=str, help="data path.")
    parser.add_argument("--output_path", required=False, type=str, help="output_path.",
                        default="./")

    parser.set_defaults()
    return parser.parse_args()


if __name__ == '__main__':
    # args = parse_args()
    # model = args.model
    # num_class = args.num_class
    # data_path = args.data_path
    # batch_size = args.batch_size
    # epochs = args.epochs
    # output_path = args.output_path
    yaml_file = '../../../config/config.yaml'
    config = ModelConfig(yaml_file)
    yaml_data = config.get_yaml_data().get('classification')
    model = './weights/mobilenetv3_large_100.pth'
    num_class = yaml_data['class_num']
    data_path = yaml_data['trans_output']
    batch_size = yaml_data['train_batch_size']
    epochs = yaml_data['epochs']

    early_stopping_setting = {
        'enable': yaml_data.get('earlystopping_enabled', False),
        'threshold': yaml_data.get('earlystopping_threshold', 0.99),
        'tolerance': yaml_data.get('earlystopping_tolerance', 10)
    }

    train_output_path = os.path.join(yaml_data['output_path'], 'train_output')
    config.set_para('classification', 'train_output', train_output_path)
    config.yaml_dump(yaml_file)
    try:
        ret = main(model, num_class, data_path, batch_size, epochs, train_output_path,early_stopping_setting)
        print(ret)
    except Exception as e:
        print(f"ERROR:{e}")