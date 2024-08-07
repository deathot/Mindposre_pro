import argparse
import os

import torch
import torch.nn as nn
import yaml
from torch.optim.lr_scheduler import CosineAnnealingLR
from torchvision.models import resnet18
from tqdm.auto import tqdm

from utils import prepare_dataset


def validate(model, val_loader, criterion, device):
    val_loss = 0.
    model.eval()
    for data in tqdm(val_loader):
        inputs, labels = data
        inputs = inputs.to(device)
        labels = labels.to(device)
        labels = labels.unsqueeze(1)

        with torch.no_grad():
            outputs = model(inputs)

        loss = criterion(outputs, labels)
        val_loss += loss.item()
    val_loss /= len(val_loader)

    return val_loss


def train(model, train_loader, criterion, optimizer, scheduler, device):
    running_loss = 0.0
    model.train()
    for data in tqdm(train_loader):
        inputs, labels = data
        inputs = inputs.to(device)
        labels = labels.to(device)
        labels = labels.unsqueeze(1)

        optimizer.zero_grad()

        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()

        optimizer.step()
        scheduler.step()

        running_loss += loss.item()

    running_loss /= len(train_loader)
    return running_loss


def export(model, height, width, save_path):
    input_names = ["input"]
    output_names = ["output"]
    dummy_input = torch.randn(1, 3, height, width)
    model.to('cpu')
    model.eval()
    torch.onnx.export(model, dummy_input, save_path, input_names=input_names,
                      output_names=output_names, opset_version=11)
    print('ONNX model exported!')


def main(config):
    dataset_path = config['common']['dataset_path']
    output_dir = config['common']['output_dir']
    os.makedirs(output_dir, exist_ok=True, mode=0o750)

    device = config['train']['device']
    resume = config['train']['resume']
    num_epoch = config['train']['epoch_num']
    batch_size = config['train']['batch_size']
    lr = config['train']['lr']

    height = config['common']['height']
    width = config['common']['width']

    if not os.path.exists(dataset_path):
        rel_path = os.path.join(os.path.dirname(__file__), dataset_path)
        raise FileNotFoundError(f'The dataset "{rel_path}" was not found')

    train_loader, val_loader = prepare_dataset(dataset_path, batch_size,
                                               height=height,
                                               width=width)

    model = resnet18(pretrained=True)
    model.fc = torch.nn.Linear(512, 1)
    model.to(device)

    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    scheduler = CosineAnnealingLR(optimizer, T_max=len(train_loader) * num_epoch, eta_min=1e-6)

    if resume:
        model.load_state_dict(torch.load(os.path.join(output_dir, 'lfnet.pth')))
        print("finish loading..")

    save_model_path = os.path.join(output_dir, 'lfnet.pth')
    best_loss = float('inf')
    print('=' * 20 + 'Start training' + '=' * 20)
    for epoch in range(num_epoch):
        train_loss = train(model, train_loader, criterion, optimizer, scheduler, device)
        print(f'Epoch {epoch + 1} training loss: {train_loss}')

        val_loss = validate(model, val_loader, criterion, device)
        print(f'Epoch {epoch + 1} validation loss: {val_loss}')

        if val_loss < best_loss:
            best_loss = val_loss
            torch.save(model.state_dict(), save_model_path)
            print('Best model saved')

    print('=' * 20 + 'Finish training' + '=' * 20)

    if config['export']['to_onnx']:
        save_onnx_path = os.path.join(output_dir, config['export']['onnx_model_name'])
        export(model, height, width, save_onnx_path)


def arg_parse():
    parser = argparse.ArgumentParser(description='PyTorch Training')
    parser.add_argument('--config', default='config.yaml', type=str, help='config file path (default: config.yaml)')

    return parser.parse_args()


if __name__ == '__main__':
    args = arg_parse()
    with open(args.config, 'r') as f:
        config_dict = yaml.load(f, Loader=yaml.SafeLoader)

    main(config_dict)
