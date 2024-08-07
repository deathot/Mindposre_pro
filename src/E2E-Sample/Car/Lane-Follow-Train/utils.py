import os
import random

import albumentations as A
import cv2
import torch
from albumentations.pytorch.transforms import ToTensorV2
from torch.utils.data import DataLoader
from torch.utils.data import Dataset


class LaneDataset(Dataset):
    def __init__(self, image_dir, labels, transform=None):
        self.image_dir = image_dir
        self.transform = transform
        self.labels = labels

    def __len__(self):
        return len(self.labels)

    def __getitem__(self, index):
        image_name, label = self.labels[index].split(' ')
        label = float(label)

        image_path = os.path.join(self.image_dir, image_name)

        image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = self.transform(image=image)['image']

        target = torch.tensor(label)
        return image, target


def split_data(data_dir, ratio=0.9, seed=42):
    """Split data into training and validation sets."""
    with open(os.path.join(data_dir, 'label.txt'), 'r') as f:
        all_labels = f.readlines()

    random.seed(seed)
    random.shuffle(all_labels)
    train_labels = all_labels[:int(len(all_labels) * ratio)]
    val_labels = all_labels[int(len(all_labels) * ratio):]
    print(f"Training data size: {len(train_labels)}")
    print(f"Validation data size: {len(val_labels)}")
    return train_labels, val_labels


def get_transforms(height, width):
    train_transform = A.Compose([
        A.Resize(height, width),
        A.RandomBrightnessContrast(p=0.5),
        A.HueSaturationValue(val_shift_limit=10, p=0.2),
        # use mean and std from ImageNet
        A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
        ToTensorV2()
    ])

    val_transform = A.Compose([
        A.Resize(height, width),
        # use mean and std from ImageNet
        A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
        ToTensorV2()
    ])

    return train_transform, val_transform


def prepare_dataset(dataset_path, batch_size, height, width):
    image_dir = os.path.join(dataset_path, "images")
    train_labels, val_labels = split_data(dataset_path)

    train_transform, val_transform = get_transforms(height, width)
    train_dataset = LaneDataset(image_dir, train_labels, transform=train_transform)
    val_dataset = LaneDataset(image_dir, val_labels, transform=val_transform)

    train_loader = DataLoader(train_dataset, batch_size=batch_size, pin_memory=True,
                              shuffle=True, num_workers=8, drop_last=False)

    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False,
                            num_workers=8, pin_memory=True)
    return train_loader, val_loader
