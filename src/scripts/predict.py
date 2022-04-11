#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from pickletools import optimize
from torch.autograd import Variable
import cv2
from torch.utils.data import DataLoader, Dataset
import torch
from torchvision import transforms, utils, datasets, models
from PIL import Image
import numpy as np
import torch.nn as nn


# Model


class CNN(nn.Module):
    def __init__(self) -> None:
        super(CNN, self).__init__()
        # self.vgg16 = vgg
        # self.classifiter = nn.Sequential(
        #     nn.Linear(200704, 256),
        #     nn.ReLU(),
        #     nn.Dropout(p=0.5),
        #     nn.Linear(256, 128),
        #     nn.ReLU(),
        #     nn.Dropout(p=0.5),
        #     nn.Linear(128, 8),
        #     nn.Softmax(dim=1))
        self.vgg16 = nn.Sequential(
            nn.Conv2d(3, 16, 3),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 4),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3),
            nn.ReLU(),
            nn.MaxPool2d(2),
        )
        self.classifiter = nn.Sequential(
            nn.Linear(12544, 256),
            nn.ReLU(),
            nn.Dropout(p=0.5),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Dropout(p=0.5),
            nn.Linear(128, 8),
            nn.Softmax(dim=1))

    def forward(self, x):
        x = self.vgg16(x)
        x = x.view(x.size(0), -1)
        output = self.classifiter(x)
        return output

# Train


def predict(img):
    data_transform = transforms.Compose([transforms.Resize([128, 128]), transforms.RandomHorizontalFlip(),
                                         transforms.ToTensor(),
                                         transforms.Normalize(mean=[0.3238427, 0.1592992, 0.15665638],
                                                              std=[0.085483715, 0.13723177, 0.13991293])])
    img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    img = data_transform(img)
    img = torch.unsqueeze(img, dim=0)
    img = img.to('cpu')
    cnn = CNN()
    cnn = cnn.to('cpu')
    pth_path = '/home/sim2real/ep_ws/src/s2r_pipeline/src/scripts/cnn_real_1.27401.pth'
    cnn.load_state_dict(torch.load(pth_path, map_location='cpu'))
    cnn.eval()
    with torch.no_grad():
        output = torch.squeeze(cnn(img))
    return(output)
    # Save the Trained Model
