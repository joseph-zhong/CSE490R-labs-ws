

import torch.nn as nn
from torch.autograd import Variable


class MotionModel(nn.Module):
    def __init__(self, D_in, H1, H2, D_out):
        super(MotionModel, self).__init__()

        self.layers = nn.Sequential(
            nn.Linear(D_in, H1),
            nn.Dropout(0.2),
            nn.Linear(H1, H2),
            nn.ReLU(),
            nn.Linear(H2, D_out)
        )

    def forward(self, x):
        out = self.layers(x)
        return out

