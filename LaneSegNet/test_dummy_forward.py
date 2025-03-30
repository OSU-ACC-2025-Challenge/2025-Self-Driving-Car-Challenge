import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), ".")))
import torch
from mmcv import Config
from laneseg.models import build_segmentor
from laneseg.utils import build_dataset

# Use a sample config
cfg = Config.fromfile("configs/openlanev2/lanesegnet_resnet18.yaml")

# Build model
model = build_segmentor(cfg.model)
model.eval()

# Use MPS or CPU (no CUDA on Mac)
device = torch.device("mps" if torch.backends.mps.is_available() else "cpu")
model = model.to(device)

# Create dummy input (1 image, 3 channels, 288x800)
dummy_input = torch.randn(1, 3, 288, 800).to(device)

# Forward pass
with torch.no_grad():
    output = model(dummy_input)
    print("Output shape:", output.shape)