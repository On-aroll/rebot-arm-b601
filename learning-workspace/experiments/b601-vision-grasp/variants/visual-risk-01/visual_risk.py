"""Visual position-risk score. No simulator truth or task metadata is accepted."""
import numpy as np
import torch
from torch import nn

from feedback import quality
from image_residual import image_patch
from policy import BOUND_M, features, valid_prediction


INPUTS = 140


def applicable(prediction):
    if not valid_prediction(prediction):
        return False
    x, y, _ = prediction["top_xyz_m"]
    return .265 <= x <= .395 and abs(y) <= .125


def risk_features(position_model, prediction, frame):
    if not applicable(prediction):
        return None
    numeric = features(prediction)
    patch = torch.from_numpy(image_patch(frame, prediction))[None]
    with torch.no_grad():
        embedding = position_model.image(patch)[0].numpy()
        delta = position_model(torch.from_numpy(numeric)[None], patch)[0].numpy()
    coverage = quality(prediction, frame["camera"])["coverage"]
    value = np.r_[numeric, embedding, delta/BOUND_M, coverage].astype(np.float32)
    assert value.shape == (INPUTS,) and np.isfinite(value).all()
    return value


class VisualRisk(nn.Module):
    def __init__(self, mean, scale):
        super().__init__()
        self.register_buffer("mean", torch.as_tensor(mean, dtype=torch.float32))
        self.register_buffer("scale", torch.as_tensor(scale, dtype=torch.float32))
        self.net = nn.Sequential(nn.Linear(INPUTS, 32), nn.ReLU(), nn.Linear(32, 1))

    def forward(self, x):
        return self.net((x-self.mean)/self.scale).squeeze(-1)

    def score(self, position_model, prediction, frame):
        value = risk_features(position_model, prediction, frame)
        if value is None:
            return None
        with torch.no_grad():
            return float(torch.sigmoid(self(torch.from_numpy(value)[None]))[0])


def load_risk(path):
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if (checkpoint["schema"] != "b601-visual-position-risk-v1" or checkpoint["hardware_executable"]
            or checkpoint["input_dimensions"] != INPUTS):
        raise ValueError("Incompatible visual risk checkpoint")
    state = checkpoint["state_dict"]
    model = VisualRisk(state["mean"], state["scale"])
    model.load_state_dict(state)
    return model.eval(), checkpoint
