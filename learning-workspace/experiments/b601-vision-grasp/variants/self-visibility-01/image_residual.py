"""Image-conditioned bounded residual; inference takes visual inputs only."""
import copy

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from policy import BOUND_M, features

PATCH_SIZE = 32


def image_patch(frame, prediction):
    rgb = np.asarray(frame["rgb"], np.float32)/255
    depth = np.clip((np.asarray(frame["depth"], np.float32)-prediction["depth_m"])/.05, -2, 2)
    value = torch.from_numpy(np.concatenate([rgb, depth[..., None]], axis=-1).transpose(2, 0, 1).copy())
    h, w = depth.shape
    offset = torch.arange(PATCH_SIZE, dtype=torch.float32)-(PATCH_SIZE-1)/2
    v, u = torch.meshgrid(offset+prediction["pixel_uv"][1], offset+prediction["pixel_uv"][0], indexing="ij")
    grid = torch.stack([2*u/(w-1)-1, 2*v/(h-1)-1], dim=-1)
    patch = F.grid_sample(value[None], grid[None], mode="bilinear", padding_mode="border", align_corners=True)[0]
    assert torch.isfinite(patch).all()
    return patch.numpy()


class ImageResidual(nn.Module):
    def __init__(self):
        super().__init__()
        self.image = nn.Sequential(nn.Conv2d(4, 8, 3, padding=1), nn.ReLU(),
                                   nn.Conv2d(8, 8, 3, padding=1), nn.ReLU(),
                                   nn.AdaptiveAvgPool2d((4, 4)), nn.Flatten())
        self.net = nn.Sequential(nn.Linear(136, 32), nn.Tanh(), nn.Linear(32, 32),
                                 nn.Tanh(), nn.Linear(32, 3), nn.Tanh())

    def initialize_scalar(self, scalar):
        with torch.no_grad():
            self.net[0].weight.zero_()
            self.net[0].weight[:, :8].copy_(scalar.net[0].weight)
            self.net[0].bias.copy_(scalar.net[0].bias)
            self.net[2].load_state_dict(scalar.net[2].state_dict())
            self.net[4].load_state_dict(scalar.net[4].state_dict())

    def forward(self, x, patch):
        return self.net(torch.cat([x, self.image(patch)], dim=1))*BOUND_M

    def correct(self, prediction, frame):
        result = copy.deepcopy(prediction)
        xyz = np.asarray(prediction["top_xyz_m"])
        if not (.265 <= xyz[0] <= .395 and abs(xyz[1]) <= .125):
            result["residual_xyz_m"] = [0., 0., 0.]
            result["action_source"] = "outside_training_workspace_geometry"
            return result
        with torch.no_grad():
            delta = self(torch.from_numpy(features(prediction))[None],
                         torch.from_numpy(image_patch(frame, prediction))[None])[0].numpy()
        result.update(raw_top_xyz_m=prediction["top_xyz_m"], residual_xyz_m=delta.tolist(),
                      top_xyz_m=(xyz+delta).tolist(), action_source="learned_bounded_visuomotor_residual",
                      policy_input="8visual_scalars_and_32x32_RGBD_patch")
        return result
