"""
Vendored copy of XFeat (https://github.com/verlab/accelerated_features).

Source commit: e92685f57f8318b18725c5c8c0bd28c7fe188d9a, Apache License 2.0 (see LICENSE).
Files: modules/xfeat.py, modules/model.py, modules/interpolator.py, weights/xfeat.pt.

Local changes to xfeat.py, all needed to use it as a subpackage:
- imports changed to relative (`from .model import *`)
- default weights path points at the xfeat.pt next to this file
- removed the unused tqdm import and the print when loading weights
- torch.load uses weights_only=True so the weights file cannot run code
"""

from .xfeat import XFeat

__all__ = ["XFeat"]
