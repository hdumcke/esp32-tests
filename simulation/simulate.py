from urdfpy import URDF
from pathlib import Path
import os
#os.environ['PYOPENGL_PLATFORM'] = 'osmesa'
import pyrender
import numpy as np


robot = URDF.load(os.path.join(Path(__file__).absolute().parent, 'urdf', 'mini-pupper.urdf'))
cfg = None

robot.show(cfg=cfg)
