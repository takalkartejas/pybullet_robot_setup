import sys
from pathlib import Path
import argparse
sys.path.append('/home/tejas/projects/egad')
sys.path.append('/home/tejas/projects/trimesh')

import trimesh
from egad.mesh import scale_mesh

meshes = trimesh.load('/home/tejas/projects/egad_train_set/processed_meshes/G08_2.obj')