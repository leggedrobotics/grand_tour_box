import os
from pathlib import Path

# Absolute path to the  boxi_root_dir repository.
BOXI_ROOT_DIR = str(Path(os.path.realpath(__file__)).parent[2])

# Absolute path to the  box repository.
BOX_ROOT_DIR = str(Path(os.path.realpath(__file__)).parent[4])
