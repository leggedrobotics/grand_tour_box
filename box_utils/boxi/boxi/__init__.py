import os
from pathlib import Path
import socket

# Absolute path to the  boxi_root_dir repository.
BOXI_ROOT_DIR = str(Path(str(os.path.realpath(__file__))).parent.parent)

# Absolute path to the  box repository.
BOX_ROOT_DIR = str(Path(os.path.realpath(__file__)).parent.parent.parent.parent)
from .os_utils import shell_run

LOCAL_HOSTNAME = socket.gethostname()

USERNAME = "rsl"