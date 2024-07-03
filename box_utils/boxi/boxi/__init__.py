import os
from pathlib import Path
import socket


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


# Absolute path to the  boxi_root_dir repository.
BOXI_ROOT_DIR = str(Path(str(os.path.realpath(__file__))).parent.parent)

# Absolute path to the  box repository.
BOX_ROOT_DIR = str(Path(os.path.realpath(__file__)).parent.parent.parent.parent)
from .os_utils import shell_run

LOCAL_HOSTNAME = socket.gethostname()
USERNAME = "rsl"
