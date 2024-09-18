import logging
import os
import socket
from pathlib import Path

import colorlog


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


class ColorLogger:
    GREEN_CHECK = "\u2714"
    YELLOW_WARNING = "\u26A0"
    RED_CROSS = "\u274C"

    @classmethod
    def get_logger(cls):
        # Define the log format with colors and emojis
        log_format = (
            '%(log_color)s%(levelname)s: %(message)s'
        )

        # Create a color mapping
        colors = {
            'DEBUG': 'cyan',
            'INFO': 'green',
            'WARNING': 'yellow',
            'ERROR': 'red',
            'CRITICAL': 'red,bg_white'
        }

        # Create a handler that writes log messages to the console
        handler = colorlog.StreamHandler()
        handler.setFormatter(colorlog.ColoredFormatter(log_format, log_colors=colors))

        # Get the root logger and set its handler and log level
        logger = colorlog.getLogger()
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        return logger
