import logging
import colorlog


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
