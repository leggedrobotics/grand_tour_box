# from importlib import import_module
# _ext = import_module(__name__ + "._compressed_depth")
# decode = _ext.decode
# __all__ = ("decode",)
from importlib import import_module

# Correct: dynamic module is named _compressed_depth (with underscore)
_ext = import_module("compressed_depth._compressed_depth")

decode = _ext.decode

__all__ = ("decode",)
