import sys
import os
import pathlib
import inspect
import numpy as np

# Automatically add box_binding to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "box_binding")))

import compressed_depth

print("package dir :", pathlib.Path(inspect.getfile(compressed_depth)).parent)
print("has decode  :", hasattr(compressed_depth, "decode"))

try:
    compressed_depth.decode(np.zeros(10, np.uint8), "compressedDepth rvl")
except Exception as e:
    print("decode ran :", type(e).__name__, e)
