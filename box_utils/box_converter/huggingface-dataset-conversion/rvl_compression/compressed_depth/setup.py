# filepath: /app/rvl_compression/compressed_depth/setup.py
from setuptools import setup, find_packages

setup(
    name="compressed_depth",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "opencv-python",
    ],
)
