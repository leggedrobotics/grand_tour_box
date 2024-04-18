from setuptools import setup, find_packages

setup(
    name="calibration_tools",
    version="0.1.0",
    author="Kappi Patterson",
    author_email="kpatterson@ethz.ch",
    description="A package for managing calibration data using dynamically generated enums.",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "pyyaml",
    ],
    python_requires=">=3.6",
    classifiers=[
        "Development Status :: 0 - Beta",
        "License :: NaN",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
    ],
)
