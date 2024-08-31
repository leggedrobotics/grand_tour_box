#
# Copyright (c) 2022-2024, ETH Zurich, Matias Mattamala, Jonas Frey.
# All rights reserved. Licensed under the MIT license.
# See LICENSE file in the project root for details.
#
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["box_auto"],
    package_dir={"": "scripts"},
    include_package_data=True,
    package_data={},
    install_requires=[],
)

setup(**d)
