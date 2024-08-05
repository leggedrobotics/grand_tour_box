#
# Copyright (c) 2022-2024, ETH Zurich, Matias Mattamala, Jonas Frey.
# All rights reserved. Licensed under the MIT license.
# See LICENSE file in the project root for details.
#
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
import subprocess


def generate_protobufs():
    proto_files = ["proto/start_recording.proto", "proto/stop_recording.proto"]
    proto_path = "proto"
    output_path = "./scripts"
    for proto_file in proto_files:
        subprocess.check_call(
            [
                "python3",
                "-m",
                "grpc_tools.protoc",
                f"-I={proto_path}",
                f"--python_out={output_path}",
                f"--grpc_python_out={output_path}",
                proto_file,
            ]
        )


generate_protobufs()

d = generate_distutils_setup(
    packages=["box_recording"],
    package_dir={"": "scripts"},
    include_package_data=True,
    package_data={"": ["*.proto"]},
    install_requires=["protobuf>=3.0.0"],
)

setup(**d)
