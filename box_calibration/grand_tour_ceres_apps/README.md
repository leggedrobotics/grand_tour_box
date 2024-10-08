# Grand Tour Ceres Apps

[![Build Status](https://github.com/fulkast/grand_tour_ceres_apps/actions/workflows/ci.yaml/badge.svg)](https://github.com/fulkast/grand_tour_ceres_apps/actions)

This project holds several apps used in calibrating sub-components of the GTBox

## Prerequisites

Make sure you have the following tools installed on your system:

- **CMake** (version 3.10 or higher)
- **C++ Compiler** (GCC, Clang, or MSVC)
- **Git** (optional but recommended)

### Installing Dependencies

#### Ubuntu/Debian

You can install all the required dependencies using the following `apt` commands:

```bash
sudo apt-get update
sudo apt-get install -y \
    libboost-all-dev \
    libceres-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev
```