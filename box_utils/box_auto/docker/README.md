Things to adapt:
box.Dockerfile -> adapt host system cuda version FROM nvidia/cuda:12.5.1-base-ubuntu20.04
open_cv.sh -> adapt compute capability of your device when building open_cv; GeForce RTX 3080 = 8.6 - https://developer.nvidia.com/cuda-gpus
zed.sh -> adapt CUDA version potentially here currently 12.1