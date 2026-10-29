# boxi_rectify_from_bag

`boxi_rectify_from_bag` is a ROS 1 package for:

- rectifying image streams stored in rosbag files,
- writing processed output bags with rectified image topics,
- visualizing already-rectified bags in a read-only OpenCV viewer.

JPEG handling uses `libjpeg-turbo` when available and falls back to OpenCV JPEG decode/encode when it is not.

## Nodes

- `rectify_and_write_bag_node`
  Validates per-camera calibration and matching acquisition timestamps, then publishes `images.bag`, `depth.bag` and `processing_summary.json` together in a new output directory.
- `visualize_rectified_from_bag_node`
  Read-only viewer for already-rectified RGB bags with optional depth and confidence overlays. It can also print the rectified RGB `CameraInfo` periodically.
- `rectify_from_bag_node`
  Legacy bag viewer for raw RGB streams that can rectify on the fly and compare distorted vs rectified views.

## Launch Files

- `launch/rectify_and_write_bag.launch`
  Batch rectification pipeline that writes processed output bags.
- `launch/rectify_viewer.launch`
  Read-only viewer for processed bags. By default it expects:
  - `/boxi/zed2i/right/image_rect/compressed`
  - `/boxi/zed2i/depth/image_rect/compressed`
  - `/boxi/zed2i/confidence/image_rect/compressed`

## Build Requirements

- ROS 1 catkin workspace
- C++17
- OpenCV 4
- `roscpp`
- `rosbag`
- `sensor_msgs`
- `cv_bridge`
- OpenCV JPEG support
- optional: `libjpeg-turbo` with TurboJPEG headers and library

## libjpeg-turbo Requirements

This package can use TurboJPEG through `turbojpeg.h` and `libturbojpeg`, but it no longer requires it to build.

CMake searches for it in this order:

1. `LIBJPEG_TURBO_ROOT`, which defaults to `/opt/libjpeg-turbo`
2. System include and library paths

The expected layout under `LIBJPEG_TURBO_ROOT` is:

- `include/turbojpeg.h`
- `lib/libturbojpeg.so` or `lib64/libturbojpeg.so`

If TurboJPEG is found, this package uses it as the fast JPEG backend.

If TurboJPEG is not found, the package still builds and uses OpenCV `imdecode()` / `imencode()` for JPEG handling instead.

### Official Download

If TurboJPEG is not installed in the expected location, use the official `libjpeg-turbo` release:

- Release page:
  `https://github.com/libjpeg-turbo/libjpeg-turbo/releases/tag/3.1.4.1`
- Direct `.deb` download:
  `https://github.com/libjpeg-turbo/libjpeg-turbo/releases/download/3.1.4.1/libjpeg-turbo-official_3.1.4.1_amd64.deb`

The official package installs under `/opt/libjpeg-turbo`, which matches this package's default `LIBJPEG_TURBO_ROOT`.

### Install From The Official `.deb`

Download and install:

```bash
wget https://github.com/libjpeg-turbo/libjpeg-turbo/releases/download/3.1.4.1/libjpeg-turbo-official_3.1.4.1_amd64.deb
sudo apt install ./libjpeg-turbo-official_3.1.4.1_amd64.deb
```

If you prefer `dpkg`:

```bash
wget https://github.com/libjpeg-turbo/libjpeg-turbo/releases/download/3.1.4.1/libjpeg-turbo-official_3.1.4.1_amd64.deb
sudo dpkg -i libjpeg-turbo-official_3.1.4.1_amd64.deb
sudo apt-get install -f
```

If your installation is not under `/opt/libjpeg-turbo`, pass a custom root to CMake. For a catkin-tools workspace:

```bash
catkin config --cmake-args -DLIBJPEG_TURBO_ROOT=/path/to/libjpeg-turbo
catkin build boxi_rectify_from_bag
```

To force the OpenCV fallback even when TurboJPEG is installed:

```bash
catkin config --cmake-args -DBOXI_USE_TURBOJPEG=OFF
catkin build boxi_rectify_from_bag
```

## Viewer Notes

- `rectify_viewer.launch` opens bags with `rosbag::bagmode::Read` only. It does not write output bags or modify existing bag contents.
- The viewer can print rectified RGB `CameraInfo` every `rgb_info_print_interval_sec` seconds.
- If `confidence_bag` is empty and `use_confidence=true`, the viewer defaults `confidence_bag` to `depth_bag`.
- If your processed bag uses a different rectified `CameraInfo` topic, override `rgb_info_topic` in the launch file or on the command line.

## Quick Start

Build the package:

```bash
catkin build boxi_rectify_from_bag
```

Write processed bags:

```bash
roslaunch boxi_rectify_from_bag rectify_and_write_bag.launch bag_dir:=/path/to/bags output_dir:=/path/to/new-output
```

Visualize processed bags:

```bash
roslaunch boxi_rectify_from_bag rectify_viewer.launch
```

## Dataset geometry and failure behavior

The shipped ZED replay uses SDK-rectified images (`save_unrectified: false`).
`input_rectified:=true` is therefore the batch default: pixels pass through
unchanged, and CameraInfo must describe rectified pixels (zero distortion,
identity R and matching K/P intrinsics).

Use `input_rectified:=false` only for raw images with complete, trusted per-camera
K, D, R and P. Each camera uses its own projection; right P retains the stereo
baseline. PNG is the default output codec for newly rectified pixels. Existing
rectified compressed images retain their original codec and bytes.

Depth and confidence retain their acquisition stamps, optical frame, resolution,
units and payload. Depth uses its own CameraInfo; ZED confidence uses the depth
pixel grid. This pipeline does not reproject depth into the right camera.

All four streams must have exact matching stamps by default. Explicit
`allow_unmatched:=true` exports their intersection and records dropped counts in
the summary. Missing calibration, duplicate stamps, changing calibration, corrupt
RGB or an existing output directory cause failure without publishing a dataset.

Export the result with `box_export/zed2i_export.launch`, supplying input/output
directories and TF bags. The exporter writes `frames.csv`, separate camera
calibrations and poses. EXR preserves float depth in metres; PNG uses uint16
millimetres, rejects finite depths above 65.535 m and encodes invalid depths as 0.
Validate using `verify_exported_images.py OUTPUT --validate-only` (set
`OPENCV_IO_ENABLE_OPENEXR=1` if your OpenCV wheel disables EXR by default).
The legacy viewers are diagnostic displays; numerical validation uses the
exported per-camera metadata and timestamp manifest.
