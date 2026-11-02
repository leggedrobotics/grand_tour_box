## Calibration-target presets

`generic_detector.launch` defaults to a 6 x 6 Aprilgrid with 83 mm tags and
gap/tag-size ratio 0.3, or a checkerboard with 7 x 8 internal corners and 80 mm
squares (`use_april_grid:=false`). Existing camera launch profiles retain these
dimensions.

`generic_detector_50mm.launch` provides the separate target geometry from
`after_oxford`:

| Target | Layout | Physical dimensions |
| --- | --- | --- |
| Aprilgrid (default) | 6 x 6 tags, tag36h11 family | 50 mm tag edges, 15 mm gaps |
| Checkerboard | 7 x 8 internal corners (8 x 9 squares) | 50 mm squares |

Launch one detector for the chosen image topic:

```bash
roslaunch grand_tour_camera_detectors generic_detector_50mm.launch \
  name:=cam1_detector image_topic:=/gt_box/alphasense_driver_node/cam1
```

Add `use_april_grid:=false` for the checkerboard. The preset also forwards
`box_namespace` and `recording_bag_name` to the generic detector. Select it
explicitly for each camera observing the 50 mm target.

Measure the physical board before using this preset. These sizes determine the
`modelpoint3d` coordinates published with each detection and consumed by the
calibration optimizer. A size mismatch changes the metric scale of the resulting
calibration; successful image detection alone cannot validate the board dimensions.

## License

This package is based on code from the Autonomous Systems Lab, ETH Zurich and Skybotix AG.  
It is licensed under the original BSD license with an advertising clause.
See `LICENSE_ASL_SKYBOTIX.txt` for full terms.
