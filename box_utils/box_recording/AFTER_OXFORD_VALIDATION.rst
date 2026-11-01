Selective after_oxford recording port
====================================

Reviewed on 2026-11-01 against main
``3dcc98fc15694f9a96d7b9ecfc60097de8d1c914`` and after_oxford
``781a917f83c3c61bd9115c76fef5511f1ddd2ef2``. This port covers review items
1, 2, 3, 4 and 7. It is prepared on ``fix/after-oxford-recording``.

Existing behavior and additions
-------------------------------

1. Main already records ZED SVO2 and implements sensor export in the separate
   ZED wrapper. Its SVO sensor-read path failed on the example recording with
   the installed SDK, as detailed below. The lightweight recorder now also
   publishes live IMU,
   magnetometer and pressure data. ``box_default`` and ``robot`` capture these
   three topics in a separate ``zed_sensors`` bag. The existing offline camera
   script still replays SVO2 through main's pinned wrapper; it does not consume
   this additional bag as a replacement for its normal output.

   Start succeeds only after the SDK reports the first frame written, allowing
   up to 30 startup grabs for encoder latency. The real SDK test below exposed
   the need for this delay; a successful first grab alone is insufficient. Start,
   stop and grab share a lock; a single worker lives for the node's lifetime.
   Existing output files and symlinks are refused. Write/capture errors,
   non-increasing image timestamps and insufficient free space stop recording
   and remain visible through status polling and the stop response. Partial
   SVO files are retained. ROS coordinator responses preserve recorder failures.
   SDK warnings with an available image are logged while raw capture continues;
   positive SDK errors stop capture.

   Sensor timestamps retain SDK nanoseconds. Gyroscope values and covariance
   use radians and radians squared, magnetometer values use tesla, and pressure
   uses pascals. Unknown covariance is zero; only the intentionally absent IMU
   orientation uses the ROS -1 marker. Sensor polling is independent of image
   capture and each sensor has its own timestamp guard.

   ROS axes match main's ZED wrapper. IMU uses the existing calibrated model
   frame; the SDK magnetometer-to-IMU transform supplies the new magnetometer
   child frame. Scalar pressure uses the existing camera-center frame. Main's
   shipped HD1080/15 Hz, H264 and separate recording/visualization launch modes
   remain selected. H265 is an explicit parameter. The SDK requirement remains
   version 4 or newer; compilation here used SDK 5.0.7.

   ROS and gRPC clients launch the bag writer with separate arguments. The
   recording script execs the C++ rosbag writer, so its owned PID remains alive
   until the bag index is flushed. Stop waits for that writer and propagates a
   nonzero exit status. gRPC start/stop calls share a lock and duplicate starts
   return an RPC failure. Hostnames and profile keys are normalized for ROS
   node/service names; original labels remain in output filenames. Both ROS
   clients and the coordinator use the same normalization. This fixes the
   invalid ROS name exposed by testing on a host with hyphens in its name.

2. ``robot_diagnostics`` is an optional superset of ``robot``. It adds clock,
   actuator, estimator, mapping and wide-angle-camera diagnostics while keeping
   every existing topic and endpoint, including ``twist_throttle``. Full-rate
   ``twist`` is an addition. Actual robot publishers and recording bandwidth
   require verification on the deployed LPC/NPC.

3. Main's live Hesai producer already publishes raw packets and the default
   recording already captures them. ``calib_lidar`` now captures these packets
   too. ``calib_lidar_robot`` adds NPC ``/lidar/point_cloud`` through the existing
   gRPC recorder. Its configured endpoint is ``192.168.0.142:50052``; handheld
   calibration continues to use ``calib_lidar`` without an NPC dependency.

4. ``box_radar`` preserves the full default box recording and adds
   ``/ti_mmwave/radar_scan_pcl_0``. OPC synchronization includes this topic.
   The radar driver remains an external installation. Its configured launch
   file is supplied explicitly::

       roslaunch launch_nuc radar.launch driver_launch:=/absolute/path/to/driver.launch

   This include does not establish radar timing or calibration. Its driver must
   provide the correct topic, timestamps and calibrated TF for the hardware.

7. Recording RViz uses the actual STIM topic and includes an optional CPT7 IMU
   display. Main's multiplot configuration already contains a STIM acceleration
   plot; this port adds a matching numeric gyroscope plot.
   ``visual_planner.launch`` displays existing
   path/goal topics in odom. It only starts RViz. The multiplot launch now loads
   the actual ``cfg/rqt_multiplot.xml`` file; its previous ``config/`` path did
   not exist.

Profile names above are the coordinator's ``yaml_file`` values, without the
``.yaml`` suffix; absolute YAML paths are also supported. Deploy the recorder,
clients and recording script together: failure propagation and the new ZED
status-service consumer are part of the same change.

Main functionality already active or superseding the branch
----------------------------------------------------------

* All ten CPT7 diagnostic commands are already selected by main's shipped
  recording launch. Main also distinguishes optional diagnostic rejection
  from failure of required receiver logs.
* Main already handles the current user's ROS log directory, recording time
  window, chosen output directory and directory-creation failures.
* Main already fixes the calibration declaration and supports explicit output
  paths. Its physical target definitions and calibration implementation remain
  in use.
* Raw Hesai packet production, wrapper-based ZED sensor export and the STIM
  acceleration plot already exist; the additions above target their specific
  recording-profile or lightweight-recorder gaps.

Main submodule contracts
-----------------------

The port initially retained all 28 top-level gitlinks and all 30 recursive
checkouts from main. The EOF fix subsequently advances the Hesai wrapper and
its nested SDK from those exact commits. The other 27 top-level gitlinks and
28 recursive checkouts remain unchanged. No submodule is downgraded.
The relevant starting contracts are:

* ZED wrapper ``d7e8c1af``: ROS axes and meter units at
  ``zed_nodelets/src/zed_nodelet/src/zed_wrapper_nodelet.cpp``; full IMU
  covariance scaling and existing SVO replay retained.
* Hesai driver ``640e74ae``: ``src/manager/source_driver_ros1.hpp`` registers
  the raw-packet callback when ``send_packet_ros`` is enabled, copies every
  packet into ``UdpFrame``, and publishes it with its timestamp/frame. The
  shipped live configuration enables this path on ``/gt_box/hesai/packets``.
* STIM driver ``ea73b08c``: ``stim320_driver_ros/src/stim320_driver.cpp``
  publishes ``/gt_box/stim320/imu``, matching recording and visualization.
* FKIE ``5e746913`` uses the existing synchronization configuration; the only
  sync-list addition here is the radar topic. Existing TF synchronization and
  TF recording supply the new magnetometer transform's transport path.

Validation performed
--------------------

* Isolated catkin build, link and install of the ZED messages, recording node
  and bringup succeeded against installed ROS Noetic and ZED SDK 5.0.7.
  Main's pinned ZED nodelets/wrapper and Hesai driver were also freshly rebuilt.
* 16 C++ tests passed: start/stop sequencing, first-frame acknowledgement,
  delayed encoder startup, bounded startup failure, failures after startup,
  existing output preservation, low disk space, SDK/write exceptions,
  timestamp failures, concurrent service/grab calls, and sensor conversions.
  These tests use a fake camera backend, not a physical camera.
* 29 Python tests passed, including main's seven existing recording follow-up
  tests. The additional tests cover the actual recorder/coordinator methods
  with mocked services, failure propagation, stop handling, shipped profiles,
  and matching recorder/coordinator names. Ruff and shell syntax checks passed.
* ROS launch loading passed for the recorder, RViz and radar wrapper with a
  fixture external driver. Missing radar launch arguments were rejected.
  All 19 recording YAML/RViz/recorder-parameter files and the multiplot XML
  parsed successfully. A real external radar launch was not available.
* All 57 source files and all 57 files in the existing SSD backup were SHA-256
  verified against its manifest before testing: 12,449,414,964 bytes per copy.
  Tests used that backup or disposable copies on the SSD, with private ROS
  masters and output directories. Generated bags, SVO copies, preview images,
  private ROS directories and temporary build files were removed. The final
  hash check verified both raw copies again. Existing local work was preserved.

Full recording transport
~~~~~~~~~~~~~~~~~~~~~~~~

The actual gRPC server started four bag writers, rejected a duplicate start,
and stopped them after replay of the six relevant streams at their original
speed. The output path deliberately contained spaces; the actual hostname
contained hyphens. All eight files, including each group's five-minute split,
were finalized before the stop response. Counts and SHA-256 hashes of serialized
messages matched exactly:

* ZED IMU: 140,812; magnetometer: 18,068; pressure: 9,007.
* STIM IMU: 183,279.
* Hesai raw-packet messages: 3,771.
* Radar point clouds: 3,784.
* Total: 358,721 messages, zero payload or header-timestamp differences.

Bag receipt times are newly generated during recording, as expected. This
validates the selected streams on this host, not simultaneous recording of
every robot diagnostic or camera stream. Earlier 220-message tests also used
the actual ROS stop helper and gRPC stop method successfully.

Two accelerated 3x runs lost 60 and 54 STIM messages respectively; the other
five streams matched exactly. The second run explicitly waited for recorder
subscriptions. Missing messages occurred in a burst during playback, not at
startup. The installed ``rosbag record`` Python launcher and the direct C++
writer select the same default buffering/chunk settings. These observations
do not establish the loss's cause; 3x recording completeness remains unproven.

ZED replay and new sensor conversions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The original SVO2 decoded all 5,493 frames with SDK 5.0.7 on the RTX 3090:
zero grab warnings, zero non-increasing timestamps, and no black image samples.
Main's freshly built, pinned wrapper exported all 5,493 left and 5,493 right
JPEG images with matching CameraInfo counts. Every image decoded at 1920x1080,
and every header/bag timestamp exactly matched the source SDK image timestamp.
This full run selected images only: depth, tracking, mapping, proprioception
and TF saving were disabled explicitly.

A separate short run used the unmodified shipped replay configuration,
including NEURAL_PLUS depth. It exported 53 synchronized sets of left/right
images, depth, confidence, CameraInfo, TF, odometry and map poses. Every depth
image decoded as uint16 PNG with valid pixels; observed depths were 300--15,000
mm and confidence values 0--100. Camera intrinsics were positive and the stereo
projection matrices retained their baseline. This verifies encoding and
timestamp consistency, not depth accuracy against ground truth.

That default run exported no IMU, magnetometer or pressure messages. The active
path is ``save_prop_bag=true`` -> ``publishSensData`` ->
``if (mSvoMode || mSensTimestampSync)`` -> SDK ``TIME_REFERENCE::IMAGE``.
The shipped ``sensors_timestamp_sync=false`` cannot select CURRENT in SVO mode;
CURRENT is implemented for live input. The SDK-only full-source probe likewise
found no available IMAGE-mode IMU samples. By contrast, CURRENT recovered 5,999
IMU, 751 magnetometer and 376 pressure samples over the first 100 image frames.
Those real samples passed through the new recorder's conversion helpers with
exact SDK nanosecond timestamps. The SDK's installed sensor-export example
also uses CURRENT and advances the SVO when its sensor window is exhausted.

Thus sensor export exists in main, but its selected SVO read mode did not
produce sensors for this file/SDK combination. The new live recorder uses
CURRENT independently of image capture; it does not change the wrapper's SVO
replay behavior. Main's wrapper submodule is unchanged.

SVO recording failure
~~~~~~~~~~~~~~~~~~~~~

The actual new recorder backend/session captured 30 frames from the source
SVO. This exposed and led to correction of delayed first-write acknowledgement:
the SDK write status became true on the second grab. Its re-recorded output,
however, returned ``CORRUPTED_FRAME`` and black images on playback.

An independent SDK-only program reproduced the problem without the recorder's
session or file reservation. Additional 60-grab tests produced H264, H265 and
LOSSLESS files with 60, 60 and 54 indexed frames respectively; playback yielded
59, 60 and 54 black frames with corruption warnings. Source images were valid.
The installed SDK recording sample accepts SVO input, so this is not dismissed
as an unsupported test. The underlying SDK/environment cause remains unknown.
Live camera capture is still required to validate the new recorder operationally.

Hesai replay
~~~~~~~~~~~~

Main's pinned decoder replayed all 3,771 raw-packet messages using the shipped
packet-replay configuration, changing only output paths. It emitted 3,770
point clouds and matching range/intensity image counts: 241,118,464 points,
zero non-finite coordinates/timestamps, valid layouts and rings 0--31.
Cloud timestamps increased strictly; each equaled its bag time and first
point timestamp. This does not establish geometric accuracy against a reference.

Before the fix, the final scan was not emitted at EOF. Source inspection shows that the SDK
emits a scan on the following scan boundary; the wrapper terminates at the
last input timestamp. Controlled 30- and 31-message runs emitted 29 and 30
clouds, with identical hashes/timestamps for their 29 common clouds. This is
a completeness limitation in the starting main decoder. The raw recording
test retained every packet message.

The fix signals EOF only after the final input message is queued. The SDK
drains packets and waits for all queued point calculations before emitting
the final available scan. Shutdown then joins the decoder before closing and
renaming the output bag. Streaming input has no EOF signal and keeps waiting
for packets; its stop path now skips file finalization when no output bag is
owned. Replay-only PTC-client pointers are initialized so teardown cannot
delete an indeterminate pointer. Packet enqueueing no longer holds the metadata
lock needed by output callbacks while waiting for space in the bounded queue.

The full fixed run produced 3,771 clouds and matching image counts, containing
241,182,464 finite points. Every one of the original 3,770 cloud payload hashes
and timestamps matched the baseline exactly; the final 64,000 points were
recovered. One-frame and truncated-tail tests cover zero, one and 50 remaining
UDP packets. A terminal cloud represents only received points, not an inferred
complete revolution. The streaming control preserved the same preceding clouds,
remained active after input paused, and exited cleanly on SIGINT. All seven
regression scenarios passed. The reusable regression script is
``hesai_lidar_ros_driver/tests/check_replay_eof.py``.

Recorded-data integrity and visualization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The entire STIM, ZED sensor and radar streams had finite measured values and
strictly increasing header timestamps. STIM averaged 499.992 Hz; radar averaged
9.9997 Hz and contained 55,455 finite points in 3,784 valid, nonempty clouds.
All 140,812 historical ZED IMU messages mark angular/acceleration covariance
with -1, incorrectly declaring those measurements absent. The recorded TF
has an IMU parent but no parent for the historical ZED magnetometer/barometer
frames. These are properties of the existing dataset; the new recorder's
covariance and frame handling are covered separately by its tests.

The corrected rqt launch and the planner RViz configuration ran under Xvfb;
rqt subscribed to 15 seconds of replayed STIM data. The recording RViz view
crashed loading ``librviz_plugin_tutorials.so`` due to an unresolved
``rviz::ColorProperty`` constructor. Loading main's unchanged recording RViz
configuration reproduced the same failure.

The IMU plugin was then rebuilt from pinned upstream release 0.11.2 against the
installed RViz 1.14.8. The missing JSK text-overlay dependency was installed from
the matching vendor build. With the optional IMU group enabled in a disposable
configuration, the recording view stayed running and subscribed to 15 seconds
of real STIM replay with no plugin errors. The matching rqt plot also ran.
The shipped displays remain optional. ``box_rviz`` now declares both runtime
dependencies and provides ``scripts/rebuild_rviz_plugin.sh`` for a reproducible
workspace-local repair. The installed repair remains in the local workspace;
generated test datasets and ROS directories were deleted.

Remaining validation before operational acceptance
-------------------------------------------------

1. On the target Jetson/ZED2i, capture and replay an SVO through main's pinned
   wrapper. Resolve the recording corruption above or verify the exact target
   SDK/camera combination. The user reports that a later ZED SDK fixes the SVO
   sensor-export issue; that wrapper path is intentionally unchanged here and
   the later SDK was not tested on this workstation. Verify image
   counts/timestamps, sensor rates/axes/TF, repeated
   start/stop, camera disconnect and encoder/disk failures. Check the deployed
   SDK version explicitly; version 4 was not compiled in this environment.
2. On the robot, verify every optional diagnostics/NPC topic and its message
   type, then measure sustained bandwidth, disk use and recording completeness.
3. With the external radar driver, verify the selected hardware configuration,
   timing, TF and sustained recording. Existing radar messages proved transport
   integrity only. Validate Hesai geometry against reference data; the EOF
   completeness regression is now fixed and tested.
4. Deploy the compatible RViz plugins on other machines. Verify sustained
   recording at the intended workload; the tested 3x workload was not lossless.

These tests exercise the relevant recording/replay paths, not the complete
dataset-processing pipeline or live hardware. The new ZED sensor bag is
additional raw data; downstream selection between that data and wrapper replay
remains explicit. The Hesai wrapper/SDK advance from main; other submodule
revisions are unchanged.

Local audit evidence
--------------------

Initial build/tests and recording-session reproduction:
``/home/tutuna/clean_box_ws/after-oxford-port-20261101-my3csljw``.

Extended replay/recording scripts, JSON results and text logs:
``/home/tutuna/clean_box_ws/after-oxford-extended-20261101-1i60mwhd``.
Only audit text, scripts and metadata are retained there; generated dataset
outputs were deleted after testing.

Final EOF and RViz repair, regression tests, baseline comparison and push audit:
``/home/tutuna/clean_box_ws/after-oxford-final-27tvu99q``.
