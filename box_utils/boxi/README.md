# Boxi

## Installation
From this directory run `pip install -e .` to install boxi 
in editable form. This is the preferred method of installation
that introduces minimal changes to `jetson` and the `nuc` when
running `boxi push`

## Apps
### Boxi set_cpt7_time: indoor CPT7/PTP startup

The CPT7 is the box's PTP grandmaster. When starting indoors without GNSS,
`SETAPPROXTIME` seeds its approximate GPS reference clock from an independent
time source. NovAtel specifies full GPS week and fractional seconds of week,
within ten minutes of actual time. This is startup initialization, not precise
UTC synchronization or GNSS holdover validation. An incorrect GPS week can cause
the receiver to reset when GNSS tracking resumes.

Use independent NTP (default `pool.ntp.org`, configurable with `--ntp-server`):

```bash
boxi set_cpt7_time --ros --dry-run
boxi set_cpt7_time --ros
```

Without `--ros`, the command uses TCP `192.168.2.98:3001` (override with `--host`
and `--port`). The ROS path uses `/gt_box/cpt7/receivers/main/Oem7Cmd`. Dry-run
obtains/validates the time and prints the command without contacting the receiver.

Without network time, `--local` explicitly trusts the PC's system clock, or
`--utc` accepts the actual current ISO-8601 time including `Z` or a UTC offset.
For example, this previews a fixed timestamp without setting the receiver:

```bash
boxi set_cpt7_time --utc 2026-11-02T12:34:56Z --dry-run
```

Use the current time for a real invocation. A PC already following the
uninitialized CPT7 is not an independent reference; seeding from it creates a
feedback loop. NTP failures do not fall back to the local clock. The old
`--manual --year` mode is rejected because a year cannot identify current time.

The conversion adds **GPS minus UTC** (18 seconds since 2017), independent of
local timezone and daylight saving. It preserves fractional seconds and carries
rounding across GPS-week boundaries. This offset differs from **TAI minus UTC**
(37 seconds since 2017), which is relevant to the PTP time scale. The bundled
historical leap table uses the IERS expiry, **2027-06-28**; after that date update
the table/expiry or supply `--gps-utc-offset` with an independently verified value.
Elapsed connection/service wait is added using a monotonic clock so a local PTP
clock step cannot change the chosen reference.

The receiver's saved PTP configuration must match the deployment:

- `PTPMODE ENABLE` allows indoor startup. `ENABLE_FINETIME` waits for
  `FINESTEERING` on initial enable and therefore cannot bootstrap a cold receiver
  without GNSS.
- Check `PTPPROFILE` and `PTPTIMESCALE` in the receiver configuration. The shipped
  NUC `phc2sys_enp45s0.service` forces `-O 0`, expecting no PHC-to-system offset;
  a UTC PTP clock matches that assumption. Jetson/Pi use `-w` to obtain the offset
  from `ptp4l`. A PTP/TAI clock requires consistent slave-side conversion; changing
  the GPS seed to compensate would corrupt receiver time.
- Inspect `LOG TIMEA ONCE` and `LOG PTPDELTATIMEA ONCE`, the elected grandmaster,
  and slave-clock offsets before recording. Command acknowledgement confirms
  acceptance only. Validate the resulting timestamps against the independent
  source on the actual CPT7/firmware, including a no-GNSS cold start.
  Check the TIME log's UTC offset/status explicitly: setting approximate GPS time
  does not supply missing almanac UTC parameters or make an invalid UTC status valid.

TCP commands are terminated and require a complete `<OK` response. Both
transports report rejection/timeouts as CLI failure. `--timeout` bounds NTP,
TCP and ROS discovery; `--ros-timeout` bounds the RPC (35 seconds by default,
because the pinned NovAtel driver retries for up to 30 seconds). A timeout cannot
undo a receiver command already sent, so inspect receiver state before retrying.
The startup tmux pane stays visible on failure.

References: [NovAtel SETAPPROXTIME](https://docs.novatel.com/OEM7/Content/Commands/SETAPPROXTIME.htm),
[PTPMODE](https://docs.novatel.com/OEM7/Content/Commands/PTPMODE.htm),
[PTPTIMESCALE](https://docs.novatel.com/OEM7/Content/Commands/PTPTIMESCALE.htm),
[PTP configuration and monitoring](https://docs.novatel.com/OEM7/Content/Configuration/PTP_Configuration.htm),
[TIME](https://docs.novatel.com/OEM7/Content/Logs/TIME.htm),
[command responses](https://docs.novatel.com/OEM7/Content/Messages/Responses.htm),
[IERS leap table](https://hpiers.obspm.fr/iers/bul/bulc/Leap_Second.dat).

Run local clock/transport regressions with
`python3 -m unittest discover -s box_utils/boxi/tests -v` from the repository root.
Tests use synthetic timestamps, mocked NTP/ROS and a loopback TCP server.

### Boxi export_raw_imu_bag
#### Usage
After a data collection run, extract a RAW_IMU_ASCII txt file using
the Novatel Convert app, available on the Ubuntu Novatel Application Suite
`boxi export_raw_imu_bag --imu_ascii_file/-i PATH_TO_RAW_IMU_ASCII.txt
--output/-o OUTPUT_ROSBAG_PATH.bag`

### Boxi export_gps_gt_trajectory_bag
#### Usage
Launch the **Inertial Explorer** app in Windows on the Boxi OPC. 
After start up, start a new project using the Project Wizard.
Select a name and location to save the project, then proceed with the Project Wizard
window to load the CPT7 .LOG file. [TODO] Insert link to the instructions on running
offline optimization of the trajectories. Once done optimizing the GPS trajectories,
export a trajectory .txt file using the profile: **GrandTour GroundTruth**.
Back over in Ubuntu, run `boxi export_gps_gt_trajectory_bag --gps_file/-g PATH_TO_GPS_TRAJECTORY.txt
--output/-o OUTPUT_ROSBAG_PATH.bag`
