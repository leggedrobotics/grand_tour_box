"""Seed a CPT7's approximate GPS clock before recording without GNSS.

SETAPPROXTIME takes GPS week/seconds, even if the receiver distributes UTC over
PTP. It is an approximate startup reference, not a precision time transfer or a
claim that PTP slaves have synchronized. See README.md for the PTP prerequisites.
https://docs.novatel.com/OEM7/Content/Commands/SETAPPROXTIME.htm
"""

from bisect import bisect_right
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from decimal import Decimal, ROUND_HALF_UP
import math
import re
import signal
import socket
import time

import ntplib


GPS_EPOCH_UNIX = 315964800  # 1980-01-06 00:00:00 UTC; independent of TZ/DST.
WEEK_MICROSECONDS = 604800 * 1_000_000
# Effective UTC dates of GPS-UTC increments. GPS was aligned with UTC at its
# epoch; TAI-GPS is 19 s. Thus the current TAI-UTC=37 s means GPS-UTC=18 s.
# Historical dates matter for explicit inputs; do not apply today's 18 s to 1996.
# https://hpiers.obspm.fr/iers/bul/bulc/Leap_Second.dat
LEAP_DATES = tuple(
    datetime.fromisoformat(day).replace(tzinfo=timezone.utc).timestamp()
    for day in (
        "1981-07-01",
        "1982-07-01",
        "1983-07-01",
        "1985-07-01",
        "1988-01-01",
        "1990-01-01",
        "1991-01-01",
        "1992-07-01",
        "1993-07-01",
        "1994-07-01",
        "1996-01-01",
        "1997-07-01",
        "1999-01-01",
        "2006-01-01",
        "2009-01-01",
        "2012-07-01",
        "2015-07-01",
        "2017-01-01",
    )
)
# IERS' table updated through Bulletin C 72 expires on 28 June 2027. Require a
# refreshed table or an explicitly verified GPS-UTC value beyond that date, rather
# than silently assuming 18 s forever. This limit also applies without internet.
# https://datacenter.iers.org/data/html/bulletinc-072.html
LEAP_TABLE_VALID_UNTIL = datetime(2027, 6, 28, tzinfo=timezone.utc).timestamp()
SERVICE_NAME = "/gt_box/cpt7/receivers/main/Oem7Cmd"


def positive_seconds(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("timeout must be finite and positive")
    return value


def parse_utc(value):
    try:
        stamp = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as error:
        raise ValueError("use an ISO-8601 timestamp, e.g. 2026-11-02T12:34:56Z") from error
    if stamp.tzinfo is None or stamp.utcoffset() is None:
        raise ValueError("timestamp must include Z or an explicit UTC offset")
    return stamp.timestamp()


def add_arguments(parser):
    parser.set_defaults(main=main)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--local", action="store_true", help="Use this PC's independently trusted UTC clock")
    source.add_argument("--utc", type=parse_utc, help="Explicit current ISO-8601 time with Z/UTC offset")
    source.add_argument("--manual", action="store_true", help="Deprecated: use --utc with a complete timestamp")
    parser.add_argument("--year", type=int, help="Deprecated: a year alone cannot set current GPS time")
    parser.add_argument("--ros", action="store_true", help="Send via the NovAtel Oem7Cmd ROS service")
    parser.add_argument("--ntp-server", default="pool.ntp.org", help="Independent NTP source (default: pool.ntp.org)")
    parser.add_argument("--host", default="192.168.2.98", help="CPT7 command-port address for TCP")
    parser.add_argument("--port", type=int, default=3001)
    parser.add_argument(
        "--timeout", type=positive_seconds, default=5.0, help="NTP/TCP/service-discovery timeout in seconds"
    )
    parser.add_argument(
        "--ros-timeout", type=positive_seconds, default=35.0, help="ROS RPC timeout; pinned driver can retry for 30 s"
    )
    parser.add_argument(
        "--gps-utc-offset", type=int, help="Explicit verified GPS minus UTC seconds; normally inferred from leap table"
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the command without contacting the receiver")
    return parser


def utc_to_gps(unix_seconds, gps_utc_offset=None):
    """Return normalized full GPS week and fractional seconds of week.

    Unix/NTP time is UTC: ADD GPS-UTC when setting the receiver. Subtracting it
    would be the inverse conversion used when exporting GPS data to UTC. Never
    add local timezone/DST, nor the TAI-UTC offset used by some PTP deployments.
    Integer microseconds retain fractional seconds and carry rounding across a
    week boundary before formatting the command (sec must stay below 604800).
    """
    if not math.isfinite(unix_seconds) or unix_seconds < GPS_EPOCH_UNIX:
        raise ValueError("UTC time must be finite and at or after the GPS epoch")
    if gps_utc_offset is None:
        if unix_seconds >= LEAP_TABLE_VALID_UNTIL:
            raise ValueError("refresh the leap-second table or supply a verified --gps-utc-offset")
        gps_utc_offset = bisect_right(LEAP_DATES, unix_seconds)
    if isinstance(gps_utc_offset, bool) or not isinstance(gps_utc_offset, int) or gps_utc_offset < 0:
        raise ValueError("GPS-UTC offset must be a non-negative integer number of seconds")
    total = Decimal(str(unix_seconds)) - GPS_EPOCH_UNIX + gps_utc_offset
    microseconds = int((total * 1_000_000).to_integral_value(rounding=ROUND_HALF_UP))
    week, remainder = divmod(microseconds, WEEK_MICROSECONDS)
    if not 0 <= week <= 9999:  # NovAtel SETAPPROXTIME field range, full week (not 10-bit).
        raise ValueError("GPS week outside NovAtel's supported range 0..9999")
    return week, remainder / 1_000_000


@dataclass(frozen=True)
class TimeReference:
    utc: float
    sampled_at: float
    source: str

    def command(self, gps_utc_offset=None):
        # PTP may step the host's wall clock while we connect/wait for ROS. Advance
        # the independent reference with monotonic elapsed time, not time.time().
        elapsed = time.monotonic() - self.sampled_at
        if not 0 <= elapsed < 600:
            raise ValueError("time reference is stale; obtain a new reference before setting CPT7")
        week, seconds = utc_to_gps(self.utc + elapsed, gps_utc_offset)
        return f"SETAPPROXTIME {week} {seconds:.6f}"


def get_reference(args):
    if args.manual or args.year is not None:
        raise ValueError("--manual/--year no longer fabricate a date; use --utc with the actual current time")
    if args.utc is not None:
        return TimeReference(args.utc, time.monotonic(), "explicit UTC")
    if args.local:
        return TimeReference(time.time(), time.monotonic(), "PC clock (must be independent of CPT7)")

    before = time.monotonic()
    reply = ntplib.NTPClient().request(args.ntp_server, version=4, timeout=args.timeout)
    after = time.monotonic()
    if reply.mode != 4 or not 1 <= reply.stratum <= 15 or reply.leap != 0:
        raise ValueError("NTP source is unsynchronized, invalid, or announcing a leap; verify the time source")
    if not math.isfinite(reply.tx_time) or reply.tx_time < GPS_EPOCH_UNIX:
        raise ValueError("NTP source returned invalid UTC time")
    # Use the server timestamp itself, not the client wall clock + NTP offset:
    # the host may still be following an uninitialized indoor CPT7. Half the
    # monotonic round-trip is only an approximate transit correction, not a
    # precision guarantee. Fail rather than silently fall back to that host.
    return TimeReference(reply.tx_time + (after - before) / 2, after, f"NTP {args.ntp_server}")


def check_response(response):
    """Accept only an explicit NovAtel ACK, not an echo or arbitrary received bytes."""
    # TCP uses '<OK'; Oem7Cmd strips the framing and returns 'OK'.
    # https://docs.novatel.com/OEM7/Content/Messages/Responses.htm
    for line in response.splitlines():
        line = line.strip().lstrip("<").strip()
        if line.startswith("ERROR:"):
            raise RuntimeError(f"CPT7 rejected command: {line}")
        if line == "OK":
            return True
    return False


def send_tcp(args, reference):
    with socket.create_connection((args.host, args.port), timeout=args.timeout) as connection:
        command = reference.command(args.gps_utc_offset)
        connection.sendall((command + "\r\n").encode("ascii"))
        deadline = time.monotonic() + args.timeout
        pending = b""
        while len(pending) < 65536:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("CPT7 acknowledgement timed out; receiver state is unverified")
            connection.settimeout(remaining)
            chunk = connection.recv(4096)
            if not chunk:
                raise RuntimeError("CPT7 closed the connection before a complete acknowledgement")
            pending += chunk
            # TCP can split '<OK\r\n' anywhere. Only parse terminated lines so
            # '<OK...' or a partial error cannot be mistaken for an ACK.
            lines = re.split(b"[\r\n]", pending)
            pending = lines.pop()
            if check_response(b"\n".join(lines).decode("ascii", errors="replace")):
                return command
        raise RuntimeError("CPT7 response exceeded the expected size without an acknowledgement")


@contextmanager
def rpc_deadline(seconds):
    # rospy's synchronous ServiceProxy has no call timeout. This one-shot Linux
    # CLI uses a wall-time alarm, independent of /clock. A timeout cannot cancel
    # a command already executing in the receiver/driver; report state unknown.
    def expired(signum, frame):
        raise TimeoutError("Oem7Cmd timed out; receiver state is unverified")

    if signal.getitimer(signal.ITIMER_REAL)[0]:
        raise RuntimeError("cannot install RPC deadline while another alarm is active")
    previous = signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)


def send_ros(args, reference):
    import rospy
    from novatel_oem7_msgs.srv import Oem7AbasciiCmd, Oem7AbasciiCmdRequest

    rospy.init_node("cpt7_set_approx_time", anonymous=True)
    rospy.wait_for_service(SERVICE_NAME, timeout=args.timeout)
    service = rospy.ServiceProxy(SERVICE_NAME, Oem7AbasciiCmd)
    try:
        command = reference.command(args.gps_utc_offset)
        # The pinned driver supplies the newline and already retries commands up
        # to ten times with 3 s waits. Repeating this RPC would resend stale time.
        with rpc_deadline(args.ros_timeout):
            response = service(Oem7AbasciiCmdRequest(cmd=command))
        if not check_response(response.rsp):
            raise RuntimeError(f"CPT7 did not acknowledge SETAPPROXTIME: {response.rsp!r}")
        return command
    finally:
        service.close()


def main(args):
    try:
        if not 1 <= args.port <= 65535:
            raise ValueError("TCP port must be in 1..65535")
        reference = get_reference(args)
        # Validate before opening any receiver connection.
        preview = reference.command(args.gps_utc_offset)
        print(f"Time source: {reference.source}")
        if args.dry_run:
            print(preview)
            return
        command = send_ros(args, reference) if args.ros else send_tcp(args, reference)
        print(f"CPT7 acknowledged: {command}")
        print("Approximate time seeded. Verify TIME/PTPDELTATIME and slave clocks before recording.")
    except Exception as error:
        # boxi's dispatcher ignores returned exit codes; explicitly fail the CLI
        # so startup scripts cannot report success after rejection or timeout.
        raise SystemExit(f"CPT7 time setting failed: {error}") from error
