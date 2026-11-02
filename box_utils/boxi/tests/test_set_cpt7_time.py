"""Clock and command contracts; mocks/loopback only, never contact a CPT7 or NTP server."""

import argparse
import contextlib
from datetime import datetime, timezone
import importlib.util
import io
import os
from pathlib import Path
import socket
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location("cpt7_clock", Path(__file__).parents[1] / "boxi/set_cpt7_time.py")
clock = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = clock
spec.loader.exec_module(clock)


def args(*options):
    return clock.add_arguments(argparse.ArgumentParser()).parse_args(options)


def utc(value):
    return datetime.fromisoformat(value).replace(tzinfo=timezone.utc).timestamp()


class ClockTests(unittest.TestCase):
    def test_known_gps_reference_and_fraction(self):
        self.assertEqual(clock.utc_to_gps(utc("2024-01-01")), (2295, 86418.0))
        self.assertEqual(clock.utc_to_gps(utc("2024-01-01") + 0.125), (2295, 86418.125))
        self.assertEqual(clock.utc_to_gps(clock.GPS_EPOCH_UNIX), (0, 0.0))

    def test_timezone_and_dst_independence(self):
        original = os.environ.get("TZ")
        try:
            for zone in ("UTC", "Europe/Zurich", "America/New_York", "Asia/Kathmandu"):
                os.environ["TZ"] = zone
                time.tzset()
                for day in ("2024-01-01", "2024-07-01"):
                    value = utc(day)
                    week, seconds = clock.utc_to_gps(value)
                    self.assertEqual(week * 604800 + seconds, value - clock.GPS_EPOCH_UNIX + 18)
        finally:
            if original is None:
                os.environ.pop("TZ", None)
            else:
                os.environ["TZ"] = original
            time.tzset()

    def test_historical_leap_transition(self):
        before = clock.utc_to_gps(utc("2016-12-31T23:59:59"))
        after = clock.utc_to_gps(utc("2017-01-01"))
        self.assertEqual(after[0] * 604800 + after[1] - before[0] * 604800 - before[1], 2)
        stamp = utc("1996-06-01")
        week, seconds = clock.utc_to_gps(stamp)
        self.assertEqual(week * 604800 + seconds, stamp - clock.GPS_EPOCH_UNIX + 11)

    def test_rounding_carries_at_gps_week_boundary(self):
        boundary = clock.GPS_EPOCH_UNIX + 2300 * 604800 - 18
        self.assertEqual(clock.utc_to_gps(boundary - 0.0000004), (2300, 0.0))
        self.assertEqual(clock.utc_to_gps(boundary - 0.000001), (2299, 604799.999999))

    def test_invalid_dates_offsets_and_expired_table(self):
        for stamp in (float("nan"), float("inf"), clock.GPS_EPOCH_UNIX - 1):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                clock.utc_to_gps(stamp)
        with self.assertRaises(ValueError):
            clock.utc_to_gps(clock.LEAP_TABLE_VALID_UNTIL)
        self.assertEqual(clock.utc_to_gps(clock.LEAP_TABLE_VALID_UNTIL, 18), (2477, 86418.0))
        for offset in (-1, True, 18.5):
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                clock.utc_to_gps(utc("2024-01-01"), offset)
        with self.assertRaises(ValueError):
            clock.utc_to_gps(clock.GPS_EPOCH_UNIX + 10000 * 604800, 18)

    def test_explicit_timezone_required(self):
        self.assertEqual(clock.parse_utc("2024-01-01T01:00:00+01:00"), utc("2024-01-01"))
        self.assertEqual(clock.parse_utc("2024-01-01T00:00:00Z"), utc("2024-01-01"))
        for value in ("2024-01-01T01:00:00", "not-a-date"):
            with self.assertRaises(ValueError):
                clock.parse_utc(value)

    def test_manual_year_and_conflicting_sources_fail(self):
        for options in (("--manual",), ("--year", "1996")):
            with self.assertRaises(ValueError):
                clock.get_reference(args(*options))
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            args("--local", "--utc", "2024-01-01T00:00:00Z")

    def test_reference_advances_using_monotonic_clock(self):
        reference = clock.TimeReference(utc("2024-01-01"), 100, "test")
        with patch.object(clock.time, "monotonic", return_value=100.25), patch.object(
            clock.time, "time", return_value=0
        ):
            self.assertEqual(reference.command(), "SETAPPROXTIME 2295 86418.250000")
        with patch.object(clock.time, "monotonic", return_value=700), self.assertRaises(ValueError):
            reference.command()

    def test_ntp_uses_server_time_and_monotonic_transit(self):
        response = SimpleNamespace(mode=4, stratum=2, leap=0, tx_time=utc("2024-01-01"))
        with patch.object(clock.ntplib, "NTPClient") as client, patch.object(
            clock.time, "monotonic", side_effect=[100, 100.2]
        ):
            client.return_value.request.return_value = response
            reference = clock.get_reference(args())
            self.assertAlmostEqual(reference.utc, response.tx_time + 0.1)
            client.return_value.request.assert_called_once_with("pool.ntp.org", version=4, timeout=5)

    def test_bad_ntp_never_falls_back_or_contacts_receiver(self):
        for changes in (
            {"stratum": 0},
            {"stratum": 16},
            {"leap": 3},
            {"leap": 1},
            {"mode": 3},
            {"tx_time": float("nan")},
        ):
            response = SimpleNamespace(
                **dict({"mode": 4, "stratum": 2, "leap": 0, "tx_time": utc("2024-01-01")}, **changes)
            )
            with patch.object(clock.ntplib, "NTPClient") as client, patch.object(clock, "send_tcp") as send:
                client.return_value.request.return_value = response
                with self.assertRaises(SystemExit):
                    clock.main(args())
                send.assert_not_called()
        with patch.object(clock.ntplib, "NTPClient") as client, patch.object(clock, "send_tcp") as send:
            client.return_value.request.side_effect = TimeoutError("NTP unavailable")
            with self.assertRaises(SystemExit):
                clock.main(args())
            send.assert_not_called()

    def test_local_and_dry_run_have_no_network(self):
        with patch.object(clock.time, "time", return_value=utc("2024-01-01")), patch.object(
            clock, "send_tcp"
        ) as send, patch.object(clock.ntplib, "NTPClient") as ntp, contextlib.redirect_stdout(io.StringIO()) as output:
            clock.main(args("--local", "--dry-run"))
            self.assertIn("SETAPPROXTIME 2295", output.getvalue())
            send.assert_not_called()
            ntp.assert_not_called()

    def test_response_requires_explicit_ack(self):
        for value in ("OK", "<OK\r\n", "\r\n<OK\r\n[ICOM1]"):
            self.assertTrue(clock.check_response(value))
        for value in ("", "NOT OK", "<OKAY", "SETAPPROXTIME 2295 10"):
            self.assertFalse(clock.check_response(value))
        with self.assertRaises(RuntimeError):
            clock.check_response("<ERROR: Invalid parameter\r\n")

    def test_tcp_loopback_fragmented_ack_and_terminator(self):
        errors, commands = [], []
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen(1)
            listener.settimeout(2)

            def server():
                try:
                    with listener.accept()[0] as connection:
                        connection.settimeout(2)
                        data = b""
                        while not data.endswith(b"\r\n"):
                            data += connection.recv(1024)
                        commands.append(data)
                        for fragment in (b"<", b"O", b"K\r", b"\n"):
                            connection.sendall(fragment)
                except Exception as error:
                    errors.append(error)

            thread = threading.Thread(target=server)
            thread.start()
            try:
                reference = clock.TimeReference(utc("2024-01-01"), time.monotonic(), "test")
                command = clock.send_tcp(
                    args("--host", "127.0.0.1", "--port", str(listener.getsockname()[1])), reference
                )
            finally:
                thread.join(3)
            self.assertFalse(thread.is_alive())
            self.assertFalse(errors, errors)
            self.assertEqual(commands, [(command + "\r\n").encode()])

    def test_tcp_rejection_disconnect_and_timeout_fail(self):
        reference = clock.TimeReference(utc("2024-01-01"), time.monotonic(), "test")
        for reply in (b"<ERROR: Invalid parameter\r\n", b"", TimeoutError("deadline")):
            with patch.object(clock.socket, "create_connection") as create:
                connection = create.return_value.__enter__.return_value
                if isinstance(reply, Exception):
                    connection.recv.side_effect = reply
                else:
                    connection.recv.return_value = reply
                with self.assertRaises((RuntimeError, TimeoutError)):
                    clock.send_tcp(args(), reference)

    def test_ros_checks_ack_and_closes_service(self):
        rospy = Mock()
        srv = ModuleType("novatel_oem7_msgs.srv")
        srv.Oem7AbasciiCmd = object
        srv.Oem7AbasciiCmdRequest = lambda **kw: SimpleNamespace(**kw)
        reference = clock.TimeReference(utc("2024-01-01"), time.monotonic(), "test")
        with patch.dict(sys.modules, {"rospy": rospy, "novatel_oem7_msgs.srv": srv}):
            for response in ("OK", "ERROR: Invalid parameter", ""):
                service = Mock(return_value=SimpleNamespace(rsp=response))
                rospy.ServiceProxy.return_value = service
                if response == "OK":
                    self.assertTrue(clock.send_ros(args("--ros"), reference).startswith("SETAPPROXTIME "))
                else:
                    with self.assertRaises(RuntimeError):
                        clock.send_ros(args("--ros"), reference)
                service.assert_called_once()
                service.close.assert_called_once()
                rospy.wait_for_service.assert_called_with(clock.SERVICE_NAME, timeout=5)

    def test_rpc_deadline_interrupts_and_restores_alarm(self):
        before = clock.signal.getsignal(clock.signal.SIGALRM)
        with self.assertRaises(TimeoutError):
            with clock.rpc_deadline(0.01):
                time.sleep(0.1)
        self.assertEqual(clock.signal.getitimer(clock.signal.ITIMER_REAL)[0], 0)
        self.assertEqual(clock.signal.getsignal(clock.signal.SIGALRM), before)

    def test_command_failure_is_nonzero_cli_exit(self):
        with patch.object(clock, "send_tcp", side_effect=TimeoutError("no ACK")), contextlib.redirect_stdout(
            io.StringIO()
        ):
            with self.assertRaises(SystemExit) as failure:
                clock.main(args("--utc", "2024-01-01T00:00:00Z"))
            self.assertNotEqual(failure.exception.code, 0)


if __name__ == "__main__":
    unittest.main()
