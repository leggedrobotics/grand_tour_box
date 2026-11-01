"""Shared, offline GPX input contracts for public and internal topic layouts."""

from datetime import datetime, timezone
import math
import operator


def acquisition_time(message, record_time):
    # A sensor header is the acquisition clock; bag time can be recorder arrival
    # time. Only headerless messages use bag time. An invalid header is an error,
    # not a reason to silently switch clocks. GPX requires an explicit UTC zone.
    stamp = message.header.stamp if hasattr(message, "header") else record_time
    seconds = stamp.to_sec()
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("GPX acquisition timestamp must be finite and positive")
    return datetime.fromtimestamp(seconds, tz=timezone.utc)


def validate_coordinates(latitude, longitude, altitude):
    values = (latitude, longitude, altitude)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("GPX coordinates must be finite")
    if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
        raise ValueError("GPX latitude/longitude are outside their valid ranges")


def decimate_coordinates(points, max_points=5000):
    max_points = operator.index(max_points)
    if max_points < 2:
        raise ValueError("max_points must be at least 2 to retain both endpoints")
    count = min(len(points), max_points)
    if count < 2:
        return [(point[0], point[1]) for point in points]
    # Integer interpolation bounds the count exactly and retains both endpoints.
    return [
        (points[i * (len(points) - 1) // (count - 1)][0], points[i * (len(points) - 1) // (count - 1)][1])
        for i in range(count)
    ]
