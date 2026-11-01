#!/usr/bin/env python3
from __future__ import annotations

# ─── IMPORTS ────────────────────────────────────────────────────────────────
import base64
import io
import json
import math
import shutil
import tempfile
import unicodedata
import warnings
import webbrowser
from pathlib import Path
from datetime import datetime
from typing import List, Tuple, Optional, Iterable, Dict

import matplotlib

matplotlib.use("Agg")  # non-interactive backend for speed / headless use
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import matplotlib.ticker as mticker
import numpy as np

# rosbag import (suppress SyntaxWarnings from rosbag import paths)
with warnings.catch_warnings():
    warnings.simplefilter("ignore", SyntaxWarning)
    import rosbag  # ROS1 bag reader

import gpxpy
import folium
import gspread
from folium.plugins import MiniMap, MeasureControl, HeatMap, Fullscreen
from geopy.geocoders import Nominatim
from geopy.extra.rate_limiter import RateLimiter

import kleinkram

from box_auto.utils.atomic_file import atomic_output
from box_auto.utils.gpx_data import acquisition_time, validate_coordinates, decimate_coordinates

from box_auto.utils import (
    get_bag,
    MISSION_DATA,
    BOX_AUTO_DIR,
    WS,
    find_and_extract_non_matching,
    get_uuid_mapping,
    read_sheet_data,
)

# ─── FEATURE FLAGS / PARAMETERS (EDIT THESE) ────────────────────────────────
SHEET_UPDATE: bool = False  # True: write stats into Google Sheet (gnss_quality_results)

# Output locations
HTML_OUT_DIR: Path = Path(MISSION_DATA) / "gps_html_out"  # where ALL HTML/GPX are written
TMP_BASE: Path = Path("/tmp")  # where remote bags are downloaded (automation)

# Local-mode browser behavior; CLI options set these values.
OPEN_HTML_INTERACTIVE: bool = False  # True: open HTML in browser, False: never open

# Automation behavior
SKIP_FIRST: int = 0  # skip first N missions in uuid mapping enumeration
MIN_REMOTE_BAG_MIB: float = 0.05  # minimum remote bag size (MiB) to consider valid

# Data-quality thresholds
MIN_GPS_MSGS: int = 10  # min messages on VALID_TOPIC required to process
MIN_GPS_POINTS: int = 10  # min parsed points required to generate outputs
MIN_COV_SAMPLES: int = 1  # min valid covariance samples to compute std-dev percentages

# How to derive a 1D std-dev (meters) from NavSatFix position_covariance (m^2)
# Options: "horizontal_rms", "horizontal_max", "3d_rms", "3d_max"
COV_STDDEV_MODE: str = "horizontal_rms"

# Reverse geocoding controls (prevents "stuck in build folium map")
ENABLE_REVERSE_GEOCODING: bool = False
REVERSE_GEOCODE_TIMEOUT_S: float = 3.0
REVERSE_GEOCODE_MIN_DELAY_S: float = 1.0
REVERSE_GEOCODE_ERROR_WAIT_S: float = 1.0  # MUST be >= REVERSE_GEOCODE_MIN_DELAY_S (geopy RateLimiter assert)
REVERSE_GEOCODE_MAX_RETRIES: int = 0  # 0/1 recommended to avoid long waits in automation

# ─── CONSTANTS ──────────────────────────────────────────────────────────────
MODE = "tc"
VALID_TOPIC = f"/boxi/inertial_explorer/{MODE}/navsatfix"
GPS_PATTERN = f"*_cpt7_ie_{MODE}.bag"

BASE_INFO_LINK = "https://grand-tour.leggedrobotics.com/"
MISSION_LINK_TEMPLATE = "https://grand-tour.leggedrobotics.com/dataset#mission-{}-on"

SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"

Q_LABELS = [f"Q {i}" for i in range(1, 7)]
STD_LABELS = [
    "0.00 - 0.10 m",
    "0.10 - 0.30 m",
    "0.30 - 1.00 m",
    "1.00 - 5.00 m",
    "5.00 m + over",
]
STD_MAP = {
    "0.00 - 0.10 m": "0_to_010m",
    "0.10 - 0.30 m": "010_to_030m",
    "0.30 - 1.00 m": "030_to_1m",
    "1.00 - 5.00 m": "1_to_5m",
    "5.00 m + over": "5_plus",
}

# Codename -> mission ID (dataset anchor uses this ID)
_CODENAME_TO_ID: Dict[str, int] = {
    "ETH-1": 1,
    "ETH-2": 2,
    "ETH-3": 3,
    "SPX-1": 4,
    "SPX-2": 5,
    "SPX-3": 6,
    "ICE-1": 7,
    "SNOW-1": 8,
    "SNOW-2": 9,
    "SNOW-3": 10,
    "EIG-1": 11,
    "EIG-2": 12,
    "GRI-1": 13,
    "CYN-1": 14,
    "CYN-2": 15,
    "HIL-1": 16,
    "RIV-1": 17,
    "PIL-1": 18,
    "PIL-2": 19,
    "ROOT-1": 20,
    "HAUS-1": 21,
    "HÖB-1": 22,
    "HÖB-2": 23,
    "HEAP-1": 24,
    "KÄB-1": 25,
    "KÄB-2": 26,
    "KÄB-3": 27,
    "TRIM-1": 28,
    "ALB-1": 29,
    "ALB-2": 30,
    "ALB-3": 31,
    "LMB-1": 32,
    "LMB-2": 33,
    "LEE-1": 34,
    "ARC-1": 35,
    "ARC-2": 36,
    "ARC-3": 37,
    "ARC-4": 38,
    "ARC-5": 39,
    "ARC-6": 40,
    "ARC-7": 41,
    "LEICA-1": 42,
    "LEICA-2": 43,
    "SBB-1": 44,
    "SBB-2": 45,
    "CON-1": 46,
    "CON-2": 47,
    "CON-3": 48,
    "CON-4": 49,
}

# ANSI color
_ORANGE = "\033[38;5;208m"
_RESET = "\033[0m"


# ─── LOGGING ────────────────────────────────────────────────────────────────
def _log(mission_id: str, stage: str, msg: str) -> None:
    print(f"[{mission_id}] [{stage}] {msg}")


def _log_orange(mission_id: str, stage: str, msg: str) -> None:
    print(f"{_ORANGE}[{mission_id}] [{stage}] {msg}{_RESET}")


# ─── LINK HELPERS ───────────────────────────────────────────────────────────
def _normalize_codename(s: str) -> str:
    return unicodedata.normalize("NFC", s or "").strip()


def _ascii_fallback_codename(s: str) -> str:
    return s.replace("Ä", "A").replace("Ö", "O").replace("Ü", "U").replace("ä", "a").replace("ö", "o").replace("ü", "u")


def mission_link_for_codename(codename: str) -> str:
    cn = _normalize_codename(codename).upper()
    mid = _CODENAME_TO_ID.get(cn)
    if mid is None:
        mid = _CODENAME_TO_ID.get(_ascii_fallback_codename(cn))
    return MISSION_LINK_TEMPLATE.format(mid) if mid is not None else BASE_INFO_LINK


# ─── GEOPY SETUP (FIXES YOUR CRASH) ─────────────────────────────────────────
# Your geopy version asserts: error_wait_seconds >= min_delay_seconds.
# If min_delay_seconds=1 and error_wait_seconds defaults to 0, it crashes at import.
_GEOL = Nominatim(user_agent="gpx_viewer")


def _reverse_impl(query):
    # Ensure bounded runtime; RateLimiter will call this
    return _GEOL.reverse(query, exactly_one=True, timeout=REVERSE_GEOCODE_TIMEOUT_S)


_REVERSE = RateLimiter(
    _reverse_impl,
    min_delay_seconds=REVERSE_GEOCODE_MIN_DELAY_S,
    error_wait_seconds=REVERSE_GEOCODE_ERROR_WAIT_S,  # MUST be >= min_delay_seconds
    max_retries=REVERSE_GEOCODE_MAX_RETRIES,
    swallow_exceptions=True,
    return_value_on_exception=None,
)


# ─── BASIC HELPERS ──────────────────────────────────────────────────────────
def pick_color(idx: int, total: int) -> str:
    if idx < 2:
        return "darkgreen"
    if idx == 2:
        return "#DAA520"
    if idx >= total - 2:
        return "red"
    return "inherit"


def ecef_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 6.69437999014e-3
    lon = np.arctan2(y, x)
    b = a * (1 - f)
    ep2 = (a**2 - b**2) / b**2
    p = np.sqrt(x**2 + y**2)
    theta = np.arctan2(z * a, p * b)
    lat = np.arctan2(
        z + ep2 * b * np.sin(theta) ** 3,
        p - e2 * a * np.cos(theta) ** 3,
    )
    N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - N
    return np.degrees(lat), np.degrees(lon), alt


def _is_finite(x: float) -> bool:
    return (x is not None) and math.isfinite(float(x))


def _sigma_from_navsatfix_cov(msg) -> Optional[float]:
    cov = getattr(msg, "position_covariance", None)
    ctype = getattr(msg, "position_covariance_type", 0)  # 0 = UNKNOWN
    if cov is None or len(cov) < 9:
        return None
    try:
        vx = float(cov[0])
        vy = float(cov[4])
        vz = float(cov[8])
    except Exception:
        return None
    if int(ctype) == 0 and (vx == 0.0 and vy == 0.0 and vz == 0.0):
        return None
    if not (_is_finite(vx) and _is_finite(vy) and _is_finite(vz)):
        return None
    if vx < 0.0 or vy < 0.0 or vz < 0.0:
        return None

    if COV_STDDEV_MODE == "horizontal_rms":
        var = 0.5 * (vx + vy)
    elif COV_STDDEV_MODE == "horizontal_max":
        var = max(vx, vy)
    elif COV_STDDEV_MODE == "3d_rms":
        var = (vx + vy + vz) / 3.0
    elif COV_STDDEV_MODE == "3d_max":
        var = max(vx, vy, vz)
    else:
        var = 0.5 * (vx + vy)

    if var <= 0.0:
        return None
    return float(math.sqrt(var))


def _std_bucket_label(sigma_m: float) -> str:
    if sigma_m < 0.10:
        return STD_LABELS[0]
    if sigma_m < 0.30:
        return STD_LABELS[1]
    if sigma_m < 1.00:
        return STD_LABELS[2]
    if sigma_m < 5.00:
        return STD_LABELS[3]
    return STD_LABELS[4]


# ─── BASEMAP LAYERS ─────────────────────────────────────────────────────────
def basemap_layers() -> Tuple[str, str, str, str, str, str, str, str, str, str]:
    esri = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    carto = "https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png"
    osm = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
    g_hyb = "https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}"
    g_rd = "https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}"
    g_sat = "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"
    rnli = '<a href="https://rnli.org/">&copy; RNLI</a>'
    return (
        esri,
        f"{rnli} | <a href='https://www.esri.com'>&copy; ESRI</a>",
        carto,
        f"{rnli} | <a href='https://www.carto.com'>&copy; Carto</a>",
        osm,
        f"{rnli} | <a href='https://www.openstreetmap.org'>&copy; OSM</a>",
        g_hyb,
        g_rd,
        g_sat,
        f"{rnli} | <a href='https://www.google.com'>&copy; Google</a>",
    )


# ─── GOOGLE SHEETS UTILS ────────────────────────────────────────────────────
def read_gspread_row(mission_id: str):
    gc = gspread.service_account(
        filename=Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json",
    )
    sheet = gc.open_by_key(SPREADSHEET_ID)
    ws = sheet.worksheet("gnss_quality_results")

    colA = ws.col_values(1)
    try:
        row_idx = next(i for i, v in enumerate(colA, start=1) if v == mission_id)
    except StopIteration as cause:
        raise ValueError(f"No row found with mission_id '{mission_id}'") from cause

    headers = ws.row_values(1)

    def col_index(name: str) -> int:
        return headers.index(name)

    row = ws.row_values(row_idx)
    good_mission = row[col_index("good_mission")].strip().upper() == "TRUE"
    codename = row[col_index("codename")] if col_index("codename") < len(row) else "Unknown"
    return good_mission, _normalize_codename(codename), row_idx, ws


def write_stats_to_sheet(gs: dict, stats: dict) -> None:
    ws = gs["worksheet"]
    row_idx = gs["row_index"]
    headers = ws.row_values(1)

    def col(name: str) -> int:
        return headers.index(name) + 1

    cells = []

    for lbl in Q_LABELS:
        key = lbl.replace(" ", "")  # Q 1 -> Q1
        if key in headers:
            cells.append(gspread.Cell(row_idx, col(key), stats.get(lbl, "N/A")))

    for lbl in STD_LABELS:
        key = STD_MAP[lbl]
        if key in headers:
            cells.append(gspread.Cell(row_idx, col(key), stats.get(lbl, "N/A")))

    if cells:
        ws.update_cells(cells)


# ─── STATS / GPX BUILDERS ───────────────────────────────────────────────────
def load_stats_optional(mission_id: str) -> dict:
    stats_file = Path(MISSION_DATA) / f"{mission_id}_cpt7_ie_{MODE}_statistics.json"
    try:
        with open(stats_file, "r") as fh:
            obj = json.load(fh)
            return obj if isinstance(obj, dict) else {}
    except Exception:
        return {}


def generate_gpx_and_cov_stats(
    bag_path: str,
    min_msgs: int = 0,
) -> Tuple[gpxpy.gpx.GPX, List[Tuple], Dict[str, str], int, int]:
    gpx = gpxpy.gpx.GPX()
    seg = gpxpy.gpx.GPXTrackSegment()
    track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(track)
    track.segments.append(seg)

    points: List[Tuple[float, float, float, datetime]] = []
    bucket_counts = {lbl: 0 for lbl in STD_LABELS}
    cov_valid = 0

    with rosbag.Bag(bag_path, "r") as bag:
        n_msgs = int(bag.get_message_count(topic_filters=[VALID_TOPIC]))
        if min_msgs > 0 and n_msgs < min_msgs:
            return gpx, points, {}, n_msgs, cov_valid

        for _, msg, ts in bag.read_messages(topics=[VALID_TOPIC]):
            if getattr(msg, "_type", "") == "sensor_msgs/NavSatFix":
                lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
                sigma = _sigma_from_navsatfix_cov(msg)
                if sigma is not None:
                    cov_valid += 1
                    bucket_counts[_std_bucket_label(sigma)] += 1
            else:
                lat, lon, alt = ecef_to_lla(msg.x, msg.y, msg.z)

            validate_coordinates(lat, lon, alt)
            timestamp = acquisition_time(msg, ts)
            seg.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon, alt, timestamp))
            points.append((lat, lon, alt, timestamp))

    std_pct: Dict[str, str] = {}
    if cov_valid >= MIN_COV_SAMPLES:
        for lbl in STD_LABELS:
            pct = 100.0 * float(bucket_counts[lbl]) / float(cov_valid)
            std_pct[lbl] = f"{pct:.1f} %"

    return gpx, points, std_pct, n_msgs, cov_valid


def build_stats_html(stats: dict) -> str:
    def make_rows(labels):
        return "\n".join(
            f"<tr><td style='padding:2px;color:{pick_color(i, len(labels))}'>{lbl}</td>"
            f"<td style='padding:2px'>{stats.get(lbl, 'N/A')}</td></tr>"
            for i, lbl in enumerate(labels)
        )

    return (
        "<strong style='font-size:13px;display:block;margin-top:4px;'>Std-Dev Range %</strong>"
        "<table style='font-size:12px;width:100%;border-collapse:collapse;'>"
        f"{make_rows(STD_LABELS)}</table>"
    )


# ─── VISUAL BUILDERS ────────────────────────────────────────────────────────
def create_elevation_plot(times: List[datetime], elevs: List[float]) -> str:
    if not times or not elevs:
        return ""

    rel = [(t - times[0]).total_seconds() for t in times]
    roboto_path = fm.findfont("Roboto", fallback_to_default=True)
    roboto = fm.FontProperties(fname=roboto_path)

    fig, ax = plt.subplots(figsize=(20, 2))
    fig.patch.set_edgecolor("black")
    fig.patch.set_linewidth(5.0)

    for spine in ax.spines.values():
        spine.set_edgecolor("black")
        spine.set_linewidth(5.0)

    ax.plot(
        rel,
        elevs,
        marker="o",
        markersize=4,
        linewidth=1,
        color="#0099D8",
        markerfacecolor="#FFFFFF",
        markeredgecolor="#0099D8",
    )

    ax.yaxis.grid(True, color="#E1E5EA", linewidth=0.8)
    ax.xaxis.grid(False)

    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    for spine in ["bottom", "left"]:
        ax.spines[spine].set_color("#6B6B6B")
        ax.spines[spine].set_linewidth(0.8)

    ax.set_title("Elevation Change", fontproperties=roboto, fontsize=14, fontweight="bold", pad=6)
    ax.xaxis.set_major_formatter(mticker.FuncFormatter(lambda x, pos: f"{int(x)} s"))
    ax.set_ylabel("Elevation (m)", fontproperties=roboto, fontsize=12, fontweight="bold", labelpad=4)

    for lbl in ax.get_xticklabels() + ax.get_yticklabels():
        lbl.set_fontproperties(roboto)
        lbl.set_fontsize(10)

    ax.margins(x=0, y=0.02)
    fig.tight_layout(pad=0)

    buf = io.BytesIO()
    plt.savefig(buf, format="png", bbox_inches="tight", pad_inches=0, transparent=True)
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode("utf-8")


def _reverse_geocode_safe(lat: float, lon: float):
    if not ENABLE_REVERSE_GEOCODING:
        return None
    try:
        return _REVERSE((lat, lon))
    except Exception:
        return None


def get_city_name(lat: float, lon: float) -> str:
    loc = _reverse_geocode_safe(lat, lon)
    try:
        addr = loc.raw.get("address", {}) if loc else {}
        return addr.get("city") or addr.get("town") or addr.get("village") or "Unknown"
    except Exception:
        return "Unknown"


def get_end_address(lat: float, lon: float) -> str:
    loc = _reverse_geocode_safe(lat, lon)
    try:
        return loc.address if loc and getattr(loc, "address", None) else "Unknown"
    except Exception:
        return "Unknown"


def _decimate_coords(points, max_points=5000):
    return decimate_coordinates(points, max_points)


def build_folium_map(
    points: List[Tuple],
    mission_name: str,
    city: str,
    end_text: str,
    stats_html: str,
    encoded_plot: str,
    codename: str = "Unknown",
    info_link: str = BASE_INFO_LINK,
) -> folium.Map:
    # IMPORTANT: no network calls in here (prevents "stuck in build folium map")
    if not points:
        raise ValueError("No points to plot.")

    points.sort(key=lambda p: p[3])

    first_lat, first_lon = points[0][:2]
    m = folium.Map(
        location=(first_lat, first_lon),
        zoom_start=18,
        max_zoom=24,
        zoomSnap=0,
        zoomDelta=0.25,
        tiles=None,
    )

    esri, esri_attr, _, _, _, _, g_hyb, _, g_sat, g_attr = basemap_layers()
    folium.TileLayer(esri, attr=esri_attr, name="ESRI World Imagery", max_zoom=24).add_to(m)
    folium.TileLayer(g_sat, attr=g_attr, name="Google Satellite", max_zoom=24).add_to(m)
    folium.TileLayer(g_hyb, attr=g_attr, name="Google Hybrid", max_zoom=24, show=True).add_to(m)

    coords = _decimate_coords(points, max_points=5000)
    folium.PolyLine(coords, color="blue", weight=3).add_to(m)

    heat_group = folium.FeatureGroup(name="Heat Map", show=True)
    HeatMap(coords).add_to(heat_group)
    heat_group.add_to(m)

    elev0 = f"{points[0][2]:.2f} m" if points[0][2] is not None else "N/A"
    popup_html = (
        "<div style='font-family:Roboto;font-size:14px;'>"
        f"<strong>Mission ID:</strong> {mission_name}<br>"
        f"<strong>Place:</strong> {city}<br>"
        f"<strong>Date:</strong> {points[0][3]}<br>"
        f"<strong>Elevation:</strong> {elev0}<br>"
        f"<a href='{info_link}' target='_blank'>More Info</a>"
        f"{stats_html}</div>"
    )

    folium.Marker(
        location=(first_lat, first_lon),
        popup=folium.Popup(popup_html, max_width=300),
        tooltip="Mission start",
        icon=folium.Icon(color="green"),
    ).add_to(m)

    end_lat, end_lon = points[-1][:2]
    folium.Marker(
        location=(end_lat, end_lon),
        popup=f"End: {end_text}",
        tooltip="Mission end",
        icon=folium.Icon(color="red"),
    ).add_to(m)

    MiniMap(toggle_display=True, position="bottomleft").add_to(m)
    MeasureControl(position="topleft", primary_length_unit="meters").add_to(m)
    Fullscreen(position="topleft").add_to(m)
    folium.LayerControl(position="topright", collapsed=False).add_to(m)

    map_name = m.get_name()

    m.get_root().html.add_child(
        folium.Element(
            f"""
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet-sidebar-v2/css/leaflet-sidebar.min.css"/>
<script src="https://unpkg.com/leaflet-sidebar-v2/js/leaflet-sidebar.min.js"></script>

<div id="sidebar" class="leaflet-sidebar collapsed">
  <div class="leaflet-sidebar-tabs">
    <ul role="tablist">
      <li><a href="#layers" role="tab"><i class="fa fa-layer-group"></i></a></li>
    </ul>
  </div>
  <div class="leaflet-sidebar-content">
    <div class="leaflet-sidebar-pane" id="layers">
      <h1 class="leaflet-sidebar-header">Layers
        <span class="leaflet-sidebar-close"><i class="fa fa-caret-left"></i></span>
      </h1>
      <div id="layerControlContainer"></div>

      <div id="heatmap-legend" style="padding: 12px 16px; font-size: 16px; font-family: Roboto, sans-serif; color: #444;">
        <strong style="display:block; margin-bottom:6px;">Heatmap Legend</strong>
        <div style="display: flex; align-items: center;">
          <span style="width: 100px; height: 12px; display: inline-block;
                       background: linear-gradient(to right, blue, lime, yellow, red);
                       border-radius: 6px;"></span>
          <span style="margin-left: 10px; font-size: 16px;">low freq. → high freq.</span>
        </div>
      </div>
    </div>
  </div>
</div>

<style>
#layerControlContainer .leaflet-control-layers {{
  position: static !important;
  width: 100% !important;
  margin: 0 !important;
  padding: 0 !important;
  border: none !important;
  box-shadow: none !important;
  background: none !important;
}}
#layerControlContainer .leaflet-control-layers-list {{
  padding: 10px;
  font-size: 16px;
  font-family: Roboto, sans-serif;
}}
</style>

<script>
setTimeout(function() {{
  var sidebar = L.control.sidebar({{ container: 'sidebar', position: 'left' }}).addTo({map_name});
  var control = document.querySelector('.leaflet-control-layers');
  var target = document.getElementById('layerControlContainer');
  if (control && target) {{
    target.appendChild(control);
  }}
}}, 1000);
</script>
"""
        )
    )

    rsl_logo_path = Path(WS) / "src/grand_tour_box/box_documentation/images/rsl_logo.webp"
    if rsl_logo_path.is_file():
        rsl_b64 = base64.b64encode(rsl_logo_path.read_bytes()).decode("utf-8")
        logo_img_html = (
            f'<img src="data:image/webp;base64,{rsl_b64}" '
            'style="width:50px; height:auto; vertical-align:middle; margin-right:8px;" />'
        )
    else:
        logo_img_html = ""

    logo_and_text = (
        f'<a href="{info_link}" target="_blank" style="text-decoration:none; color:inherit;">'
        f"{logo_img_html}"
        '<span style="vertical-align:middle; font-weight:bold;">[The GrandTour Dataset]</span>'
        "</a>"
    )

    dynamic_title = (
        f"{logo_and_text} Mission: | "
        f'<a href="{info_link}" target="_blank" style="color:#1E90FF; text-decoration:underline;">{codename}</a> | '
        f"{city} | {mission_name}"
    )

    title_html = f"""
<div style="
  position: fixed;
  top: 10px; left: 50%;
  transform: translateX(-50%);
  z-index: 9999;
  background-color: rgba(255,255,255,0.9);
  padding: 8px 12px;
  border: 2px solid black;
  border-radius: 4px;
  font-family: Roboto;
">
  <h2 style="margin:0; font-size:18px; line-height:1.2;">
    {dynamic_title}
  </h2>
</div>
"""
    m.get_root().html.add_child(folium.Element(title_html))

    if encoded_plot:
        plot_html = (
            "<div id='elevation-overlay' "
            "style='display:none; position:fixed; bottom:10px; right:10px; z-index:9999;"
            " background:rgba(255,255,255,0.9); padding:5px; border:3px solid black;"
            " transition:width 0.2s ease;'>"
            f"<img src='data:image/png;base64,{encoded_plot}' style='width:100%; height:auto;'/>"
            "</div>"
            "<script>"
            "  const overlay = document.getElementById('elevation-overlay');"
            "  function updateOverlayWidth() {"
            "    if (overlay) overlay.style.width = `${window.innerWidth * 0.8}px`;"
            "  }"
            "  window.addEventListener('resize', updateOverlayWidth);"
            "  updateOverlayWidth();"
            "</script>"
        )
        m.get_root().html.add_child(folium.Element(plot_html))

        toggle_button = """
<div style="position:fixed;top:10px;right:10px;z-index:9999;">
  <button id="toggleElevBtn" onclick="toggleElev()"
          style="
            background-color:#ccc;
            color:#333;
            font-weight:bold;
            font-size:18px;
            padding:8px 12px;
            border:none;
            border-radius:4px;
            box-shadow:0 2px 6px rgba(0,0,0,0.2);
            font-family:roboto;
            cursor:pointer;">
    Toggle Elevation
  </button>
</div>
"""
        m.get_root().html.add_child(folium.Element(toggle_button))

        resize_slider = """
<div id="sliderContainer" style="
  display:none;
  position:fixed;
  top:60px;
  right:10px;
  z-index:9999;
  background:#f8f9fa;
  padding:8px 12px;
  border-radius:6px;
  box-shadow:0 2px 6px rgba(0,0,0,0.1);
  font-family:roboto;
  font-size:18px;
  color:#333;">
  <label for="elevSize" style="display:block; margin-bottom:4px; font-weight:bold;">Width:</label>
  <input type="range" id="elevSize" min="400" max="2500" value="2200"
         style="width:100%; -webkit-appearance:none; height:6px; border-radius:3px; background:#ddd; outline:none;"
         oninput="document.getElementById('elevation-overlay').style.width=this.value + 'px';"/>
</div>
"""
        m.get_root().html.add_child(folium.Element(resize_slider))

        toggle_js = """
<script>
function toggleElev() {
  var overlay = document.getElementById('elevation-overlay');
  var slider = document.getElementById('sliderContainer');
  var button = document.getElementById('toggleElevBtn');
  if (!overlay) return;

  var visible = overlay.style.display !== 'none';
  overlay.style.display = visible ? 'none' : 'block';
  slider.style.display = visible ? 'none' : 'block';

  if (visible) { button.style.backgroundColor = '#ccc'; button.style.color = '#333'; }
  else { button.style.backgroundColor = '#1E90FF'; button.style.color = '#fff'; }
}
</script>
"""
        m.get_root().html.add_child(folium.Element(toggle_js))

    return m


# ─── KLEINKRAM REMOTE HELPERS ───────────────────────────────────────────────
def select_best_remote_bag(mission_uuid_release: str, patterns: list[str], min_mib: float) -> Optional[str]:
    candidates = kleinkram.list_files(mission_ids=[mission_uuid_release], file_names=patterns)
    candidates = [f for f in candidates if (f.size / 1024 / 1024) > min_mib]
    if not candidates:
        return None
    candidates.sort(key=lambda f: f.size, reverse=True)
    return candidates[0].file_name if hasattr(candidates[0], "file_name") else candidates[0].name


def download_remote_files(mission_uuid_release: str, file_names: Iterable[str], dest: Path) -> None:
    kleinkram.download(
        mission_ids=[mission_uuid_release],
        file_names=list(file_names),
        dest=dest,
        verbose=True,
    )


def find_local_file(pattern_or_name: str, directory: Path) -> Path:
    try:
        p = get_bag(pattern_or_name, directory=directory)
        return Path(p)
    except Exception as cause:
        cand = directory / Path(pattern_or_name).name
        if cand.exists():
            return cand
        matches = list(directory.glob(pattern_or_name))
        if len(matches) == 1:
            return matches[0]
        raise FileNotFoundError(f"Could not resolve local file for '{pattern_or_name}' in {directory}") from cause


# ─── PIPELINE CORE ──────────────────────────────────────────────────────────
def process_one_mission(
    mission_id: str,
    bag_path: Path,
    out_dir: Path,
    open_html: bool,
    note_list: List[str],
    sheet_update: bool = False,
) -> Optional[Path]:
    _log(mission_id, "PROCESS", f"begin (bag={bag_path})")
    out_dir.mkdir(parents=True, exist_ok=True)

    codename = "Unknown"
    gs_info = None
    if sheet_update:
        _log(mission_id, "SHEET", "lookup gnss_quality_results row")
        try:
            _good, codename, row_idx, ws = read_gspread_row(mission_id)
            gs_info = {
                "good_mission": _good,
                "codename": codename,
                "row_index": row_idx,
                "worksheet": ws,
            }
            _log(mission_id, "SHEET", f"found row={row_idx}, codename='{codename}', good_mission={_good}")
        except Exception as exc:
            _log_orange(mission_id, "SHEET", f"lookup failed: {exc} (continuing without sheet row)")
            gs_info = None

    info_link = mission_link_for_codename(codename)
    _log(mission_id, "LINK", f"info_link={info_link}")

    _log(mission_id, "STATS", "load optional statistics JSON (if present)")
    stats = load_stats_optional(mission_id)
    _log(mission_id, "STATS", "loaded JSON stats" if stats else "no JSON stats found")

    _log(mission_id, "BAG", f"parse topic='{VALID_TOPIC}' (min_msgs={MIN_GPS_MSGS})")
    try:
        gpx, points, std_pct, n_msgs, cov_valid = generate_gpx_and_cov_stats(str(bag_path), min_msgs=MIN_GPS_MSGS)
    except Exception as exc:
        note_list.append(mission_id)
        _log_orange(mission_id, "BAG", f"failed to read/parse bag: {exc}")
        return None

    _log(mission_id, "BAG", f"message_count={n_msgs}, parsed_points={len(points)}, cov_valid={cov_valid}")

    if n_msgs < MIN_GPS_MSGS:
        note_list.append(mission_id)
        _log_orange(mission_id, "QUALITY", f"insufficient messages: {n_msgs} < MIN_GPS_MSGS({MIN_GPS_MSGS})")
        return None
    if len(points) < MIN_GPS_POINTS:
        note_list.append(mission_id)
        _log_orange(mission_id, "QUALITY", f"insufficient points: {len(points)} < MIN_GPS_POINTS({MIN_GPS_POINTS})")
        return None

    _log(mission_id, "STATS", f"compute std-dev % from covariance (mode={COV_STDDEV_MODE})")
    if cov_valid < MIN_COV_SAMPLES or not std_pct:
        note_list.append(mission_id)
        _log_orange(
            mission_id, "STATS", f"insufficient covariance samples: {cov_valid} < MIN_COV_SAMPLES({MIN_COV_SAMPLES})"
        )
        for lbl in STD_LABELS:
            stats[lbl] = "N/A"
    else:
        for lbl in STD_LABELS:
            stats[lbl] = std_pct.get(lbl, "0.0 %")
        _log(mission_id, "STATS", "std-dev bucket percentages computed")

    for lbl in Q_LABELS:
        stats.setdefault(lbl, "N/A")

    if sheet_update and gs_info is not None:
        _log(mission_id, "SHEET", "write stats into row (Q + STD buckets)")
        try:
            write_stats_to_sheet(gs_info, stats)
            _log(mission_id, "SHEET", "stats written")
        except Exception as exc:
            _log_orange(mission_id, "SHEET", f"stats update failed: {exc}")

    _log(
        mission_id, "GEO", f"reverse geocode (enabled={ENABLE_REVERSE_GEOCODING}, timeout={REVERSE_GEOCODE_TIMEOUT_S}s)"
    )
    city = get_city_name(*points[0][:2])
    end_text = get_end_address(*points[-1][:2])
    _log(mission_id, "GEO", f"city='{city}'")
    _log(mission_id, "GEO", f"end_address='{end_text}'")

    _log(mission_id, "HTML", "build stats popup HTML")
    stats_html = build_stats_html(stats)

    _log(mission_id, "HTML", "build elevation plot")
    step = max(1, len(points) // 200)
    times = [p[3] for p in points[::step]]
    elevs = [p[2] for p in points[::step]]
    encoded_plot = create_elevation_plot(times, elevs)
    _log(mission_id, "HTML", f"elevation_plot={'yes' if bool(encoded_plot) else 'no'}")

    _log(mission_id, "MAP", "build folium map (no network calls inside)")
    try:
        fmap = build_folium_map(
            points=points,
            mission_name=mission_id,
            city=city,
            end_text=end_text,
            stats_html=stats_html,
            encoded_plot=encoded_plot,
            codename=codename,
            info_link=info_link,
        )
        _log(mission_id, "MAP", "folium map built")
    except Exception as exc:
        note_list.append(mission_id)
        _log_orange(mission_id, "MAP", f"failed to build folium map: {exc}")
        return None

    html_file = out_dir / f"{mission_id}_gps_map_{MODE}.html"
    _log(mission_id, "OUTPUT", f"write HTML -> {html_file}")
    try:
        # Render both artifacts before publishing either one. Each file is
        # published atomically and existing reports are preserved.
        html = fmap.get_root().render()
        gpx_xml = gpx.to_xml()
        gpx_path = out_dir / f"{mission_id}_{MODE}.gpx"
        if html_file.exists() or gpx_path.exists():
            raise FileExistsError(f"Report already exists for {mission_id} in {out_dir}")
        with atomic_output(gpx_path, overwrite=False) as pending:
            pending.write_text(gpx_xml, encoding="utf-8")
        try:
            with atomic_output(html_file, overwrite=False) as pending:
                pending.write_text(html, encoding="utf-8")
        except Exception:
            gpx_path.unlink()  # This invocation created it; keep the pair consistent.
            raise
        _log(mission_id, "OUTPUT", "GPX and HTML written")
    except Exception as exc:
        note_list.append(mission_id)
        _log_orange(mission_id, "OUTPUT", f"failed to write HTML: {exc}")
        return None

    if open_html:
        _log(mission_id, "BROWSER", "open HTML in browser")
        try:
            webbrowser.open(f"file://{html_file.resolve()}")
            _log(mission_id, "BROWSER", "open requested")
        except Exception as exc:
            _log_orange(mission_id, "BROWSER", f"failed to open browser: {exc}")
    else:
        _log(mission_id, "BROWSER", "open suppressed")

    _log(mission_id, "DONE", f"success (html={html_file.name})")
    return html_file


# ─── MODES ──────────────────────────────────────────────────────────────────
def run_interactive() -> int:
    _log("INTERACTIVE", "START", "resolve local bag + mission id")
    bag_path = Path(get_bag(GPS_PATTERN, directory=MISSION_DATA))
    mission_id = find_and_extract_non_matching(MISSION_DATA, GPS_PATTERN)
    _log(mission_id, "START", f"local bag resolved: {bag_path}")

    notes: List[str] = []
    saved = process_one_mission(
        mission_id=mission_id,
        bag_path=bag_path,
        out_dir=HTML_OUT_DIR,
        open_html=OPEN_HTML_INTERACTIVE,
        note_list=notes,
        sheet_update=SHEET_UPDATE,
    )
    if saved is not None:
        _log(mission_id, "RESULT", f"saved: {saved}")
    else:
        _log_orange(mission_id, "RESULT", "not saved (see logs above)")

    return 0 if saved is not None else 1


def run_automation() -> int:
    _log("AUTOMATION", "START", "load uuid mapping + sheet data")
    uuid_mappings = get_uuid_mapping()
    _, sheet_missions = read_sheet_data(SPREADSHEET_ID)
    _log("AUTOMATION", "START", f"missions_in_mapping={len(uuid_mappings)} missions_in_sheet={len(sheet_missions)}")

    failures = 0
    successes = 0
    missing_tc: List[str] = []
    not_good_or_insufficient: List[str] = []

    HTML_OUT_DIR.mkdir(parents=True, exist_ok=True)
    _log("AUTOMATION", "OUTPUT", f"HTML_OUT_DIR={HTML_OUT_DIR}")

    for j, (mission_id, data) in enumerate(uuid_mappings.items(), start=1):
        if j <= SKIP_FIRST:
            _log(mission_id, "SKIP", f"skipping due to SKIP_FIRST ({j} <= {SKIP_FIRST})")
            continue

        _log(mission_id, "START", f"begin mission #{j}")

        _log(mission_id, "SHEET", "check mission exists in sheet mapping")
        if mission_id not in sheet_missions:
            not_good_or_insufficient.append(mission_id)
            _log_orange(mission_id, "SHEET", "mission not found in sheet (treated as insufficient)")
            continue

        good_flag = str(sheet_missions[mission_id].get("GOOD_MISSION", "")).upper()
        _log(mission_id, "SHEET", f"GOOD_MISSION={good_flag}")
        if good_flag != "TRUE":
            not_good_or_insufficient.append(mission_id)
            _log_orange(mission_id, "QUALITY", "GOOD_MISSION is not TRUE (treated as insufficient)")
            continue

        uuid_release = data["uuid_release"]
        _log(mission_id, "REMOTE", f"uuid_release={uuid_release}")

        _log(mission_id, "REMOTE", f"select best bag pattern='{GPS_PATTERN}', min_mib={MIN_REMOTE_BAG_MIB}")
        remote_tc_name = select_best_remote_bag(uuid_release, patterns=[GPS_PATTERN], min_mib=MIN_REMOTE_BAG_MIB)
        if remote_tc_name is None:
            failures += 1
            missing_tc.append(mission_id)
            _log_orange(mission_id, "REMOTE", f"no remote _{MODE} bag available (treated as missing)")
            continue
        _log(mission_id, "REMOTE", f"selected bag='{remote_tc_name}'")

        TMP_BASE.mkdir(parents=True, exist_ok=True)
        tmp_folder = Path(tempfile.mkdtemp(prefix="box_auto_download_", dir=TMP_BASE))
        _log(mission_id, "DOWNLOAD", f"prepare tmp dir: {tmp_folder}")
        try:
            tmp_folder.mkdir(parents=True, exist_ok=True)

            _log(mission_id, "DOWNLOAD", f"download '{remote_tc_name}' -> {tmp_folder}")
            download_remote_files(uuid_release, [remote_tc_name], tmp_folder)
            _log(mission_id, "DOWNLOAD", "download complete")

            _log(mission_id, "LOCAL", "resolve downloaded bag path")
            local_tc = find_local_file(remote_tc_name, tmp_folder)
            _log(mission_id, "LOCAL", f"resolved bag path: {local_tc}")

            _log(mission_id, "PIPELINE", "generate GPX + HTML")
            saved = process_one_mission(
                mission_id=mission_id,
                bag_path=local_tc,
                out_dir=HTML_OUT_DIR,
                open_html=False,  # automation: never open
                note_list=not_good_or_insufficient,
                sheet_update=SHEET_UPDATE,
            )
            if saved is not None:
                successes += 1
                _log(mission_id, "RESULT", f"saved: {saved}")
            else:
                failures += 1
                _log_orange(mission_id, "RESULT", "not saved (treated as insufficient)")

        except Exception as exc:
            failures += 1
            not_good_or_insufficient.append(mission_id)
            _log_orange(mission_id, "EXCEPTION", f"{exc}")
        finally:
            _log(mission_id, "CLEANUP", f"remove tmp dir: {tmp_folder}")
            shutil.rmtree(tmp_folder, ignore_errors=True)
            _log(mission_id, "CLEANUP", "done")

    _log("AUTOMATION", "SUMMARY", "complete")

    if missing_tc:
        _log_orange("AUTOMATION", "SUMMARY", f"MISSING _{MODE} BAG missions: {len(missing_tc)}")
        for m in missing_tc:
            _log_orange(m, "SUMMARY", "missing remote _tc bag")
    else:
        _log("AUTOMATION", "SUMMARY", f"MISSING _{MODE} BAG missions: 0")

    uniq_insufficient = sorted(set(not_good_or_insufficient))
    if uniq_insufficient:
        _log_orange("AUTOMATION", "SUMMARY", f"INSUFFICIENT/NOTES missions: {len(uniq_insufficient)}")
        for m in uniq_insufficient:
            _log_orange(m, "SUMMARY", "insufficient/noted (see per-mission logs)")
    else:
        _log("AUTOMATION", "SUMMARY", "INSUFFICIENT/NOTES missions: 0")

    return 0 if successes and not failures else 1


def main(argv=None):
    import argparse

    global MISSION_DATA, HTML_OUT_DIR, SHEET_UPDATE, OPEN_HTML_INTERACTIVE, ENABLE_REVERSE_GEOCODING
    parser = argparse.ArgumentParser(description="Generate GNSS mission maps")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--mission-data", type=Path)
    mode.add_argument("--batch", action="store_true", help="Download and process remote missions")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--update-sheet", action="store_true")
    parser.add_argument("--open-browser", action="store_true")
    parser.add_argument("--reverse-geocode", action="store_true", help="Look up places online")
    args = parser.parse_args(argv)
    if args.mission_data:
        MISSION_DATA = str(args.mission_data.resolve())
    HTML_OUT_DIR = args.output_dir.resolve()
    SHEET_UPDATE = args.update_sheet
    OPEN_HTML_INTERACTIVE = args.open_browser
    ENABLE_REVERSE_GEOCODING = args.reverse_geocode
    if args.batch:
        return run_automation()
    else:
        return run_interactive()


if __name__ == "__main__":
    raise SystemExit(main())
