#!/usr/bin/env python3
# ─── IMPORTS ────────────────────────────────────────────────────────────────
import base64
import io
import json
import webbrowser
from pathlib import Path
from datetime import datetime
from typing import List, Tuple

import folium
import gpxpy
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import matplotlib.ticker as mticker
import numpy as np
import rosbag
import gspread
from folium.plugins import (
    MiniMap,
    MeasureControl,
    HeatMap,
    Fullscreen,
    # TimestampedGeoJson,  # kept for feature parity (not used by default)
    # MarkerCluster,  # kept for feature parity (not used by default)
    # Search,  # kept for feature parity (not used by default)
    # LocateControl,  # kept for feature parity (not used by default)
    # FloatImage,  # kept for feature parity (not used by default)
    # DualMap,  # kept for feature parity (not used by default)
)
from geopy.geocoders import Nominatim
from geopy.extra.rate_limiter import RateLimiter

from box_auto.utils import (
    get_bag,
    MISSION_DATA,
    BOX_AUTO_DIR,
    WS,
    find_and_extract_non_matching,
)

# ─── CONSTANTS ──────────────────────────────────────────────────────────────
MODE = "tc"
VALID_TOPIC = f"/gt_box/inertial_explorer/{MODE}/navsatfix"
GPS_PATTERN = f"*_cpt7_ie_{MODE}.bag"
INFO_LINK = "https://grand-tour.leggedrobotics.com/"

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

# Shared geopy objects (rate‑limited)
_GEOL = Nominatim(user_agent="gpx_viewer")
_REVERSE = RateLimiter(_GEOL.reverse, min_delay_seconds=1)

# ─── BASIC HELPERS ──────────────────────────────────────────────────────────


def pick_color(idx: int, total: int) -> str:
    """Color helper mimicking the original traffic‑light logic."""
    if idx < 2:
        return "darkgreen"
    if idx == 2:
        return "#DAA520"  # golden‑rod for the middle bucket
    if idx >= total - 2:
        return "red"
    return "inherit"


def ecef_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert ECEF coordinates → lat/long/alt (WGS‑84)."""
    a = 6378137.0  # semi‑major axis
    f = 1 / 298.257223563
    e2 = 6.69437999014e-3  # first eccentricity squared

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


# ─── BASEMAP LAYERS ─────────────────────────────────────────────────────────


def basemap_layers() -> Tuple[str, str, str, str, str, str, str, str, str, str]:
    """Return tile URLs + attributions for all basemap layers."""
    # URLs
    esri = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    carto = "http://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png"
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


def read_gspread_row(date_str: str):
    """Return (good_mission, codename, row_index, worksheet) for *date_str*."""
    gc = gspread.service_account(
        filename=Path(BOX_AUTO_DIR) / "../.." / ".secrets/halogen-oxide-451108-u4-67f470bcc02e.json",
    )
    sheet = gc.open_by_key("1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg")
    ws = sheet.worksheet("gnss_quality_results")

    data = ws.get_all_values("A1:C75")
    try:
        row_idx = next(i for i, r in enumerate(data) if r[0] == date_str) + 1
    except StopIteration:
        raise ValueError(f"No row found with mission_id '{date_str}'")

    headers = data[0]
    col = lambda name: headers.index(name)
    row = data[row_idx - 1]

    good_mission = row[col("good_mission")].strip().upper() == "TRUE"
    codename = row[col("codename")]

    return good_mission, codename, row_idx, ws


def write_stats_to_sheet(gs: dict, stats: dict) -> None:
    """Write *stats* dict to the Google Sheet row described by *gs*."""
    ws = gs["worksheet"]
    row_idx = gs["row_index"]
    headers = ws.row_values(1)

    def col(name: str) -> int:
        return headers.index(name) + 1

    cells = [gspread.Cell(row_idx, col(lbl.replace(" ", "")), stats[lbl]) for lbl in Q_LABELS] + [
        gspread.Cell(row_idx, col(STD_MAP[lbl]), stats[lbl]) for lbl in STD_LABELS
    ]

    ws.update_cells(cells)


# ─── STATS / GPX BUILDERS ───────────────────────────────────────────────────


def load_stats(date_str: str) -> dict:
    """Load statistics from JSON file or return placeholder if not found."""
    stats_file = Path(MISSION_DATA) / f"{date_str}_cpt7_ie_{MODE}_statistics.json"
    try:
        with open(stats_file, "r") as fh:
            return json.load(fh)
    except FileNotFoundError:
        print(f"Warning: Stats file not found: {stats_file}")
        # Create placeholder stats
        fake_stats = {"FILE_NOT_FOUND": "true"}
        # Add placeholders for all required labels
        for label in Q_LABELS:
            fake_stats[label] = "N/A"
        for label in STD_LABELS:
            fake_stats[label] = "N/A"
        return fake_stats


def generate_gpx(bag_path: str) -> Tuple[gpxpy.gpx.GPX, List[Tuple]]:
    """Parse *bag_path* and return (gpx_obj, list_of_points)."""
    gpx = gpxpy.gpx.GPX()
    seg = gpxpy.gpx.GPXTrackSegment()
    track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(track)
    track.segments.append(seg)

    points = []
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, ts in bag.read_messages(topics=[VALID_TOPIC]):
            if msg._type == "sensor_msgs/NavSatFix":
                lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
            else:
                lat, lon, alt = ecef_to_lla(msg.x, msg.y, msg.z)

            timestamp = datetime.fromtimestamp(ts.secs + ts.nsecs * 1e-9)
            seg.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon, alt, timestamp))
            points.append((lat, lon, alt, timestamp))
    return gpx, points


def build_stats_html(stats: dict) -> str:
    """Return an HTML snippet for the folium popup."""

    def make_rows(labels):
        return "\n".join(
            f"<tr><td style='padding:2px;color:{pick_color(i,len(labels))}'>{lbl}</td>"
            f"<td style='padding:2px'>{stats[lbl]}</td></tr>"
            for i, lbl in enumerate(labels)
        )

    return (
        # "<hr style='margin:6px 0;'/>"
        # "<strong style='font-size:13px;display:block;'>Quality Number %</strong>"
        # "<table style='font-size:12px;width:100%;border-collapse:collapse;'>"
        # f"{make_rows(Q_LABELS)}</table>"
        "<strong style='font-size:13px;display:block;margin-top:4px;'>Std‑Dev Range %</strong>"
        "<table style='font-size:12px;width:100%;border-collapse:collapse;'>"
        f"{make_rows(STD_LABELS)}</table>"
    )


# ─── VISUAL BUILDERS ────────────────────────────────────────────────────────


def create_elevation_plot(times: List[datetime], elevs: List[float]) -> str:
    """Return base-64 PNG of the elevation graph, styled per Datawrapper guidelines."""
    # compute relative seconds
    rel = [(t - times[0]).total_seconds() for t in times]

    # find & register Roboto
    roboto_path = fm.findfont("Roboto", fallback_to_default=True)
    roboto = fm.FontProperties(fname=roboto_path)

    # make figure: narrower width, small height
    fig, ax = plt.subplots(figsize=(20, 2))

    fig.patch.set_edgecolor("black")
    fig.patch.set_linewidth(5.0)

    for spine in ax.spines.values():
        spine.set_edgecolor("black")
        spine.set_linewidth(5.0)

    # plot line + markers in a clean blue
    ax.plot(
        rel,
        elevs,
        marker="o",
        markersize=4,
        linewidth=1,
        color="#0099D8",  # Datawrapper Blue
        markerfacecolor="#FFFFFF",
        markeredgecolor="#0099D8",
    )

    # only horizontal gridlines, very light
    ax.yaxis.grid(True, color="#E1E5EA", linewidth=0.8)
    ax.xaxis.grid(False)

    # remove top/right spines; slim remaining spines
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    for spine in ["bottom", "left"]:
        ax.spines[spine].set_color("#6B6B6B")
        ax.spines[spine].set_linewidth(0.8)

    # labels + title in Roboto bold
    ax.set_title(
        "Elevation Change",
        fontproperties=roboto,
        fontsize=14,
        fontweight="bold",
        pad=6,
    )
    # ax.set_xlabel(
    #     "Time (s)",
    #     fontproperties=roboto,
    #     fontsize=12,
    #     fontweight="bold",
    #     labelpad=4,
    # )

    ax.xaxis.set_major_formatter(mticker.FuncFormatter(lambda x, pos: f"{int(x)} s"))
    ax.set_ylabel(
        "Elevation (m)",
        fontproperties=roboto,
        fontsize=12,
        fontweight="bold",
        labelpad=4,
    )

    # ticks in Roboto, slightly smaller
    for lbl in ax.get_xticklabels() + ax.get_yticklabels():
        lbl.set_fontproperties(roboto)
        lbl.set_fontsize(10)

    # zero margins on x, small vertical margin
    ax.margins(x=0, y=0.02)

    # tighten everything up
    fig.tight_layout(pad=0)

    # save to PNG with no padding
    buf = io.BytesIO()
    plt.savefig(
        buf,
        format="png",
        bbox_inches="tight",
        pad_inches=0,
        transparent=True,
    )
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode("utf-8")


def get_city_name(lat: float, lon: float) -> str:
    """Reverse geocode → city / town."""
    try:
        loc = _REVERSE((lat, lon), exactly_one=True)
        addr = loc.raw.get("address", {}) if loc else {}
        return addr.get("city") or addr.get("town") or addr.get("village") or "Unknown"
    except Exception as exc:
        return f"Error: {exc}"


def build_folium_map(
    points: List[Tuple],
    mission_name: str,
    city: str,
    stats_html: str,
    encoded_plot: str,
    codename: str = "Unknown",
) -> folium.Map:
    """Create folium map with overlays and UI controls."""
    if not points:
        raise ValueError("No points to plot.")

    # Sort points by timestamp to ensure correct start/end order
    points.sort(key=lambda p: p[3])  # Sort by timestamp (4th element)

    first_lat, first_lon = points[0][:2]
    m = folium.Map(
        location=(first_lat, first_lon),
        zoom_start=18,
        max_zoom=24,
        zoomSnap=0,
        zoomDelta=0.25,
        tiles=None,
    )

    # Basemaps
    (
        esri,
        esri_attr,
        _,
        _,
        _,
        _,
        g_hyb,
        _,
        g_sat,
        g_attr,
    ) = basemap_layers()

    folium.TileLayer(esri, attr=esri_attr, name="ESRI World Imagery", max_zoom=24).add_to(m)
    # folium.TileLayer(carto, attr=carto_attr, name="CartoDB Dark", max_zoom=24).add_to(m)
    # folium.TileLayer(osm, attr=osm_attr, name="OpenStreetMap", max_zoom=24).add_to(m)
    # folium.TileLayer(g_rd, attr=g_attr, name="Google Road", max_zoom=24).add_to(m)
    folium.TileLayer(g_sat, attr=g_attr, name="Google Satellite", max_zoom=24).add_to(m)
    folium.TileLayer(g_hyb, attr=g_attr, name="Google Hybrid", max_zoom=24, show=True).add_to(m)

    # Track & heatmap
    coords = [(lat, lon) for lat, lon, *_ in points]
    folium.PolyLine(coords, color="blue", weight=3).add_to(m)
    HeatMap(coords).add_to(folium.FeatureGroup(name="Heat Map", show=True).add_to(m))

    # Popup at first point (start of mission)
    elev0 = f"{points[0][2]:.2f} m" if points[0][2] is not None else "N/A"
    popup_html = (
        "<div style='font-family:Roboto;font-size:14px;'>"
        f"<strong>Mission ID:</strong> {mission_name}<br>"
        f"<strong>Place:</strong> {city}<br>"
        f"<strong>Date:</strong> {points[0][3]}<br>"
        f"<strong>Elevation:</strong> {elev0}<br>"
        f"<a href='{INFO_LINK}' target='_blank'>More Info</a>"
        f"{stats_html}</div>"
    )

    folium.Marker(
        location=(first_lat, first_lon),
        popup=folium.Popup(popup_html, max_width=300),
        tooltip="Mission start",
        icon=folium.Icon(color="green"),
    ).add_to(m)

    # End marker
    end_lat, end_lon = points[-1][:2]
    end_loc = _REVERSE((end_lat, end_lon))
    folium.Marker(
        location=(end_lat, end_lon),
        popup=f"End: {end_loc.address}",
        tooltip="Mission end",
        icon=folium.Icon(color="red"),
    ).add_to(m)

    # Controls
    MiniMap(toggle_display=True, position="bottomleft").add_to(m)
    MeasureControl(position="topleft", primary_length_unit="meters").add_to(m)
    Fullscreen(position="topleft").add_to(m)
    folium.LayerControl(position="topright", collapsed=False, overlay=True, draggable=True).add_to(m)

    # Dynamic JS variable name for the map
    map_name = m.get_name()

    # Inject sidebar + styling + JS
    m.get_root().html.add_child(
        folium.Element(
            f"""
    <!-- Font Awesome for icons -->
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css"/>

    <!-- Leaflet Sidebar v2 CSS and JS -->
    <link rel="stylesheet" href="https://unpkg.com/leaflet-sidebar-v2/css/leaflet-sidebar.min.css"/>
    <script src="https://unpkg.com/leaflet-sidebar-v2/js/leaflet-sidebar.min.js"></script>

    <!-- Sidebar HTML -->
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
    /* Make LayerControl fit cleanly inside the sidebar */
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

    # # Simple CSS tweak
    # m.get_root().html.add_child(
    #     folium.Element("<style>.leaflet-control-layers{font-family:'Roboto';font-size:20px;}</style>")
    # )

    # Logos
    # icon_path = Path(WS) / "src/grand_tour_box/box_documentation/images/icon.png"
    rsl_logo_path = Path(WS) / "src/grand_tour_box/box_documentation/images/rsl_logo.webp"
    # icon_b64 = base64.b64encode(icon_path.read_bytes()).decode("utf-8")
    rsl_b64 = base64.b64encode(rsl_logo_path.read_bytes()).decode("utf-8")

    # logo_html = (
    #     "<div style='position:fixed;bottom:200px;left:10px;z-index:9999;"
    #     "background:rgba(255,255,255,0.8);padding:5px;border:1px solid grey;'>"
    #     f"<img src='data:image/png;base64,{icon_b64}' style='width:100px;height:auto;'>"
    #     "</div>"
    # )
    # m.get_root().html.add_child(folium.Element(logo_html))

    # Title banner with logo and text
    logo_and_text = (
        f'<a href="{INFO_LINK}" target="_blank" '
        'style="text-decoration:none; color:inherit;">'
        f'<img src="data:image/webp;base64,{rsl_b64}" '
        'style="width:50px; height:auto; vertical-align:middle; margin-right:8px;" />'
        '<span style="vertical-align:middle; font-weight:bold;">[The GrandTour Dataset]</span>'
        "</a>"
    )

    dynamic_title = (
        f"{logo_and_text} Mission: | "
        f'<a href="{INFO_LINK}" target="_blank" style="color:#1E90FF; text-decoration:underline;">{codename}</a> | '
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
        "    if (overlay) overlay.style.width = `${window.innerWidth * 0.8}px`;"  # Set width to 80% of innerWidth
        "  }"
        "  window.addEventListener('resize', updateOverlayWidth);"
        "  updateOverlayWidth();"  # Initial call to set width
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
                cursor:pointer;
            ">
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
        color:#333;
    ">
    <label for="elevSize" style="display:block; margin-bottom:4px; font-weight:bold;">
        Width:
    </label>
    <input type="range" id="elevSize" min="400" max="2500" value="2200"
            style="
            width: 100%;
            -webkit-appearance: none;
            height: 6px;
            border-radius: 3px;
            background: #ddd;
            outline: none;
            "
            oninput="
            document
                .getElementById('elevation-overlay')
                .style.width=this.value + 'px';
            "/>
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

    // Change button color based on state
    if (visible) {
        button.style.backgroundColor = '#ccc'; // OFF: gray
        button.style.color = '#333';
    } else {
        button.style.backgroundColor = '#1E90FF'; // ON: blue
        button.style.color = '#fff';
    }
    }
    </script>
    """
    m.get_root().html.add_child(folium.Element(toggle_js))

    return m


# ─── MAIN PIPELINE ──────────────────────────────────────────────────────────


def main() -> None:
    bag_path = get_bag(GPS_PATTERN)
    mission_id = find_and_extract_non_matching(MISSION_DATA, GPS_PATTERN)

    good, codename, row_idx, ws = read_gspread_row(mission_id)
    gs_info = {
        "good_mission": good,
        "codename": codename,
        "row_index": row_idx,
        "worksheet": ws,
    }

    if not good:
        print(f"Mission {mission_id} is not marked as good; skipping.")
        return

    stats = load_stats(mission_id)
    write_stats_to_sheet(gs_info, stats)

    gpx, points = generate_gpx(bag_path)

    # Save GPX next to bag
    gpx_path = Path(bag_path).with_suffix(".gpx")
    gpx_path.write_text(gpx.to_xml())

    city = get_city_name(*points[0][:2])
    stats_html = build_stats_html(stats)

    # Sparse subset for plot to avoid excessive markers
    step = max(1, len(points) // 200)
    times = [p[3] for p in points[::step]]
    elevs = [p[2] for p in points[::step]]
    encoded_plot = create_elevation_plot(times, elevs)

    fmap = build_folium_map(points, mission_id, city, stats_html, encoded_plot, codename)

    html_file = Path(MISSION_DATA) / f"{mission_id}_gps_map_{MODE}.html"
    fmap.save(str(html_file))
    webbrowser.open(f"file://{html_file.resolve()}")


if __name__ == "__main__":
    main()
