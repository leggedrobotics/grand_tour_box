import rosbag
import base64
import io
import matplotlib.pyplot as plt
import gpxpy
from pathlib import Path
import folium
from folium.plugins import HeatMap
import numpy as np
import webbrowser
from box_auto.utils import get_bag, MISSION_DATA, WS, find_and_extract_non_matching
from datetime import datetime
from geopy.geocoders import Nominatim
from geopy.extra.rate_limiter import RateLimiter


def get_city_name(lat, lon):
    try:
        # Perform the reverse geocoding
        location = reverse((lat, lon), exactly_one=True)
        if location and "address" in location.raw:
            address = location.raw["address"]
            city = address.get("city", address.get("town", address.get("village", "Unknown")))
        else:
            city = "Unknown"
    except Exception as e:
        city = f"Error: {str(e)}"
    return city


def ecef_to_lla(x, y, z):
    # WGS84 ellipsoid constants
    a = 6378137.0  # Semi-major axis (meters)
    e2 = 6.69437999014e-3  # Square of first eccentricity
    f = 1 / 298.257223563  # Flattening

    # Compute longitude
    lon = np.arctan2(y, x)

    # Iterative computation for latitude and altitude
    b = a * (1 - f)
    ep2 = (a**2 - b**2) / b**2
    p = np.sqrt(x**2 + y**2)
    theta = np.arctan2(z * a, p * b)

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    lat = np.arctan2(z + ep2 * b * sin_theta**3, p - e2 * a * cos_theta**3)

    # Compute altitude
    N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - N

    # Convert radians to degrees
    lat = np.degrees(lat)
    lon = np.degrees(lon)

    return lat, lon, alt


def basemap_lyrs():
    # ESRI World Imagery
    esri_imagery_url = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    esri_attribution = '<a href="https://rnli.org/">&copy; RNLI</a> | <a href="https://www.esri.com">&copy; ESRI</a>'

    # CartoDB Dark
    cartodb_dark_url = "http://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png"
    cartodb_attribution = (
        '<a href="https://rnli.org/">&copy; RNLI</a> | <a href="https://www.carto.com">&copy; Carto</a>'
    )

    # OpenStreetMap
    osm_url = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
    osm_attribution = (
        '<a href="https://rnli.org/">&copy; RNLI</a> | <a href="https://www.openstreetmap.org">&copy; OSM</a>'
    )

    # Google Maps layers
    google_hybrid_url = "https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}"
    google_roads_url = "https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}"
    google_satellite_url = "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"
    google_attribution = (
        '<a href="https://rnli.org/">&copy; RNLI</a> | <a href="https://www.google.com">&copy; Google</a>'
    )

    return (
        esri_imagery_url,
        esri_attribution,
        cartodb_dark_url,
        cartodb_attribution,
        osm_url,
        osm_attribution,
        google_hybrid_url,
        google_roads_url,
        google_satellite_url,
        google_attribution,
    )


if __name__ == "__main__":

    mode = "dgps"
    # valid_topics = f"/gt_box/inertial_explorer/{mode}/raw/position_ecef"
    valid_topics = f"/gt_box/inertial_explorer/{mode}/navsatfix"

    GPS_PATTERN = f"*_cpt7_ie_{mode}.bag"
    bag_path = get_bag(GPS_PATTERN)

    time_as_string = find_and_extract_non_matching(MISSION_DATA, GPS_PATTERN)

    gpx = gpxpy.gpx.GPX()
    # Create first track in our GPX:
    gpx_track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(gpx_track)

    # Create first segment in our GPX track:
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)
    # Create points

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, ts in bag.read_messages(topics=[valid_topics]):

            if msg._type == "sensor_msgs/NavSatFix":
                # Convert lat, lon, alt to degrees
                lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
            else:
                # Convert ECEF to lat, lon, alt
                lat, lon, alt = ecef_to_lla(msg.x, msg.y, msg.z)

            # convert msg.header to datetime
            timestamp = datetime.fromtimestamp(ts.secs + ts.nsecs * 1e-9)

            # Add points to the GPS path
            gpx_segment.points.append(
                gpxpy.gpx.GPXTrackPoint(latitude=lat, longitude=lon, elevation=alt, time=timestamp)
            )
    out = bag_path.replace(".bag", ".gpx")
    with open(out, "w") as f:
        f.write(gpx.to_xml())

    # Initialize the geolocator
    geolocator = Nominatim(user_agent="gpx_viewer")

    # Use RateLimiter to avoid hitting the service rate limits
    reverse = RateLimiter(geolocator.reverse, min_delay_seconds=1)

    # TODO check if RSL and ETHz logos are permitted.
    # Load project icon
    image_path = Path(WS) / "src" / "grand_tour_box" / "box_documentation" / "images" / "icon.png"
    if not image_path.exists():
        raise FileNotFoundError(f"Image not found at: {image_path}")

    with open(image_path, "rb") as img_file:
        encoded_image = base64.b64encode(img_file.read()).decode("utf-8")
    data_url = f"data:image/png;base64,{encoded_image}"

    # Get 2D points from the GPX file
    points = [
        (point.latitude, point.longitude, point.elevation, point.time)
        for track in gpx.tracks
        for segment in track.segments
        for point in segment.points
    ]

    if not points:
        raise ValueError("No points found in the GPX file.")

    city_name = get_city_name(points[0][0], points[0][1])

    # Create a folium map centered on the first point's latitude and longitude
    m = folium.Map(location=(points[0][0], points[0][1]), zoom_start=18, max_zoom=24, zoomSnap=0, zoomDelta=0.25)

    # Get basemap layers
    (
        esri_imagery_url,
        esri_attribution,
        cartodb_dark_url,
        cartodb_attribution,
        osm_url,
        osm_attribution,
        google_hybrid_url,
        google_roads_url,
        google_satellite_url,
        google_attribution,
    ) = basemap_lyrs()

    # Draw the GPX track using a PolyLine (using only lat and lon)
    track_coords = [(lat, lon) for lat, lon, elevation, point_time in points]
    folium.PolyLine(track_coords, color="blue", weight=3).add_to(m)

    # Optionally add sparse markers to avoid performance issues with too many markers
    marker_interval = max(1, len(points) // 200)
    first_point = True

    mission_name = f"Mission ID: {str(time_as_string)}"
    place_name = city_name
    info_link = "https://grand-tour.leggedrobotics.com/"

    times = []
    elevations = []
    for lat, lon, elevation, point_time in points[::marker_interval]:
        # Add a small red circle marker
        folium.CircleMarker(location=(lat, lon), radius=2, color="red", fill=True).add_to(m)
        times.append(point_time)
        elevations.append(elevation)

        if first_point:
            # Create a marker with a popup that displays the time (if available)
            elev_text = f"{elevation:.2f} m" if elevation is not None else "N/A"
            popup_html = f"""
            <div style="font-family: Roboto; font-size: 14px;">
                <strong>Mission:</strong> {mission_name}<br>
                <strong>Place:</strong> {place_name}<br>
                <strong>Date:</strong> {point_time if point_time else 'No date info'}<br>
                <strong>Elevation:</strong> {elev_text}<br>
                <a href="{info_link}" target="_blank">More Info</a>
            </div>
            """

            # Create the popup object and add the marker
            popup = folium.Popup(popup_html, max_width=300)

            folium.Marker(
                location=[lat, lon],
                popup=popup,
                tooltip="Mission info",
                show=True,
            ).add_to(m)
            first_point = False

    start_time = times[0]
    relative_seconds = [(t - start_time).total_seconds() for t in times]

    fig, ax = plt.subplots(figsize=(8, 6))  # adjust size as needed
    ax.plot(relative_seconds, elevations, marker="o", linestyle="-")
    ax.set_xlabel("Time")
    ax.set_ylabel("Elevation (m)")
    ax.set_title("Elevation Change")
    fig.autofmt_xdate()  # formats dates nicely on the x-axis

    # Save the plot to a bytes buffer
    buf = io.BytesIO()
    plt.savefig(buf, format="png", bbox_inches="tight", transparent=True)
    buf.seek(0)
    encoded_plot = base64.b64encode(buf.read()).decode("utf-8")
    buf.close()
    plt.close(fig)  # close the figure

    # Add a HeatMap layer using only lat and lon coordinates
    hm_lyr = folium.FeatureGroup(name="Heat Map", overlay=True, control=True, show=True)
    hm_lyr.add_to(m)
    HeatMap(track_coords).add_to(hm_lyr)

    # Add basemap tile layers with increased maximum zoom
    folium.TileLayer(
        tiles=esri_imagery_url, attr=esri_attribution, name="ESRI World Imagery", control=True, max_zoom=24, show=False
    ).add_to(m)

    folium.TileLayer(
        tiles=cartodb_dark_url,
        attr=cartodb_attribution,
        name="CartoDB Dark Imagery",
        control=True,
        max_zoom=24,
        show=False,
    ).add_to(m)

    folium.TileLayer(
        tiles=osm_url, attr=osm_attribution, name="Open Street Map", control=True, max_zoom=24, show=False
    ).add_to(m)

    folium.TileLayer(
        tiles=google_hybrid_url, attr=google_attribution, name="Google Hybrid", control=True, max_zoom=24, show=True
    ).add_to(m)

    folium.TileLayer(
        tiles=google_roads_url, attr=google_attribution, name="Google Road", control=True, max_zoom=24, show=False
    ).add_to(m)

    folium.TileLayer(
        tiles=google_satellite_url,
        attr=google_attribution,
        name="Google Satellite",
        control=True,
        max_zoom=24,
        show=False,
    ).add_to(m)

    folium.LayerControl(position="topright", collapsed=False, overlay=True, draggable=True).add_to(m)

    custom_css = """
    <style>
        .leaflet-control-layers {
            font-family: 'Roboto';
            font-size: 20px;
        }
    </style>
    """
    m.get_root().html.add_child(folium.Element(custom_css))

    dynamic_title = "GrandTour Mission: " + str(time_as_string) + " | " + str(city_name)

    # Create a HTML element with your title # noqa
    title_html = f"""<div style="position: fixed;
                top: 10px; left: 50%;
                transform: translate(-50%, 0);
                z-index: 9999;
                background-color: rgba(255, 255, 255, 0.9);
                padding: 10px;
                border: 2px solid black;">
        <h2>{dynamic_title}</h2>
    </div>"""
    m.get_root().html.add_child(folium.Element(title_html))  # noqa

    # noqa
    image_html = f"""
    <div style="
        position: fixed;
        bottom: 10px;
        left: 10px;
        z-index: 9999;
        background-color: rgba(255, 255, 255, 0.8);
        padding: 5px;
        border: 1px solid grey;
    ">
        <img src="{data_url}" style="width: 100px; height: auto;">
    </div>
    """

    # Add the HTML element to the map
    m.get_root().html.add_child(folium.Element(image_html))  # noqa

    plot_html = f"""
    <div style="
        position: fixed;
        bottom: 10px;
        right: 10px;
        z-index: 9999;
        background-color: rgba(255, 255, 255, 0.9);
        padding: 5px;
        border: 1px solid grey;
    ">
        <img src="data:image/png;base64,{encoded_plot}" style="width:600px; height:auto;">
    </div>
    """

    # Inject the plot HTML into the map's HTML
    m.get_root().html.add_child(folium.Element(plot_html))

    # Save the interactive map to an HTML file and open it as a clickable URL
    html_filename = f"{MISSION_DATA}{str(time_as_string)}_gps_map_{mode}.html"
    m.save(html_filename)

    # Get absolute file path and open in the default web browser
    webbrowser.open(f"file://{html_filename}")
