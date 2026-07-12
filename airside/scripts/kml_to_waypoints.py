#!/usr/bin/env python3
"""
Convert a KML file (exported from Google My Maps) into the engine's
waypoints.yaml config format.

The placemark named "Home" becomes the `home` entry. Waypoints are taken from
the first Polygon's outer ring (dropping the repeated closing vertex). If the
KML has no polygon, all other Point placemarks are used in document order.
"""

import argparse
import sys
import xml.etree.ElementTree as ET

KML_NS = {"kml": "http://www.opengis.net/kml/2.2"}

HEADER = """\
# Lap waypoints. `home` orients the waypoints
# and is the returning location at the end
# of the mission.\
"""


def parse_coordinates(text):
    """Parse a KML <coordinates> block into a list of (lat, lon) tuples."""
    points = []
    for token in text.split():
        parts = token.split(",")
        lon, lat = float(parts[0]), float(parts[1])
        points.append((lat, lon))
    return points


def extract(root):
    """Return (home, waypoints) as (lat, lon) tuples from the KML root."""
    home = None
    polygon_points = None
    point_placemarks = []

    for placemark in root.iter("{http://www.opengis.net/kml/2.2}Placemark"):
        name_el = placemark.find("kml:name", KML_NS)
        name = (name_el.text or "").strip() if name_el is not None else ""

        point_coords = placemark.find("kml:Point/kml:coordinates", KML_NS)
        if point_coords is not None:
            point = parse_coordinates(point_coords.text)[0]
            if name.lower() == "home":
                home = point
            else:
                point_placemarks.append(point)
            continue

        ring_coords = placemark.find(
            "kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates",
            KML_NS,
        )
        if ring_coords is not None and polygon_points is None:
            ring = parse_coordinates(ring_coords.text)
            if len(ring) > 1 and ring[0] == ring[-1]:
                ring = ring[:-1]
            polygon_points = ring

    waypoints = polygon_points if polygon_points is not None else point_placemarks
    return home, waypoints


def format_yaml(home, waypoints, alt):
    lines = [HEADER]
    lines.append("home:")
    lines.append(f"  lat: {home[0]}")
    lines.append(f"  lon: {home[1]}")
    lines.append(f"  alt: {alt}")
    lines.append("")
    lines.append("waypoints:")
    for lat, lon in waypoints:
        lines.append(f"  - lat: {lat}")
        lines.append(f"    lon: {lon}")
        lines.append(f"    alt: {alt}")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(
        description="Convert a KML file to the engine waypoints.yaml format."
    )
    parser.add_argument("kml", help="input KML file")
    parser.add_argument(
        "-o", "--output", help="output YAML file (default: print to stdout)"
    )
    parser.add_argument(
        "--alt", type=float, default=10, help="altitude for all points (default: 10)"
    )
    args = parser.parse_args()

    root = ET.parse(args.kml).getroot()
    home, waypoints = extract(root)

    if home is None:
        sys.exit('error: no Point placemark named "Home" found in KML')
    if not waypoints:
        sys.exit("error: no waypoints found in KML (no polygon or point placemarks)")

    alt = int(args.alt) if args.alt == int(args.alt) else args.alt
    yaml_text = format_yaml(home, waypoints, alt)

    if args.output:
        with open(args.output, "w") as f:
            f.write(yaml_text)
        print(f"wrote {len(waypoints)} waypoints + home to {args.output}")
    else:
        sys.stdout.write(yaml_text)


if __name__ == "__main__":
    main()
