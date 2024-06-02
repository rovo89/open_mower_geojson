#!/usr/bin/env python3
import argparse
import geojson
import os
from pathlib import Path
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
import utm

parser = argparse.ArgumentParser(
  prog='bag_to_geojson.py',
  description='Converts map.bag to GeoJSON',
)
parser.add_argument('input', help='map.bag filename')
parser.add_argument('output', help='GeoJSON filename')
parser.add_argument('-x', '--datum-lat', type=float, required='OM_DATUM_LAT' not in os.environ, help='Datum Latitude')
parser.add_argument('-y', '--datum-long', type=float, required='OM_DATUM_LONG' not in os.environ, help='Datum Longitude')
args = parser.parse_args()

geojson.geometry.DEFAULT_PRECISION = 15

datum_lat = args.datum_lat if args.datum_lat is not None else float(os.environ['OM_DATUM_LAT'])
datum_lon = args.datum_long if args.datum_long is not None else float(os.environ['OM_DATUM_LONG'])
(ORIGIN_X, ORIGIN_Y, ORIGIN_ZONE_NUMBER, ORIGIN_ZONE_LETTER) = utm.from_latlon(datum_lat, datum_lon)

AREA_PROPS = {
  'mowing_areas': {'type': 'lawn', 'fill': '#00ff00'},
  'navigation_areas': {'type': 'navigation', 'fill': '#ffffff'},
  'obstacle': {'type': 'obstacle', 'fill': '#ff0000'},
}

def pos_to_lonlat(point):
  (lat, lon) = utm.to_latlon(ORIGIN_X + point.x, ORIGIN_Y + point.y, ORIGIN_ZONE_NUMBER, ORIGIN_ZONE_LETTER)
  return (lon, lat)

def area_to_geojson_feature(polygon, type):
  geometry = geojson.Polygon([[pos_to_lonlat(point) for point in polygon.points]])
  return geojson.Feature(geometry=geometry, properties=AREA_PROPS[type])

typestore = get_typestore(Stores.ROS1_NOETIC)
typestore.register(get_types_from_msg(Path('MapArea.msg').read_text(), 'mower_map/msg/MapArea'))

features = []
with Reader(args.input) as reader:
  for connection, timestamp, rawdata in reader.messages():
    if connection.topic == 'mowing_areas' or connection.topic == 'navigation_areas':
      msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
      features.append(area_to_geojson_feature(msg.area, connection.topic))
      for obstacle in msg.obstacles:
        features.append(area_to_geojson_feature(obstacle, 'obstacle'))
    elif connection.topic == 'docking_point':
      # TODO
      pass

with open(args.output, 'w') as f:
  geojson.dump(geojson.FeatureCollection(features), f)
