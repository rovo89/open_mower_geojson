#!/usr/bin/env python3
import argparse
import geojson
import math
import os
from pathlib import Path
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
import shapely
import time
import utm

parser = argparse.ArgumentParser(
  prog='geojson_to_bag.py',
  description='Converts GeoJSON to map.bag',
)
parser.add_argument('input', help='GeoJSON filename')
parser.add_argument('output', help='map.bag filename')
parser.add_argument('-x', '--datum-lat', type=float, required='OM_DATUM_LAT' not in os.environ, help='Datum Latitude')
parser.add_argument('-y', '--datum-long', type=float, required='OM_DATUM_LONG' not in os.environ, help='Datum Longitude')
args = parser.parse_args()

geojson.geometry.DEFAULT_PRECISION = 15

datum_lat = args.datum_lat if args.datum_lat is not None else float(os.environ['OM_DATUM_LAT'])
datum_lon = args.datum_long if args.datum_long is not None else float(os.environ['OM_DATUM_LONG'])
(ORIGIN_X, ORIGIN_Y, ORIGIN_ZONE_NUMBER, ORIGIN_ZONE_LETTER) = utm.from_latlon(datum_lat, datum_lon)

# Initialize ROS types.
typestore = get_typestore(Stores.ROS1_NOETIC)
typestore.register(get_types_from_msg(Path('MapArea.msg').read_text(), 'mower_map/msg/MapArea'))
ROS_MapArea = typestore.types['mower_map/msg/MapArea']
ROS_Polygon = typestore.types['geometry_msgs/msg/Polygon']
ROS_Point32 = typestore.types['geometry_msgs/msg/Point32']
ROS_Pose = typestore.types['geometry_msgs/msg/Pose']
ROS_Point = typestore.types['geometry_msgs/msg/Point']
ROS_Quaternion = typestore.types['geometry_msgs/msg/Quaternion']

def lonlat_to_pos(point):
  (x, y, *_) = utm.from_latlon(point[1], point[0], ORIGIN_ZONE_NUMBER, ORIGIN_ZONE_LETTER)
  return (x - ORIGIN_X, y - ORIGIN_Y)

def area_to_shapely_polygon(feature):
  return shapely.Polygon(lonlat_to_pos(point) for point in feature.geometry.coordinates[0])

def polygon_shapely_to_ros(polygon):
  return ROS_Polygon(points=[ROS_Point32(x=p[0], y=p[1], z=0.0) for p in polygon.exterior.coords])

with open(args.input, 'r') as f:
  collection = geojson.load(f)

Path(args.output).unlink(missing_ok=True)
with Writer(args.output) as writer:
  # Add connections.
  mow_conn = writer.add_connection('mowing_areas', ROS_MapArea.__msgtype__, typestore=typestore)
  nav_conn = writer.add_connection('navigation_areas', ROS_MapArea.__msgtype__, typestore=typestore)
  dock_conn = writer.add_connection('docking_point', ROS_Pose.__msgtype__, typestore=typestore)

  # Detect and prepare the features.
  obstacles = []
  areas = []
  for feature in collection.features:
    if feature.properties['type'] == 'obstacle':
      obstacles.append(area_to_shapely_polygon(feature))
    elif feature.properties['type'] in ['lawn', 'navigation']:
      areas.append(feature)
    elif feature.properties['type'] == 'docking_station':
      pos = lonlat_to_pos(feature.geometry.coordinates[1])
      approach_pos = lonlat_to_pos(feature.geometry.coordinates[0])
      angle = math.atan2(pos[1] - approach_pos[1], pos[0] - approach_pos[0])
      quaternion = ROS_Quaternion(x=0, y=0, z=math.sin(angle / 2), w=math.cos(angle / 2))
      message = ROS_Pose(position=ROS_Point(x=pos[0], y=pos[1], z=0), orientation=quaternion)
      writer.write(dock_conn, time.time_ns(), typestore.serialize_ros1(message, ROS_Pose.__msgtype__))

  # Process mowing and navigation areas.
  for feature in areas:
    area = area_to_shapely_polygon(feature)

    # First reduce the area by those obstacles which are on its boundary.
    for obstacle in obstacles:
      if area.overlaps(obstacle):
        area = area.difference(obstacle)

    # Now collect the remaining obstacles which are completely within the area.
    ros_obstacles = []
    for obstacle in obstacles:
      if area.contains(obstacle):
        ros_obstacles.append(polygon_shapely_to_ros(obstacle))

    # Write the area to the bag.
    conn = mow_conn if feature.properties['type'] == 'lawn' else nav_conn
    message = ROS_MapArea(name='', area=polygon_shapely_to_ros(area), obstacles=ros_obstacles)
    writer.write(conn, time.time_ns(), typestore.serialize_ros1(message, ROS_MapArea.__msgtype__))

# TODO: Write docking point.
