# Description
This repository contains tools to convert between `map.bag` and GeoJSON format.

The latter is used by the [ROS2 port](https://jkaflik.github.io/OpenMowerROS2/) of OpenMower and can
be viewed and modified e.g. using https://play.placemark.io/.

# Installation
The scripts don't need the ROS environment, i.e. they can run on any host as long as you transfer
the `map.bag` to it. You might need to set `OM_DATUM_LAT` and `OM_DATUM_LONG` or use the script
options to define the GPS datum in that case.

It's recommended to run the scripts in a virtual environment:

```bash
sudo apt update && apt install python3 python3-venv
git clone https://github.com/rovo89/open_mower_geojson
cd open_mower_geojson
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

# Running scripts
Before every script invocation, execute:
```bash
source .venv/bin/activate
source /boot/openmower/mower_config.txt
```

One round-trip is as simple as:
```bash
./bag_to_geojson.py map.bag map.geojson
./geojson_to_bag.py map.geojson map.bag
```
