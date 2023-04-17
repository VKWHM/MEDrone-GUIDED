from modules.medrone import GPSLocation
import json
import sys

if len(sys.argv) < 3:
    sys.stderr.write(f'usage: python3 {sys.argv[0]} [text, json] "<<WAY POINT>>"\n')
    exit()
points = []
for point in sys.argv[2:]:
    lat, lon, alt = point.split(',')
    lat, lon, alt = lat.strip(), lon.strip(), alt.strip()
    points.append(GPSLocation(lat, lon, alt).export())
if sys.argv[1].strip() == 'json':
    _list = [{'coordinates': wp} for wp in points]
    print(json.dumps({'points': _list}, indent=3))
else:
    for point in points:
        for coor in point:
            print(coor)
