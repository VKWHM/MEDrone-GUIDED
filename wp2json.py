from modules.medrone import MEDrone
import json
import sys

def main():
    drone = MEDrone(sys.argv[1])
    wps = []
    servoIds = [7,8,6]
    while len(wps) < 3:
        input("Press Enter To Take Way Point...")
        print([*drone.location[:2],10])
        wps.append({'coordinates': [*drone.location[:2],10], 'servoId': servoIds[len(wps)]})

    with open('src/wp.json', 'w') as f:
        json.dump({'points': wps}, f, indent=4)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.stderr.write("usage: python3 {} <connection address>\n".format(sys.argv[0]))
        sys.exit(1)
    main()

