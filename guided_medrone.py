from modules.object_tracking import ObjectTracker
from modules.medrone import MEDrone, get_direction, is_drone_in_target, get_distance_metres
from modules.manager import CaptureManager, WindowManager
from dronekit import LocationGlobalRelative, VehicleMode
from modules.cvclient import CVClient

import cv2
import logging
import json
import argparse
import time
import os


class CameraError(Exception):
    pass


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "address",
        default=0,
        help="Drone connection address.",
    )
    parser.add_argument(
        "-c",
        "--camera",
        dest='camera',
        required=True,
        help="Video file or a capturing device or an IP video stream for video capturing.",
    )
    parser.add_argument(
        "-n",
        "--network",
        dest='host',
        action='store_true',
        help="Receive Frames stream from network",
    )
    parser.add_argument(
        '-f',
        '--file',
        type=str,
        required=True,
        help='File for way points.'
    )
    parser.add_argument(
        "-t",
        "--threshold",
        dest="thresh",
        default=0.6,
        type=float,
        help="Trashold value.",
    )
    parser.add_argument(
        "-l",
        "--log-level",
        dest="log",
        default="warning",
        choices=["critical", "error", "warning", "info", "debug"],
        help="Level for logging",
    )
    return parser.parse_args()


def sort_wp(wp_list, current_location, home_location=None):
    if home_location is None:
        home_location = current_location
    coor_combinations = []
    def generate_combinations(digits, _list, prefix=""):
        if len(digits) == 0:
            coor_combinations.append([_list[int(i)] for i in [int(prefix[j]) for j in range(len(prefix))]])
        else:
            for i in range(len(digits)):
                generate_combinations(digits[:i] + digits[i+1:], _list, prefix + digits[i])

    finally_list = []
    current_combination = []
    if len(wp_list) > 1:
        generate_combinations(''.join(str(i) for i in range(len(wp_list))), wp_list)
    elif len(wp_list):
        current_combination = wp_list
    min_distance = 10e100
    for combination in coor_combinations:
        distance = get_distance_metres(current_location, LocationGlobalRelative(*combination[0]['coordinates']))
        for j in range(len(combination)-1):
            distance += get_distance_metres(LocationGlobalRelative(*combination[j]['coordinates']), LocationGlobalRelative(*combination[j+1]['coordinates']))
        else:
            distance += get_distance_metres(LocationGlobalRelative(*combination[j+1]['coordinates']), home_location)
            if distance < min_distance:
                min_distance = distance
                current_combination = combination
    if current_combination is not None:
        finally_list.extend(current_combination)
    return finally_list 


def get_wp(filename, current_location):
    if not os.path.exists(filename):
        raise FileNotFoundError('The Way Points File Name Not Exists')

    with open(filename, 'r') as f:
        wp_data = json.load(f)
        if wp_data.get('sort_status', 0) == 1:
            return [LocationGlobalRelative(*wp['coordinates']) for wp in wp_data['points']]
    wps = []
    urgency_wp = [wp for wp in wp_data['points'] if wp.get('urgency', 0) == 1]
    wps.extend(sort_wp(urgency_wp, current_location))
    normal_wp = [wp for wp in wp_data['points'] if wp.get('urgency', 0) == 0]
    if len(wps):
        wps.extend(sort_wp(normal_wp, LocationGlobalRelative(*wps[-1]['coordinates']), current_location))
    else:
        wps.extend(sort_wp(normal_wp, current_location))

    return [LocationGlobalRelative(*wp['coordinates']) for wp in wps]

def main():
    args = parse_args()
    try:
        import colorlog
        colorlog.basicConfig(
            format="[%(asctime)s.%(msecs)03d] [%(log_color)s%(levelname)s%(reset)s] (%(name)s): %(log_color)s%(message)s%(reset)s",
            level=getattr(logging, args.log.upper()),
            datefmt="%H:%M:%S",
        )
    except ImportError:
        logging.basicConfig(
            format="[%(asctime)s.%(msecs)03d] [%(levelname)s] (%(name)s): %(message)s",
            level=getattr(logging, args.log.upper()),
            datefmt="%H:%M:%S",
        )

    window = WindowManager("Medrone Hedef Tespiti")
    if args.host:
        camera = CVClient(args.camera)
    else:
        camera = cv2.VideoCapture(args.camera if len(args.camera) > 2 else int(args.camera))

    if not camera.isOpened():
        raise CameraError("Can't Open Camera")
    cap = CaptureManager(camera, window)

    detector = ObjectTracker(os.path.join(os.path.realpath('src'), 'medrone_hedef.cfg'),
                             os.path.join(os.path.realpath('src'), 'medrone_hedef.weights'), threshold=args.thresh
                             )

    drone = MEDrone(args.address)
    way_points = get_wp(args.file, LocationGlobalRelative(*drone.location))
    drone.takeoff(10)

    for point in way_points:
        logging.info(f"{list(point.__dict__.values())[:3]} Noktasına Gidiliyor...")
        drone.simple_goto(point)
        logging.info("Görüntü İşleme Modülü Çalışıyor...")
        window.createWindow()
        objects = {}
        while window.isWindowCreated:
            with cap as frame:
                if frame is not None:
                    frame, targets = detector.track_objects(frame)
                    if len(targets) == 1:
                        target = targets[0]
                        if len(objects) and (objects.get(target['id'], 0) > 60):
                            logging.debug(f"Detected {target['class']} on {target['bbox']}")
                            box = target['bbox']
                            if is_drone_in_target(frame, box, 40):
                                break
                            else:
                                logging.debug("{} {} {}".format(*drone.location))
                                drone.send_ned_velocity(
                                    *get_direction(*frame.shape[:2], box)
                                )
                        objects.update({target['id']: 1 + objects.get(target['id'], 0)})
                #cap.frame = frame
            window.processEvent()
        window.destroyWindow()

        # drone.set_servo(CHANNEL, VALUE)
        logging.info("Servo Açılmasını Bekleniyor")
        time.sleep(4)
        logging.info("Servo Kapanıyor")
        time.sleep(4)

    logging.info("Home Noktasina İniyor... ")
    drone.vehicle.mode = VehicleMode("RTL")


if __name__ == '__main__':
    main()
