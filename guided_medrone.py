from modules.object_tracking import HaarObjectTracker
from modules.medrone import MEDrone, get_direction, is_drone_in_target, get_distance_metres
from modules.manager import CaptureManager, WindowManager
from dronekit import LocationGlobalRelative, VehicleMode
from modules.cvclient import CVClient

import cv2
import threading
import queue
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
        help="Drone connection address.",
    )

    subparsers = parser.add_subparsers(dest='source')

    camera_parser = subparsers.add_parser('camera', help='Capture video from a camera or video file')
    camera_parser.add_argument(
        "ID",
        type=str,
        help="Video file or a capturing device or an IP video stream for video capturing.",
    )
    camera_parser.add_argument(
        "-n",
        "--network",
        dest='host',
        action='store_true',
        help="Receive Frames stream from network",
    )
    camera_parser.add_argument(
        "-t",
        "--threshold",
        dest="thresh",
        default=0.5,
        type=float,
        help="Threshold value.",
    )

    parser.add_argument(
        '-f',
        '--file',
        type=str,
        required=True,
        help='File for way points.'
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
            coor_combinations.append(
                [_list[int(i)] for i in [int(prefix[j]) for j in range(len(prefix))]])
        else:
            for i in range(len(digits)):
                generate_combinations(
                    digits[:i] + digits[i+1:], _list, prefix + digits[i])

    finally_list = []
    current_combination = []
    if len(wp_list) > 1:
        generate_combinations(''.join(str(i)
                              for i in range(len(wp_list))), wp_list)
    elif len(wp_list):
        current_combination = wp_list
    min_distance = 10e100
    for combination in coor_combinations:
        distance = get_distance_metres(
            current_location, LocationGlobalRelative(*combination[0]['coordinates']))
        for j in range(len(combination)-1):
            distance += get_distance_metres(LocationGlobalRelative(
                *combination[j]['coordinates']), LocationGlobalRelative(*combination[j+1]['coordinates']))
        else:
            distance += get_distance_metres(LocationGlobalRelative(
                *combination[j+1]['coordinates']), home_location)
            if distance < min_distance:
                min_distance = distance
                current_combination = combination
    if current_combination is not None:
        finally_list.extend(current_combination)
    return finally_list


def get_wp(filename, current_location):
    if not os.path.exists(filename):
        logging.error(f'{filename} File Is Not Exists')
        raise FileNotFoundError('The Way Points File Name Not Exists')

    with open(filename, 'r') as f:
        wp_data = json.load(f)
        if wp_data.get('sort_status', 0) == 1:
            return wp_data['points']
    wps = []
    urgency_wp = [wp for wp in wp_data['points'] if wp.get('urgency', 0) == 1]
    wps.extend(sort_wp(urgency_wp, current_location))
    normal_wp = [wp for wp in wp_data['points'] if wp.get('urgency', 0) == 0]
    if len(wps):
        wps.extend(sort_wp(normal_wp, LocationGlobalRelative(
            *wps[-1]['coordinates']), current_location))
    else:
        wps.extend(sort_wp(normal_wp, current_location))
    logging.info(f"En kısa rota: {[wp['coordinates'] for wp in wps]}")
    return wps


def image_processing(cap, window, drone, detector):
    counter = 90
    start_time = time.time()
    window.createWindow()
    objects = {}
    send_ordering_status = True
    _queue = queue.Queue()

    def send_orders(q):
        while send_ordering_status:
            drone.send_ned_velocity(*q.get())
            time.sleep(0.1)

    thread = threading.Thread(target=send_orders, args=(_queue,), daemon=True)
    thread.start()
    while window.isWindowCreated:
        with cap as frame:
            if frame is not None:
                frame, targets = detector.track_objects(frame)
                if len(targets) == 1:
                    target = targets[0]
                    if len(objects) and (objects.get(target['id'], 0) > 60):
                        logging.debug(f"Detected {target['class']} on {target['bbox']}")
                        box = target['bbox']
                        if is_drone_in_target(frame, box, 50):
                            break
                        else:
                            logging.debug("{} {} {}".format(*drone.location))
                            if _queue.empty():
                                _queue.put(
                                    get_direction(*frame.shape[:2], box)
                                )
                            start_time += 0.1
                            counter -= 0.1
        
                    objects.update({target['id']: 1 + objects.get(target['id'], 0)})
        window.processEvent()
        if time.time() - start_time > 20 or counter < 0:
            logging.warning('Image Processing Timeout! Break.')
            break
    window.destroyWindow()
    send_ordering_status = False
    thread.join()

def main():
    RISE_ALTITUDE = 5
    DESCEND_ALTITUDE = 3

    args = parse_args()
    _format = "[%(asctime)s.%(msecs)03d] [%(levelname)s] (%(name)s): %(message)s"

    try:
        import colorlog
        stream_handler = colorlog.StreamHandler()
        stream_handler.setFormatter(colorlog.ColoredFormatter("[%(asctime)s.%(msecs)03d] [%(log_color)s%(levelname)s%(reset)s] (%(name)s): %(log_color)s%(message)s%(reset)s", datefmt="%H:%M:%S"))
        stream_handler.setLevel(getattr(logging, args.log.upper()))
    except ImportError:
        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(logging.Formatter(_format, datefmt="%H:%M:%S"))
        stream_handler.setLevel(getattr(logging, args.log.upper()))

    file_handler = logging.FileHandler('src/MEDBox.log', 'w')
    formatter = logging.Formatter(_format, datefmt="%H:%M:%S")
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging.DEBUG)

    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)
    root_logger.addHandler(file_handler)
    root_logger.addHandler(stream_handler)

    loggers = [logging.getLogger(logger) for logger in ['MEDrone', 'autopilot', 'CaptureManager', 'WindowManager', 'CVClient']]
    for logger in loggers:
        logger.setLevel(logging.DEBUG)

    if args.source is not None:
        window = WindowManager("Medrone Hedef Tespiti")
        if args.host:
            camera = CVClient(args.ID)
        else:
            camera = cv2.VideoCapture(args.ID if len(args.ID) > 2 else int(args.ID))

        if not camera.isOpened():
            raise CameraError("Can't Open Camera")
        cap = CaptureManager(camera, window)

        detector = HaarObjectTracker(os.path.join(os.path.realpath('src'), 'cascade.xml'), threshold=args.thresh)

    drone = MEDrone(args.address)
    way_points = get_wp(args.file, LocationGlobalRelative(*drone.location))
    try:
        drone.takeoff(RISE_ALTITUDE)
        for wp in way_points:
            point = LocationGlobalRelative(*wp['coordinates'])
            logging.info(f"{wp['coordinates']} Noktasına Gidiliyor...")
            drone.simple_goto(point)
            logging.debug(f"Drone {DESCEND_ALTITUDE} metreye kadar alçalıyor...")
            drone.vehicle.simple_goto(LocationGlobalRelative(*wp['coordinates'][:2], DESCEND_ALTITUDE))
            while True:
                current_alt = drone.location[2]
                if current_alt <= DESCEND_ALTITUDE:
                    break
            if args.source is not None:
                logging.info("Görüntü İşleme Modülü Çalışıyor...")
                image_processing(cap, window, drone, detector)

            if wp.get('servoId') is not None:
                try:
                    logging.info("Servo Açılmasını Bekleniyor")
                    drone.set_servo(wp['servoId'], 900)
                    time.sleep(4)
                    logging.info("Servo Kapanıyor")
                    drone.set_servo(wp['servoId'], 2000)
                    time.sleep(4)
                except Exception as e:
                    logging.error(f"Servo Error: {e}")
            else:
                logging.warning(f"Way Point'e ({wp['coordinates']}) ait bir servo id bulunamadı !!")
                time.sleep(2)

            logging.debug(f"Drone {RISE_ALTITUDE} metreye kadar yükseliyor...")
            drone.vehicle.simple_goto(LocationGlobalRelative(*wp['coordinates'][:2], RISE_ALTITUDE))
            while True:
                current_alt = drone.location[2]
                if current_alt >= RISE_ALTITUDE:
                    break

            time.sleep(3)

        logging.info("Home Noktasina İniyor... ")
        drone.vehicle.mode = VehicleMode("RTL")
        if args.source is not None:
            cap.close()
    except KeyboardInterrupt:
        logging.debug("CTRL-C Detected. Mode RTL...")
        drone.vehicle.mode = VehicleMode("RTL")
        if args.source is not None:
            cap.close()
    except Exception as e:
        logging.critical(e)
        drone.vehicle.mode = VehicleMode("RTL")
        if args.source is not None:
            cap.close()


if __name__ == '__main__':
    main()
