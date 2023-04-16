from modules.object_tracking import ObjectTracker
from modules.medrone import MEDrone, get_direction, is_drone_in_target
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


def get_wp(filename):
    if os.path.exists(filename):
        with open(filename, 'r') as f:
            return [LocationGlobalRelative(*wp) for wp in json.load(f)]
    raise FileNotFoundError('The Way Points File Name Not Exists')


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

    way_points = get_wp(args.file)
    drone = MEDrone(args.address)
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
                        if len(objects) and (objects.get(target['id'], 0) > 90):
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
