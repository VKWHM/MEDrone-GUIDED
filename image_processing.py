from modules.object_tracking import YoloObjectTracker, HaarObjectTracker
from modules.manager import CaptureManager, WindowManager
from modules.cvclient import CVClient

import argparse
import logging
logging.basicConfig(level=logging.INFO)
import os
import cv2

def parse_args():
    parser = argparse.ArgumentParser()
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
        "-t",
        "--threshold",
        dest="thresh",
        default=0.6,
        type=float,
        help="Trashold value.",
    )
    return parser.parse_args()

def main():
    args = parse_args()
    if args.host:
        camera = CVClient(args.camera)
    else:
        camera = cv2.VideoCapture(args.camera if len(args.camera) > 2 else int(args.camera))
    Cameo(camera, trashold=args.thresh).run()

class Cameo(object):
    def __init__(self, Capture, logger="Cameo", detect=None, trashold=None):
        self._logger = logging.getLogger(logger)
        self._windowManager = WindowManager("Window", self.onKeypress)
        self._captureManager = CaptureManager(
            Capture,
            self._windowManager,
        )

        self.shouldTracking = False
        #self._track = YoloObjectTracker(os.path.join(os.path.realpath('src'), 'medrone_hedef.cfg'),
        #                     os.path.join(os.path.realpath('src'), 'medrone_hedef.weights'), threshold=trashold
        #                     )
        self._track = HaarObjectTracker(os.path.join(os.path.realpath('src'), 'cascade.xml'), threshold=trashold)


    def run(self):
        """
        Run Main loop.
        """
        self._showFPS = False
        self._windowManager.createWindow()
        while self._windowManager.isWindowCreated:
            with self._captureManager as frame:
                if frame is not None:
                    if self._showFPS:
                        fps_text = self._captureManager.fps
                        cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    if self._track is not None and self.shouldTracking:
                        self._track.track_objects(frame)

            self._windowManager.processEvent()

    def onKeypress(self, keycode):
        """
        Handle a keypress.
        space -> Take a screenshot
        x -> Start/Stop Face Tracking
        tab -> Start/Stop recording a screencast.
        escape -> Quit.
        """
        if keycode == 32:  # space
            self._captureManager.writeImage("screenshot.png")
        elif keycode == 9:  # TAB
            if not self._captureManager.isWritingVideo:
                self._captureManager.startWriteVideo(
                    "screencast.mp4", cv2.VideoWriter_fourcc(*'mp4v')
                )
            else:
                self._captureManager.stopWriteVideo()
        elif keycode == ord("f"):
            self._showFPS = not self._showFPS
        elif keycode == ord("j"):
            self._logger.info(
                "Object Tracking "
                + ("Started" if not self.shouldTracking else "Stoped")
            )
            self.shouldTracking = not self.shouldTracking
        elif keycode == 27:  # ESC
            self._windowManager.destroyWindow()

if __name__ == '__main__':
    main()
