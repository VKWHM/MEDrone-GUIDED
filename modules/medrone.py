from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import logging
import math
import pyproj
import re


class MEDrone:
    def __init__(self, connection_port, logger='MEDrone'):
        self._logger = logging.getLogger(logger)
        if 'dev' in connection_port:
            self.vehicle = connect(
                connection_port, buad=57600, wait_ready=True)
        else:
            self.vehicle = connect(connection_port, wait_ready=False)
        self.vehicle.mode = VehicleMode("GUIDED")
        self._logger.debug('Guided Moduna Alınmayı Bekleniyor.')
        while not self.vehicle.mode.name == "GUIDED":
            time.sleep(1)
        else:
            self._logger.info('Guided Moda Alındı')
        self.vehicle.armed = True
        while self.vehicle.is_armable is not True:
            self._logger.warning("IHA ARM edilebilir durumda değil.")
            time.sleep(2)

    @property
    def location(self):
        return list(self.vehicle.location.global_relative_frame.__dict__.values())[:3]

    def takeoff(self, altitude):
        self._logger.debug('Kalkış İçin Hazırlanıyor...')
        self._logger.debug('ARM Olunmayı Bekleniyor.')
        while not self.vehicle.armed:
            time.sleep(1)
        else:
            self._logger.info("ARM olundu")

        self._logger.info("Kalkış yapılıyor...")
        self.vehicle.simple_takeoff(altitude)

        while True:
            self._logger.debug(
                f"Anlık İrtifa: {self.vehicle.location.global_relative_frame.alt:.2f}m")
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                self._logger.info("Hedef irtifaya ulaşıldı")
                break
            time.sleep(1)

    def send_ned_velocity(self, velocity_x, velocity_y):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        self._logger.debug(
            "Net Velocity mesajı: x-{}, y-{}".format(velocity_x, velocity_y))
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, 0,  # x, y, z velocity in m/s
            # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0, 0,
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        self.vehicle.send_mavlink(msg)

    def set_servo(self, channel, pwm_value):
        """
        Set the PWM value for a specific channel of the servo.

        :param channel: The channel of the servo to control.
        :param pwm_value: The PWM value to set for the servo.
        """
        self._logger.debug("Servolar Çalıştırılıyor...")
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,  # confirmation
            channel,  # servo number
            pwm_value,  # servo PWM value
            0, 0, 0, 0, 0)  # unused parameters
        self.vehicle.send_mavlink(msg)

    def simple_goto(self, targetLocation):
        currentLocation = self.vehicle.location.global_relative_frame
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        self._logger.info(
            "Hedef Noktasına Olan Uzaklık: {:.2f}m".format(targetDistance))

        self.vehicle.simple_goto(targetLocation)

        while True:
            currentLocation = self.vehicle.location.global_relative_frame
            targetDistance = get_distance_metres(
                currentLocation, targetLocation)

            if targetDistance <= 1:
                self._logger.info("Hedef Noktasına Ulaşıldı.")
                break

            self._logger.info(
                "Hedef Noktasına Olan Uzaklık: {:.2f}m".format(targetDistance))
            time.sleep(1)


class GPSLocation(LocationGlobalRelative):
    def __init__(self, *args):
        lat = args[0]
        lon = args[1]
        pattern = re.compile("(([0-9\.]+)°)?(([0-9\.]+)')?(([0-9\.]+)'')?")
        lat_result = pattern.search(lat)
        lat_deg = float(lat_result.group(
            2) if lat_result.group(2) is not None else 0)
        lat_min = float(lat_result.group(
            4) if lat_result.group(4) is not None else 0)
        lat_sec = float(lat_result.group(
            6) if lat_result.group(6) is not None else 0)
        lon_result = pattern.search(lon)
        lon_deg = float(lon_result.group(
            2) if lon_result.group(2) is not None else 0)
        lon_min = float(lon_result.group(
            4) if lon_result.group(4) is not None else 0)
        lon_sec = float(lon_result.group(
            6) if lon_result.group(6) is not None else 0)
        lat_dd = lat_deg + (lat_min / 60) + (lat_sec / 3600)
        lon_dd = lon_deg + (lon_min / 60) + (lon_sec / 3600)
        wgs84 = pyproj.CRS('EPSG:4326')
        utm = pyproj.CRS('EPSG:32618')
        transformer = pyproj.Transformer.from_crs(wgs84, utm)
        easting, northing = transformer.transform(lon_dd, lat_dd)
        super().__init__(easting, northing, float(args[2]))

    def export(self):
        return [self.lat, self.lon, self.alt]


def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_direction(image_height, image_width, box):
    target_center_x, target_center_y = (
        box[0] + box[2])//2, (box[1] + box[3])//2
    error_x = target_center_x - image_width / 2
    error_y = target_center_y - image_height / 2
    v_x = -error_y / 100
    v_y = error_x / 100
    v_x = max(min(v_x, 1), -1)
    v_y = max(min(v_y, 1), -1)
    return (v_x, v_y)


def is_drone_in_target(img, box, distance=20):
    x_min, y_min, x_max, y_max = box[0], box[1], box[2], box[3]
    # find center
    (h, w) = img.shape[:2]
    box_w = x_max - x_min
    box_y = y_max - y_min
    box_center_x = x_min + box_w//2
    box_center_y = y_min + box_y//2

    fark_w = box_center_x - w//2
    fark_y = box_center_y - h//2
    if (abs(fark_w) < distance) and (abs(fark_y) < distance):
        return True
    return False
