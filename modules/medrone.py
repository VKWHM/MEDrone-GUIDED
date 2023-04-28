from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import warnings
import logging
import math
import re
import threading
with warnings.catch_warnings():
    warnings.filterwarnings("ignore", category=FutureWarning)
    from coordinates.converter import convert_degrees_to_decimal


class MEDrone:
    def __init__(self, connection_port, logger='MEDrone'):
        self._logger = logging.getLogger(logger)
        if 'dev' in connection_port:
            self.vehicle = connect(
                connection_port, baud=57600, wait_ready=True)
        else:
            self.vehicle = connect(connection_port, wait_ready=False)
        self.vehicle.mode = VehicleMode("GUIDED")
        self._logger.debug('Guided Moduna Alınmayı Bekleniyor.')
        while not self.vehicle.mode.name == "GUIDED":
            time.sleep(1)
        else:
            self._logger.info('Guided Moda Alındı')
        while self.vehicle.is_armable is not True:
            self._logger.warning("IHA ARM edilebilir durumda değil.")
            time.sleep(2)
        threading.Thread(target=self.instant_status, daemon=True).start()

    @property
    def location(self):
        return list(self.vehicle.location.global_relative_frame.__dict__.values())[:3]

    def takeoff(self, altitude):
        self._logger.debug('Kalkış İçin Hazırlanıyor...')
        self.vehicle.armed = True
        self._logger.debug('ARM Olunmayı Bekleniyor.')
        self.vehicle.flush()
        start_time = time.time()
        while not self.vehicle.armed:
            time.sleep(1)
            if time.time() - start_time > 3:
                break
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

        time.sleep(3)

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
        self.vehicle.channels.overrides[str(channel)] = pwm_value
        #self._logger.debug("Servolar Çalıştırılıyor...")
        #msg = self.vehicle.message_factory.command_long_encode(
        #    0, 0,  # target system, target component
        #    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        #    0,  # confirmation
        #    channel,  # servo number
        #    pwm_value,  # servo PWM value
        #    0, 0, 0, 0, 0)  # unused parameters
        #self.vehicle.send_mavlink(msg)

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
        time.sleep(3)

    def instant_status(self):
        while True:
            self._logger.debug(" Global Location: %s" % self.vehicle.location.global_frame)
            self._logger.debug(" Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)
            self._logger.debug(" Local Location: %s" % self.vehicle.location.local_frame)
            self._logger.debug(" Attitude: %s" % self.vehicle.attitude)
            self._logger.debug(" Velocity: %s" % self.vehicle.velocity)
            self._logger.debug(" GPS: %s" % self.vehicle.gps_0)
            self._logger.debug(" Gimbal status: %s" % self.vehicle.gimbal)
            self._logger.debug(" Battery: %s" % self.vehicle.battery)
            self._logger.debug(" EKF OK?: %s" % self.vehicle.ekf_ok)
            self._logger.debug(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
            self._logger.debug(" Rangefinder: %s" % self.vehicle.rangefinder)
            self._logger.debug(" Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
            self._logger.debug(" Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
            self._logger.debug(" Heading: %s" % self.vehicle.heading)
            self._logger.debug(" Is Armable?: %s" % self.vehicle.is_armable)
            self._logger.debug(" System status: %s" % self.vehicle.system_status.state)
            self._logger.debug(" Groundspeed: %s" % self.vehicle.groundspeed)    # settable
            self._logger.debug(" Airspeed: %s" % self.vehicle.airspeed)    # settable
            self._logger.debug(" Mode: %s" % self.vehicle.mode.name)    # settable
            self._logger.debug(" Armed: %s" % self.vehicle.armed)    # settable
            time.sleep(10)

## 32°21'21" 
class GPSLocation(LocationGlobalRelative):
    def __init__(self, *args):
        lat = args[0]
        lon = args[1]
        pattern = re.compile("(([0-9\.-]+)°)?(([0-9\.-]+)')?(([0-9\.-]+)[('')|\"])?")
        lat_result = pattern.search(lat)
        lon_result = pattern.search(lon)
        if lat_result.group(0) != '' and lon_result.group(0) != '':
            lat_deg = float(lat_result.group(
                2) if lat_result.group(2) is not None else 0)
            lat_min = float(lat_result.group(
                4) if lat_result.group(4) is not None else 0)
            lat_sec = float(lat_result.group(
                6) if lat_result.group(6) is not None else 0)
            lon_deg = float(lon_result.group(
                2) if lon_result.group(2) is not None else 0)
            lon_min = float(lon_result.group(
                4) if lon_result.group(4) is not None else 0)
            lon_sec = float(lon_result.group(
                6) if lon_result.group(6) is not None else 0)
            lat_dd = convert_degrees_to_decimal(lat_deg ,lat_min ,lat_sec)
            lon_dd = convert_degrees_to_decimal(lon_deg ,lon_min ,lon_sec)
        else:
            (lat_dd, lon_dd) = float(lat), float(lon)
        super().__init__(lat_dd, lon_dd, float(args[2]))
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
    return (v_x//2, v_y//2)


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
