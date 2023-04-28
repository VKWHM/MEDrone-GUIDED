from modules.medrone import MEDrone
import logging
import time
from dronekit import VehicleMode, connect
logging.basicConfig(level=logging.DEBUG)
vehicle = connect(
    '/dev/ttyACM0', baud=115200, wait_ready=True)
def set_servo(self, channel, pwm_value):
    self.channels.overrides[str(channel)] = pwm_value

#try:
#    drone.takeoff(5)
#    time.sleep(10)
#    drone.vehicle.mode = VehicleMode('LAND')
#except KeyboardInterrupt:
#    drone.vehicle.mode = VehicleMode('LAND')

set_servo(vehicle, 7,900)
time.sleep(2)
set_servo(vehicle, 7,2000)
time.sleep( 3)
set_servo(vehicle, 8,900)
time.sleep( 2)
set_servo(vehicle, 8,2000)
time.sleep( 3)
set_servo(vehicle, 6,900)
time.sleep(2)
set_servo(vehicle, 6,2000)
time.sleep(3)

