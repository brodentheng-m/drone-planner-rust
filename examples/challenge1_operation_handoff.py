import sys
import time
from codrone_edu.drone import *

def run_leg(drone):
    drone.set_drone_LED(0, 200, 255, 100)
    drone.takeoff()
    drone.hover(1.5)

    drone.move_forward(152, units="cm", speed=0.8)
    drone.circle(speed=60, direction=1)
    drone.hover(1.0)

    drone.go("down", 35, 0.8)
    drone.move_forward(122, units="cm", speed=0.7)
    drone.move_forward(122, units="cm", speed=0.7)
    drone.move_forward(152, units="cm", speed=0.7)
    drone.set_drone_LED(126, 87, 194, 100)
    drone.land()

def return_leg(drone):
    drone.set_drone_LED(255, 165, 0, 100)
    drone.takeoff()
    drone.hover(1.0)
    drone.turn_right(180)
    drone.go("up", 35, 0.8)
    drone.move_forward(548, units="cm", speed=1.0)
    drone.set_drone_LED(0, 255, 0, 100)
    drone.land()

drone = Drone()
drone.pair()

try:
    battery = drone.get_battery()
    print(f"Battery: {battery}%")
    if battery is not None and battery < 20:
        print("Battery too low for flight")
        sys.exit(1)

    drone.reset_gyro()
    time.sleep(1)

    run_leg(drone)
    time.sleep(1)
    return_leg(drone)
    time.sleep(2)
    run_leg(drone)

except KeyboardInterrupt:
    drone.emergency_stop()

finally:
    drone.close()
