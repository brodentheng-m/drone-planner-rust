import sys
import time
from codrone_edu.drone import *

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

    drone.set_drone_LED(255, 87, 34, 100)
    drone.takeoff()
    drone.hover(1.5)

    drone.turn_left(16)
    drone.move_forward(213, units="cm", speed=0.8)
    drone.hover(1.0)

    drone.turn_right(95)
    drone.go("down", 35, 0.8)
    drone.move_forward(183, units="cm", speed=0.7)

    drone.turn_left(117)
    drone.move_forward(152, units="cm", speed=0.8)
    drone.hover(1.0)

    drone.turn_right(91)
    drone.move_forward(152, units="cm", speed=0.8)
    drone.set_drone_LED(126, 87, 194, 100)
    drone.land()

except KeyboardInterrupt:
    drone.emergency_stop()

finally:
    drone.close()
