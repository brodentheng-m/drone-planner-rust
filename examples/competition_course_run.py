from codrone_edu.drone import *
import time

drone = Drone()
drone.pair()

drone.takeoff()
drone.hover(1.5)
drone.turn_right(106)
drone.move_forward(213, speed=1.0)
drone.hover(1.0)
drone.turn_left(95)
drone.go("down", 35, 0.8)
drone.move_forward(183, speed=0.9)
drone.turn_right(117)
drone.move_forward(152, speed=1.0)
drone.hover(1.0)
drone.turn_left(83)
drone.move_forward(165, speed=1.0)
drone.land()

drone.close()
