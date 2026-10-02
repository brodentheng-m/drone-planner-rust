from codrone_edu.drone import *
import math
import time

def navigate_to_waypoint(drone, target_x, target_y, target_z=80, tolerance=15, timeout=60):
    start_time = time.time()
    evasion_direction = 1
    stuck_counter = 0
    while time.time() - start_time < timeout:
        cur_x = drone.get_pos_x("cm")
        cur_y = drone.get_pos_y("cm")
        cur_z = drone.get_pos_z("cm")
        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)
        if dist <= tolerance:
            drone.hover(0.5)
            break
        target_heading = math.degrees(math.atan2(dy, dx))
        drone.turn_degree(int(target_heading), timeout=2, p_value=10)
        front_dist = drone.get_front_range("cm")
        if drone.detect_wall(50) or front_dist < 50:
            drone.avoid_wall(1.5, 40)
            if evasion_direction == 1:
                drone.move_right(35, speed=0.5)
            else:
                drone.move_left(35, speed=0.5)
            stuck_counter += 1
            if stuck_counter > 3:
                evasion_direction *= -1
                stuck_counter = 0
        else:
            stuck_counter = 0
            step = min(dist, 40)
            drone.move_forward(int(step), speed=0.6)
    drone.hover(1.0)

if __name__ == "__main__":
    drone = Drone()
    drone.pair()
    drone.takeoff()
    
    # Example: Navigate to (x=200cm, y=150cm, z=80cm) while avoiding obstacles
    navigate_to_waypoint(drone, 200, 150, 80)
    
    drone.land()
    drone.close()
