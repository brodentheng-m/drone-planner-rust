import sys
import time
import threading
from codrone_edu.drone import *

class DroneCrashException(Exception):
    def __init__(self, stage, reason, details):
        super().__init__(f"CRASH DETECTED at {stage}: {reason} ({details})")
        self.stage = stage
        self.reason = reason
        self.details = details

class CourseController:
    def __init__(self):
        self.drone = Drone()
        self.current_stage = "Initialization"
        self.initial_accidents = 0
        self.crashed = threading.Event()
        self.crash_info = None
        self.stop_watchdog = threading.Event()
        self.watchdog_thread = None

    def connect(self):
        self.current_stage = "Pairing"
        print("Connecting to drone controller...")
        self.drone.pair()
        battery = self.drone.get_battery()
        print(f"Connected. Battery: {battery}%")
        if battery is not None and battery < 20:
            print("Battery level below 20%, aborting flight")
            self.drone.close()
            sys.exit(1)
        self.initial_accidents = self.drone.get_accident_count() or 0
        print("Calibrating gyroscope on takeoff pad...")
        self.drone.reset_gyro()
        time.sleep(1.0)
        print("Gyroscope calibrated. Ready for flight.")

    def start_watchdog(self):
        self.stop_watchdog.clear()
        self.crashed.clear()
        self.crash_info = None
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

    def stop_watchdog_monitor(self):
        self.stop_watchdog.set()
        if self.watchdog_thread and self.watchdog_thread.is_alive():
            self.watchdog_thread.join(timeout=0.5)

    def _watchdog_loop(self):
        while not self.stop_watchdog.is_set():
            try:
                accidents = self.drone.get_accident_count()
                state = self.drone.get_flight_state()
                roll = self.drone.get_angle_x()
                pitch = self.drone.get_angle_y()

                reason = None
                if accidents is not None and accidents > self.initial_accidents:
                    reason = "Hardware accident flag triggered"
                elif state in (ModeFlight.Accident, ModeFlight.Error):
                    reason = f"Flight state changed to {state}"
                elif roll is not None and abs(roll) > 55:
                    reason = f"Excessive roll angle ({roll} deg)"
                elif pitch is not None and abs(pitch) > 55:
                    reason = f"Excessive pitch angle ({pitch} deg)"

                if reason:
                    self.crash_info = (self.current_stage, reason, f"roll={roll}, pitch={pitch}, state={state}")
                    self.crashed.set()
                    self.drone.emergency_stop()
                    break
            except Exception:
                pass
            time.sleep(0.04)

    def execute_step(self, stage_name, action):
        self.current_stage = stage_name
        print(f"[STAGE] {stage_name}")
        if self.crashed.is_set():
            stage, reason, details = self.crash_info
            raise DroneCrashException(stage, reason, details)
        action()
        if self.crashed.is_set():
            stage, reason, details = self.crash_info
            raise DroneCrashException(stage, reason, details)

    def run_solution(self):
        self.start_watchdog()
        try:
            self.execute_step("Takeoff", lambda: (
                self.drone.set_drone_LED(255, 87, 34, 100),
                self.drone.takeoff(),
                self.drone.hover(1.5)
            ))

            self.execute_step("Station 2: Approach Bar Stool (7 ft)", lambda: (
                self.drone.set_drone_LED(92, 58, 33, 100),
                self.drone.turn_left(16),
                self.drone.move_forward(213, units="cm", speed=0.8),
                self.drone.hover(1.0)
            ))

            self.execute_step("Station 3: Desk Underpass Alignment", lambda: (
                self.drone.set_drone_LED(229, 181, 106, 100),
                self.drone.turn_right(95),
                self.drone.go("down", 35, 0.8)
            ))

            self.execute_step("Station 3: Fly Under Desk (6 ft)", lambda: (
                self.drone.move_forward(183, units="cm", speed=0.7),
                self.drone.hover(1.0)
            ))

            self.execute_step("Station 4: Knock Ball off Mini Cone (5 ft)", lambda: (
                self.drone.set_drone_LED(142, 36, 170, 100),
                self.drone.turn_left(117),
                self.drone.move_forward(152, units="cm", speed=0.8),
                self.drone.hover(1.0)
            ))

            self.execute_step("Station 5: Align with Purple Landing Pad (5 ft)", lambda: (
                self.drone.set_drone_LED(126, 87, 194, 100),
                self.drone.turn_right(91),
                self.drone.move_forward(152, units="cm", speed=0.8)
            ))

            self.execute_step("Station 5: Land on Purple Pad", lambda: (
                self.drone.land(),
                self.drone.set_drone_LED(0, 255, 0, 100),
                self.drone.drone_buzzer("C5", 0.3)
            ))

            print("MISSION COMPLETE: Drone successfully completed Challenge 2 and landed on the pad.")

        except DroneCrashException as crash:
            print("\n" + "=" * 50)
            print("DRONE CRASH DETECTED!")
            print(f"Stage:   {crash.stage}")
            print(f"Reason:  {crash.reason}")
            print(f"Details: {crash.details}")
            print("=" * 50)
            try:
                self.drone.emergency_stop()
                self.drone.set_drone_LED(255, 0, 0, 100)
                self.drone.drone_buzzer("A3", 0.5)
            except Exception:
                pass
        except KeyboardInterrupt:
            print("\nOperator abort triggered.")
            self.drone.emergency_stop()
        finally:
            self.stop_watchdog_monitor()
            self.drone.close()
            print("Drone connection closed.")

if __name__ == "__main__":
    controller = CourseController()
    controller.connect()
    controller.run_solution()
