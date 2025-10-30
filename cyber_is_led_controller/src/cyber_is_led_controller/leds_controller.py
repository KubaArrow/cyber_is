from cyber_is_led_controller.leds_front import Front
from cyber_is_led_controller.leds_side  import Side
import rospy

class LedsController:
    def __init__(self):
        self.side_leds  = rospy.get_param("/led_controller/side_leds", 9)
        self.front_leds = rospy.get_param("/led_controller/front_leds", 6)

        self.f  = Front(self.front_leds)
        self.ls = Side(self.side_leds,  True)   # ← True z wielkiej litery
        self.rs = Side(self.side_leds,  False)

    # --------------- publikacja ---------------

    def get_leds(self):
        # zwracamy PŁASKĄ listę bajtów RGBA
        return (
            self.ls.get_leds_rgba()
            + self.f.get_leds_rgba()
            + self.rs.get_leds_rgba()
        )

    # --------------- sterowanie trybem ---------------

    def set_mode(self, mode: str):
        if mode.startswith("FRONT_"):
            self.f.set_mode(mode)
        elif mode.startswith("SIDE_"):
            self.ls.set_mode(mode)
            self.rs.set_mode(mode)
        else:
            rospy.logwarn(f"Nieobsługiwany tryb: {mode}")

    # --------------- animacje ---------------

    def tick(self):
        self.ls.animation_tick()
        self.rs.animation_tick()
        self.f.animation_tick()
