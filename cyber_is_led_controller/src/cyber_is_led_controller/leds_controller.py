from cyber_is_led_controller.leds_front import Front
from cyber_is_led_controller.leds_side import Side
import logging

class LedsController:
    def __init__(self, side_leds: int, front_leds: int):
        self.side_leds = int(side_leds)
        self.front_leds = int(front_leds)

        self.f = Front(self.front_leds)
        self.ls = Side(self.side_leds, True)
        self.rs = Side(self.side_leds, False)

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
            logging.getLogger(__name__).warning(f"Nieobsługiwany tryb: {mode}")

    # --------------- animacje ---------------

    def tick(self):
        self.ls.animation_tick()
        self.rs.animation_tick()
        self.f.animation_tick()
