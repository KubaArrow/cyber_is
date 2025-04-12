from cyber_is_led_controller.leds_front import Front
from cyber_is_led_controller.leds_side import Side
from cyber_is_led_controller.leds_back import Back
import rospy

class Modes:
    def __init__(self):
        side_leds = rospy.get_param("side_leds", 5)
        front_leds = rospy.get_param("front_leds", 4)
        back_leds = rospy.get_param("back_leds", 4)
        self.f = Front()
        self.s = Side()
        self.b = Back()
        self.base_hue = 0.0

    def rainbow(self):
        animation_speed = 0.005
        data = (self.s.full_rainbow(self.base_hue,True)+self.f.full_rainbow(self.base_hue)+
                self.s.full_rainbow(self.base_hue,False)+self.b.full_rainbow(self.base_hue))
        self.base_hue = (self.base_hue + animation_speed) % 1.0
        return data
    
    
    # sprawdza pelna tecza i zbiera wartosci z poszczegolnych klas ledow
    def get_leds(self):
        if self.mode == "FULL_RAINBOW":
            return self.rainbow()
        else:
            data = (self.s.set_color(True)+self.f.set_color()+
                    self.s.set_color(False)+self.b.set_color())
            return data
        
    # odczytuje wywolanie, zapisuje tryb, przekazuje do odpowiedniej klasy ledow
    def set_mode(self,mode): 
        self.mode = mode
        if "FRONT" in self.mode:
            self.f.get_mode(self.mode)
        elif "SIDE" in self.mode:
            self.s.get_mode(self.mode)
        elif "BACK" in self.mode:
            self.b.get_mode(self.mode)
