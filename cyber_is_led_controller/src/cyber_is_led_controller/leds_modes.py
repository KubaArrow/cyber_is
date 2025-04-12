from cyber_is_led_controller.leds_side_left import SideLeft
from cyber_is_led_controller.leds_front import Front
from cyber_is_led_controller.leds_side_right import SideRight
from cyber_is_led_controller.leds_back_right import BackRight
from cyber_is_led_controller.leds_back_left import BackLeft
import rospy

class Modes:
    def __init__(self):
        side_leds = rospy.get_param("side_leds", 5)
        front_leds = rospy.get_param("front_leds", 4)
        back_leds = rospy.get_param("back_leds", 4)
        self.sl = SideLeft()
        self.f = Front()
        self.sr = SideRight()
        self.br = BackRight()
        self.bl = BackLeft()
        self.base_hue = 0.0
        self.on_off = True
    
    def manual(self):
        data = (self.sl.manual()+self.f.manual()+self.sr.manual())
        if True:
            data = self.turn_left(data)
        else:
            data = data + self.br.manual()+self.bl.manual()
        return data

    def rainbow(self):
        animation_speed = 0.005
        data = (self.sl.rainbow(self.base_hue)+self.f.rainbow(self.base_hue)+
                self.sr.rainbow(self.base_hue)+self.br.rainbow(self.base_hue)+
                self.bl.rainbow(self.base_hue))
        self.base_hue = (self.base_hue + animation_speed) % 1.0
        return data
    
    def turn_left(self,data):
        t_data = data + self.br.manual()+self.bl.turn(self.on_off)
        self.on_off = -self.on_off
        return t_data
    
    def get_leds(self,mode):
        if mode == "MANUAL_MODE":
            return self.manual()
        elif mode == "FINAL":
            return self.rainbow()
        
    def set_mode(self,mode):
        if mode == "MANUAL_MODE":
            rospy.loginfo("Manual Animator uruchomiony.")
            return 2
        elif mode == "FINAL":
            rospy.loginfo("Final Animator uruchomiony.")
            return 30