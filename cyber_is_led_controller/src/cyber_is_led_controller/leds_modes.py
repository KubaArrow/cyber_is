from cyber_is_led_controller.leds_front import Front
from cyber_is_led_controller.leds_side import Side
from cyber_is_led_controller.leds_back import Back
import rospy

class Modes:
    def __init__(self):
        self.side_leds = rospy.get_param("/led_controller/side_leds", 9)
        self.front_leds = rospy.get_param("/led_controller/front_leds", 5)
        self.back_leds = rospy.get_param("/led_controller/back_leds", 0)
        all_leds = self.side_leds*2 + self.front_leds + self.back_leds
        self.f = Front(self.front_leds, all_leds)
        self.s = Side(self.side_leds, all_leds)
        self.b = Back(self.back_leds, all_leds)
        self.iteration = 0 #parametr do opozniania migania
        self.on_off = True #miganie wlacz/wylacz
        self.brightness = 178
        self.up_down = True
        self.animation_speed = 0.005
        self.base_hue = 0.0
        self.mode=""

    def rainbow(self):
        
        data = (self.s.full_rainbow(self.base_hue,True,self.front_leds)+self.f.full_rainbow(self.base_hue,self.side_leds)+
                self.s.full_rainbow(self.base_hue,False,self.front_leds)+self.b.full_rainbow(self.base_hue))
        self.base_hue = (self.base_hue + self.animation_speed) % 1.0
        return data
    
    # ustawia wartosc ledow
    def get_leds(self):
        if self.mode == "FULL_RAINBOW": #sprawdza czy full_rainbow - osobna funkcja
            return self.rainbow()
        else:
            if "STROBE" in self.mode: # steruje miganiem
                if self.iteration == 9:
                    self.on_off = -self.on_off
                    self.iteration = 0
                self.iteration += 1
            elif "BREATH" in self.mode: # steruje zmianami jasnosci
                if self.up_down == True and self.brightness < 255:
                    self.brightness += 3
                elif self.brightness == 255:
                    self.up_down = False
                    self.brightness -=3
                elif self.brightness == 0:
                    self.up_down = True
                    self.brightness +=3
                else:
                    self.brightness -=3
            data = (self.s.set_color(self.base_hue,True,self.on_off,self.brightness)+self.f.set_color()+ #zbiera wartosci ze wszystkich klas
                    self.s.set_color(self.base_hue,False,self.on_off,self.brightness)+self.b.set_color())
            if self.mode == "SIDE_RAINBOW":
                self.base_hue = (self.base_hue + self.animation_speed) % 1.0
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
