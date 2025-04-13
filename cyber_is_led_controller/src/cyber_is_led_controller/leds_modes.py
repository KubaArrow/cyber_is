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
        self.iteration = 0 #parametr do opozniania migania
        self.on_off = True #miganie wlacz/wylacz
        self.brightness = 178
        self.up_down = True
        self.animation_speed = 0.005

    def rainbow(self):
        data = (self.s.full_rainbow(self.base_hue,True)+self.f.full_rainbow(self.base_hue)+
                self.s.full_rainbow(self.base_hue,False)+self.b.full_rainbow(self.base_hue))
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
                    self.brightness += 1
                elif self.brightness == 255:
                    self.up_down = False
                    self.brightness -=1
                elif self.brightness == 0:
                    self.up_down = True
                    self.brightness +=1
                else:
                    self.brightness -=1
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
