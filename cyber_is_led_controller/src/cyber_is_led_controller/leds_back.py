from colorsys import hsv_to_rgb

class Back:
    
    def __init__(self,leds,all_leds):
        self.leds_count = leds
        self.all_leds = all_leds
        self.mode = ""

    def get_mode(self,mode):
        self.mode = mode
        
    def set_color(self):
        if self.mode == "BACK_FULL":
            return self.back_full()
        elif self.mode == "BACK_HALF":
            return self.back_half()
        elif self.mode == "BACK_OFF":
            return self.back_off()
        else:
            return self.back_off()

    def back_full(self):
        data = [255,0,0,255]
        return (data*self.leds_count)
    
    def back_half(self):
        data = [255,0,0,122]
        return (data*self.leds_count)
    
    def back_off(self):
        data = [255,0,0,0]
        return (data*self.leds_count)
    
    def full_rainbow(self,base_hue):
        data = []
        for i in range(self.leds_count):
            hue = (base_hue + ((i+(self.all_leds-self.leds_count))/self.all_leds)) % 1.0
            # Konwersja z HSV (pełne nasycenie i jasność) do RGB
            r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
            # Przeliczenie wartości do zakresu 0-255
            r_int = int(r * 255)
            g_int = int(g * 255)
            b_int = int(b * 255)
            a_int = 255  # pełna przezroczystość
            data.extend([r_int, g_int, b_int, a_int])
        return data