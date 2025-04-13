from colorsys import hsv_to_rgb

class Front:
    
    def __init__(self, leds, all_leds):
        self.leds_count = leds
        self.all_leds = all_leds
        self.mode = ""

    def get_mode(self,mode):
        self.mode = mode
        
    def set_color(self):
        if self.mode == "FRONT_FULL":
            return self.front_full()
        elif self.mode == "FRONT_HALF":
            return self.front_half()
        elif self.mode == "FRONT_MIN":
            return self.front_min()
        elif self.mode == "FRONT_OFF":
            return self.front_off()
        else:
            return self.front_off()
        

    def front_full(self):
        data = [255,255,255,255]
        return (data*self.leds_count)
    
    def front_half(self):
        data = [255,255,255,122]
        return (data*self.leds_count)
    
    def front_min(self):
        data = [255,255,255,25]
        return (data*self.leds_count)
    
    def front_off(self):
        data = [255,255,255,0]
        return (data*self.leds_count)
    
    def full_rainbow(self,base_hue,leds_before):
        data = []
        for i in range(self.leds_count):
            hue = (base_hue + ((i+leds_before)/self.all_leds)) % 1.0
            # Konwersja z HSV (pełne nasycenie i jasność) do RGB
            r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
            # Przeliczenie wartości do zakresu 0-255
            r_int = int(r * 255)
            g_int = int(g * 255)
            b_int = int(b * 255)
            a_int = 255  # pełna przezroczystość
            data.extend([r_int, g_int, b_int, a_int])
        return data