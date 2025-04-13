from colorsys import hsv_to_rgb

class Side:
    
    def __init__(self):
        self.leds_count = 5
        self.mode = ""

    def get_mode(self,mode):
        self.mode = mode
        
    def set_color(self, base_hue, reverse, on_off, brightness):
        if self.mode == "SIDE_BLUE":
            return self.side_blue()
        elif self.mode == "SIDE_BLUE_STROBE":
            return self.side_blue_strobe(on_off)
        elif self.mode == "SIDE_BLUE_BREATH":
            return self.side_blue_breath(brightness)
        elif self.mode == "SIDE_RED":
            return self.side_red()
        elif self.mode == "SIDE_RED_STROBE":
            return self.side_red_strobe(on_off)
        elif self.mode == "SIDE_RED_BREATH":
            return self.side_red_breath(brightness)
        elif self.mode == "SIDE_GREEN":
            return self.side_green()
        elif self.mode == "SIDE_GREEN_STROBE":
            return self.side_green_strobe(on_off)
        elif self.mode == "SIDE_GREEN_BREATH":
            return self.side_green_breath(brightness)
        elif self.mode == "SIDE_RAINBOW":
            return self.side_rainbow(base_hue,reverse)
        else:
            return self.side_off()

    def side_blue(self):
        data = [65,105,225,178]
        return (data*self.leds_count) 
    
    def side_blue_strobe(self, on_off):
        if on_off == True:
            data = [65,105,225,178]
        else:
            data = [65,105,225,0]
        return (data*self.leds_count) 
    
    def side_blue_breath(self, brightness):
        data = [65,105,225,brightness]
        return (data*self.leds_count) 
    
    def side_red(self):
        data = [255,0,0,178]
        return (data*self.leds_count) 
    
    def side_red_strobe(self, on_off):
        if on_off == True:
            data = [255,0,0,178]
        else:
            data = [255,0,0,0]
        return (data*self.leds_count) 
    
    def side_red_breath(self, brightness):
        data = [255,0,0,brightness]
        return (data*self.leds_count) 
    
    def side_green(self):
        data = [57,255,20,178]
        return (data*self.leds_count) 
    
    def side_green_strobe(self, on_off):
        if on_off == True:
            data = [57,255,20,178]
        else:
            data = [57,255,20,0]
        return (data*self.leds_count)
    
    def side_green_breath(self, brightness):
        data = [57,255,20,brightness]
        return (data*self.leds_count) 

    def side_off(self):
        data = [0,0,0,0]
        return (data*self.leds_count)
    
    def side_rainbow(self,base_hue,reverse):
        data = []
        if reverse == True:
            for i in range(self.leds_count):
                hue = (base_hue + ((i)/10)) % 1.0
                # Konwersja z HSV (pełne nasycenie i jasność) do RGB
                r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
                # Przeliczenie wartości do zakresu 0-255
                r_int = int(r * 255)
                g_int = int(g * 255)
                b_int = int(b * 255)
                a_int = 255  # pełna przezroczystość
                data.extend([r_int, g_int, b_int, a_int])
        else:
            for i in range(self.leds_count):
                hue = (base_hue + ((i+5)/10)) % 1.0
                # Konwersja z HSV (pełne nasycenie i jasność) do RGB
                r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
                # Przeliczenie wartości do zakresu 0-255
                r_int = int(r * 255)
                g_int = int(g * 255)
                b_int = int(b * 255)
                a_int = 255  # pełna przezroczystość
                data.extend([r_int, g_int, b_int, a_int])
        return data
    
    def full_rainbow(self,base_hue,reverse):
        data = []
        if reverse == True:
            for i in range(self.leds_count):
                hue = (base_hue + ((i)/18)) % 1.0
                # Konwersja z HSV (pełne nasycenie i jasność) do RGB
                r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
                # Przeliczenie wartości do zakresu 0-255
                r_int = int(r * 255)
                g_int = int(g * 255)
                b_int = int(b * 255)
                a_int = 255  # pełna przezroczystość
                data.extend([r_int, g_int, b_int, a_int])
        else:
            for i in range(self.leds_count):
                hue = (base_hue + ((i+9)/18)) % 1.0
                # Konwersja z HSV (pełne nasycenie i jasność) do RGB
                r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
                # Przeliczenie wartości do zakresu 0-255
                r_int = int(r * 255)
                g_int = int(g * 255)
                b_int = int(b * 255)
                a_int = 255  # pełna przezroczystość
                data.extend([r_int, g_int, b_int, a_int])
        return data