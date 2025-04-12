from colorsys import hsv_to_rgb

class SideRight:
    
    def __init__(self):
        self.leds_count = 5
        #self.mode = mode
        
    def manual(self):
        data = []
        for i in range(self.leds_count):
            data.extend([65,105,225,255])
        return data
    
    def rainbow(self,base_hue):
        data = []
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