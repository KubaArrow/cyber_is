#!/usr/bin/env python3
"""
Front‑LED controller for Cyber‑IS mobile robot (v2.1)
===================================================

## Statyczne tryby  `FRONT_<COLOR>_<INT>`
`<COLOR>` = RED / GREEN / BLUE / **WHITE**   `<INT>` = 100 / 75 / 50 / 25

## Animacje  `FRONT_<EFFECT>` lub `FRONT_<EFFECT>_<COLOR>`
| Efekt | Opis |
|-------|------|
| **KITT_<COLOR>** | 3‑diodowy skaner ping‑pong 50‑100‑50 % (RED / GREEN / BLUE) |
| **<COLOR>_BREATH** | Oddychanie sinusoidą |
| **<COLOR>_BLINK** | Szybkie miganie on/off |
| **RED_SOS** | Morse SOS (… ― …) czerwony |

### Polecenia bash
```bash
# biały statyczny 75 %
rostopic pub -1 /leds_mode std_msgs/String "data: 'FRONT_WHITE_75'"

# niebieski KITT
rostopic pub -1 /leds_mode std_msgs/String "data: 'FRONT_KITT_BLUE'"
```
"""

from enum import Enum
from math import sin, pi
from typing import List, Tuple, Optional

RGB = Tuple[int, int, int]

# ───────────────────────────────────────────────────────────────────────────────
#  Enum trybów
# ───────────────────────────────────────────────────────────────────────────────
class FrontLedMode(Enum):
    FRONT_OFF = (0, 0, 0)

    # statyczne RGBW
    FRONT_RED_100 = (255, 0, 0)
    FRONT_RED_75  = (191, 0, 0)
    FRONT_RED_50  = (128, 0, 0)
    FRONT_RED_25  = (64,  0, 0)

    FRONT_GREEN_100 = (0, 255, 0)
    FRONT_GREEN_75  = (0, 191, 0)
    FRONT_GREEN_50  = (0, 128, 0)
    FRONT_GREEN_25  = (0,  64, 0)

    FRONT_BLUE_100 = (0, 0, 255)
    FRONT_BLUE_75  = (0, 0, 191)
    FRONT_BLUE_50  = (0, 0, 128)
    FRONT_BLUE_25  = (0, 0,  64)

    FRONT_WHITE_100 = (255, 255, 255)
    FRONT_WHITE_75  = (191, 191, 191)
    FRONT_WHITE_50  = (128, 128, 128)
    FRONT_WHITE_25  = (64,  64,  64)

    # animacje – sentinele
    FRONT_KITT_RED   = (-1, -1, -1)
    FRONT_KITT_GREEN = (-1, -1, -2)
    FRONT_KITT_BLUE  = (-1, -1, -3)

    FRONT_RED_BREATH   = (-2, -2, -2)
    FRONT_GREEN_BREATH = (-2, -2, -3)
    FRONT_BLUE_BREATH  = (-2, -2, -4)

    FRONT_RED_BLINK   = (-3, -3, -3)
    FRONT_GREEN_BLINK = (-3, -3, -4)
    FRONT_BLUE_BLINK  = (-3, -3, -5)

    FRONT_RED_SOS = (-4, -4, -4)

# wsteczny alias
FrontLedMode.FRONT_KITT = FrontLedMode.FRONT_KITT_RED

# ───────────────────────────────────────────────────────────────────────────────
#  Klasy animacji – bez zmian
# ───────────────────────────────────────────────────────────────────────────────
class KittAnimation:
    _REL = (0.5, 1.0, 0.5)
    def __init__(self, n: int, color: RGB):
        self.n, self.c, self.center, self.dir = n, color, 0, 1
    def next_frame(self) -> List[RGB]:
        leds = [(0, 0, 0)] * self.n
        for off, rel in zip((-1, 0, 1), self._REL):
            idx = self.center + off
            if 0 <= idx < self.n:
                r, g, b = self.c
                leds[idx] = (int(r*rel), int(g*rel), int(b*rel))
        self.center += self.dir
        if self.center in (0, self.n-1):
            self.dir *= -1
        return leds

class BreathAnimation:
    def __init__(self, n: int, c: RGB, p: int = 60):
        self.n, self.c, self.p, self.t = n, c, p, 0
    def next_frame(self):
        br = 0.5*(1+sin(2*pi*self.t/self.p)); self.t=(self.t+1)%self.p
        r,g,b=self.c; scale=lambda x:int(x*br)
        return [(scale(r),scale(g),scale(b))]*self.n

class BlinkAnimation:
    def __init__(self, n:int,c:RGB):
        self.n,self.c,self.on=n,c,False
    def next_frame(self):
        self.on=not self.on
        return [self.c if self.on else (0,0,0)]*self.n

class SOSAnimation:
    _U=4; _PAT=[1,0]*3+[1]*3+[0]+[1,0]*3
    def __init__(self,n:int,c:RGB):
        self.n,self.c=n,c
        self.seq=[b for bit in self._PAT for b in [bit]*self._U]
        self.i=0
    def next_frame(self):
        bit=self.seq[self.i]; self.i=(self.i+1)%len(self.seq)
        return [self.c if bit else (0,0,0)]*self.n

# ───────────────────────────────────────────────────────────────────────────────
#  Controller
# ───────────────────────────────────────────────────────────────────────────────
class FrontLedsController:
    _COLOR_BASE={"RED":(255,0,0),"GREEN":(0,255,0),"BLUE":(0,0,255)}
    def __init__(self,n:int):
        self.n=n; self._mode=FrontLedMode.FRONT_OFF; self._anim=None
        self._leds=[(0,0,0)]*n
    def set_mode(self,name:str):
        if name not in FrontLedMode.__members__:
            raise ValueError(f"Nieznany tryb: {name}")
        self._mode=FrontLedMode[name]
        if name.startswith("FRONT_KITT"):
            color_key=name.split("_")[-1]; self._anim=KittAnimation(self.n,self._COLOR_BASE[color_key])
        elif name.endswith("_BREATH"):
            self._anim=BreathAnimation(self.n,self._COLOR_BASE[name.split("_")[1]])
        elif name.endswith("_BLINK"):
            self._anim=BlinkAnimation(self.n,self._COLOR_BASE[name.split("_")[1]])
        elif name=="FRONT_RED_SOS":
            self._anim=SOSAnimation(self.n,self._COLOR_BASE["RED"])
        elif name.count("_")==2 and name.startswith("FRONT_"):
            _,col,perc=name.split("_");f=int(perc)/100
            if col=="WHITE":
                self._leds=[(int(255*f),)*3]*self.n
            else:
                r,g,b=self._COLOR_BASE[col];self._leds=[(int(r*f),int(g*f),int(b*f))]*self.n
            self._anim=None;return
        else:
            self._anim=None; self._leds=[self._mode.value]*self.n;return
        self._leds=self._anim.next_frame()
    def animation_tick(self):
        if self._anim: self._leds=self._anim.next_frame()
    def get_leds(self):return list(self._leds)
    def get_leds_rgba(self):return [b for (r,g,b) in self._leds for b in (r,g,b,255)]
    def __repr__(self):return f"FrontLedsController({self.n} LED, {self._mode.name})"

class Front:
    def __init__(self,n:int):
        self.ctrl=FrontLedsController(n)
        self.ctrl.set_mode("FRONT_WHITE_25")
    def set_mode(self,m:str):self.ctrl.set_mode(m)
    def animation_tick(self):self.ctrl.animation_tick()
    def get_leds_rgba(self):return self.ctrl.get_leds_rgba()
