#!/usr/bin/env python3
"""
Side‑LED controller for Cyber‑IS robot (v2)
==========================================

**Statyczne tryby**  (nazwy ➜ `SIDE_<COLOR>_<INT>`)
    • kolory: RED / GREEN / BLUE / ORANGE
    • <INT>  : 100 / 75 / 50 / 25 (%)

**Animacje**  (nazwy ➜ `SIDE_<COLOR>_<EFFECT>`) – dostępne efekty
    BREATH   – płynne oddychanie
    BLINK    – szybkie miganie on/off
    TRIPLE   – pakiet 3 diodek 50‑100‑50 % sunie w górę (↕) kolumny
    LOAD     – ładowanie / rozładowanie (teraz 2× wolniej)
    WAVE     – fala sinusoidy wędruje w górę
    CHAOS    – losowe iskierki (20 % LED‑ów zapalone co tick)
    HAZARD   – naprzemienne błyski (duty ≈ ½)

---
**Przykładowe polecenia bash**  (publikacja na topic `/leds_mode`)
```
# fala zielona
rostopic pub -1 /leds_mode std_msgs/String "data: 'SIDE_BLUE_WAVE'"

# losowe błyski pomarańczowe
rostopic pub -1 /leds_mode std_msgs/String "data: 'SIDE_ORANGE_CHAOS'"

# efekt hazard (żółto‑pomarańczowy) – lewa i prawa naprzemiennie
rostopic pub -1 /leds_mode std_msgs/String "data: 'SIDE_ORANGE_HAZARD'"
```
"""

from math import sin, pi
from random import random, randint
from typing import List, Tuple, Optional

RGB = Tuple[int, int, int]

# ────────────────────────────────────────────────
#  Animacje pomocnicze
# ────────────────────────────────────────────────
class BreathingAnimation:
    """Oddychanie sinusoidalne całego paska."""
    def __init__(self, n: int, color: RGB, period: int = 60):
        self.n, self.c, self.p, self.t = n, color, period, 0

    def next_frame(self) -> List[RGB]:
        br = 0.5 * (1 + sin(2 * pi * self.t / self.p))
        self.t = (self.t + 1) % self.p
        scale = lambda x: int(x * br)
        r, g, b = self.c
        return [(scale(r), scale(g), scale(b))] * self.n


class BlinkFastAnimation:
    """Szybkie miganie – co klatkę on/off."""
    def __init__(self, n: int, color: RGB):
        self.n, self.c, self.on = n, color, False

    def next_frame(self) -> List[RGB]:
        self.on = not self.on
        return [self.c if self.on else (0, 0, 0)] * self.n


class TripleScrollAnimation:
    """Pakiet 3 LED‑ów (50‑100‑50 %) sunie w górę/ w dół kolumny."""
    _REL = (0.5, 1.0, 0.5)

    def __init__(self, n: int, color: RGB, is_left: bool):
        self.n, self.c, self.is_left = n, color, is_left
        self.head = -3 if is_left else n

    def next_frame(self) -> List[RGB]:
        leds = [(0, 0, 0)] * self.n
        for o, rel in enumerate(self._REL):
            idx = self.head + o if self.is_left else self.head - o
            if 0 <= idx < self.n:
                r, g, b = self.c
                leds[idx] = (int(r * rel), int(g * rel), int(b * rel))
        self.head += 1 if self.is_left else -1
        if self.is_left and self.head >= self.n:
            self.head = -3
        if not self.is_left and self.head < -3:
            self.head = self.n
        return leds


class LoadUnloadAnimation:
    """Ładowanie / rozładowanie – **2× wolniej** niż dotąd."""
    def __init__(self, n: int, color: RGB):
        self.n, self.c = n, color
        self.progress, self.dir, self.tick = 0, 1, 0

    def next_frame(self) -> List[RGB]:
        # aktualizuj co drugi tick
        if self.tick % 2 == 0:
            self.progress += self.dir
            if self.progress == self.n or self.progress == 0:
                self.dir *= -1
        self.tick += 1
        return [self.c] * self.progress + [(0, 0, 0)] * (self.n - self.progress)


class WaveAnimation:
    """Pionowa fala sinusoidalna przemieszczająca się w górę kolumny."""
    def __init__(self, n: int, color: RGB, is_left: bool, wavelength: int = 8):
        self.n, self.c, self.is_left = n, color, is_left
        self.phase, self.wl = 0, wavelength

    def next_frame(self) -> List[RGB]:
        leds = []
        for i in range(self.n):
            # kierunek zależy od strony (symetria)
            idx = i if self.is_left else (self.n - 1 - i)
            val = 0.5 * (1 + sin(2 * pi * (idx + self.phase) / self.wl))  # 0..1
            r, g, b = self.c
            leds.append((int(r * val), int(g * val), int(b * val)))
        self.phase = (self.phase + 1) % self.wl
        return leds if self.is_left else list(reversed(leds))


class ChaosAnimation:
    """Losowe iskry – ~20 % LED‑ów zapalone pełną jasnością."""
    def __init__(self, n: int, color: RGB):
        self.n, self.c = n, color

    def next_frame(self) -> List[RGB]:
        return [self.c if random() < 0.2 else (0, 0, 0) for _ in range(self.n)]


class HazardAnimation:
    """Błysk/ciemność z duty≈½; lewa & prawa błyskają synchronicznie."""
    def __init__(self, n: int, color: RGB):
        self.n, self.c, self.t = n, color, 0

    def next_frame(self) -> List[RGB]:
        self.t += 1
        on = (self.t // 8) % 2 == 0  # 8 ticków on, 8 off
        return [self.c if on else (0, 0, 0)] * self.n


# ────────────────────────────────────────────────
#  Główna klasa Side
# ────────────────────────────────────────────────
class Side:
    """Sterownik bocznego pionowego paska LED‑ów."""
    _COLOR_BASE = {
        "RED":    (255, 0, 0),
        "GREEN":  (0, 255, 0),
        "BLUE":   (0, 0, 255),
        "ORANGE": (255, 160, 0),
    }
    _PERCENT = {"100": 1.0, "75": 0.75, "50": 0.5, "25": 0.25}

    def __init__(self, num_leds: int, is_left: bool):
        self.n, self.is_left = num_leds, is_left
        self._leds: List[RGB] = [(0, 0, 0)] * num_leds
        self._anim: Optional[object] = None
        self.set_mode("SIDE_GREEN_50")

    # -------------------------------------------------------------- set_mode --
    def set_mode(self, mode: str) -> None:
        if not mode.startswith("SIDE_"):
            raise ValueError("Tryb musi zaczynać się od 'SIDE_'")

        parts = mode.split("_")
        _, color_key, *rest = parts
        color_key = color_key.upper()
        if color_key not in self._COLOR_BASE:
            raise ValueError(f"Nieznany kolor: {color_key}")
        base = self._COLOR_BASE[color_key]

        # ---------- statyczne ----------
        if rest and rest[0] in self._PERCENT:
            f = self._PERCENT[rest[0]]
            scale = lambda x: int(x * f)
            self._anim = None
            r, g, b = base
            self._leds = [(scale(r), scale(g), scale(b))] * self.n
            return

        tag = "_".join(rest).upper()
        if tag == "BREATH":
            self._anim = BreathingAnimation(self.n, base)
        elif tag == "BLINK":
            self._anim = BlinkFastAnimation(self.n, base)
        elif tag == "TRIPLE":
            self._anim = TripleScrollAnimation(self.n, base, self.is_left)
        elif tag == "LOAD":
            self._anim = LoadUnloadAnimation(self.n, base)
        elif tag == "WAVE":
            self._anim = WaveAnimation(self.n, base, self.is_left)
        elif tag == "CHAOS":
            self._anim = ChaosAnimation(self.n, base)
        elif tag == "HAZARD":
            self._anim = HazardAnimation(self.n, base)
        else:
            raise ValueError(f"Nieznany tryb: {mode}")

        self._leds = self._anim.next_frame()

    # ------------------------------------------------------------ animation --
    def animation_tick(self):
        if self._anim:
            self._leds = self._anim.next_frame()

    # -------------------------------------------------------------- publish --
    def get_leds_rgba(self) -> List[int]:
        arr = self._leds if self.is_left else list(reversed(self._leds))
        return [byte for (r, g, b) in arr for byte in (r, g, b, 255)]

    def __repr__(self):
        return f"<Side {'LEFT' if self.is_left else 'RIGHT'}, {self.n} LED-ów>"
