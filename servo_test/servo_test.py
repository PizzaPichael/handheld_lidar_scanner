#!/usr/bin/env python3
"""Testet einen SG90 Micro-Servo per Pfeiltasten am Raspberry Pi 5.

Verkabelung (SG90 → Raspberry Pi 5 Header):
    Orange (Signal) → Pin 12 (GPIO18)
    Rot    (5 V)    → Pin 2  (5 V)   (Pin 4 geht auch)
    Braun  (GND)    → Pin 14 (GND)   (Pin 6 ist schon von der MPU6050 belegt)

Hinweis: Servo zieht beim Anlaufen Stromspitzen. Bei Pi-Resets oder Zittern
→ externe 5-V-Quelle für Rot/Braun nutzen und deren GND mit Pi-GND verbinden.

Abhängigkeiten (Ubuntu 24.04):
    sudo apt install python3-gpiozero python3-lgpio

Start:
    python3 servo_test.py      (bei "Permission denied" mit sudo)

Bedienung: Pfeil hoch/runter = Winkel ändern, q = beenden.
"""

import curses

from gpiozero import AngularServo

SIGNAL_GPIO = 18
STEP_DEG = 10
MIN_ANGLE = 0
MAX_ANGLE = 180

# SG90 erwartet Pulse von ca. 0,5 ms (0°) bis 2,4 ms (180°) alle 20 ms.
# gpiozero-Default ist 1–2 ms → ohne diese Werte nur ca. 90° Stellweg.
MIN_PULSE_S = 0.5 / 1000
MAX_PULSE_S = 2.4 / 1000


def run(stdscr):
    servo = AngularServo(
        SIGNAL_GPIO,
        min_angle=MIN_ANGLE,
        max_angle=MAX_ANGLE,
        min_pulse_width=MIN_PULSE_S,
        max_pulse_width=MAX_PULSE_S,
    )

    angle = (MIN_ANGLE + MAX_ANGLE) // 2
    servo.angle = angle

    stdscr.addstr(0, 0, "Pfeil hoch/runter: Winkel aendern, q: beenden")
    try:
        while True:
            # Pulsbreite linear zwischen MIN_PULSE_S und MAX_PULSE_S interpoliert
            pulse_ms = (MIN_PULSE_S + (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE)
                        * (MAX_PULSE_S - MIN_PULSE_S)) * 1000
            stdscr.addstr(2, 0, f"Winkel: {angle:3d} deg   Puls: {pulse_ms:.2f} ms   ")
            stdscr.refresh()

            # getch() blockiert, bis Taste gedrückt wird
            key = stdscr.getch()
            if key == curses.KEY_UP:
                angle = min(angle + STEP_DEG, MAX_ANGLE)
            elif key == curses.KEY_DOWN:
                angle = max(angle - STEP_DEG, MIN_ANGLE)
            elif key in (ord("q"), ord("Q")):
                break
            servo.angle = angle
    finally:
        # Signal abschalten → Servo hält nicht mehr aktiv, kein Brummen
        servo.detach()
        servo.close()


if __name__ == "__main__":
    # wrapper stellt Terminal nach Ende/Exception wieder her
    curses.wrapper(run)
