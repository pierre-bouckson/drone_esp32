#!/usr/bin/env python3
"""
Diagnostic manette : affiche en direct les axes et boutons vus par pygame.

Sert a trouver les bons index avant de les reporter dans gamepad_pilot.py
(AX_YAW / AX_THROTTLE / AX_ROLL / AX_PITCH, BTN_ARM / BTN_DISARM).

    ./venv/bin/python pad_test.py

Bouger un stick a la fois et noter quel numero d'axe reagit, et dans quel sens.
Ctrl-C pour quitter.
"""
import sys
import time

import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Aucune manette detectee.")
    print("  - manette bien branchee en USB (cable data, pas un cable de charge) ?")
    print("  - sur DualShock 4, la barre lumineuse doit etre allumee")
    print("  - macOS : Reglages > Confidentialite et securite > Surveillance des")
    print("    entrees, autoriser le Terminal, puis relancer le Terminal")
    sys.exit(1)

js = pygame.joystick.Joystick(0)
js.init()
print(f"Manette : {js.get_name()}")
print(f"  axes    : {js.get_numaxes()}")
print(f"  boutons : {js.get_numbuttons()}")
print(f"  hats    : {js.get_numhats()}")
print(f"  guid    : {js.get_guid()}")
print("\nBouger les commandes. Ctrl-C pour quitter.\n")

try:
    while True:
        pygame.event.pump()

        axes = " ".join(f"{i}:{js.get_axis(i):+.2f}" for i in range(js.get_numaxes()))
        btns = "".join(str(js.get_button(i)) for i in range(js.get_numbuttons()))
        hats = " ".join(str(js.get_hat(i)) for i in range(js.get_numhats()))

        # \r : on reecrit la meme ligne, la console reste lisible.
        sys.stdout.write(f"\rAXES {axes}  |  BTN {btns}  |  HAT {hats}   ")
        sys.stdout.flush()
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nfin")
finally:
    pygame.quit()
