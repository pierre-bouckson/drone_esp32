#!/usr/bin/env python3
"""
drone_pilot.py — Contrôle d'un drone Tello sans ROS
Lit la manette via pygame, envoie des commandes UDP au drone.

Dépendances : pip install pygame
Lancement   : python drone_pilot.py
"""

import socket
import threading
import time
import os
import queue
import tkinter as tk
from tkinter import simpledialog
import pygame

# ─── Configuration réseau ─────────────────────────────────────────────────────
TELLO_IP           = '192.168.10.1'   # IP par défaut du Tello en mode AP
TELLO_COMMAND_PORT = 8889
SELF_PORT          = 8890
COMMAND_TIMEOUT    = 3.0
RC_INTERVAL        = 0.05             # 20 Hz

# ─── Mapping manette Xbox One (identique à tello_joy_node.hpp) ───────────────
JOY_AXIS_YAW      = 0   # Stick gauche G/D  → rotation
JOY_AXIS_THROTTLE = 1   # Stick gauche H/B  → montée/descente
JOY_AXIS_ROLL     = 3   # Stick droit  G/D  → gauche/droite (strafe)
JOY_AXIS_PITCH    = 4   # Stick droit  H/B  → avance/recule
JOY_BTN_TAKEOFF   = 7   # Bouton Menu       → décollage
JOY_BTN_LAND      = 6   # Bouton View       → atterrissage
NUM_AXES          = 6
NUM_BUTTONS       = 11

# ─── Couleurs ─────────────────────────────────────────────────────────────────
_DARK_BG  = '#1e1e2e'
_PANEL_BG = '#2a2a3e'
_ACCENT   = '#7c7cff'
_TEXT     = '#cdd6f4'
_GREEN    = '#a6e3a1'
_RED      = '#f38ba8'
_ORANGE   = '#fab387'
_YELLOW   = '#f9e2af'
_GREY     = '#45475a'


# ══════════════════════════════════════════════════════════════════════════════
#  Couche réseau UDP
# ══════════════════════════════════════════════════════════════════════════════

class UDPLink:
    def __init__(self, port: int):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('', port))

        self._response: tuple[str, str] | None = None  # (message, 'new'|'old')
        self._lock = threading.Lock()

        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True, name='udp-recv').start()

    def send(self, command: str, ip: str) -> None:
        self._sock.sendto(command.encode('utf-8'), (ip, TELLO_COMMAND_PORT))

    def send_wait(self, command: str, ip: str, timeout: float = COMMAND_TIMEOUT) -> str | None:
        with self._lock:
            self._response = None

        self._sock.sendto(command.encode('utf-8'), (ip, TELLO_COMMAND_PORT))

        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._response and self._response[1] == 'new':
                    msg = self._response[0]
                    self._response = (msg, 'old')
                    return msg
            time.sleep(0.01)
        return None

    def _recv_loop(self):
        self._sock.settimeout(1.0)
        while self._running:
            try:
                data, _ = self._sock.recvfrom(1024)
                msg = data.decode('utf-8', errors='replace').strip()
                with self._lock:
                    self._response = (msg, 'new')
            except socket.timeout:
                pass
            except Exception:
                break

    def close(self):
        self._running = False
        self._sock.close()


# ══════════════════════════════════════════════════════════════════════════════
#  Drone
# ══════════════════════════════════════════════════════════════════════════════

class Drone:
    def __init__(self, ip: str, link: UDPLink):
        self.ip          = ip
        self.link        = link
        self.connected   = False
        self.rc_enabled  = False
        self.battery     = '?'
        self.log_queue: queue.Queue[tuple[str, str]] = queue.Queue()

    def init(self) -> bool:
        resp = self.link.send_wait('command', self.ip)
        self.connected = (resp == 'ok')
        if self.connected:
            self._log(f'Connecté ({self.ip})', 'recv')
            self._refresh_battery()
        else:
            self._log(f'Pas de réponse ({self.ip})', 'recv')
        return self.connected

    def takeoff(self):
        if not self.connected:
            return
        self._log('→ takeoff', 'action')
        self.link.send_wait('command', self.ip)
        time.sleep(0.5)
        resp = self.link.send_wait('takeoff', self.ip, timeout=10.0)
        if resp == 'ok':
            self.rc_enabled = True
            self._log('Décollage OK', 'recv')
        else:
            self._log(f'Échec décollage: {resp}', 'recv')

    def land(self):
        self.rc_enabled = False
        self._log('→ land', 'action')
        resp = self.link.send_wait('land', self.ip, timeout=10.0)
        self._log(f'Atterrissage: {resp}', 'recv')

    def emergency(self):
        self.rc_enabled = False
        self.link.send('emergency', self.ip)
        self._log('!!! EMERGENCY !!!', 'action')

    def send_rc(self, roll: int, pitch: int, throttle: int, yaw: int):
        if not self.rc_enabled:
            self.link.send('rc 0 0 0 0', self.ip)
            return
        self.link.send(f'rc {roll} {pitch} {throttle} {yaw}', self.ip)

    def poll_battery(self):
        resp = self.link.send_wait('battery?', self.ip)
        if resp:
            self.battery = resp
            self._log(f'Batterie: {resp}%', 'recv')

    def _log(self, msg: str, tag: str = 'send'):
        self.log_queue.put((msg, tag))


# ══════════════════════════════════════════════════════════════════════════════
#  Lecture manette (thread pygame)
# ══════════════════════════════════════════════════════════════════════════════

class JoystickReader:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self.joy: pygame.joystick.JoystickType | None = None
        self.axes:    list[float] = [0.0] * NUM_AXES
        self.buttons: list[int]   = [0]   * NUM_BUTTONS
        self.available = False
        self.name = 'Aucune manette détectée'

        self._lock = threading.Lock()
        self._try_connect()

        self._running = True
        threading.Thread(target=self._loop, daemon=True, name='joystick').start()

    def _try_connect(self):
        if pygame.joystick.get_count() > 0:
            self.joy = pygame.joystick.Joystick(0)
            self.joy.init()
            self.available = True
            self.name = self.joy.get_name()
            self.axes    = [0.0] * max(self.joy.get_numaxes(),    NUM_AXES)
            self.buttons = [0]   * max(self.joy.get_numbuttons(), NUM_BUTTONS)

    def get_state(self) -> tuple[list[float], list[int]]:
        with self._lock:
            return list(self.axes), list(self.buttons)

    def get_rc(self) -> tuple[int, int, int, int]:
        axes, _ = self.get_state()

        def ax(i):
            return axes[i] if i < len(axes) else 0.0

        def clamp(v):
            return int(max(min(v * 100, 100), -100))

        roll     = clamp(-ax(JOY_AXIS_ROLL))
        pitch    = clamp( ax(JOY_AXIS_PITCH))
        throttle = clamp( ax(JOY_AXIS_THROTTLE))
        yaw      = clamp( ax(JOY_AXIS_YAW))
        return roll, pitch, throttle, yaw

    def _loop(self):
        while self._running:
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED and not self.available:
                    self._try_connect()
                elif event.type == pygame.JOYDEVICEREMOVED:
                    self.available = False
                    self.name = 'Manette déconnectée'
                    self.joy  = None
                    with self._lock:
                        self.axes    = [0.0] * NUM_AXES
                        self.buttons = [0]   * NUM_BUTTONS

            if self.joy and self.available:
                with self._lock:
                    for i in range(min(self.joy.get_numaxes(), len(self.axes))):
                        self.axes[i] = self.joy.get_axis(i)
                    for i in range(min(self.joy.get_numbuttons(), len(self.buttons))):
                        self.buttons[i] = self.joy.get_button(i)
            time.sleep(0.02)

    def stop(self):
        self._running = False
        pygame.quit()


# ══════════════════════════════════════════════════════════════════════════════
#  Interface graphique tkinter
# ══════════════════════════════════════════════════════════════════════════════

class JoystickPanel:
    _STICK_R = 42
    _DOT_R   = 6

    def __init__(self, parent: tk.Widget, joy: JoystickReader):
        self.joy = joy

        self.frame = tk.LabelFrame(
            parent, text=' Manette ',
            fg=_ACCENT, bg=_PANEL_BG,
            font=('Consolas', 10, 'bold'),
            padx=8, pady=6, relief='groove', bd=2
        )

        self._name_var = tk.StringVar(value=joy.name)
        tk.Label(self.frame, textvariable=self._name_var,
                 fg=_YELLOW, bg=_PANEL_BG, font=('Consolas', 9)).pack(anchor='w')

        sticks_row = tk.Frame(self.frame, bg=_PANEL_BG)
        sticks_row.pack(pady=4)

        self._left_canvas,  self._left_dot  = self._make_stick(sticks_row, 'Stick G\nYaw / Throttle')
        self._right_canvas, self._right_dot = self._make_stick(sticks_row, 'Stick D\nRoll / Pitch')

        # Valeurs numériques
        vals = tk.Frame(self.frame, bg=_PANEL_BG)
        vals.pack(fill=tk.X, pady=2)
        self._val_vars: dict[str, tk.StringVar] = {}
        for name, color in [('Throttle', _ACCENT), ('Yaw', _ACCENT),
                             ('Roll', _GREEN),      ('Pitch', _GREEN)]:
            f = tk.Frame(vals, bg=_PANEL_BG)
            f.pack(side=tk.LEFT, padx=10)
            tk.Label(f, text=name, fg=color, bg=_PANEL_BG,
                     font=('Consolas', 8, 'bold')).pack()
            v = tk.StringVar(value='  0%')
            self._val_vars[name] = v
            tk.Label(f, textvariable=v, fg=_TEXT, bg=_PANEL_BG,
                     font=('Consolas', 11)).pack()

        # Indicateurs boutons
        btn_row = tk.Frame(self.frame, bg=_PANEL_BG)
        btn_row.pack(pady=4)
        self._btn_leds: dict[int, tk.Canvas] = {}
        btn_labels = {JOY_BTN_TAKEOFF: 'Menu\n(Takeoff)', JOY_BTN_LAND: 'View\n(Land)'}
        for i in range(NUM_BUTTONS):
            col = tk.Frame(btn_row, bg=_PANEL_BG)
            col.pack(side=tk.LEFT, padx=3)
            tk.Label(col, text=btn_labels.get(i, f'B{i}'),
                     fg=_GREY, bg=_PANEL_BG,
                     font=('Consolas', 7), justify='center').pack()
            c = tk.Canvas(col, width=16, height=16, bg=_PANEL_BG, highlightthickness=0)
            c.pack()
            c.create_oval(2, 2, 14, 14, fill=_GREY, outline='', tags='dot')
            self._btn_leds[i] = c

        self.frame.after(50, self._refresh)

    def _make_stick(self, parent, label):
        f = tk.Frame(parent, bg=_PANEL_BG)
        f.pack(side=tk.LEFT, padx=14)
        tk.Label(f, text=label, fg=_TEXT, bg=_PANEL_BG,
                 font=('Consolas', 8), justify='center').pack()
        r = self._STICK_R
        size = r * 2 + 10
        c = tk.Canvas(f, width=size, height=size,
                      bg='#11111b', highlightthickness=1, highlightbackground=_GREY)
        c.pack()
        cx = cy = size // 2
        c.create_oval(cx-r, cy-r, cx+r, cy+r, outline=_GREY, fill='#11111b')
        c.create_line(cx-r, cy, cx+r, cy, fill=_GREY)
        c.create_line(cx, cy-r, cx, cy+r, fill=_GREY)
        d = self._DOT_R
        dot = c.create_oval(cx-d, cy-d, cx+d, cy+d, fill=_ACCENT, outline='')
        return c, dot

    def _move_dot(self, canvas, dot, x: float, y: float):
        r = self._STICK_R
        d = self._DOT_R
        size = r * 2 + 10
        cx = cy = size // 2
        px = cx + int(x * r)
        py = cy - int(y * r)
        canvas.coords(dot, px-d, py-d, px+d, py+d)

    def _refresh(self):
        axes, buttons = self.joy.get_state()

        def ax(i):
            return axes[i] if i < len(axes) else 0.0

        yaw      = ax(JOY_AXIS_YAW)
        throttle = ax(JOY_AXIS_THROTTLE)
        roll     = ax(JOY_AXIS_ROLL)
        pitch    = ax(JOY_AXIS_PITCH)

        self._move_dot(self._left_canvas,  self._left_dot,  yaw,  throttle)
        self._move_dot(self._right_canvas, self._right_dot, roll, pitch)

        self._val_vars['Throttle'].set(f'{throttle*100:+5.0f}%')
        self._val_vars['Yaw'].set(     f'{yaw*100:+5.0f}%')
        self._val_vars['Roll'].set(    f'{roll*100:+5.0f}%')
        self._val_vars['Pitch'].set(   f'{pitch*100:+5.0f}%')

        btn_colors = {JOY_BTN_TAKEOFF: _GREEN, JOY_BTN_LAND: _RED}
        for i, c in self._btn_leds.items():
            pressed = i < len(buttons) and bool(buttons[i])
            c.itemconfig('dot', fill=btn_colors.get(i, _ACCENT) if pressed else _GREY)

        self._name_var.set(self.joy.name)
        self.frame.after(50, self._refresh)


class DronePanel:
    def __init__(self, parent: tk.Widget, drone: Drone, takeoff_cb, land_cb, emergency_cb):
        self.drone = drone

        self.frame = tk.LabelFrame(
            parent, text=' Drone ',
            fg=_ACCENT, bg=_PANEL_BG,
            font=('Consolas', 10, 'bold'),
            padx=8, pady=6, relief='groove', bd=2
        )

        # ── Ligne status ──
        status_row = tk.Frame(self.frame, bg=_PANEL_BG)
        status_row.pack(fill=tk.X, pady=(0, 6))

        self._led_canvas = tk.Canvas(status_row, width=20, height=20,
                                     bg=_PANEL_BG, highlightthickness=0)
        self._led_canvas.pack(side=tk.LEFT, padx=(0, 6))
        self._led = self._led_canvas.create_oval(2, 2, 18, 18, fill=_RED, outline='')

        self._info_var = tk.StringVar(value=f'IP: {drone.ip}   Batterie: --')
        tk.Label(status_row, textvariable=self._info_var,
                 fg=_TEXT, bg=_PANEL_BG, font=('Consolas', 10)).pack(side=tk.LEFT, padx=6)

        self._rc_var = tk.StringVar(value='RC:   0   0   0   0')
        tk.Label(status_row, textvariable=self._rc_var,
                 fg=_YELLOW, bg=_PANEL_BG, font=('Consolas', 10)).pack(side=tk.LEFT, padx=12)

        # ── Boutons de commande ──
        btn_row = tk.Frame(self.frame, bg=_PANEL_BG)
        btn_row.pack(fill=tk.X, pady=(0, 6))

        _btn = dict(font=('Consolas', 10, 'bold'), relief='flat',
                    padx=14, pady=5, cursor='hand2')
        tk.Button(btn_row, text='⬆  Takeoff', command=takeoff_cb,
                  bg=_ACCENT, fg='#1e1e2e', activebackground=_GREEN,
                  **_btn).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_row, text='⬇  Land', command=land_cb,
                  bg=_GREY, fg=_TEXT, activebackground=_ACCENT,
                  **_btn).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_row, text='🔴  EMERGENCY', command=emergency_cb,
                  bg=_RED, fg='#1e1e2e', activebackground=_ORANGE,
                  **_btn).pack(side=tk.RIGHT, padx=4)

        # ── Zone de log ──
        log_frame = tk.Frame(self.frame, bg=_PANEL_BG)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self._log = tk.Text(log_frame, height=12,
                            bg='#11111b', fg=_TEXT,
                            font=('Consolas', 9), relief='flat',
                            insertbackground=_TEXT)
        self._log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        sb = tk.Scrollbar(log_frame, command=self._log.yview, bg=_GREY)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._log.config(yscrollcommand=sb.set)

        self._log.tag_configure('send',   foreground=_TEXT)
        self._log.tag_configure('recv',   foreground=_GREEN)
        self._log.tag_configure('action', foreground=_ORANGE)

        self.frame.after(80, self._flush_logs)

    def update_status(self):
        d = self.drone
        self._info_var.set(f'IP: {d.ip}   Batterie: {d.battery}%')
        self._led_canvas.itemconfig(self._led, fill=_GREEN if d.connected else _RED)

    def update_rc(self, roll: int, pitch: int, throttle: int, yaw: int):
        self._rc_var.set(f'RC: {roll:+4d}  {pitch:+4d}  {throttle:+4d}  {yaw:+4d}')

    def _flush_logs(self):
        q = self.drone.log_queue
        while not q.empty():
            msg, tag = q.get_nowait()
            self._log.insert(tk.END, msg + '\n', tag)
            self._log.see(tk.END)
        self.frame.after(80, self._flush_logs)


# ══════════════════════════════════════════════════════════════════════════════
#  Application principale
# ══════════════════════════════════════════════════════════════════════════════

class DronePilot:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Drone Pilot')
        self.root.configure(bg=_DARK_BG)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        # Demande l'IP au démarrage
        ip = simpledialog.askstring(
            'Adresse IP du drone',
            'IP du drone Tello :',
            initialvalue=TELLO_IP,
            parent=self.root
        )
        if not ip:
            self.root.destroy()
            return

        self.link  = UDPLink(SELF_PORT)
        self.joy   = JoystickReader()
        self.drone = Drone(ip.strip(), self.link)

        # Layout principal
        main = tk.Frame(self.root, bg=_DARK_BG)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        self.joy_panel = JoystickPanel(main, self.joy)
        self.joy_panel.frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))

        self.panel = DronePanel(
            main, self.drone,
            takeoff_cb=self._takeoff,
            land_cb=self._land,
            emergency_cb=self._emergency
        )
        self.panel.frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Connexion au drone
        threading.Thread(target=self._connect, daemon=True).start()

        # Boucle RC à 20 Hz
        self._rc_running = True
        self._btns_prev  = [0] * NUM_BUTTONS
        threading.Thread(target=self._rc_loop, daemon=True, name='rc').start()

        # Polling batterie
        threading.Thread(target=self._battery_loop, daemon=True, name='bat').start()

    def _connect(self):
        self.drone.init()
        self.root.after(0, self.panel.update_status)

    def _takeoff(self):
        threading.Thread(target=self.drone.takeoff, daemon=True).start()

    def _land(self):
        threading.Thread(target=self.drone.land, daemon=True).start()

    def _emergency(self):
        self.drone.emergency()

    def _rc_loop(self):
        while self._rc_running:
            _, buttons = self.joy.get_state()

            btn_takeoff  = JOY_BTN_TAKEOFF < len(buttons) and bool(buttons[JOY_BTN_TAKEOFF])
            btn_land     = JOY_BTN_LAND    < len(buttons) and bool(buttons[JOY_BTN_LAND])
            prev_takeoff = JOY_BTN_TAKEOFF < len(self._btns_prev) and bool(self._btns_prev[JOY_BTN_TAKEOFF])
            prev_land    = JOY_BTN_LAND    < len(self._btns_prev) and bool(self._btns_prev[JOY_BTN_LAND])

            if btn_takeoff and not prev_takeoff:
                self._takeoff()
            if btn_land and not prev_land:
                self._land()
            self._btns_prev = buttons

            roll, pitch, throttle, yaw = self.joy.get_rc()
            if self.drone.connected:
                self.drone.send_rc(roll, pitch, throttle, yaw)

            self.root.after(0, lambda r=roll, p=pitch, t=throttle, y=yaw:
                            self.panel.update_rc(r, p, t, y))

            time.sleep(RC_INTERVAL)

    def _battery_loop(self):
        time.sleep(5.0)
        while True:
            if self.drone.connected:
                self.drone.poll_battery()
                self.root.after(0, self.panel.update_status)
            time.sleep(30.0)

    def _on_close(self):
        self._rc_running = False
        if self.drone.connected:
            try:
                self.drone.link.send('land', self.drone.ip)
            except Exception:
                pass
        self.joy.stop()
        self.link.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    app = DronePilot()
    app.run()


if __name__ == '__main__':
    main()
