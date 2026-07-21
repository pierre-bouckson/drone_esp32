#!/usr/bin/env python3
"""
Pilotage du drone a la manette PS4 (USB) + reglage PID en direct.

Regroupe tout ce qu'il faut pour un essai en vol :
  - lecture de la manette et envoi des commandes RC a 50 Hz (port 8889)
  - sliders Kp / Ki / Kd envoyes au firmware sans recompiler
  - orientation temps reel (courbes d'angles)
  - correction PID appliquee a chaque moteur, en barres et en courbes

Se connecter au point d'acces WiFi de l'ESP32, brancher la manette, puis :
    ./venv/bin/python gamepad_pilot.py

Securite, du plus doux au plus radical :
  TRIANGLE  arme ; sans cela les gaz restent a zero
  CERCLE    desarme cote outil, envoie "rc 0 0 0 0" immediatement
  CROIX     arret d'urgence : envoie "stop" au firmware, qui coupe les moteurs
            et se verrouille jusqu'au redemarrage de l'ESP32
Perdre la manette en cours de vol desarme automatiquement.
"""
import sys
import socket

import numpy as np
import pygame
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QColor, QPainter, QPen, QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QProgressBar, QPushButton, QSlider, QVBoxLayout,
    QWidget,
)

from udp_link import UdpTelemetry

DRONE_IP = "192.168.4.1"   # point d'acces de l'ESP32
CMD_PORT = 8889            # localPort dans include/AppConfig.h
RC_PERIOD_MS = 20          # 50 Hz d'envoi des commandes pilote
UI_PERIOD_MS = 33          # ~30 Hz de rafraichissement graphique
TEXT_DIVIDER = 6           # les libelles verbeux ne bougent qu'un tick sur 6
N = 300                    # points affiches par courbe

# Amplitude envoyee au firmware sur les axes roll/pitch. Le firmware divise par
# STICK_TO_ANGLE (20) pour obtenir une consigne d'angle : 400 => +/- 20 deg.
STICK_RANGE = 400
YAW_RANGE = 400
THROTTLE_MAX = 100         # firmware : duty = throttle * THROTTLE_GAIN (2.5)
THROTTLE_RATE = 60.0       # unites/s en mode incremental
DEADZONE = 0.08

# Index des axes/boutons pygame pour une DualShock 4. Les pilotes ne sont pas
# tous d'accord sur la numerotation : le panneau "Manette" affiche les valeurs
# brutes, il suffit de corriger ces constantes si un axe est inverse ou decale.
AX_YAW, AX_THROTTLE, AX_ROLL, AX_PITCH = 0, 1, 2, 3
BTN_STOP, BTN_ARM, BTN_DISARM = 0, 3, 1   # Croix / Triangle / Cercle

# La commande "stop" verrouille le firmware jusqu'au reboot : c'est le seul
# message dont la perte serait vraiment genante, on le repete.
STOP_REPEAT = 5

# L'antialiasing des courbes coute cher pour 8 traces de 300 points redessinees
# en continu : c'est la premiere cause de saccades sur cette IHM.
pg.setConfigOptions(antialias=False)


def deadzone(v: float) -> float:
    """Annule le bruit autour du centre et reetale le reste sur [-1, 1]."""
    if abs(v) < DEADZONE:
        return 0.0
    return (v - DEADZONE * (1 if v > 0 else -1)) / (1.0 - DEADZONE)


# ---------------------------------------------------------------------------
#  Manette
# ---------------------------------------------------------------------------
class Gamepad:
    """Acces non bloquant a la premiere manette branchee, avec rebranchement."""

    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.js = None
        self.axes = []
        self.buttons = []
        self.pressed = set()      # boutons passes a 1 depuis le dernier poll
        self.connect()

    @property
    def connected(self) -> bool:
        return self.js is not None

    @property
    def name(self) -> str:
        return self.js.get_name() if self.js else "aucune manette"

    def connect(self) -> bool:
        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.js = None
            self.axes, self.buttons = [], []
            return False
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.axes = [0.0] * self.js.get_numaxes()
        self.buttons = [0] * self.js.get_numbuttons()
        return True

    def poll(self):
        """Met a jour axes/boutons. Renvoie False si la manette a disparu."""
        self.pressed.clear()
        if self.js is None:
            return False
        try:
            # get() vide la file ; pump() la laisse se remplir jusqu'a saturation
            # et le traitement des evenements accumules finit par couter cher.
            pygame.event.get()
            self.axes = [self.js.get_axis(i) for i in range(self.js.get_numaxes())]
            new = [self.js.get_button(i) for i in range(self.js.get_numbuttons())]
        except pygame.error:
            self.js = None
            return False

        for i, (old, cur) in enumerate(zip(self.buttons, new)):
            if cur and not old:
                self.pressed.add(i)
        self.buttons = new
        return True

    def axis(self, index: int) -> float:
        """Axe filtre par la zone morte, 0.0 si l'index n'existe pas."""
        if index >= len(self.axes):
            return 0.0
        return deadzone(self.axes[index])


# ---------------------------------------------------------------------------
#  Barre signee (correction moteur, axes manette)
# ---------------------------------------------------------------------------
class SignedBar(QWidget):
    """Barre partant du centre : positif a droite, negatif a gauche."""

    # Pinceaux partages : les recreer a chaque paintEvent est du gaspillage pur.
    BG = QColor(35, 35, 40)
    POS = QColor(80, 200, 120)
    NEG = QColor(220, 90, 90)
    AXIS = QPen(QColor(120, 120, 130), 1)
    TEXT = QColor(230, 230, 230)
    FONT = QFont("Menlo", 9)

    def __init__(self, span: float = 100.0):
        super().__init__()
        self.setFixedHeight(20)
        self.setMinimumWidth(120)
        self.span = span
        self.value = 0.0

    def set_value(self, v: float):
        # Sous le demi-pourcent d'echelle le rendu serait identique au pixel
        # pres : on evite un repaint pour rien a chaque tick.
        if abs(v - self.value) < self.span * 0.005:
            return
        self.value = v
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        w, h = self.width(), self.height()
        mid = w / 2

        p.fillRect(0, 0, w, h, self.BG)
        ratio = max(-1.0, min(1.0, self.value / self.span))
        length = ratio * (mid - 2)
        color = self.POS if ratio >= 0 else self.NEG
        if length >= 0:
            p.fillRect(QRectF(mid, 2, length, h - 4), color)
        else:
            p.fillRect(QRectF(mid + length, 2, -length, h - 4), color)

        p.setPen(self.AXIS)
        p.drawLine(int(mid), 0, int(mid), h)
        p.setPen(self.TEXT)
        p.setFont(self.FONT)
        p.drawText(self.rect(), Qt.AlignCenter, f"{self.value:+.0f}")


# ---------------------------------------------------------------------------
#  Slider PID (slider entier + spinbox flottante synchronises)
# ---------------------------------------------------------------------------
class GainSlider(QWidget):
    """Un gain PID : slider grossier + saisie fine, toujours d'accord."""

    STEPS = 1000   # resolution du slider entier sous-jacent

    def __init__(self, name: str, maximum: float, default: float, on_change):
        super().__init__()
        self.maximum = maximum
        self.on_change = on_change
        self._syncing = False

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, self.STEPS)
        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(3)
        self.spin.setRange(0.0, maximum)
        self.spin.setSingleStep(maximum / 200)
        self.spin.setFixedWidth(90)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        label = QLabel(name)
        label.setFixedWidth(30)
        lay.addWidget(label)
        lay.addWidget(self.slider)
        lay.addWidget(self.spin)

        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)
        self.set_value(default)

    def value(self) -> float:
        return self.spin.value()

    def set_value(self, v: float):
        self._syncing = True
        self.spin.setValue(v)
        self.slider.setValue(int(round(v / self.maximum * self.STEPS)))
        self._syncing = False

    def _from_slider(self, raw: int):
        if self._syncing:
            return
        self._syncing = True
        self.spin.setValue(raw / self.STEPS * self.maximum)
        self._syncing = False
        self.on_change()

    def _from_spin(self, v: float):
        if self._syncing:
            return
        self._syncing = True
        self.slider.setValue(int(round(v / self.maximum * self.STEPS)))
        self._syncing = False
        self.on_change()


# ---------------------------------------------------------------------------
#  Fenetre principale
# ---------------------------------------------------------------------------
class Pilot(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone - pilotage manette & reglage PID")
        self.resize(1400, 860)

        self.pad = Gamepad()
        self.link = UdpTelemetry()
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.armed = False
        self.stopped = False         # "stop" envoye : le firmware est verrouille
        self.throttle = 0.0          # 0..THROTTLE_MAX, entretenu par la boucle RC
        self.last_rc = (0, 0, 0, 0)
        self.tele = None             # derniere trame recue

        # La boucle RC tourne a 50 Hz mais ne touche aucun widget : elle depose
        # ici ce que la boucle d'affichage ira lire a 30 Hz. Sans ca, chaque tick
        # RC declenchait un setText + quatre repaints, d'ou les saccades.
        self.sticks = (0.0, 0.0, 0.0, 0.0)    # roll, pitch, yaw, gaz
        self.rc_text = "rc 0 0 0 0"
        self._rc_shown = None
        self._text_tick = 0

        self.x = np.arange(N)
        # Tampons numpy fixes : plus d'allocation ni de conversion deque->array
        # a chaque rafraichissement.
        self.bufs = {k: np.zeros(N) for k in
                     ("roll", "pitch", "cr", "cp", "m1", "m2", "m3", "m4")}
        self._curves_dirty = False

        root = QHBoxLayout(self)
        root.addLayout(self._build_left(), 0)
        root.addLayout(self._build_right(), 1)

        try:
            self.link.open()
            self.state_tele.setText("telemetrie : a l'ecoute")
        except OSError as e:
            self.state_tele.setText(f"telemetrie : {e}")

        # Deux cadences : le drone a besoin de commandes regulieres, l'ecran non.
        self.rc_timer = QTimer(self)
        self.rc_timer.timeout.connect(self.rc_tick)
        self.rc_timer.start(RC_PERIOD_MS)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.ui_tick)
        self.ui_timer.start(UI_PERIOD_MS)

    # ---- construction de l'interface ------------------------------------
    def _build_left(self):
        col = QVBoxLayout()
        col.setSpacing(8)

        # --- etat / armement ---
        box = QGroupBox("Vol")
        v = QVBoxLayout(box)

        self.lbl_armed = QLabel("DESARME")
        self.lbl_armed.setAlignment(Qt.AlignCenter)
        self.lbl_armed.setFont(QFont("Menlo", 15, QFont.Bold))
        self._paint_armed()
        v.addWidget(self.lbl_armed)

        row = QHBoxLayout()
        self.btn_arm = QPushButton("Armer (TRIANGLE)")
        self.btn_arm.clicked.connect(self.arm)
        kill = QPushButton("STOP (CROIX)")
        kill.clicked.connect(self.stop)
        kill.setToolTip("Envoie 'stop' : le firmware coupe les moteurs "
                        "jusqu'au redemarrage de l'ESP32.")
        kill.setStyleSheet("background:#a03030; color:white; font-weight:bold;")
        soft = QPushButton("Desarmer (CERCLE)")
        soft.clicked.connect(self.disarm)
        row.addWidget(self.btn_arm)
        row.addWidget(soft)
        row.addWidget(kill)
        v.addLayout(row)

        self.chk_hold = QCheckBox("Gaz incrementaux (stick relache = gaz maintenus)")
        self.chk_hold.setChecked(True)
        self.chk_hold.setToolTip(
            "Le stick gauche d'une PS4 revient au centre : en mode direct, le "
            "lacher coupe les gaz. En incrementiel il fait monter ou descendre "
            "une consigne qui reste en place."
        )
        v.addWidget(self.chk_hold)

        self.bar_throttle = QProgressBar()
        self.bar_throttle.setRange(0, THROTTLE_MAX)
        self.bar_throttle.setFormat("Gaz %v")
        v.addWidget(self.bar_throttle)

        self.lbl_rc = QLabel("rc 0 0 0 0")
        self.lbl_rc.setFont(QFont("Menlo", 10))
        v.addWidget(self.lbl_rc)
        col.addWidget(box)

        # --- manette ---
        box = QGroupBox("Manette")
        v = QVBoxLayout(box)
        self.lbl_pad = QLabel("...")
        self.lbl_pad.setWordWrap(True)
        v.addWidget(self.lbl_pad)

        grid = QGridLayout()
        self.pad_bars = {}
        for r, (key, title) in enumerate((("roll", "Roll"), ("pitch", "Pitch"),
                                          ("yaw", "Yaw"), ("thr", "Stick gaz"))):
            bar = SignedBar(1.0)
            self.pad_bars[key] = bar
            grid.addWidget(QLabel(title), r, 0)
            grid.addWidget(bar, r, 1)
        v.addLayout(grid)

        self.lbl_axes = QLabel("")
        self.lbl_axes.setFont(QFont("Menlo", 9))
        self.lbl_axes.setWordWrap(True)
        v.addWidget(self.lbl_axes)

        rescan = QPushButton("Rechercher la manette")
        rescan.clicked.connect(self.pad.connect)
        v.addWidget(rescan)
        col.addWidget(box)

        # --- gains PID ---
        box = QGroupBox("Gains PID (envoyes en direct)")
        v = QVBoxLayout(box)
        self.gains = {
            "kp": GainSlider("Kp", 10.0, 1.0, self.send_pid),
            "ki": GainSlider("Ki", 5.0, 0.0, self.send_pid),
            "kd": GainSlider("Kd", 5.0, 0.0, self.send_pid),
        }
        for g in self.gains.values():
            v.addWidget(g)

        row = QHBoxLayout()
        resend = QPushButton("Renvoyer")
        resend.clicked.connect(self.send_pid)
        zero = QPushButton("Tout a zero")
        zero.clicked.connect(self.zero_gains)
        row.addWidget(resend)
        row.addWidget(zero)
        v.addLayout(row)

        self.lbl_pid = QLabel("")
        self.lbl_pid.setFont(QFont("Menlo", 9))
        v.addWidget(self.lbl_pid)
        col.addWidget(box)

        # --- valeurs mesurees ---
        box = QGroupBox("Mesures")
        g = QGridLayout(box)
        self.readouts = {}
        rows = (("roll", "Roll"), ("pitch", "Pitch"),
                ("cr", "Corr. roll"), ("cp", "Corr. pitch"))
        for i, (key, title) in enumerate(rows):
            lab = QLabel("--")
            lab.setFont(QFont("Menlo", 12, QFont.Bold))
            self.readouts[key] = lab
            g.addWidget(QLabel(title), i, 0)
            g.addWidget(lab, i, 1)

        self.state_tele = QLabel("telemetrie : arretee")
        g.addWidget(self.state_tele, len(rows), 0, 1, 2)
        col.addWidget(box)

        col.addStretch()
        return col

    def _build_right(self):
        col = QVBoxLayout()
        col.setSpacing(8)

        box = QGroupBox("Correction PID par moteur (gaz exclus)")
        g = QGridLayout(box)
        self.motor_bars = []
        # M1 avant-gauche, M2 avant-droit, M3 arriere-droit, M4 arriere-gauche
        # (memes indices que le mixage en X du firmware).
        for i, place in enumerate(("M1 av-g", "M2 av-d", "M3 ar-d", "M4 ar-g")):
            bar = SignedBar(100.0)
            self.motor_bars.append(bar)
            g.addWidget(QLabel(place), i, 0)
            g.addWidget(bar, i, 1)
        col.addWidget(box, 0)

        self.p_angle = self._plot("Angles mesures (deg)")
        self.p_angle.addLegend()
        self.c_roll = self.p_angle.plot(pen=pg.mkPen("c", width=2), name="roll")
        self.c_pitch = self.p_angle.plot(pen=pg.mkPen("y", width=2), name="pitch")
        col.addWidget(self.p_angle, 1)

        self.p_corr = self._plot("Sortie PID (deg/s)")
        self.p_corr.addLegend()
        self.c_cr = self.p_corr.plot(pen=pg.mkPen("m", width=2), name="corr roll")
        self.c_cp = self.p_corr.plot(pen=pg.mkPen("g", width=2), name="corr pitch")
        col.addWidget(self.p_corr, 1)

        self.p_mot = self._plot("Correction moteurs", auto=True)
        self.p_mot.addLegend()
        self.c_mot = [self.p_mot.plot(pen=pg.mkPen(c, width=1), name=f"M{i+1}")
                      for i, c in enumerate(("r", "y", "g", "b"))]
        col.addWidget(self.p_mot, 1)
        return col

    def _plot(self, title, auto=False):
        pw = pg.PlotWidget(title=title)
        pw.showGrid(x=True, y=True, alpha=0.3)
        if auto:
            pw.enableAutoRange(axis="y")
        else:
            pw.setYRange(-60, 60)
        # L'axe X est fige sur la fenetre glissante : sans ca pyqtgraph
        # recalcule l'etendue a chaque setData.
        pw.setXRange(0, N - 1, padding=0)
        pw.setMouseEnabled(x=False, y=auto)
        pw.hideButtons()
        pw.setMenuEnabled(False)
        return pw

    # ---- armement --------------------------------------------------------
    def arm(self):
        if self.stopped:
            return                   # verrou firmware : armer n'aurait aucun effet
        self.armed = True
        self.throttle = 0.0          # on repart toujours gaz coupes
        self._paint_armed()

    def disarm(self):
        self.armed = False
        self.throttle = 0.0
        self._paint_armed()
        self.send_rc(0, 0, 0, 0)     # coupure immediate, sans attendre le tick

    def stop(self):
        """Arret d'urgence : coupe les moteurs cote firmware, definitivement."""
        self.stopped = True
        self.armed = False
        self.throttle = 0.0
        for _ in range(STOP_REPEAT):
            try:
                self.tx.sendto(b"stop", (DRONE_IP, CMD_PORT))
            except OSError as e:
                self.rc_text = f"stop  [echec: {e}]"
                break
        else:
            self.rc_text = f"stop  (x{STOP_REPEAT})"
        self._paint_armed()

    def _paint_armed(self):
        if self.stopped:
            # Le firmware ne relache pilot_stop qu'au redemarrage : autant le dire.
            self.lbl_armed.setText("STOP - rebooter l'ESP32")
            self.lbl_armed.setStyleSheet("background:#a03030; color:white; padding:6px;")
        elif self.armed:
            self.lbl_armed.setText("ARME")
            self.lbl_armed.setStyleSheet("background:#207040; color:white; padding:6px;")
        else:
            self.lbl_armed.setText("DESARME")
            self.lbl_armed.setStyleSheet("background:#404048; color:#dddddd; padding:6px;")

    # ---- emission --------------------------------------------------------
    def send_rc(self, left, forward, up, yaw):
        """Emet la trame RC. L'affichage est laisse a ui_tick (voir rc_text)."""
        self.last_rc = (left, forward, up, yaw)
        msg = f"rc {left} {forward} {up} {yaw}"
        try:
            self.tx.sendto(msg.encode(), (DRONE_IP, CMD_PORT))
        except OSError as e:
            msg = f"{msg}   [echec: {e}]"
        self.rc_text = msg

    def send_pid(self):
        kp, ki, kd = (self.gains[k].value() for k in ("kp", "ki", "kd"))
        msg = f"pid {kp:.3f} {ki:.3f} {kd:.3f}"
        try:
            self.tx.sendto(msg.encode(), (DRONE_IP, CMD_PORT))
        except OSError as e:
            self.lbl_pid.setText(f"echec: {e}")
            return
        # Le firmware n'accuse pas reception : l'affichage reflete l'envoi seul.
        self.lbl_pid.setText(f"envoye  {msg}")

    def zero_gains(self):
        for g in self.gains.values():
            g.set_value(0.0)
        self.send_pid()

    # ---- boucle de commande (50 Hz) -------------------------------------
    def rc_tick(self):
        if not self.pad.poll():
            if self.armed:
                self.disarm()        # manette perdue : on ne laisse pas les gaz
            # On continue d'emettre des zeros a 50 Hz au lieu de couper
            # l'emission : le "rc 0 0 0 0" du disarm est un datagramme unique,
            # s'il se perd les suivants prennent le relais. Le drone recoit donc
            # une consigne nulle en continu tant que la manette manque.
            self.throttle = 0.0
            self.send_rc(0, 0, 0, 0)
            self.sticks = (0.0, 0.0, 0.0, 0.0)
            return

        # Le stop prime sur tout le reste : teste en premier.
        if BTN_STOP in self.pad.pressed:
            self.stop()
        elif BTN_DISARM in self.pad.pressed:
            self.disarm()
        elif BTN_ARM in self.pad.pressed:
            self.arm()

        roll = self.pad.axis(AX_ROLL)
        pitch = self.pad.axis(AX_PITCH)
        yaw = self.pad.axis(AX_YAW)
        thr_stick = -self.pad.axis(AX_THROTTLE)   # stick vers le haut = axe negatif

        if self.armed:
            if self.chk_hold.isChecked():
                self.throttle += thr_stick * THROTTLE_RATE * (RC_PERIOD_MS / 1000.0)
            else:
                self.throttle = max(0.0, thr_stick) * THROTTLE_MAX
            self.throttle = max(0.0, min(float(THROTTLE_MAX), self.throttle))
        else:
            self.throttle = 0.0

        # Le firmware calcule pitch a partir de rc.left et roll a partir de
        # rc.forward : on garde ses noms de champs, pas les notres.
        left = int(round(pitch * STICK_RANGE))
        forward = int(round(roll * STICK_RANGE))
        self.send_rc(left, forward, int(round(self.throttle)),
                     int(round(yaw * YAW_RANGE)))

        self.sticks = (roll, pitch, yaw, thr_stick)

    # ---- boucle d'affichage (30 Hz) -------------------------------------
    def _push(self, key, value):
        """Decale le tampon d'un cran et depose la nouvelle mesure a la fin."""
        buf = self.bufs[key]
        buf[:-1] = buf[1:]
        buf[-1] = value

    def ui_tick(self):
        for f in self.link.poll():
            self.tele = f
            self._curves_dirty = True
            self._push("roll", f.roll)
            self._push("pitch", f.pitch)
            self._push("cr", f.corr_roll)
            self._push("cp", f.corr_pitch)
            for i in range(4):
                self._push(f"m{i+1}", f.motors[i])

        if self.tele is not None:
            t = self.tele
            self.readouts["roll"].setText(f"{t.roll:+7.2f} deg")
            self.readouts["pitch"].setText(f"{t.pitch:+7.2f} deg")
            self.readouts["cr"].setText(f"{t.corr_roll:+7.2f}")
            self.readouts["cp"].setText(f"{t.corr_pitch:+7.2f}")
            for i, bar in enumerate(self.motor_bars):
                bar.set_value(t.motors[i])

        # Sans nouvelle trame les courbes sont deja a jour : redessiner huit
        # traces pour un resultat identique ne ferait que manger du temps.
        if self._curves_dirty:
            self._curves_dirty = False
            self.c_roll.setData(self.x, self.bufs["roll"])
            self.c_pitch.setData(self.x, self.bufs["pitch"])
            self.c_cr.setData(self.x, self.bufs["cr"])
            self.c_cp.setData(self.x, self.bufs["cp"])
            for i, curve in enumerate(self.c_mot):
                curve.setData(self.x, self.bufs[f"m{i+1}"])

        roll, pitch, yaw, thr_stick = self.sticks
        self.pad_bars["roll"].set_value(roll)
        self.pad_bars["pitch"].set_value(pitch)
        self.pad_bars["yaw"].set_value(yaw)
        self.pad_bars["thr"].set_value(thr_stick)

        if self.rc_text != self._rc_shown:
            self._rc_shown = self.rc_text
            self.lbl_rc.setText(self.rc_text)
        self.bar_throttle.setValue(int(self.throttle))

        # Le pave d'axes bruts est un outil de diagnostic : il passe par un
        # QLabel a retour a la ligne, dont le relayout coute cher a 30 Hz.
        self._text_tick += 1
        if self._text_tick % TEXT_DIVIDER:
            return
        if self.pad.connected:
            self.lbl_pad.setText(f"connectee : {self.pad.name}")
            self.lbl_axes.setText(
                "axes  " + " ".join(f"{i}:{v:+.2f}" for i, v in enumerate(self.pad.axes))
                + "\nbtns  " + "".join(str(b) for b in self.pad.buttons)
            )
        else:
            self.lbl_pad.setText("aucune manette detectee - brancher puis relancer la recherche")
            self.lbl_axes.setText("")

    def closeEvent(self, event):
        if self.armed:
            self.disarm()
        self.link.close()
        pygame.quit()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = Pilot()
    w.show()
    sys.exit(app.exec_())
