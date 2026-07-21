#!/usr/bin/env python3
"""
Reglage PID du drone - minimal.

Ecoute la telemetrie diffusee en UDP (port 8894) et trace, par axe, l'angle
mesure et la correction PID sur le meme graphique. Permet d'envoyer de
nouveaux gains au drone (port 8889).

Se connecter au point d'acces WiFi de l'ESP32, puis lancer :
    ./venv/bin/python drone_tuner.py
"""
import sys
import socket
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QMessageBox
)
from PyQt5.QtCore import QTimer

from udp_link import UdpTelemetry

N = 300                      # points affiches
DRONE_IP = "192.168.4.1"     # IP du point d'acces de l'ESP32
CMD_PORT = 8889              # localPort dans include/AppConfig.h

pg.setConfigOptions(antialias=False)


class Tuner(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone PID Tuner")
        self.resize(1000, 700)

        self.link = UdpTelemetry()
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.x = np.arange(N)
        self.bufs = {k: deque([0.0] * N, maxlen=N) for k in
                     ("roll", "roll_acc", "cr", "pitch", "pitch_acc", "cp",
                      "m1", "m2", "m3", "m4")}

        layout = QVBoxLayout(self)
        layout.addLayout(self._build_bar())

        # Un graphe par axe : angle filtre, angle brut et correction PID
        # superposes, pour voir directement l'effet des gains sur l'angle.
        self.p_roll = self._plot("ROLL", layout)
        self.c_roll = self.p_roll.plot(pen=pg.mkPen("c", width=2), name="Kalman")
        self.c_racc = self.p_roll.plot(pen=pg.mkPen("g", width=1), name="Accelero")
        self.c_cr = self.p_roll.plot(pen=pg.mkPen("m", width=2), name="Correction")
        self.p_roll.addLegend()

        self.p_pitch = self._plot("PITCH", layout)
        self.c_pitch = self.p_pitch.plot(pen=pg.mkPen("c", width=2))
        self.c_pacc = self.p_pitch.plot(pen=pg.mkPen("g", width=1))
        self.c_cp = self.p_pitch.plot(pen=pg.mkPen("m", width=2))

        # Duties moteurs : a part, l'echelle n'a rien a voir avec des degres.
        self.p_mot = self._plot("MOTEURS (correction, gaz exclus)", layout)
        self.p_mot.enableAutoRange(axis="y")
        self.c_mot = [self.p_mot.plot(pen=pg.mkPen(c, width=1), name=f"M{i+1}")
                      for i, c in enumerate(("r", "y", "g", "b"))]
        self.p_mot.addLegend()

        self.timer = QTimer(self)
        self.timer.setInterval(5)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    def _plot(self, title, layout):
        pw = pg.PlotWidget(title=title)
        pw.showGrid(x=True, y=True, alpha=0.3)
        pw.setYRange(-60, 60)
        layout.addWidget(pw)
        return pw

    def _build_bar(self):
        bar = QHBoxLayout()

        self.btn = QPushButton("Ecouter")
        self.btn.clicked.connect(self.toggle)
        bar.addWidget(self.btn)
        self.state = QLabel("arrete")
        bar.addWidget(self.state)

        bar.addStretch()

        self.edits = {}
        for name, default in (("kp", "1.0"), ("ki", "0.0"), ("kd", "0.0")):
            e = QLineEdit(default)
            e.setFixedWidth(70)
            e.returnPressed.connect(self.send_pid)
            self.edits[name] = e
            bar.addWidget(QLabel(name.upper()))
            bar.addWidget(e)

        send = QPushButton("Envoyer PID")
        send.clicked.connect(self.send_pid)
        bar.addWidget(send)

        self.sent = QLabel("")
        bar.addWidget(self.sent)
        return bar

    # ---- reseau ----------------------------------------------------------
    def toggle(self):
        if self.link.is_open:
            self.link.close()
            self.btn.setText("Ecouter")
            self.state.setText("arrete")
        else:
            try:
                self.link.open()
            except OSError as e:
                QMessageBox.critical(self, "Erreur", str(e))
                return
            self.btn.setText("Arreter")
            self.state.setText("a l'ecoute")

    def send_pid(self):
        try:
            kp, ki, kd = (float(self.edits[k].text()) for k in ("kp", "ki", "kd"))
        except ValueError:
            QMessageBox.warning(self, "Erreur", "Gains invalides")
            return

        msg = f"pid {kp} {ki} {kd}"
        try:
            self.tx.sendto(msg.encode(), (DRONE_IP, CMD_PORT))
        except OSError as e:
            self.sent.setText(f"echec: {e}")
            return
        # Envoi non confirme : le firmware n'accuse pas reception des gains.
        self.sent.setText(f"envoye  {msg}")

    # ---- boucle d'affichage ---------------------------------------------
    def refresh(self):
        if not self.link.is_open:
            return

        for f in self.link.poll():
            self.bufs["roll"].append(f.roll)
            self.bufs["roll_acc"].append(f.roll_acc)
            self.bufs["cr"].append(f.corr_roll)
            self.bufs["pitch"].append(f.pitch)
            self.bufs["pitch_acc"].append(f.pitch_acc)
            self.bufs["cp"].append(f.corr_pitch)
            for i in range(4):
                self.bufs[f"m{i+1}"].append(f.motors[i])

        b = {k: np.array(v) for k, v in self.bufs.items()}
        self.c_roll.setData(self.x, b["roll"])
        self.c_racc.setData(self.x, b["roll_acc"])
        self.c_cr.setData(self.x, b["cr"])
        self.c_pitch.setData(self.x, b["pitch"])
        self.c_pacc.setData(self.x, b["pitch_acc"])
        self.c_cp.setData(self.x, b["cp"])
        for i, curve in enumerate(self.c_mot):
            curve.setData(self.x, b[f"m{i+1}"])

    def closeEvent(self, event):
        self.link.close()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Tuner()
    w.show()
    sys.exit(app.exec_())
