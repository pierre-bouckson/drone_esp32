"""
Kalman Visualizer - Minimal
Ecoute la telemetrie UDP diffusee par le drone (voir udp_link.py).
pip install pyqt5 pyqtgraph numpy
"""
import sys
import numpy as np
from collections import deque
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

from udp_link import UdpTelemetry, TELEMETRY_PORT

N = 300  # points affichés

pg.setConfigOptions(antialias=False, useOpenGL=False)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kalman Visualizer")
        self.resize(1000, 600)

        self.link = UdpTelemetry()
        self.bufs = {k: deque([0.0]*N, maxlen=N) for k in
                     ["roll_k","roll_acc","pitch_k","pitch_acc"]}
        self.x = np.arange(N)

        # UI
        w = QWidget(); self.setCentralWidget(w)
        layout = QVBoxLayout(w)

        # Barre connexion
        bar = QHBoxLayout()
        self.edit_port = QLineEdit(str(TELEMETRY_PORT))
        self.edit_port.setFixedWidth(80)
        self.btn = QPushButton("Ecouter")
        self.btn.clicked.connect(self.toggle)
        self.lbl_state = QLabel("Arrete")
        bar.addWidget(QLabel("Port UDP:")); bar.addWidget(self.edit_port)
        bar.addWidget(self.btn); bar.addWidget(self.lbl_state); bar.addStretch()
        layout.addLayout(bar)

        # Graphiques
        self.pw_roll  = pg.PlotWidget(title="ROLL")
        self.pw_pitch = pg.PlotWidget(title="PITCH")
        for pw in (self.pw_roll, self.pw_pitch):
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setYRange(-180, 180)
        layout.addWidget(self.pw_roll)
        layout.addWidget(self.pw_pitch)

        self.c_rk  = self.pw_roll.plot( pen=pg.mkPen("c", width=2), name="Kalman")
        self.c_ra  = self.pw_roll.plot( pen=pg.mkPen("g", width=1), name="Accelero")
        self.c_pk  = self.pw_pitch.plot(pen=pg.mkPen("c", width=2))
        self.c_pa  = self.pw_pitch.plot(pen=pg.mkPen("g", width=1))
        self.pw_roll.addLegend()

        # Timer lecture + affichage 20 Hz
        self.timer = QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update)
        self.timer.start()

    def toggle(self):
        if self.link.is_open:
            self.link.close()
            self.btn.setText("Ecouter")
            self.lbl_state.setText("Arrete")
        else:
            try:
                self.link.port = int(self.edit_port.text())
                self.link.open()
                self.btn.setText("Arreter")
                self.lbl_state.setText("A l'ecoute")
            except (ValueError, OSError) as e:
                QMessageBox.critical(self, "Erreur", str(e))

    def update(self):
        if not self.link.is_open:
            return

        for f in self.link.poll():
            self.bufs["roll_k"].append(f.roll)
            self.bufs["roll_acc"].append(f.roll_acc)
            self.bufs["pitch_k"].append(f.pitch)
            self.bufs["pitch_acc"].append(f.pitch_acc)

        self.c_rk.setData(self.x, np.array(self.bufs["roll_k"]))
        self.c_ra.setData(self.x, np.array(self.bufs["roll_acc"]))
        self.c_pk.setData(self.x, np.array(self.bufs["pitch_k"]))
        self.c_pa.setData(self.x, np.array(self.bufs["pitch_acc"]))

    def closeEvent(self, event):
        self.link.close()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow(); w.show()
    sys.exit(app.exec_())