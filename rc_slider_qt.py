#!/usr/bin/env python3
"""
Petit emetteur RC Qt pour le firmware ESP32.
Inclut un visualiseur temps-reel des angles roll/pitch via port serie.

Protocole cote ESP32:
  - commande UDP vers 192.168.4.1:8889
  - message RC: "rc <left> <forward> <up> <yaw>"
  - serie 115200: "roll : <deg>pitch : <deg>"  (Serial.print, pas println)

Dependances:
  pip install PySide6 pyserial

Lancement:
  python rc_slider_qt.py
"""

from __future__ import annotations

import collections
import re
import socket
import sys
import time
from math import isfinite

try:
    from PySide6.QtCore import QThread, QTimer, Signal, Qt
    from PySide6.QtGui import QColor, QFont, QPainter, QPen
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QPushButton,
        QSlider,
        QSpinBox,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    try:
        from PyQt6.QtCore import QThread, QTimer, Qt, pyqtSignal as Signal
        from PyQt6.QtGui import QColor, QFont, QPainter, QPen
        from PyQt6.QtWidgets import (
            QApplication,
            QCheckBox,
            QComboBox,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QLineEdit,
            QMainWindow,
            QPushButton,
            QSlider,
            QSpinBox,
            QTextEdit,
            QVBoxLayout,
            QWidget,
        )
    except ImportError:
        from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal as Signal
        from PyQt5.QtGui import QColor, QFont, QPainter, QPen
        from PyQt5.QtWidgets import (
            QApplication,
            QCheckBox,
            QComboBox,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QLineEdit,
            QMainWindow,
            QPushButton,
            QSlider,
            QSpinBox,
            QTextEdit,
            QVBoxLayout,
            QWidget,
        )

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


DRONE_IP = "192.168.4.1"
DRONE_PORT = 8889
LOCAL_RESPONSE_PORT = 8894
RC_MIN = -100
RC_MAX = 100
RC_INTERVAL_MS = 50
ANGLE_HISTORY = 300  # nombre d'echantillons gardes dans le graph


def horizontal_orientation():
    return Qt.Orientation.Horizontal if hasattr(Qt, "Orientation") else Qt.Horizontal


def _qt_align(attr: str):
    """Retourne la constante d'alignement Qt independamment de la version."""
    af = getattr(Qt, "AlignmentFlag", None)
    if af is not None:
        return getattr(af, attr)
    return getattr(Qt, attr)


# ---------------------------------------------------------------------------
# Threads
# ---------------------------------------------------------------------------

class UdpReceiver(QThread):
    """Ecoute sur le socket partage et emet un signal par message recu."""

    message_received = Signal(str, str)

    def __init__(self, sock: socket.socket) -> None:
        super().__init__()
        self._sock = sock
        self._running = True

    def run(self) -> None:
        self._sock.settimeout(0.25)
        while self._running:
            try:
                data, address = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            text = data.decode("utf-8", errors="replace").strip()
            self.message_received.emit(f"{address[0]}:{address[1]}", text)

    def stop(self) -> None:
        self._running = False


class SerialReader(QThread):
    """Lit le port serie et extrait les angles roll/pitch en continu."""

    angles_received = Signal(float, float)
    status = Signal(str)

    # Format ESP32: "roll : 1.23pitch : -4.56" (pas de newline entre les deux)
    _PATTERN = re.compile(r"roll\s*:\s*([-\d.eE+]+)\s*pitch\s*:\s*([-\d.eE+]+)")

    def __init__(self, port: str, baud: int = 115200) -> None:
        super().__init__()
        self._port = port
        self._baud = baud
        self._running = True

    def run(self) -> None:
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.25)
        except serial.SerialException as exc:
            self.status.emit(f"Erreur port serie: {exc}")
            return

        self.status.emit(f"Serie connecte: {self._port} @ {self._baud}")
        buf = ""
        while self._running:
            try:
                chunk = ser.read(256).decode("utf-8", errors="replace")
            except serial.SerialException as exc:
                self.status.emit(f"Lecture serie perdue: {exc}")
                break
            if not chunk:
                continue
            buf += chunk
            last_end = 0
            for m in self._PATTERN.finditer(buf):
                try:
                    roll = float(m.group(1))
                    pitch = float(m.group(2))
                except ValueError:
                    continue
                if isfinite(roll) and isfinite(pitch):
                    self.angles_received.emit(roll, pitch)
                last_end = m.end()
            # Garde uniquement la fin non encore parsee
            if last_end:
                buf = buf[last_end:]
            elif len(buf) > 512:
                buf = buf[-256:]

        ser.close()
        self.status.emit("Serie deconnecte.")

    def stop(self) -> None:
        self._running = False


# ---------------------------------------------------------------------------
# Widgets
# ---------------------------------------------------------------------------

class AngleGraph(QWidget):
    """
    Graphique defilant roll (bleu) / pitch (orange).
    Affiche aussi les valeurs numeriques courantes en bas.
    """

    _BG = QColor(18, 18, 30)
    _GRID = QColor(55, 55, 85)
    _ZERO_LINE = QColor(90, 90, 130)
    _ROLL_COL = QColor(90, 180, 255)
    _PITCH_COL = QColor(255, 165, 70)
    _TEXT_COL = QColor(210, 210, 235)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(240)
        self._roll: collections.deque[float] = collections.deque(maxlen=ANGLE_HISTORY)
        self._pitch: collections.deque[float] = collections.deque(maxlen=ANGLE_HISTORY)
        self._y_range = 45.0  # demi-echelle en degres

    def add_sample(self, roll: float, pitch: float) -> None:
        self._roll.append(roll)
        self._pitch.append(pitch)
        all_vals = list(self._roll) + list(self._pitch)
        if all_vals:
            peak = max(abs(v) for v in all_vals)
            self._y_range = max(20.0, peak * 1.20)
        self.update()

    def clear(self) -> None:
        self._roll.clear()
        self._pitch.clear()
        self._y_range = 45.0
        self.update()

    # ---- dessin -----------------------------------------------------------

    def paintEvent(self, event) -> None:  # noqa: N802
        p = QPainter(self)
        hint = getattr(getattr(QPainter, "RenderHint", None), "Antialiasing", None)
        if hint is None:
            hint = QPainter.Antialiasing
        p.setRenderHint(hint)

        W, H = self.width(), self.height()
        pad_l, pad_r, pad_t, pad_b = 50, 12, 10, 30

        p.fillRect(0, 0, W, H, self._BG)

        pw = W - pad_l - pad_r
        ph = H - pad_t - pad_b

        y_rng = self._y_range

        def ypx(deg: float) -> int:
            return int(pad_t + ph * (1.0 - (deg + y_rng) / (2.0 * y_rng)))

        def xpx(i: int, n: int) -> int:
            return int(pad_l + (i / max(n - 1, 1)) * pw)

        # --- grille horizontale ---
        step = 10 if y_rng <= 40 else (30 if y_rng <= 90 else 45)
        ticks = sorted({t for k in range(0, int(y_rng) + step, step) for t in (k, -k)
                        if abs(t) <= y_rng})
        small_font = QFont("Consolas", 8)
        p.setFont(small_font)
        for t in ticks:
            yp = ypx(t)
            pen = QPen(self._ZERO_LINE if t == 0 else self._GRID)
            pen.setWidth(1 if t != 0 else 2)
            p.setPen(pen)
            p.drawLine(pad_l, yp, pad_l + pw, yp)
            p.setPen(self._TEXT_COL)
            p.drawText(0, yp - 8, pad_l - 4, 16, _qt_align("AlignRight"), f"{t:+.0f}")

        # --- courbes ---
        def draw_curve(data, color):
            pts = list(data)
            n = len(pts)
            if n < 2:
                return
            pen = QPen(color)
            pen.setWidth(2)
            p.setPen(pen)
            for i in range(1, n):
                p.drawLine(xpx(i - 1, n), ypx(pts[i - 1]),
                           xpx(i, n),     ypx(pts[i]))

        draw_curve(self._roll, self._ROLL_COL)
        draw_curve(self._pitch, self._PITCH_COL)

        # --- valeurs courantes en bas ---
        roll_now = self._roll[-1] if self._roll else float("nan")
        pitch_now = self._pitch[-1] if self._pitch else float("nan")

        big_font = QFont("Consolas", 13)
        big_font.setBold(True)
        p.setFont(big_font)

        ly = pad_t + ph + 2
        lh = pad_b - 2

        roll_txt = f"ROLL  {roll_now:+7.2f}°" if isfinite(roll_now) else "ROLL   ---"
        pitch_txt = f"PITCH {pitch_now:+7.2f}°" if isfinite(pitch_now) else "PITCH  ---"

        p.setPen(self._ROLL_COL)
        p.drawText(pad_l, ly, 230, lh, _qt_align("AlignLeft"), roll_txt)
        p.setPen(self._PITCH_COL)
        p.drawText(pad_l + 240, ly, 230, lh, _qt_align("AlignLeft"), pitch_txt)

        p.end()


class RcControl(QWidget):
    value_changed = Signal()

    def __init__(self, name: str, description: str) -> None:
        super().__init__()
        self.name = name

        label = QLabel(name)
        label.setMinimumWidth(74)

        hint = QLabel(description)
        hint.setMinimumWidth(132)

        self.slider = QSlider(horizontal_orientation())
        self.slider.setRange(RC_MIN, RC_MAX)
        self.slider.setTickPosition(QSlider.TickPosition.TicksBelow if hasattr(QSlider, "TickPosition") else QSlider.TicksBelow)
        self.slider.setTickInterval(25)

        self.spin = QSpinBox()
        self.spin.setRange(RC_MIN, RC_MAX)
        self.spin.setSingleStep(1)
        self.spin.setMinimumWidth(72)

        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self.slider.setValue)
        self.slider.valueChanged.connect(self.value_changed.emit)
        self.spin.valueChanged.connect(self.value_changed.emit)

        layout = QHBoxLayout(self)
        layout.addWidget(label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin)
        layout.addWidget(hint)

    def value(self) -> int:
        return self.spin.value()

    def set_value(self, value: int) -> None:
        self.spin.setValue(value)


# ---------------------------------------------------------------------------
# Fenetre principale
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ESP32 RC Slider + IMU")
        self.resize(820, 860)

        # --- UDP ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(("", LOCAL_RESPONSE_PORT))
        except OSError as exc:
            print(f"Impossible de binder le port {LOCAL_RESPONSE_PORT}: {exc}")

        self.receiver = UdpReceiver(self.sock)
        self.receiver.message_received.connect(self.on_udp_message)
        self.receiver.start()

        self.timer = QTimer(self)
        self.timer.setInterval(RC_INTERVAL_MS)
        self.timer.timeout.connect(self.send_rc)

        # --- Controles reseau ---
        self.ip_edit = QLineEdit(DRONE_IP)
        self.port_edit = QSpinBox()
        self.port_edit.setRange(1, 65535)
        self.port_edit.setValue(DRONE_PORT)

        self.interval_edit = QSpinBox()
        self.interval_edit.setRange(10, 1000)
        self.interval_edit.setValue(RC_INTERVAL_MS)
        self.interval_edit.setSuffix(" ms")
        self.interval_edit.valueChanged.connect(self.timer.setInterval)

        # --- RC sliders ---
        self.controls = {
            "left": RcControl("left", "gauche / droite"),
            "forward": RcControl("forward", "avant / arriere"),
            "up": RcControl("up", "montee / descente"),
            "yaw": RcControl("yaw", "rotation"),
        }
        for control in self.controls.values():
            control.value_changed.connect(self.refresh_preview)

        self.preview = QLabel()
        self.preview.setObjectName("preview")

        self.send_once_btn = QPushButton("Envoyer une fois")
        self.send_once_btn.clicked.connect(self.send_rc)

        self.command_btn = QPushButton("Envoyer command")
        self.command_btn.clicked.connect(lambda: self.send_raw("command"))

        self.zero_btn = QPushButton("Stop / zero")
        self.zero_btn.clicked.connect(self.zero_and_send)

        self.continuous_check = QCheckBox("Envoi continu")
        self.continuous_check.toggled.connect(self.toggle_continuous)

        self.response_label = QLabel("En attente de reponse...")
        self.response_label.setObjectName("response")

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        # --- Serie / IMU ---
        self._serial_reader: SerialReader | None = None

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(120)

        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(9600, 2000000)
        self.baud_spin.setValue(115200)
        self.baud_spin.setSingleStep(9600)

        self.refresh_ports_btn = QPushButton("Rafraichir")
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)

        self.serial_btn = QPushButton("Connecter serie")
        self.serial_btn.setCheckable(True)
        self.serial_btn.toggled.connect(self.toggle_serial)
        if not SERIAL_AVAILABLE:
            self.serial_btn.setEnabled(False)
            self.serial_btn.setToolTip("pip install pyserial")

        self.angle_graph = AngleGraph()

        self.setCentralWidget(self.build_ui())
        self.refresh_preview()
        self.refresh_ports()
        self.log_line(f"Pret. Socket lie au port {LOCAL_RESPONSE_PORT}. Connecte-toi au WiFi peter_pan.")
        if not SERIAL_AVAILABLE:
            self.log_line("pyserial non installe — 'pip install pyserial' pour activer la serie.")

    # -----------------------------------------------------------------------
    # Construction de l'UI
    # -----------------------------------------------------------------------

    def build_ui(self) -> QWidget:
        root = QWidget()
        main = QVBoxLayout(root)

        # -- Reseau UDP --
        network = QGroupBox("Reseau UDP")
        nl = QGridLayout(network)
        nl.addWidget(QLabel("IP ESP32"), 0, 0)
        nl.addWidget(self.ip_edit, 0, 1)
        nl.addWidget(QLabel("Port UDP"), 0, 2)
        nl.addWidget(self.port_edit, 0, 3)
        nl.addWidget(QLabel("Periode"), 0, 4)
        nl.addWidget(self.interval_edit, 0, 5)

        # -- Message RC --
        sliders = QGroupBox("Message RC")
        sl = QVBoxLayout(sliders)
        for key in ("left", "forward", "up", "yaw"):
            sl.addWidget(self.controls[key])
        sl.addWidget(self.preview)

        buttons = QHBoxLayout()
        buttons.addWidget(self.command_btn)
        buttons.addWidget(self.send_once_btn)
        buttons.addWidget(self.continuous_check)
        buttons.addStretch(1)
        buttons.addWidget(self.zero_btn)

        # -- Port serie --
        serial_box = QGroupBox("Port serie (IMU)")
        serl = QHBoxLayout(serial_box)
        serl.addWidget(QLabel("Port"))
        serl.addWidget(self.port_combo, 1)
        serl.addWidget(QLabel("Baud"))
        serl.addWidget(self.baud_spin)
        serl.addWidget(self.refresh_ports_btn)
        serl.addWidget(self.serial_btn)

        # -- Graphique IMU --
        imu_box = QGroupBox("IMU — Roll / Pitch en temps reel")
        imul = QVBoxLayout(imu_box)
        imul.addWidget(self.angle_graph, 1)

        main.addWidget(network)
        main.addWidget(sliders)
        main.addLayout(buttons)
        main.addWidget(self.response_label)
        main.addWidget(serial_box)
        main.addWidget(imu_box, 1)
        main.addWidget(QLabel("Journal"))
        main.addWidget(self.log)

        root.setStyleSheet(
            """
            QWidget { font-size: 14px; }
            QGroupBox {
                font-weight: 600;
                margin-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
            }
            QLabel#preview {
                font-family: Consolas, monospace;
                font-size: 18px;
                padding: 8px 0;
            }
            QLabel#response {
                font-family: Consolas, monospace;
                font-size: 16px;
                font-weight: bold;
                padding: 6px 8px;
                border-radius: 4px;
                background: #2a2a3e;
                color: #a6e3a1;
            }
            QPushButton {
                min-height: 32px;
                padding: 4px 12px;
            }
            QPushButton:checked {
                background: #3a5a3a;
                color: #a6e3a1;
            }
            """
        )

        return root

    # -----------------------------------------------------------------------
    # RC / UDP
    # -----------------------------------------------------------------------

    def rc_values(self) -> tuple[int, int, int, int]:
        return (
            self.controls["left"].value(),
            self.controls["forward"].value(),
            self.controls["up"].value(),
            self.controls["yaw"].value(),
        )

    def rc_command(self) -> str:
        left, forward, up, yaw = self.rc_values()
        return f"rc {left} {forward} {up} {yaw}"

    def refresh_preview(self) -> None:
        self.preview.setText(self.rc_command())

    def target(self) -> tuple[str, int]:
        return self.ip_edit.text().strip(), int(self.port_edit.value())

    def send_raw(self, command: str) -> None:
        ip, port = self.target()
        if not ip:
            self.log_line("IP vide, commande non envoyee.")
            return
        try:
            self.sock.sendto(command.encode("utf-8"), (ip, port))
        except OSError as exc:
            self.log_line(f"Erreur envoi: {exc}")
            return
        self.log_line(f"> {command}")

    def send_rc(self) -> None:
        self.send_raw(self.rc_command())

    def zero_and_send(self) -> None:
        for control in self.controls.values():
            control.set_value(0)
        self.send_rc()

    def toggle_continuous(self, enabled: bool) -> None:
        if enabled:
            self.timer.start()
            self.log_line(f"Envoi continu actif ({self.timer.interval()} ms).")
        else:
            self.timer.stop()
            self.log_line("Envoi continu arrete.")

    def on_udp_message(self, sender: str, message: str) -> None:
        self.response_label.setText(f"Reponse de {sender} : {message}")
        color = "#a6e3a1" if message == "ok" else "#fab387"
        self.response_label.setStyleSheet(
            f"font-family: Consolas, monospace; font-size: 16px; font-weight: bold;"
            f"padding: 6px 8px; border-radius: 4px; background: #2a2a3e; color: {color};"
        )
        self.log_line(f"< {sender}  {message}")

    # -----------------------------------------------------------------------
    # Serie / IMU
    # -----------------------------------------------------------------------

    def refresh_ports(self) -> None:
        self.port_combo.clear()
        if not SERIAL_AVAILABLE:
            self.port_combo.addItem("pyserial manquant")
            return
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        for p in ports:
            desc = p.description or p.device
            self.port_combo.addItem(f"{p.device}  —  {desc}", userData=p.device)
        if not ports:
            self.port_combo.addItem("Aucun port detecte")

    def toggle_serial(self, checked: bool) -> None:
        if checked:
            idx = self.port_combo.currentIndex()
            port = self.port_combo.itemData(idx)
            if not port:
                self.log_line("Aucun port serie valide selectionne.")
                self.serial_btn.setChecked(False)
                return
            baud = self.baud_spin.value()
            self._serial_reader = SerialReader(port, baud)
            self._serial_reader.angles_received.connect(self.on_angles)
            self._serial_reader.status.connect(self.on_serial_status)
            self._serial_reader.start()
            self.serial_btn.setText("Deconnecter serie")
        else:
            if self._serial_reader:
                self._serial_reader.stop()
                self._serial_reader.wait(1500)
                self._serial_reader = None
            self.serial_btn.setText("Connecter serie")
            self.angle_graph.clear()

    def on_serial_status(self, msg: str) -> None:
        self.log_line(f"[SERIE] {msg}")

    def on_angles(self, roll: float, pitch: float) -> None:
        self.angle_graph.add_sample(roll, pitch)

    # -----------------------------------------------------------------------
    # Utilitaires
    # -----------------------------------------------------------------------

    def log_line(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log.append(f"[{timestamp}] {text}")

    def closeEvent(self, event) -> None:  # noqa: N802
        self.timer.stop()
        self.receiver.stop()
        self.receiver.wait(1000)
        self.sock.close()
        if self._serial_reader:
            self._serial_reader.stop()
            self._serial_reader.wait(1500)
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
