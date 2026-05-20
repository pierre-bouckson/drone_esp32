#!/usr/bin/env python3
"""
Petit emetteur RC Qt pour le firmware ESP32.

Protocole cote ESP32:
  - commande UDP vers 192.168.4.1:8889
  - message RC: "rc <left> <forward> <up> <yaw>"

Dependances:
  pip install PySide6

Lancement:
  python rc_slider_qt.py
"""

from __future__ import annotations

import socket
import sys
import time


try:
    from PySide6.QtCore import QThread, QTimer, Signal, Qt
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
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
        from PyQt6.QtWidgets import (
            QApplication,
            QCheckBox,
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
        from PyQt5.QtWidgets import (
            QApplication,
            QCheckBox,
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


DRONE_IP = "192.168.4.1"
DRONE_PORT = 8889
LOCAL_RESPONSE_PORT = 8894
RC_MIN = -100
RC_MAX = 100
RC_INTERVAL_MS = 50


def horizontal_orientation():
    return Qt.Orientation.Horizontal if hasattr(Qt, "Orientation") else Qt.Horizontal


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


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ESP32 RC Slider")
        self.resize(780, 560)

        # Socket unique pour envoyer ET recevoir.
        # En le bindant sur LOCAL_RESPONSE_PORT, l'ESP32 voit ce port comme
        # port source et y renvoie sa reponse.
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

        self.ip_edit = QLineEdit(DRONE_IP)
        self.port_edit = QSpinBox()
        self.port_edit.setRange(1, 65535)
        self.port_edit.setValue(DRONE_PORT)

        self.interval_edit = QSpinBox()
        self.interval_edit.setRange(10, 1000)
        self.interval_edit.setValue(RC_INTERVAL_MS)
        self.interval_edit.setSuffix(" ms")
        self.interval_edit.valueChanged.connect(self.timer.setInterval)

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

        # Indicateur de reponse visible
        self.response_label = QLabel("En attente de reponse...")
        self.response_label.setObjectName("response")

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        self.setCentralWidget(self.build_ui())
        self.refresh_preview()
        self.log_line(f"Pret. Socket lie au port {LOCAL_RESPONSE_PORT}. Connecte-toi au WiFi peter_pan.")

    def build_ui(self) -> QWidget:
        root = QWidget()
        main = QVBoxLayout(root)

        network = QGroupBox("Reseau")
        network_layout = QGridLayout(network)
        network_layout.addWidget(QLabel("IP ESP32"), 0, 0)
        network_layout.addWidget(self.ip_edit, 0, 1)
        network_layout.addWidget(QLabel("Port UDP"), 0, 2)
        network_layout.addWidget(self.port_edit, 0, 3)
        network_layout.addWidget(QLabel("Periode"), 0, 4)
        network_layout.addWidget(self.interval_edit, 0, 5)

        sliders = QGroupBox("Message RC")
        sliders_layout = QVBoxLayout(sliders)
        for key in ("left", "forward", "up", "yaw"):
            sliders_layout.addWidget(self.controls[key])
        sliders_layout.addWidget(self.preview)

        buttons = QHBoxLayout()
        buttons.addWidget(self.command_btn)
        buttons.addWidget(self.send_once_btn)
        buttons.addWidget(self.continuous_check)
        buttons.addStretch(1)
        buttons.addWidget(self.zero_btn)

        main.addWidget(network)
        main.addWidget(sliders)
        main.addLayout(buttons)
        main.addWidget(self.response_label)
        main.addWidget(QLabel("Journal"))
        main.addWidget(self.log, 1)

        root.setStyleSheet(
            """
            QWidget {
                font-size: 14px;
            }
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
            """
        )

        return root

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

    def log_line(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log.append(f"[{timestamp}] {text}")

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        self.timer.stop()
        self.receiver.stop()
        self.receiver.wait(1000)
        self.sock.close()
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
