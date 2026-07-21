#!/usr/bin/env python3
"""
IMU 3D Visualizer — telemetrie UDP diffusee par le drone (voir udp_link.py).
Se connecter au point d'acces WiFi de l'ESP32, puis ecouter le port 8894.
"""

import sys
import math
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QFrame, QSizePolicy, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette, QLinearGradient, QPainter
from PyQt5.QtOpenGL import QGLWidget

from udp_link import UdpTelemetry, TELEMETRY_PORT

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    HAS_OPENGL = True
except ImportError:
    HAS_OPENGL = False
    print("PyOpenGL not found. Install with: pip install PyOpenGL PyOpenGL_accelerate")


# ──────────────────────────────────────────────
#  OpenGL 3-D board widget
# ──────────────────────────────────────────────
class IMUGLWidget(QGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll  = 0.0
        self.pitch = 0.0
        self._anim_roll  = 0.0
        self._anim_pitch = 0.0
        self.setMinimumSize(480, 380)

        # smooth animation timer
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._animate)
        self._timer.start(16)   # ~60 fps

    def set_angles(self, roll: float, pitch: float):
        self.roll  = roll
        self.pitch = pitch

    def _animate(self):
        # exponential smoothing toward target
        alpha = 0.12
        self._anim_roll  += alpha * (self.roll  - self._anim_roll)
        self._anim_pitch += alpha * (self.pitch - self._anim_pitch)
        self.updateGL()

    # ── OpenGL setup ──────────────────────────
    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_LINE_SMOOTH)
        glShadeModel(GL_SMOOTH)
        glClearColor(0.06, 0.07, 0.10, 1.0)

        light_pos = [2.0, 4.0, 3.0, 1.0]
        glLightfv(GL_LIGHT0, GL_POSITION,  light_pos)
        glLightfv(GL_LIGHT0, GL_AMBIENT,   [0.25, 0.25, 0.30, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE,   [0.90, 0.90, 0.95, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR,  [0.60, 0.60, 0.60, 1.0])

    def resizeGL(self, w, h):
        glViewport(0, 0, w, max(h, 1))
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, w / max(h, 1), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # camera
        gluLookAt(0, 2.2, 5.5,
                  0, 0,   0,
                  0, 1,   0)

        # ── world grid ──────────────────────────
        self._draw_grid()
        self._draw_axes_world()

        # ── board with IMU rotations ────────────
        glPushMatrix()
        glRotatef(-self._anim_pitch, 1, 0, 0)   # pitch around X
        glRotatef( self._anim_roll,  0, 0, 1)   # roll  around Z

        self._draw_board()
        self._draw_axes_body()
        glPopMatrix()

    # ── scene elements ─────────────────────────
    def _draw_grid(self):
        glDisable(GL_LIGHTING)
        glLineWidth(1.0)
        N, S = 10, 0.4
        glBegin(GL_LINES)
        for i in range(-N, N + 1):
            t = i * S
            # dim teal lines
            alpha = 0.15 if i % 5 != 0 else 0.35
            glColor4f(0.20, 0.55, 0.55, alpha)
            glVertex3f(-N * S, -0.01, t); glVertex3f(N * S, -0.01, t)
            glVertex3f(t, -0.01, -N * S); glVertex3f(t, -0.01,  N * S)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_axes_world(self):
        """Small XYZ indicator at origin."""
        glDisable(GL_LIGHTING)
        glLineWidth(1.5)
        length = 0.5
        glBegin(GL_LINES)
        glColor3f(0.90, 0.25, 0.25); glVertex3f(0,0,0); glVertex3f(length,0,0)
        glColor3f(0.25, 0.85, 0.25); glVertex3f(0,0,0); glVertex3f(0,length,0)
        glColor3f(0.25, 0.45, 0.95); glVertex3f(0,0,0); glVertex3f(0,0,length)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_board(self):
        """PCB-like board: dark green slab + gold pads."""
        W, H, D = 1.8, 0.08, 1.2   # width, thickness, depth

        # ── main PCB slab ──────────────────────
        glColor4f(0.05, 0.35, 0.18, 1.0)
        self._solid_box(W, D, H)

        # ── silkscreen top ─────────────────────
        glColor4f(0.85, 0.85, 0.85, 0.25)
        self._solid_box(W * 0.96, D * 1.002, H * 0.96, offset_y=0)

        # ── IMU chip (dark square on top) ──────
        cx, cz = 0.0, 0.0
        glPushMatrix()
        glTranslatef(cx, H / 2 + 0.001, cz)
        glColor4f(0.12, 0.12, 0.14, 1.0)
        self._flat_box(0.35, 0.35, 0.04)
        # chip label dot
        glColor4f(0.90, 0.90, 0.15, 1.0)
        self._flat_box(0.04, 0.04, 0.041, ox=-0.13, oz=-0.12)
        glPopMatrix()

        # ── gold edge pads ─────────────────────
        glColor4f(0.85, 0.65, 0.10, 1.0)
        pad_w, pad_h = 0.06, 0.04
        n_pads = 8
        for i in range(n_pads):
            t = -0.45 + i * (0.90 / (n_pads - 1))
            glPushMatrix()
            glTranslatef(t, 0, -D / 2 - 0.001)
            self._flat_box(pad_w, pad_h, 0.002)
            glPopMatrix()
            glPushMatrix()
            glTranslatef(t, 0, D / 2 + 0.001)
            self._flat_box(pad_w, pad_h, 0.002)
            glPopMatrix()

        # ── up-arrow on top face ───────────────
        glDisable(GL_LIGHTING)
        glColor4f(0.30, 0.90, 0.55, 0.80)
        glLineWidth(2.5)
        y = H / 2 + 0.003
        glBegin(GL_LINE_STRIP)
        glVertex3f( 0.00, y, -0.35)
        glVertex3f( 0.00, y,  0.25)
        glEnd()
        glBegin(GL_LINES)
        glVertex3f(-0.10, y, -0.25)
        glVertex3f( 0.00, y, -0.35)
        glVertex3f( 0.10, y, -0.25)
        glVertex3f( 0.00, y, -0.35)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_axes_body(self):
        """Body-frame axes attached to board."""
        glDisable(GL_LIGHTING)
        glLineWidth(3.0)
        L = 1.1
        glBegin(GL_LINES)
        glColor3f(1.0, 0.30, 0.30); glVertex3f(0,0,0); glVertex3f(L, 0, 0)
        glColor3f(0.30, 1.0, 0.30); glVertex3f(0,0,0); glVertex3f(0, L, 0)
        glColor3f(0.35, 0.55, 1.0); glVertex3f(0,0,0); glVertex3f(0, 0, L)
        glEnd()
        glEnable(GL_LIGHTING)

    # ── geometry helpers ───────────────────────
    def _solid_box(self, w, d, h, offset_y=0):
        """Draw a box centered at origin (xz-plane), height along y."""
        x, y, z = w / 2, h / 2, d / 2
        o = offset_y
        faces = [
            # top
            ([0, 1, 0],  [(-x,y+o,z),(x,y+o,z),(x,y+o,-z),(-x,y+o,-z)]),
            # bottom
            ([0,-1, 0],  [(-x,-y+o,-z),(x,-y+o,-z),(x,-y+o,z),(-x,-y+o,z)]),
            # front
            ([0, 0, 1],  [(-x,-y+o,z),(x,-y+o,z),(x,y+o,z),(-x,y+o,z)]),
            # back
            ([0, 0,-1],  [(-x,y+o,-z),(x,y+o,-z),(x,-y+o,-z),(-x,-y+o,-z)]),
            # right
            ([1, 0, 0],  [(x,-y+o,-z),(x,y+o,-z),(x,y+o,z),(x,-y+o,z)]),
            # left
            ([-1,0, 0],  [(-x,-y+o,z),(-x,y+o,z),(-x,y+o,-z),(-x,-y+o,-z)]),
        ]
        glBegin(GL_QUADS)
        for normal, verts in faces:
            glNormal3fv(normal)
            for v in verts:
                glVertex3fv(v)
        glEnd()

    def _flat_box(self, w, d, h, ox=0, oz=0):
        """Flat box (thin slab) centered at ox, oz, on XZ plane."""
        x, y, z = w / 2 + ox, h / 2, d / 2 + oz
        bx, bz  = ox - w / 2, oz - d / 2
        tx, tz  = ox + w / 2, oz + d / 2
        glBegin(GL_QUADS)
        glNormal3f(0, 1, 0)
        glVertex3f(bx, y, bz); glVertex3f(tx, y, bz)
        glVertex3f(tx, y, tz); glVertex3f(bx, y, tz)
        glNormal3f(0,-1, 0)
        glVertex3f(bx,-y, tz); glVertex3f(tx,-y, tz)
        glVertex3f(tx,-y, bz); glVertex3f(bx,-y, bz)
        glEnd()


# ──────────────────────────────────────────────
#  Gauge widget  (roll or pitch)
# ──────────────────────────────────────────────
class GaugeWidget(QWidget):
    def __init__(self, label: str, color: str, parent=None):
        super().__init__(parent)
        self.label = label
        self.color = QColor(color)
        self.value = 0.0
        self.setMinimumSize(160, 160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, v: float):
        self.value = v
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        r = min(cx, cy) - 18

        # background disc
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(18, 22, 32))
        p.drawEllipse(cx - r, cy - r, 2 * r, 2 * r)

        # arc track
        from PyQt5.QtGui import QPen
        pen = QPen(QColor(40, 55, 75), 6)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawArc(cx - r + 3, cy - r + 3, 2*(r-3), 2*(r-3), 0, 360 * 16)

        # value arc  (−90 .. +90 → full arc = 270°, centered at bottom)
        span = max(-90, min(90, self.value))
        start_angle = 225 * 16   # 225° from right = start at lower-left
        arc_span    = int(-span / 90 * 135 * 16)   # 135° each side
        pen2 = QPen(self.color, 6)
        pen2.setCapStyle(Qt.RoundCap)
        p.setPen(pen2)
        p.drawArc(cx - r + 3, cy - r + 3, 2*(r-3), 2*(r-3), start_angle, arc_span)

        # center value text
        p.setPen(QColor(230, 235, 245))
        p.setFont(QFont("Courier New", int(r * 0.36), QFont.Bold))
        p.drawText(0, 0, w, h, Qt.AlignCenter, f"{self.value:+.1f}°")

        # label
        p.setFont(QFont("Courier New", 9))
        p.setPen(self.color.lighter(140))
        p.drawText(0, h - 26, w, 22, Qt.AlignCenter, self.label)

        # tick marks
        pen3 = QPen(QColor(60, 80, 100), 1)
        p.setPen(pen3)
        import math as _m
        for deg in range(-90, 91, 30):
            angle_rad = _m.radians(225 - deg * 1.5)
            x1 = cx + (r - 8)  * _m.cos(angle_rad)
            y1 = cy - (r - 8)  * _m.sin(angle_rad)
            x2 = cx + (r - 14) * _m.cos(angle_rad)
            y2 = cy - (r - 14) * _m.sin(angle_rad)
            p.drawLine(int(x1), int(y1), int(x2), int(y2))


# ──────────────────────────────────────────────
#  History mini-plot
# ──────────────────────────────────────────────
class PlotWidget(QWidget):
    """Simple scrolling line plot for roll & pitch history."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self._roll  = deque(maxlen=200)
        self._pitch = deque(maxlen=200)
        self.setMinimumHeight(90)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def add(self, roll, pitch):
        self._roll.append(roll)
        self._pitch.append(pitch)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(12, 16, 24))

        # grid
        from PyQt5.QtGui import QPen
        p.setPen(QPen(QColor(30, 45, 60), 1))
        for pct in [0.25, 0.5, 0.75]:
            y = int(h * pct)
            p.drawLine(0, y, w, y)
        p.setPen(QPen(QColor(40, 60, 80), 1))
        p.drawLine(0, h//2, w, h//2)

        def draw_series(data, color):
            if len(data) < 2:
                return
            pen = QPen(QColor(color), 1.5)
            p.setPen(pen)
            n = len(data)
            pts = list(data)
            for i in range(1, n):
                x1 = int((i - 1) / (n - 1) * w)
                x2 = int(i       / (n - 1) * w)
                y1 = int(h / 2 - pts[i-1] / 90 * h / 2.2)
                y2 = int(h / 2 - pts[i]   / 90 * h / 2.2)
                p.drawLine(x1, y1, x2, y2)

        draw_series(self._roll,  "#FF6B6B")
        draw_series(self._pitch, "#4ECDC4")

        # legend
        p.setFont(QFont("Courier New", 8))
        p.setPen(QColor("#FF6B6B")); p.drawText(6,  14, "ROLL")
        p.setPen(QColor("#4ECDC4")); p.drawText(6,  28, "PITCH")


# ──────────────────────────────────────────────
#  Main window
# ──────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU 3D Visualizer")
        self.resize(960, 700)

        self.link = UdpTelemetry()

        # Vidage de la file UDP a 50 Hz : plus rapide que l'emission du drone
        # (20 Hz), donc aucune trame ne s'accumule.
        self._poll = QTimer(self)
        self._poll.setInterval(20)
        self._poll.timeout.connect(self._drain)
        self._poll.start()

        # dark palette
        pal = QPalette()
        pal.setColor(QPalette.Window,     QColor(10, 13, 20))
        pal.setColor(QPalette.WindowText, QColor(200, 210, 225))
        pal.setColor(QPalette.Base,       QColor(16, 20, 30))
        pal.setColor(QPalette.Text,       QColor(200, 210, 225))
        QApplication.instance().setPalette(pal)
        self.setStyleSheet("""
            QMainWindow { background: #0a0d14; }
            QLabel      { color: #c8d2e1; font-family: 'Courier New'; }
            QComboBox   { background: #141820; color: #a0c4ff;
                          border: 1px solid #203050; padding: 4px 8px;
                          font-family: 'Courier New'; border-radius: 4px; }
            QComboBox::drop-down { border: none; }
            QPushButton { background: #1a2840; color: #7ec8e3;
                          border: 1px solid #2a5070; padding: 6px 18px;
                          font-family: 'Courier New'; font-weight: bold;
                          border-radius: 4px; }
            QPushButton:hover   { background: #22385a; color: #a0e0ff; }
            QPushButton:checked { background: #0d4030; color: #3ddc97;
                                  border-color: #3ddc97; }
        """)

        self._build_ui()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(14, 10, 14, 10)
        main.setSpacing(10)

        # ── top bar ─────────────────────────────
        top = QHBoxLayout()
        title = QLabel("◈ IMU 3D VISUALIZER")
        title.setFont(QFont("Courier New", 14, QFont.Bold))
        title.setStyleSheet("color: #7ec8e3; letter-spacing: 3px;")

        self.port_edit = QLineEdit(str(TELEMETRY_PORT))
        self.port_edit.setFixedWidth(90)
        self.port_edit.setAlignment(Qt.AlignCenter)

        self.connect_btn = QPushButton("⏵  LISTEN")
        self.connect_btn.setCheckable(True)
        self.connect_btn.toggled.connect(self._toggle_link)

        self.status_lbl = QLabel("● OFFLINE")
        self.status_lbl.setStyleSheet("color: #ff5555; font-family: 'Courier New';")

        top.addWidget(title)
        top.addStretch()
        top.addWidget(QLabel("Port UDP:"))
        top.addWidget(self.port_edit)
        top.addWidget(self.connect_btn)
        top.addSpacing(12)
        top.addWidget(self.status_lbl)
        main.addLayout(top)

        # ── separator ───────────────────────────
        sep = QFrame(); sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #1a2840;")
        main.addWidget(sep)

        # ── content area ────────────────────────
        content = QHBoxLayout()
        content.setSpacing(12)

        # 3-D view
        if HAS_OPENGL:
            self.gl_widget = IMUGLWidget()
        else:
            self.gl_widget = QLabel("⚠ PyOpenGL not installed\npip install PyOpenGL")
            self.gl_widget.setAlignment(Qt.AlignCenter)
            self.gl_widget.setStyleSheet("background:#0d1018; color:#ff6b6b;")
            self.gl_widget.setMinimumSize(480, 380)

        content.addWidget(self.gl_widget, 3)

        # right panel
        right = QVBoxLayout()
        right.setSpacing(10)

        # gauges
        gauges_row = QHBoxLayout()
        self.roll_gauge  = GaugeWidget("ROLL",  "#FF6B6B")
        self.pitch_gauge = GaugeWidget("PITCH", "#4ECDC4")
        gauges_row.addWidget(self.roll_gauge)
        gauges_row.addWidget(self.pitch_gauge)
        right.addLayout(gauges_row)

        # numeric readout
        num_frame = QFrame()
        num_frame.setStyleSheet("background:#0d1018; border:1px solid #1a2840; border-radius:6px;")
        num_layout = QGridLayout(num_frame)
        num_layout.setContentsMargins(12, 8, 12, 8)

        def make_val_label():
            l = QLabel("---")
            l.setFont(QFont("Courier New", 18, QFont.Bold))
            l.setAlignment(Qt.AlignCenter)
            return l

        lbl_r = QLabel("ROLL"); lbl_r.setStyleSheet("color:#FF6B6B; font-family:'Courier New'; font-size:10px;"); lbl_r.setAlignment(Qt.AlignCenter)
        lbl_p = QLabel("PITCH"); lbl_p.setStyleSheet("color:#4ECDC4; font-family:'Courier New'; font-size:10px;"); lbl_p.setAlignment(Qt.AlignCenter)
        self.roll_val_lbl  = make_val_label(); self.roll_val_lbl.setStyleSheet("color:#FF6B6B;")
        self.pitch_val_lbl = make_val_label(); self.pitch_val_lbl.setStyleSheet("color:#4ECDC4;")

        num_layout.addWidget(lbl_r,  0, 0)
        num_layout.addWidget(lbl_p,  0, 1)
        num_layout.addWidget(self.roll_val_lbl,  1, 0)
        num_layout.addWidget(self.pitch_val_lbl, 1, 1)
        right.addWidget(num_frame)

        # ── corrections PID / moteurs ───────────
        corr_frame = QFrame()
        corr_frame.setStyleSheet("background:#0d1018; border:1px solid #1a2840; border-radius:6px;")
        corr_layout = QGridLayout(corr_frame)
        corr_layout.setContentsMargins(12, 8, 12, 8)

        corr_title = QLabel("CORRECTION MOTEURS")
        corr_title.setStyleSheet("color:#3a5070; font-family:'Courier New'; font-size:9px; border:none;")
        corr_title.setAlignment(Qt.AlignCenter)
        corr_layout.addWidget(corr_title, 0, 0, 1, 4)

        self.corr_lbl = QLabel("CR  ---   CP  ---")
        self.corr_lbl.setFont(QFont("Courier New", 11, QFont.Bold))
        self.corr_lbl.setStyleSheet("color:#c8a0ff; border:none;")
        self.corr_lbl.setAlignment(Qt.AlignCenter)
        corr_layout.addWidget(self.corr_lbl, 1, 0, 1, 4)

        # M1..M4 : contribution des PID a chaque moteur, gaz exclus.
        self.motor_lbls = []
        for i in range(4):
            name = QLabel(f"M{i + 1}")
            name.setStyleSheet("color:#3a5070; font-family:'Courier New'; font-size:9px; border:none;")
            name.setAlignment(Qt.AlignCenter)
            val = QLabel("---")
            val.setFont(QFont("Courier New", 13, QFont.Bold))
            val.setStyleSheet("color:#7ec8e3; border:none;")
            val.setAlignment(Qt.AlignCenter)
            corr_layout.addWidget(name, 2, i)
            corr_layout.addWidget(val,  3, i)
            self.motor_lbls.append(val)

        right.addWidget(corr_frame)

        # history plot
        plot_lbl = QLabel("HISTORY (last 200 samples)")
        plot_lbl.setStyleSheet("color:#3a5070; font-family:'Courier New'; font-size:9px;")
        right.addWidget(plot_lbl)
        self.plot = PlotWidget()
        right.addWidget(self.plot)

        right.addStretch()
        content.addLayout(right, 2)
        main.addLayout(content)

    # ── UDP control ────────────────────────────
    def _toggle_link(self, checked: bool):
        if checked:
            try:
                self.link.port = int(self.port_edit.text())
                self.link.open()
            except (ValueError, OSError) as e:
                self._on_error(str(e))
                return
            self.connect_btn.setText("⏹  STOP")
            self.status_lbl.setText("● LISTENING")
            self.status_lbl.setStyleSheet("color:#3ddc97; font-family:'Courier New';")
        else:
            self.link.close()
            self.connect_btn.setText("⏵  LISTEN")
            self.status_lbl.setText("● OFFLINE")
            self.status_lbl.setStyleSheet("color:#ff5555; font-family:'Courier New';")

    def _drain(self):
        """Traite les trames en attente ; seule la derniere pilote l'affichage 3D."""
        for frame in self.link.poll():
            self._on_data(frame)

    def _on_data(self, f):
        if HAS_OPENGL:
            self.gl_widget.set_angles(f.roll, f.pitch)
        self.roll_gauge.set_value(f.roll)
        self.pitch_gauge.set_value(f.pitch)
        self.roll_val_lbl.setText(f"{f.roll:+.2f}°")
        self.pitch_val_lbl.setText(f"{f.pitch:+.2f}°")
        self.plot.add(f.roll, f.pitch)

        # Corrections PID : sortie des regulateurs et repartition par moteur.
        self.corr_lbl.setText(f"CR {f.corr_roll:+.2f}   CP {f.corr_pitch:+.2f}")
        for lbl, duty in zip(self.motor_lbls, f.motors):
            lbl.setText(f"{duty:+d}")

    def _on_error(self, msg: str):
        self.status_lbl.setText(f"⚠ {msg[:40]}")
        self.status_lbl.setStyleSheet("color:#ffaa33; font-family:'Courier New';")
        self.connect_btn.setChecked(False)

    def closeEvent(self, event):
        self.link.close()
        super().closeEvent(event)


# ──────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("IMU 3D Visualizer")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())