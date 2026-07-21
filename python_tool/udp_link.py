"""
Reception de la telemetrie du drone en UDP.

Le firmware diffuse a 20 Hz sur le sous-reseau de son point d'acces
(192.168.4.255) une ligne de texte par cycle :

    ROLL: 1.23 deg, PITCH: -4.56 deg | RA:1.10 PA:-4.30 | CR:2.10 CP:-3.40 | M1:5 M2:-1 M3:-5 M4:1

    ROLL/PITCH : angles fusionnes par le filtre de Kalman (degres)
    RA/PA      : angles bruts de l'accelerometre, avant filtrage (degres)
    CR/CP      : sortie des PID de roll et pitch
    M1..M4     : contribution des PID a chaque moteur, gaz exclus

Les champs apres ROLL/PITCH sont optionnels dans le parsing : une trame
tronquee reste exploitable.
"""

import re
import socket
from dataclasses import dataclass, field

# Port d'ecoute, identique a hostPort dans include/AppConfig.h.
TELEMETRY_PORT = 8894

_PATTERN = re.compile(
    r"ROLL:\s*(-?[\d.]+)\s*deg.*?PITCH:\s*(-?[\d.]+)\s*deg"
    r"(?:.*?RA:\s*(-?[\d.]+).*?PA:\s*(-?[\d.]+))?"
    r"(?:.*?CR:\s*(-?[\d.]+).*?CP:\s*(-?[\d.]+))?"
    r"(?:.*?M1:\s*(-?\d+).*?M2:\s*(-?\d+).*?M3:\s*(-?\d+).*?M4:\s*(-?\d+))?"
)


@dataclass
class Telemetry:
    """Un cycle de telemetrie decode."""
    roll: float = 0.0
    pitch: float = 0.0
    roll_acc: float = 0.0    # brut accelerometre ; retombe sur roll si absent
    pitch_acc: float = 0.0
    corr_roll: float = 0.0   # sortie PID
    corr_pitch: float = 0.0
    motors: tuple = (0, 0, 0, 0)


def parse(line: str):
    """Decode une ligne de telemetrie, ou None si elle ne correspond pas."""
    m = _PATTERN.search(line)
    if not m:
        return None

    g = m.groups()
    roll, pitch = float(g[0]), float(g[1])

    return Telemetry(
        roll=roll,
        pitch=pitch,
        # Sans champ RA/PA on retombe sur l'angle filtre : les deux courbes se
        # superposent au lieu de s'ecraser a zero.
        roll_acc=float(g[2]) if g[2] else roll,
        pitch_acc=float(g[3]) if g[3] else pitch,
        corr_roll=float(g[4]) if g[4] else 0.0,
        corr_pitch=float(g[5]) if g[5] else 0.0,
        motors=tuple(int(v) for v in g[6:10]) if g[6] else (0, 0, 0, 0),
    )


class UdpTelemetry:
    """
    Socket d'ecoute non bloquante.

    Usage : open(), puis poll() regulierement depuis un timer d'IHM.
    poll() vide la file de reception et rend les trames decodees.
    """

    def __init__(self, port: int = TELEMETRY_PORT):
        self.port = port
        self._sock = None

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def open(self):
        """Ouvre l'ecoute. Leve OSError si le port est deja pris."""
        if self._sock is not None:
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Permet a plusieurs outils d'ecouter la meme diffusion en parallele.
        if hasattr(socket, "SO_REUSEPORT"):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

        try:
            sock.bind(("", self.port))   # toutes interfaces, pour capter le broadcast
        except OSError:
            sock.close()
            raise

        sock.setblocking(False)
        self._sock = sock

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def poll(self):
        """Rend la liste des trames recues depuis le dernier appel."""
        if self._sock is None:
            return []

        out = []
        while True:
            try:
                data, _ = self._sock.recvfrom(2048)
            except BlockingIOError:
                break            # plus rien en attente
            except OSError:
                break            # socket fermee entre-temps

            for line in data.decode("utf-8", errors="ignore").splitlines():
                frame = parse(line)
                if frame is not None:
                    out.append(frame)
        return out
