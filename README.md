## Compilation et flash

Compiler :

```bash
pio run
```

Flasher via USB (port série à adapter) :

```bash
pio run -t upload
```

## Utilitaire RC Qt

Installer Qt pour Python :

```bash
pip install PySide6
```

Lancer l'interface a curseurs :

```bash
python rc_slider_qt.py
```

L'ESP32 ecoute par defaut sur `192.168.4.1:8889` et recoit les messages
`rc <left> <forward> <up> <yaw>`.
