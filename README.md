## Compilation et flash

Compiler :

```bash
pio run
```

Flasher via USB (port série à adapter) :

```bash
pio run -t upload
```

## Console série

Écouter la sortie série (115200 bauds, lu depuis `monitor_speed` dans `platformio.ini`) :

```bash
pio device monitor
```

En précisant le port explicitement :

```bash
pio device monitor -p /dev/cu.usbserial-0001 -b 115200
```

Lister les ports disponibles :

```bash
pio device list
```

Quitter le moniteur : `Ctrl-C`.

Sortie attendue au démarrage :

```
Console ready !
AP IP address: 192.168.4.1
Initialisation LSM6DSOX...
LSM6DSOX détecté !
IMU init
```

Si rien ne s'affiche, la carte n'a pas redémarré depuis l'ouverture du port : appuyer sur le bouton `EN`/`RST`.
