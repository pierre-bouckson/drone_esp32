## OTA update

For compile

```bash
pio run
```

To Flash

```bash
cd
python3 /home/mea/.platformio/packages/framework-arduinoespressif32/tools/espota.py -i 192.168.4.1 -p 3232 -a esp32pass -f ~/Arduino/ws/peter_pan/.pio/build/esp32dev/firmware.bin
```