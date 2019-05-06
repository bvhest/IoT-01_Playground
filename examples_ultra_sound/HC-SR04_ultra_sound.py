#
# Voorbeeld ultra_sound.py om een ultra-sound-sensor uit te lezen.
# Deze opzet werkt voor de "Ultrasonische Sensor - HC-SR04".
#
# Bron:
#   - https://github.com/rsc1975/micropython-hcsr04
#
# BvH 2019-05-06
#

from hcsr04 import HCSR04

sensor = HCSR04(trigger_pin=16, echo_pin=0, echo_timeout_us=10000)

# voorbeeld met foutafhandeling (bijv. door een time-out bij het uitlezen):
try:
    distance = sensor.distance_cm()
    print('Distance:', distance, 'cm')
except OSError as ex:
    print('ERROR getting distance:', ex)
