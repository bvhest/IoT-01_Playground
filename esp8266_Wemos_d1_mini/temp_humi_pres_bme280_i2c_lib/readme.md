# Wemos D1 Mini en BME280 meteo station

## Wemos D1 Mini pinout

![pinout](./docs-hardware/esp8266-wemos-d1-mini-pinout.png)

## Connectie via USB

Bij aansluiten op usb-poort Linux laptop, dan komt de D1 Mini beschikbaar op "/dev/ttyUSB0".

Met `lsusb`

	Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
	Bus 001 Device 002: ID 04f2:b512 Chicony Electronics Co., Ltd 
	Bus 001 Device 008: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter <--- Wemos D1 Mini
	Bus 001 Device 005: ID 8087:0a2b Intel Corp. 
	Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
	
Eenvoudiger via

	dmesg | grep  "converter now attached"

Dit geeft als resultaat

	[13901.137965] usb 1-3: ch341-uart converter now attached to ttyUSB0

Dus de Wemos D1 Mini hangt aan  "/dev/**ttyUSB0**"

Zie 

   * [esp8266-basics](https://steve.fi/Hardware/esp8266-basics/)
   * [micropython-getting-started](https://lemariva.com/blog/2017/10/micropython-getting-started)

By default the permissions are configured such that I cannot read/write to the device, so we'll fix that, and create a handy symlink for ease of future identification. We'll do this by creating a local udev rule in the file /etc/udev/rules.d/99-wemos.rules - create that file, and give it the following contents:

	SUBSYSTEM=="tty", GROUP="plugdev", MODE="0660"
	ACTION=="add", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="wemos"

NOTE: The numbers in bold come from the output of lsusb which we saw earlier. You'll want to use the numbers you see.

Now that the new udev rule is in place we need to reload the service, unplug the device, and plug it in again. That will allow the new rule to be recognized and applied. Reloading the rules can be achieved via:

	# /etc/init.d/udev reload

Once that is complete we'll see we have the symlink, and better permissions.

	$ ls -l /dev/wemos /dev/ttyUSB0
	lrwxrwxrwx 1 root root         7 Jan  5 08:15 /dev/wemos -> ttyUSB0
	crw-rw---- 1 root plugdev 166, 1 Jan  5 08:15 /dev/ttyUSB0

NOTE: My user is a member of the plugdev group, so I can now read/write to the device.

## uploaden bestanden en D1 Mini console

Bestanden uploaden via Ampy:

	# list files
	ampy --port /dev/ttyUSB0 ls -l
	# upload files
	ampy --port /dev/ttyUSB0 put main.py
	ampy --port /dev/ttyUSB0 put bme280_float.py
	# download files
	ampy --port /dev/ttyUSB0 get boot.py > boot.py
]
Console:

	screen /dev/ttyUSB0


## BME280 library

  - https://github.com/triplepoint/micropython_bme280_i2c

## Adafruit IO account & feed

Feed name: weerdata_esp8266_bme280
Feed Key : weerdata-esp8266-bme280

**Current Endpoints**:

Web = https://io.adafruit.com/uden/feeds/weerdata-esp8266-bme280

API = https://io.adafruit.com/api/v2/uden/feeds/weerdata-esp8266-bme280

MQTT by Key = uden/feeds/weerdata-esp8266-bme280


## Example projects

  - https://github.com/arendst/Sonoff-Tasmota/wiki/Wemos-D1-Mini-and-BME280-Temperature,-Humidity-and-Pressure-Sensor
  - http://www.esp8266learning.com/esp8266-and-bme280-temperature-sensor-example.php
  - http://domoticx.com/esp8266-wifi-temperatuur-luchtvochtigheid-barometer-bme280-arduinoide/
  - https://www.instructables.com/id/ESP8266-NodeMCU-With-BME280-Gauges-Chart/
  - http://embedded-lab.com/blog/making-simple-weather-web-server-using-esp8266-bme280/
  - https://robotzero.one/esp8266-and-bme280-temp-pressure-and-humidity-sensor-spi/
  - https://lastminuteengineers.com/bme280-esp8266-weather-station/
  - https://www.hackster.io/bobbyleonard84/python-micropython-sensor-logger-with-google-sheets-fd2b93


## Issues

  - [instructies uitwerken](https://www.instructables.com/id/The-Super-Easy-Micropython-ESP8266-Guide-No-Guessw/)

## ToDo

  - [instructies uitwerken](https://www.instructables.com/id/The-Super-Easy-Micropython-ESP8266-Guide-No-Guessw/)