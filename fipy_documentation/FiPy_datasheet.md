# FiPy datasheet

source: https://docs.pycom.io/datasheets/development/fipy.html

### WiFi

By default, upon boot the FiPy will create a WiFi access point with the SSID `fipy-wlan-XXXX`, where `XXXX` is a random 4-digit number, and the password `www.pycom.io`.

The RF switch that selects between the on-board and external antenna is connected to `P12`, for this reason using `P12`should be avoided unless WiFi is disabled in your application.

### Power

The `Vin` pin on the FiPy can be supplied with a voltage ranging from `3.5v` to `5.5v`. The `3.3v` pin on the other hand is output **only**, and must not be used to feed power into the FiPy, otherwise the on-board regulator will be damaged.

### AT Commands

The AT commands for the Sequans Monarch modem on the FiPy are available in a PDF file.

[AT Commands for Sequans](https://docs.pycom.io/.gitbook/assets/Monarch-LR5110-ATCmdRefMan-rev6_noConfidential.pdf)

## Tutorials

Tutorials on how to the FiPy module can be found in the [examples](https://docs.pycom.io/tutorials/introduction.html) section of this documentation. The following tutorials might be of specific interest for the FiPy:

- [WiFi connection](https://docs.pycom.io/tutorials/all/wlan.html)
- [LoRaWAN node](https://docs.pycom.io/tutorials/lora/lorawan-abp.html)
- [LoRaWAN nano gateway](https://docs.pycom.io/tutorials/lora/lorawan-nano-gateway.html)
- [Sigfox](https://docs.pycom.io/tutorials/sigfox.html)
- [LTE CAT-M1](https://docs.pycom.io/tutorials/lte/cat-m1.html)
- [NB-IoT](https://docs.pycom.io/tutorials/lte/nb-iot.html)
- [BLE](https://docs.pycom.io/tutorials/all/ble.html)