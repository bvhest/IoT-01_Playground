## Help voor ESP8266

### Pymakr en ampy

* de Pymakr plugin maakt de ESP8266-Python REPL toegankelijk; daarmee kun je interactief Python-opdrachten geven, om eenvoudige dingen uit te proberen;
* deze REPL bereik je via het Pymakr-venster; dit open je met de knop helemaal rechts onderin.
* met de Pymakr RUN opdracht kun je een Python-bestand direct uitvoeren op de ESP8266. Dit is handig voor grotere experimenten en testen.
* helaas werkt de Pymakr synchronisatie (nog) niet met de ESP8266/NodeMCU. In plaats daarvan gebruiken we `ampy` (via de commandoregel.)
  * zorg er dan voor dat het Pymakr-venster rechts onderin gesloten is: anders blokkeert dit de USB-verbinding met de ESP8266.

`ampy` opdrachten:

```
$ ampy -p /dev/tty.SLAB_USBtoUART put boot.py
$ ampy -p /dev/tty.SLAB_USBtoUART ls lib
$ ampy -p /dev/tty.SLAB_USBtoUART rm lib
$ ampy -p /dev/tty.SLAB_USBtoUART put lib
```

Met `ampy` kun je bestanden van en naar de ESP8266 kopiëren.

Voordat je de directory `lib` (met alle bestanden daarin) kopieert naar de ESP8266, moet je eerst de bestaande `lib` verwijderen.

#### Configureren van Pymakr

* via Pymakr-menu (rechts onder): Settings->Global, invullen: device address, bijvoorbeeld: `/dev/tty.SLAB_USBtoUART`
* Settings->Project: project-specifieke instellingen.

### config.py

Het bestand config.py bevat de waarden voor je lokale configuratie, zoals de naam en het wachtwoord voor het WiFi-netwerk, en de naam van de mqtt-server. Deze waarde proberen we buiten het versie-beheer te houden, om te voorkomen dat deze via GitHub e.d. verspreid worden. (zie `.gitignore`)

* kopieer `config.py.example` naar `config.py`

### boot.py

Het bestand boot.py wordt bij het opstarten van de ESP8266 automatisch uitgevoerd. Dit is ideaal om bijv. het WiFI-netwerk op te zetten, en systeeminstellingen aan te passen zoals het zoekpad.

Kennelijk heeft de ESP8266 geen `main.py` die automatisch
opgestart wordt, zoals in het geval van de ESP32. We kunnen dit oplossen door in `boot.py` deze main aan te roepen.

### webREPL

webREPL maakt de Python REPL beschikbaar via de WiFi-verbinding. Bovendien kun je hiermee bestanden van en naar de ESP8266 kopiëren.

Enable (or disable) webrepl (interactief, via REPL):

`import webrepl_setup`

Starten van webrepl (als dat niet bij boot gebeurt):

```py
import webrepl
webrepl.start()
```

### i2c

* default-pins voor I2C: 4 (SDA) en 5 (SCL); dit zijn de GPIO-nrs. De NodeMCU
pins zijn:
* D1 (GPIO5)
* D2 (GPIO4) --- dus net andersom!

(Je kunt zelf ook andere pins kiezen, maar dit werkt in elk geval...)
