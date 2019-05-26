### MQTT

Voor het maken van de MQTT-verbinding moeten we controleren of er een
WiFI-verbinding is. In de Arduino-code ziet dit er als volgt uit:

```

```

Een MQTT-verbinding kan verbroken worden

In de Arduino-code:

```{cpp}
if (!client.connected()) {
   if client.(connect("clientName")) {
     client.publish("clients", "reconnect");
   }
 }
```

 Bij mijn weten heeft de MicroPython umqtt geen functie voor `connected`.

 NB: de verbinding kan door allerlei oorzaken verbroken worden, ook bijv. doordat
 een server opnieuw opgestart wordt.

 De Arduino mqtt-method `client.loop()` zorgt voor het onderhouden van de
 communicatie met de host: o.a. om in het geval van een ontvangen bericht,
 de callback-functie aan te roepen.

 In het geval van de ESP8266 kunnen we `client.check_msg()` gebruiken: die heeft
 ongeveer dezelfde functie.

 Kan een "publish" actie resulteren in een exception - bijv. als een verbinding
 verbroken is? Kan het wachten op een message resulteren in een exception?


 De Arduino loop:

```{cpp}
 void loop() {
   if (!client.connected()) {
     reconnect();
   }
   client.loop();

   if (digitalRead(button0)) {
     sensor0Publish();
     delay(200); // limit button repetition rate
   }

   if (millis() >= sensor1Timer) {
     sensor1Publish();
     sensor1Timer = sensor1Timer + sensor1Period;
   }
```

In de `reconnect` van umqtt.robust wordt een `delay` gebruikt; dit kan eigenlijk
geen kwaad omdat er bij gebrek aan verbinding toch geen andere actie kan
plaatsvinden.

In veel opzichten lijkt de module `umqtt.robust` te bieden wat we nodig hebben;
maar, deze heeft geen versie van `check_msg`. Er is alleen een *blokkerende*
versie: `wait_msg`.

De umqtt gebruik *blokkerende sockets*: dat betekent dat je eigenlijk met
timeouts moet werken, om een echt robuuste implementatie te krijgen. En wat
moet je dan doen met een blokkerende socket? Kun je die opruimen o.i.d.?

* zie: https://forum.micropython.org/viewtopic.php?t=2239
* en: https://github.com/micropython/micropython/issues/2568
* en: https://github.com/micropython/micropython-lib/issues/103

Ander issue: Raspberry Pi Mosquitto-versie:

* http://diy.viktak.com/2015/05/installing-mosquitto-for-use-with.html

Eventueel kunnen we de Arduino-versie (pubsubclient) als voorbeeld gebruiken;
volgens mij is die behoorlijk robuust, ik heb deze al maanden zonder problemen
in gebruik. Een essentieel element daarvan is volgens mij het gebruik van
niet-blokkerende operaties.
