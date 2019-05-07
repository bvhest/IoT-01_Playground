# MQTT

## MQTT Client

Before we begin the next section, you might want another application handy—a standalone MQTT client. You could try:

  * [MQTT.fx](http://mqttfx.org/) (GUI; Windows/Mac)
  * [MQTTBox](http://workswithweb.com/mqttbox.html) (GUI; Windows/Mac/Linux)
  * mosquitto-clients from [Mosquitto](https://mosquitto.org/) is available via package manager (CLI; Linux/Mac)
  * Various free clients on app stores (iOS/Android)
  * [Node-RED](https://nodered.org/) can also connect to an MQTT broker (Web; Windows/Mac/Linux)

Using one isn’t strictly necessary, but will aid experimentation

Source: taken from [micropython-on-esp32-part-2](https://boneskull.com/micropython-on-esp32-part-2/)


## MQTT broker

### Mosquitto.org

*public* MQTT broker [`test.mosquitto.org`](http://test.mosquitto.org/), by the [Mosquitto](https://mosquitto.org/) project, is a public broker.

As a member of the public, you can use it! Just be aware: **any data or information you publish on a public MQTT broker is alsopublic**. Don’t publish anything you wouldn’t want *everyone* to know about.

The server listens on the following ports:

  * 1883 : MQTT, unencrypted
  * 8883 : MQTT, encrypted
  * 8884 : MQTT, encrypted, client certificate required
  * 8080 : MQTT over WebSockets, unencrypted
  * 8081 : MQTT over WebSockets, encrypted

The encrypted ports support TLS v1.2, v1.1 or v1.0 with x509 certificates and require client support to connect. In all cases you should use the certificate authority file ([mosquitto.org.crt (PEM format)](http://test.mosquitto.org/ssl/mosquitto.org.crt), or [mosquitto.org.der (DER format)](http://test.mosquitto.org/ssl/mosquitto.org.der)) to verify the server connection. Port 8884 requires clients to provide a certificate to authenticate their connection. It is now possible to[generate your own certificate](http://test.mosquitto.org/ssl).

*Important to note:* the “time and date” you see in the payload detail does **not** mean “when the originating client sent the message.” Rather, it means “when the receiving client received the message.” MQTT messages do not contain a “sent on” timestamp unless you add one yourself!

###  Watson IoT Platform

#### Quickstart

You can experiment with this platform without needing to sign up for an account.

1. Visit the [Quickstart](https://quickstart.internetofthings.ibmcloud.com/#/) page:
   ![Watson IoT Platform Screenshot](https://boneskull.com/content/images/2018/01/quickstart.png)Watson IoT Platform's Quickstart Page
2. Tick “I Accept” after carefully reading the entire terms of use.
3. Enter a unique device identifier in the input box. I’m calling mine “boneskull-esp32-test”. Click “Go”.

Keep this browser window open; you’re now ready to send data, and see the result in real-time

Watson IoT Platform has a free tier. To sign up, you need to:

1. [Register with IBM Cloud](https://console.bluemix.net/registration) (no credit card needed)
2. [Create a Watson IoT Platform service instance](https://console.bluemix.net/catalog/services/internet-of-things-platform) using the “free plan” from the catalog
3. Click “Launch” to explore the platform.
4. Also, check out [the docs](https://console.bluemix.net/docs/services/IoT/index.html).



#### The `micropython-watson-iot` module

The `micropython-watson-iot` library offers a few “quality of life” benefits—as IoT platforms typically do—when compared to a vanilla MQTT client and/or broker:

1. Messages contain metadata such as “published on” time, handled by the cloud platform
2. You can group devices via logical “device types”
3. Structured data can be automatically encoded/decoded to/from JSON (it does this by default)
4. Create your own custom encoders and decoders (e.g., numeric, Base64)
5. Create custom “command handlers,” which cause the device to react upon reception of a “command”-style MQTT message. For example, you could send a command to blink an onboard LED or reboot the device.

> I’ve committed a few [micropython-watson-iot examples](https://github.com/boneskull/micropython-watson-iot/tree/master/example); you can use adapt these patterns to your own code.

[micropython-watson-iot](https://github.com/boneskull/micropython-watson-iot) is the module I referenced earlier. Its README contains installation instructions using `upip`, but essentially it’s the same as before, via the REPL:

```python
import upip
upip.install('micropython-watson-iot')
```

To verify installation, run:

```python
from watson_iot import Device
```

Assuming that didn’t throw an exception, we can use it like so:

```python
d = Device(device_id='boneskull-esp32-test')
d.connect()
d.publishEvent('temperature', {'degrees': 68.5, 'unit': 'F'})
```