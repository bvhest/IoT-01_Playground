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

### test.mosquitto.org

*public* MQTT broker [`test.mosquitto.org`](http://test.mosquitto.org/), by the [Mosquitto](https://mosquitto.org/) project, is a public broker.

As a member of the public, you can use it! Just be aware: **any data or information you publish on a public MQTT broker is alsopublic**. Don’t publish anything you wouldn’t want *everyone* to know about.

The server listens on the following ports:

  * 1883 : MQTT, unencrypted
  * 8883 : MQTT, encrypted
  * 8884 : MQTT, encrypted, client certificate required
  * 8080 : MQTT over WebSockets, unencrypted
  * 8081 : MQTT over WebSockets, encrypted

The encrypted ports support TLS v1.2, v1.1 or v1.0 with x509 certificates and require client support to connect. In all cases you should use the certificate authority file ([mosquitto.org.crt (PEM format)](http://test.mosquitto.org/ssl/mosquitto.org.crt), or [mosquitto.org.der (DER format)](http://test.mosquitto.org/ssl/mosquitto.org.der)) to verify the server connection. Port 8884 requires clients to provide a certificate to authenticate their connection. It is now possible to[generate your own certificate](http://test.mosquitto.org/ssl).
