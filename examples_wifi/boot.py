#
# Voorbeeld boot.py om een wifi-verbinding op te zetten
#
# Bron: https://boneskull.com/micropython-on-esp32-part-1/
#
# BvH 2019-05-06
#
def no_debug():
    import esp
    # this can be run from the REPL as well
    esp.osdebug(None)

def connect():
    import network
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect('<YOUR WIFI SSID>', '<YOUR WIFI PASS>')
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ifconfig())

# disable the info-berichten. NB print() werkt nog wel.
no_debug()
# verbind met, in connect(), geconfigureerde netwerk.
# na verbinden wordt ip-adres getoond.
connect()
