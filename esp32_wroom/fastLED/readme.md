# MicroPython performance example

## Intro

Although micropython is an interpreted language, it can be pretty fast. And with some tricks, it can be made to run even faster.

This code shows some examples.

## Results

1. fastLed1.py
	LED flashes with 1 Hz (based on delay)
2. fastLed2.py
	8.404 s, 84.036 uSec/blink :    11.54 kHz/s
3. fastLed3.py
	 	3.548 s, 35.483 uSec/blink :    28.18 kHz/s
4. fastLed4.py
	3.218 s, 32.181 uSec/blink :    31.07 kHz/s
5. fastLed5.py
	0.459 s,  4.589 uSec/blink :   217.91 kHz/s
6. fastLed6.py
	0.378 s,  3.777 uSec/blink :   264.76 kHz/s
7. fastLed7.py
	native 20% sneller dan versie 6.
	... maar niet standaard ondersteund. Vereist een her-compilatie
	  van de micropython interpreter met een aangepaste vlag.
8. fastLed8.py
	viper is een factor 6(!!!) sneller dan de native variant.
	 ... maar niet standaard ondersteund. Vereist een her-compilatie
	     van de micropython interpreter met een aangepaste vlag.
