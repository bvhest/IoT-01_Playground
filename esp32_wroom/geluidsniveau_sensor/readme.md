# Geluids(overlast) sensor

## Intro

Geluidsoverlast is vervelend. Is het ook aan te tonen obv metingen?

## Let op

Zie ESP32 documentatie voor de ADC, filtering en andere technische details.

## Nuttige info

Uit [How to convert sound sensor output reading to decibel value?](https://forum.arduino.cc/index.php?topic=534279.0)

We can't give you an exact formula/expression because your microphone & preamp are uncalibrated.

But, I'll give you a made-up example -  

Let's say your real SPL meter reads 80dB when the Arduino reads 270.   270 is your reference analog reading AREF at 80dB SPL.     

Now the volume increases and we get a new analog reading of A = 540 and we can calculate the dB difference  or change.   dB = 20 log(A/AREF) = 20 log (540/270) = +6dB. 

That's 6dB higher than your reference so we have 86dB SPL.  

So in this case the formula would be  dB SPL = (20log(A/270)) + 80      ...That's the formula, not the C++ expression.   


...That assumes that your '270' readings are "true" and proportional to loudness and they are unbiased (so silence reads zero, although you'll never get total silence).     

## Voorbeeldprojecten

  - [IOT SOUNDSCAPE](https://core-electronics.com.au/projects/iot-soundscape) By Andrew on 07 January 2019

## Projecten

  * [Arduino sound level meter and spectrum analyzer](https://blog.yavilevich.com/2016/08/arduino-sound-level-meter-and-spectrum-analyzer/) 27 Aug 2016, with a solid theoretical background.
  * [Arduino Sound Level Meter](https://www.dfrobot.com/blog-980.html) DFRobot    May 22 2018, with some theoretical background.
  * [Measuring Sound Levels](https://learn.adafruit.com/adafruit-microphone-amplifier-breakout/measuring-sound-levels) Jan 13, 2013
  * [Measure Sound/Noise Level in dB with Microphone and Arduino
ARDUINO](https://circuitdigest.com/microcontroller-projects/arduino-sound-level-measurement) ByAswinth Raj Jan 04, 2018
  * [Arduino Projects: Arduino Decibel Meter](https://tutorial45.com/arduino-projects-arduino-decibel-meter/). Voorbeeld met *slimme code*!
  * [Wireless acoustic sensor based on ESP32 board](https://www.researchgate.net/figure/Wireless-acoustic-sensor-based-on-ESP32-board_fig4_319339998), het hele [artikel](https://www.researchgate.net/publication/319339998_A_Low_Cost_Wireless_Acoustic_Sensor_for_Ambient_Assisted_Living_Systems)
  * [EspAudioSensor](https://revspace.nl/EspAudioSensor), includes measurements over different frequency bands. Werk-in-uitvoering, 2019-05-02.

