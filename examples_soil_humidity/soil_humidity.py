#
# Voorbeeld soil_humidity.py om een bodem-vochtigheids-sensor uit te lezen.
# Deze opzet werkt voor de "Capacitieve Bodemvochtsensor Module met Kabel
#   - Model: SOILMOISWCABLE"
#
# Bron:
#   - https://forum.micropython.org/viewtopic.php?f=18&t=4487&start=10
#   - https://www.switchdoc.com/2018/11/tutorial-capacitive-moisture-sensor-grove/
#
# BvH 2019-05-06
#
# Idee:
#   gebruik ADC-ingang van de ESP32. Zet deze op 3.3V max. ipv 1.0V of
#   gebruik een voltage-deler. Zie discussie in 1e link.
#
#   Code voorbeeld hieronder is gebaseerd op 2e link en is in C (dus
#   herschrijven in microPython).
#
"""
/***************************************************
 This example reads Capacitive Soil Moisture Sensor.

 Created 2015-10-21
 By berinie Chen &lt;bernie.chen@dfrobot.com&gt;

 GNU Lesser General Public License.
 See &lt;http://www.gnu.org/licenses/&gt; for details.
 All above must be included in any redistribution
 ****************************************************/


const int AirValue = 520;
const int WaterValue = 260;
int intervals = (AirValue - WaterValue)/3;
int soilMoistureValue = 0;

void setup() {
  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
}

void loop() {
soilMoistureValue = analogRead(A0);  //put Sensor insert into soil
if(soilMoistureValue &gt; WaterValue &amp;&amp; soilMoistureValue &lt; (WaterValue + intervals)) { Serial.println("Very Wet"); } else if(soilMoistureValue &gt; (WaterValue + intervals) &amp;&amp; soilMoistureValue &lt; (AirValue - intervals))
{
  Serial.println("Wet");
}
else if(soilMoistureValue &lt; AirValue &amp;&amp; soilMoistureValue &gt; (AirValue - intervals))
{
  Serial.println("Dry");
}
delay(100);
}
"""
