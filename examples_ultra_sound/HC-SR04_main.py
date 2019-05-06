##
#
# Source: http://www.learnmicropython.com/pyboard/hc-sr04-ultrasonic-ranging-module-and-pyboard-example-in-micropython.php
#
##
import pyb
import ultrasonic

# setting pins to accomodate Ultrasonic Sensor (HC-SR04)
sensor1_trigPin = pyb.Pin.board.X1
sensor1_echoPin = pyb.Pin.board.X2

# sensor needs 5V and ground to be connected to pyboard's ground
# creating two Ultrasonic Objects using the above pin config
sensor1 = ultrasonic.Ultrasonic(sensor1_trigPin, sensor1_echoPin)

# using USR switch to print the sensor's values when pressed
switch = pyb.Switch()

# function that prints each sensor's distance
def print_sensor_values():
	# get sensor1's distance in cm
	distance1 = sensor1.distance_in_cm()
	print("Sensor1 (Metric System)", distance1, "cm")

# prints values every second
while True:
	print_sensor_values()
	pyb.delay(1000)
