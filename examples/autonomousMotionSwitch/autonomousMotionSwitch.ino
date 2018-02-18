/*
 * Demonstration of the autonomous motion switch function
 *
 * autonomousMotionSwitch() is an implementation of a use case described in the
 * datasheet for the ADXL362 sensor. It can be used to trigger an alarm or turn
 * equipment on when motion is detected, and turn it back off when the motion
 * goes away. In this example an LED is turned on or off.
 *
 * The sensor is configured to pull its INT1 pin high as soon as motion is
 * detected, and pull it low when a period of inactivity is detected. The demo
 * sketch goes into low lower sleep, waiting for a change to happen, and turns
 * the LED on or off respectively. If an error is detected, the LED blinks at a
 * faster rate.
 */

#define CS_PIN 10  // CS or slave select pin
#define MCU_INT 3 // MCU pin connected to INT1 on the sensor
#define LED_PIN 4 // Can usually be LED_BUILTIN, except on boards where that pin is used for SPI such as Atmega 328

#include <ADXL362ng.h>
#include <LowPower.h>

ADXL362ng motion(CS_PIN);

void setup() {
	while(!Serial); // Wait for Serial connection (only works on USB boards)
	Serial.begin(115200);
	delay(3000);    // Delay to give the serial viewer time to connect before we start
	Serial.println(F("ADXL362 autonomous motion switch demo"));

	pinMode(MCU_INT,INPUT);
	pinMode(LED_PIN,OUTPUT);

	if(motion.begin()) {
		Serial.println(F("ADXL362 Sensor detected"));
	}
	else {
		Serial.println(F("Could not identify ADXL362 sensor. Check your connections"));
		error();
	}

	// Select INT1 or INT2 on the sensor
	int8_t intPin=1;
	// Required minimum force (in mg) before the sensor triggers
	int16_t activityThreshold=250;
	// Required number of consequtive samples exceeding the threshold required to trigger
	int8_t activityTime=1;
	// Required maximum force (in mg) to detect inactivity
	int16_t inactivityThreshold=150;
	// Required number of consequtive samples under the inactivity threshold to consider the situation as inactivity
	int16_t inactivityTime=30;

	if(motion.autonomousMotionSwitch(intPin, activityThreshold, activityTime, inactivityThreshold, inactivityTime) && motion.ready()) {
		attachInterrupt(digitalPinToInterrupt(MCU_INT),interruptFunction,CHANGE);
		Serial.println(F("Ready. Move the device and the LED should glow until you let it go for a while"));
		delay(1000); // Wait for the text above to be printed before going to sleep in loop()
	}
	else {
		Serial.println(F("Could not set desired operating mode"));
		error();
	}

}

void loop() {
	digitalWrite(LED_PIN,digitalRead(MCU_INT)); // Light up on motion, go dark on inactivity
	LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void interruptFunction() {
}

void error() {
	while(1) {
		digitalWrite(LED_PIN,!digitalRead(LED_PIN)); // Toggle LED
		delay(100);
	}
}
