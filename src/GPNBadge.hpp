#pragma once

#include <ESP8266WiFi.h>
#include <IRremoteESP8266.h>
#include <Adafruit_NeoPixel.h>
#include <TFT_ILI9163C.h>
#include <Wire.h>
#include <SPI.h>


#define GPIO_LCD_DC 0
#define GPIO_TX     1
#define GPIO_WS2813 4
#define GPIO_RX     3
#define GPIO_DN     2
#define GPIO_DP     5

#define GPIO_BOOT   16
#define GPIO_MOSI   13
#define GPIO_CLK    14
#define GPIO_LCD_CS 15
#define GPIO_BNO    12

#define MUX_JOY 0
#define MUX_BAT 1
#define MUX_LDR 2
#define MUX_ALK 4
#define MUX_IN1 5

#define VIBRATOR 3
#define MQ3_EN   4
#define LCD_LED  5
#define IR_EN    6
#define OUT1     7

#define UP      730
#define DOWN    590
#define RIGHT   496
#define LEFT    970
#define OFFSET  30

#define I2C_PCA 0x25

#define NUM_LEDS    4

#define BAT_CRITICAL 3300
#define BAT_FULL 4200
#define BAT_PLATEAU 3600

extern IRsend irsend;
extern IRrecv irrecv;

extern Adafruit_NeoPixel pixels;
extern TFT_ILI9163C tft;

enum class JoystickState {
	BTN_NOTHING,
	BTN_ENTER,
	BTN_LEFT,
	BTN_RIGHT,
	BTN_UP,
	BTN_DOWN
};

class Badge {
public:
	void init() { //initialize the badge
  		WiFi.disconnect();
		WiFi.mode(WIFI_OFF);
		// Next 2 line seem to be needed to connect to wifi after Wake up
  		delay(20);
        WiFi.persistent(false);
	 	Wire.begin(9, 10);

  		pinMode(GPIO_BOOT, INPUT_PULLDOWN_16);
  		pinMode(GPIO_WS2813, OUTPUT);

  		//the ESP is very power-sensitive during startup, so...
  		Wire.beginTransmission(I2C_PCA);
  		Wire.write(0b00000000); //...clear the I2C extender to switch off vobrator and LCD backlight
  		Wire.endTransmission();


 		pixels.begin(); //initialize the WS2813
  		pixels.clear();
  		pixels.show();

  		irsend.begin();
  		irrecv.enableIRIn();
  		Serial.begin(115200);

  		delay(100);

  		tft.begin(); //initialize the tft. This also sets up SPI to 80MHz Mode 0
  		tft.setRotation(2);
  		tft.scroll(32);

 		pixels.clear(); //clear the WS2813 another time, in case they catched up some noise
 		pixels.show();
	}

	void setGPIO(byte channel, boolean level) {
  		bitWrite(shiftConfig, channel, level);
  		Wire.beginTransmission(I2C_PCA);
  		Wire.write(shiftConfig);
  		Wire.endTransmission();
	}
	void setAnalogMUX(byte channel) {
  		shiftConfig = shiftConfig & 0b11111000;
  		shiftConfig = shiftConfig | channel;
  		Wire.beginTransmission(I2C_PCA);
  		Wire.write(shiftConfig);
  		Wire.endTransmission();
	}
	void setBacklight(bool on) {
		setGPIO(LCD_LED, on?HIGH:LOW);
	}
	void setVibrator(bool on) {
		setGPIO(VIBRATOR, on?HIGH:LOW);
	}
	uint16_t getBatLvl() {
 		setAnalogMUX(MUX_BAT);
		delay(20);
  		uint16_t avg = 0;
  		for (byte i = 0; i < 10; i++) {
    			avg += analogRead(A0);
  		}
		return (avg / 10);
	}

	uint16_t getBatVoltage() { //battery voltage in mV
  		return (getBatLvl() * 4.8);
	}

	uint16_t getLDRLvl() {
  		setAnalogMUX(MUX_LDR);
        delay(20);
  		uint16_t avg = 0;
  		for (byte i = 0; i < 10; i++) {
    			avg += analogRead(A0);
  		}
  		return (avg / 10);
	}

	JoystickState getJoystickState() {
		this->setAnalogMUX(MUX_JOY);
 	 	delay(10);
  		uint16_t adc = analogRead(A0);

  		if (adc < UP + OFFSET && adc > UP - OFFSET) {
			return JoystickState::BTN_UP;
		}
  		else if (adc < DOWN + OFFSET && adc > DOWN - OFFSET) {
			return JoystickState::BTN_DOWN;
  		}
  		else if (adc < RIGHT + OFFSET && adc > RIGHT - OFFSET) {
			return JoystickState::BTN_RIGHT;
  		}
  		else if (adc < LEFT + OFFSET && adc > LEFT - OFFSET) {
			return JoystickState::BTN_LEFT;
  		}
  		else if (digitalRead(GPIO_BOOT) == HIGH) {
			return JoystickState::BTN_ENTER;
  		}
		else {
			return JoystickState::BTN_NOTHING;
		}
	}

private:
	byte shiftConfig = 0; //stores the 74HC595 config
};


