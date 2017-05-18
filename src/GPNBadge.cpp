#include <GPNBadge.hpp>

IRsend irsend(GPIO_DP); //an IR led is connected to GPIO pin 4 (D2)
IRrecv irrecv(GPIO_DN);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, GPIO_WS2813, NEO_GRB + NEO_KHZ800);
TFT_ILI9163C tft(GPIO_LCD_CS, GPIO_LCD_DC);

