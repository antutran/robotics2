#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

// PB7 = SDA, PB6 = SCL
TwoWire myWire(PB7, PB6);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &myWire, -1);

void setup() {
  Serial.begin(115200);
  delay(500);

  myWire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("OLED TEST");
  display.println("SSD1306");
  display.println("PB6 = SCL");
  display.println("PB7 = SDA");

  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.display();
}

void loop() {
}