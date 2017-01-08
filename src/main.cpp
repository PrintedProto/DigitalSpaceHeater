



#include <SPI.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Software SPI

#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

void setup()   {
  //Serial.begin(9600);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //display.display();
  //delay(2000);

  // Clear the buffer.
  display.clearDisplay();
}

void loop() {
  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(" Current     SetPoint");
  display.println("");
  display.setTextSize(2);
  display.println(" 88C   88C");
  display.setTextSize(1);
  display.println("");
  display.println("Fan.I 88C    Coil ON");
  display.setTextSize(1);
  display.println("Fan.O 88C    hh:mm:ss");
  //display.setCursor(32,0);
  //display.print("88");
  //display.setCursor(32,64);
  //display.print("88");
  display.display();
  delay(2000);
  display.clearDisplay();
}
