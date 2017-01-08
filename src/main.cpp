



#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
//#include <string.h>

// Software SPI
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 8
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

//rotary encoder
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile int encoderPos = 20; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile int oldEncPos = 20; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
static int encButton = 4;

//encoder functions
void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}
//encoder functions

void setup()   {
  Serial.begin(9600);

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

  //encoder
  pinMode(encButton, INPUT_PULLUP); // buton press
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  //attachInterrupt(digitalPinToInterrupt(pinA),PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  //attachInterrupt(digitalPinToInterrupt(pinB),PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  pinMode(13, OUTPUT);
}



void updatescrn(uint8_t dhttemp, uint8_t settemp, uint8_t fintemp, uint8_t foutemp){

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(" Current     SetPoint");
  display.println("");
  display.setTextSize(2);
  display.println(" " + String(dhttemp) + "C   " + String(settemp) + "C");
  display.setTextSize(1);
  display.println("");
  display.println("Fan.I " + String(fintemp) + "C    Coil ON");
  display.println("Fan.O " + String(foutemp) + "C    hh:mm:ss");
  display.display();
}

void loop() {
  //.println("Loop start");
  uint8_t dhttemp,fintemp, foutemp, butpres; // settemp,
  dhttemp = 88;
  //settemp = 20;
  fintemp = 90;
  foutemp = 91;
  butpres = digitalRead(encButton);
  //digitalWrite(13, !butpres);
  //Serial.println("butpres =" + String(butpres));
  if (butpres==0){
    //Serial.println("IF statement triggered");
    attachInterrupt(digitalPinToInterrupt(pinA),PinA,RISING); // set an interrupt on PinA and B, looking for a rising edge signal
    attachInterrupt(digitalPinToInterrupt(pinB),PinB,RISING); // and executing the PinA() Interrupt Service Routine
    Serial.println("Interrupts attached");
    while (butpres==0){
      //Serial.println("While loop triggered");
      if(oldEncPos != encoderPos) {
        if(encoderPos <= 0){
          encoderPos = 0;
        }
        else if(encoderPos >= 80){
          encoderPos = 80;
        }
        oldEncPos = encoderPos;
        Serial.println("EncoderPos" + String(encoderPos));
        }
        updatescrn(dhttemp, encoderPos, fintemp, foutemp);
        //Serial.println("update screen triggered");
        if(!digitalRead(encButton)){
          delay(1000);
          butpres = !digitalRead(encButton);
        }
        //digitalWrite(13, !butpres);
        //Serial.println("butpres =" + String(butpres));
      }
    detachInterrupt(digitalPinToInterrupt(pinA)); // Removes interrupts so values will only increment when needed
    detachInterrupt(digitalPinToInterrupt(pinB));
    Serial.println("Interrupts dettached");
    delay(60000);
    }

  updatescrn(dhttemp, encoderPos, fintemp, foutemp);


}
