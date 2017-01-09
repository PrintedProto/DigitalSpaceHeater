


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_RHT03.h>
#include <OneWire.h>

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

//DHT22/rht03/am2302 sensor
const int RHT03_DATA_PIN = 14; // RHT03 data pin
RHT03 rht; // This creates a RTH03 object, which we'll use to interact with the sensor

//DS18B20
#define DS18S20_ID 0x10
#define DS18B20_ID 0x28
OneWire  ds(6);

//screen variables
volatile int butpres, settemp;
volatile float dhttemp, fintemp, foutemp, dhtrh;

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

  //dht22 sensor
  rht.begin(RHT03_DATA_PIN); //initializes dht22

  settemp = 20;
  dhttemp = 1;
  dhtrh = 1;


}

//updates the screen
//void updatescrn(uint8_t dhttemp, uint8_t dhtrh, uint8_t settemp, uint8_t fintemp, uint8_t foutemp){
void updatescrn(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(" Current     SetPoint\n"); //line 1
  display.setTextSize(2);
  display.print(" ");                         //line 2
  display.print(dhttemp,0);
  display.print("C   ");
  display.print(settemp,0);
  display.print("C\n");
  display.setTextSize(1);
  display.print("Fan.I ");                  //line 3
  display.print(fintemp,0);
  display.print("C    Coil ON");
  display.print("\nFan.O ");                  //line 4
  display.print(foutemp,0);
  display.print("C      ");
  display.print(dhtrh,0);
  display.print("%RH");
  display.display();
}

//adjusts Temp setpoint
void adjustT(){
  //Serial.println("IF statement triggered");
  attachInterrupt(digitalPinToInterrupt(pinA),PinA,RISING); // set an interrupt on PinA and B, looking for a rising edge signal
  attachInterrupt(digitalPinToInterrupt(pinB),PinB,RISING); // and executing the PinA() Interrupt Service Routine
  //Serial.println("Interrupts attached");
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
      //Serial.println("EncoderPos" + String(encoderPos));
      }
      updatescrn();
      //Serial.println("update screen triggered");
      if(!digitalRead(encButton)){
        delay(1000);
        butpres = !digitalRead(encButton);
      }
    }
    detachInterrupt(digitalPinToInterrupt(pinA)); // Removes interrupts so values will only increment when needed
    detachInterrupt(digitalPinToInterrupt(pinB));
  //  Serial.println("Interrupts dettached");
}

void getDhtinfo(){
                                 // Call rht.update() to get new humidity and temperature values from the sensor.
  int updateRet = rht.update(); // If successful, the update() function will return 1.
                                // If update fails, it will return a value <0
  if (updateRet == 1) {
    //float latestHumidity = rht.humidity();// The humidity(), tempC(), and tempF() functions can be called -- after
    //float latestTempC = rht.tempC();// a successful update() -- to get the last humidity and temperature
    //Serial.println("Humidity: " + String(latestHumidity, 1) + " %");
    //Serial.println("Temp (C): " + String(latestTempC, 1) + " deg C");
    dhttemp = rht.tempC();
    dhtrh = rht.humidity();
  }
  else {
    delay(RHT_READ_INTERVAL_MS);// If the update failed, try delaying for RHT_READ_INTERVAL_MS ms before
  }
  delay(1000);
}

void getDs18b20(){
   byte i;
   byte present = 0;
   byte data[12];
   byte addr[8];
   //find a device
   if (!ds.search(addr)) {
   ds.reset_search();
   return false;
   }
   if (OneWire::crc8( addr, 7) != addr[7]) {
   return false;
   }
   if (addr[0] != DS18S20_ID && addr[0] != DS18B20_ID) {
   return false;
   }
   ds.reset();
   ds.select(addr);
   // Start conversion
   ds.write(0x44, 1);
   // Wait some time...
   delay(850);
   present = ds.reset();
   ds.select(addr);
   // Issue Read scratchpad command
   ds.write(0xBE);
   // Receive 9 bytes
   for ( i = 0; i < 9; i++) {
   data[i] = ds.read();
   }
   // Calculate temperature value
   fintemp = ( (data[1] << 8) + data[0] )*0.0625;
   return true;
}

void loop() {
  Serial.println("Loop");
  fintemp = 90;
  foutemp = 91;
  Serial.println("Polling DHT");
  getDhtinfo();
  Serial.println("Temp= " + String(dhttemp));
  Serial.println("Humi= " + String(dhtrh));
  butpres = digitalRead(encButton);
  //digitalWrite(13, !butpres);
  //Serial.println("butpres =" + String(butpres));
  if (butpres==0){
    adjustT();
      }

  updatescrn();
  Serial.println("Scrn updated");

}
