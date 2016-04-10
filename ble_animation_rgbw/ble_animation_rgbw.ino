#include <Adafruit_NeoPixel.h> //Requires latest RGBW Adafruit Library
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    PIN                       Which pin on the Arduino is connected to the NeoPixels?
    NUMPIXELS                 How many NeoPixels are attached to the Arduino?
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE     0

    #define PIN                     6
    #define NUMPIXELS               16
/*=========================================================================*/

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
//additional variables

//Color
    uint8_t red = 100;
    uint8_t green = 100;
    uint8_t blue = 100;
    //uint8_t white = 100; //Added by Sam

    //New Colour Units
  //int white = min(red, green);
  //white = min(white, blue);
  //red = red - white;
  //green = green - white;
  //blue = blue - white;
uint8_t animationState = 1;

    int pos = 0, dir = 1; // Position, direction of "eye" for larson scanner animation

void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  // turn off neopixel
  pixel.begin(); // This initializes the NeoPixel library.
  for(uint8_t i=0; i<NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0,0,0,0)); // off
  }
  colorWipe(pixel.Color(100, 100, 100, 0), 20); // do a quick colorWipe to show that the pixels are all working, even before Bluefruit connection established
  colorWipe(pixel.Color(0, 0, 0, 0), 20);
  colorWipe(pixel.Color(0,0,0,100), 20);
  colorWipe(pixel.Color(0,0,0,0), 20); 
  pixel.show();

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Neopixel Color Picker Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  //if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    red = packetbuffer[2];
    green = packetbuffer[3];
    blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);

// this part colors ALL the pixels according to the app's color picker:
//    for(uint8_t i=0; i<NUMPIXELS; i++) {
//      pixel.setPixelColor(i, pixel.Color(red,green,blue));
//   }
//    pixel.show(); // This sends the updated pixel color to the hardware.
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
 
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    animationState = buttnum;
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }
    
  if (animationState == 1){ // button labeled "1" in control pad
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0,0));
        }
    //Animation
    rainbow(10);
     pixel.show(); // This sends the updated pixel color to the hardware.
   }
  
  if (animationState == 2){ // button labeled "2" in control pad
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0,0));
        }
    //Animation
    colorWipe(pixel.Color(0, 0, 0, 255), 100); // White RGBW
    delay(20);
    colorWipe(pixel.Color(0, 0, 0, 0), 100); // White RGBW
    pixel.show();
  }

  if (animationState == 3){ // button labeled "3" in control pad
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0,0));
        }
    //Animation
    colorWipe(pixel.Color(green, red, blue, 0), 20);
    pixel.show(); // This sends the updated pixel color to the hardware.
    colorWipe(pixel.Color(0, 0, 0, 0), 20);
    pixel.show();
    pixel.show(); // This sends the updated pixel color to the hardware.
  }
  
  if (animationState == 4){ // button labeled "4" in control pad
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0,0));
        }
    //Animation
    for(uint8_t i=0; i<NUMPIXELS; i++) {
      pixel.setPixelColor(i, pixel.Color(green,red,blue,0));
    }
    pixel.show(); // This sends the updated pixel color to the hardware.
  }

}

//Animations

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixel.numPixels(); i++) {
    pixel.setPixelColor(i, c);
    pixel.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel(((i * 256 / pixel.numPixels()) + j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, c);    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (uint16_t i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3, 0);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  WheelPos -= 170;
  return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}
