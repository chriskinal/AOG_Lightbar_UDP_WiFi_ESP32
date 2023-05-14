/*
INFO: This Software is made for my private use. I like to share, but for further details read the LICENCE file!

To use with https://github.com/farmerbriantee/AgOpenGPS V5

AGOpenGPS Lightbar for WLAN/UDP connection with ESP32 and led strip ws2812b

inspired by https://github.com/mnltake/AOGLightbar_M5stickC/blob/master/AOGLightbar_M5stickC_BT_v5_1.ino as well as parts of the code are copied and modified into this sketch

do not forget to include Libraries:
	fastled/FastLED@^3.4.0

ESP32 firmware
Basic selection via #define in the top of this code

by hagre 
05 2021
*/

//version
#define VERSION 0
#define SUB_VERSION 3

//features
#define DEBUG_UART_ENABLED //enable or disable with // in front of #define     //self explaining (more or less just for me to use)
//#define LED_BRIGHTNESS_CONTROL_ENABLED //enable or disable with // in front of #define   you need to connect a potentiometer to the pin

//WIFI settings
#define YOUR_WIFI_HOSTNAME "AOG_Lightbar"
#define PORT_TO_LISTEN 8888 // local port to listen for UDP packets
#define UDP_PACKET_SIZE 1024
#define MAX_WAIT_TIME_UDP 2000  //ms timeout for waiting on valide UDP msg

//LED Settings
#define NUMPIXELS   35                 // Odd number dont use =0 
#define Neopixel_Pin 13                //GPIO16 Set this to the pin number you are using for the Neopixel strip controll line
#define cmPerLightbarPixel  16          // Must be a multiple of cmPerDistInt
#define cmPerDistInt  2                // The number of centimeters represented by a change in 1 of the AOG cross track error byte
#define LED_UPDATE_TIME 1000  //ms
#define LED_BRIGHTNESS 100    //0-255
#ifdef  LED_BRIGHTNESS_CONTROL_ENABLED
  #define ANALOG_PIN 34 // GPIO 34 (Analog ADC1_CH6) 
  #define MIN_LED_BRIGHTNESS 20 // 0-255
  #define MAX_LED_BRIGHTNESS 150 // 0-255
#endif
#define LED_TYPE WS2812B
#define LED_COLOR_SETTING GRB
#define LED_CORRECTION TypicalLEDStrip

//Serial Port config - do not change
#define DEBUG_UART_HARDWARE_PORT 0 //config as required // USB == 0
#define DEBUG_UART_BOUD 115200 //config as required 
#define UART0RX 3 //ESP32 GPIOs
#define UART0TX 1
//++++++++++++++++++++++++++++++++++++++++++++++++ END of CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <Arduino.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <FastLED.h>

int8_t LANStatus = -5; //Connected to Network //WIFI or ETHERNET //-5 = init, -2 = just disconnected, -1 = wait to reconnect, 0 = disconnected, 1 = connecting, 2 = just connected,  3 = still connected
int8_t LANStatusOld = -6;

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
unsigned int localPort = PORT_TO_LISTEN;      // local port to listen for UDP packets
byte packetBuffer[UDP_PACKET_SIZE]; //buffer to hold incoming packets
unsigned long lastvalideXtrackreceived = 0;


CRGBArray <NUMPIXELS> leds;
const uint8_t centerpixel = (NUMPIXELS-1)/2;
uint8_t Neopixel_Brightness = LED_BRIGHTNESS;// default brightness value between 0 and 255
unsigned long lastLEDUpdate = 0;

const uint8_t cmPerLBPixel = cmPerLightbarPixel / cmPerDistInt;
int16_t cross_track_error = 0;
int16_t cross_track_errorOld = 0;

int8_t statusOfProgram = -3; // -3 init / -2 No WLAN connection / -1 no UDP msg arrive or timeout / 0 not in AutoMode (255) / 1 receiving valid distance 
int8_t statusOfProgramOld = -3;

uint32_t serialDebugPortBoud = DEBUG_UART_BOUD;
HardwareSerial SerialDebug(DEBUG_UART_HARDWARE_PORT);

void setup() {
  // put your setup code here, to run once:
  WiFiManager wm; // wifimanger instance
  #ifdef DEBUG_UART_ENABLED
    SerialDebug.begin (serialDebugPortBoud, SERIAL_8N1, UART0RX, UART0TX); //Debug output, usually USB Port
    SerialDebug.println ("Setup starting");
  #endif

  statusOfProgram = -2;

  #ifdef DEBUG_UART_ENABLED
    SerialDebug.println ("Starting WIFI");
   #endif
 
  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  // wm.resetSettings();

  //set custom ip for portal
  wm.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //set hostname
  wm.setHostname(YOUR_WIFI_HOSTNAME);
  
  bool res;
  // res = wm.autoConnect(); // auto generated AP name from chipid
  res = wm.autoConnect("AOG_Lightbar_AP"); // anonymous ap
  // res = wm.autoConnect("AutoConnectAP", "passowrd"); // password protected ap
  if(!res) {
      SerialDebug.println("Failed to connect");
      // ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      SerialDebug.print("WiFi IP of lightbar: "); SerialDebug.println(WiFi.localIP());        
  }
  
  FastLED.addLeds<LED_TYPE ,Neopixel_Pin,LED_COLOR_SETTING>(leds, NUMPIXELS ).setCorrection(LED_CORRECTION);
  lastLEDUpdate = millis();

  delay (500);
}

// End of setup --------------------------------------------

void loop() {
  // put your main code here, to run repeatedly:
  LANStatus = WiFi.status();
  SerialDebug.print("LANStatus: ");SerialDebug.print(LANStatus);

  #ifdef DEBUG_UART_ENABLED
    // SerialDebug.print ("Status: ");
    // SerialDebug.println (statusOfProgramOld);
  #endif
 
  if (LANStatus != LANStatusOld) { 
    if (LANStatusOld < 3 && LANStatus == 3){
      #ifdef DEBUG_UART_ENABLED
        SerialDebug.println ("Starting UDP");
      #endif
      Udp.begin(localPort);
      statusOfProgram = -1;
    }
    if (LANStatusOld == 3 && LANStatus <= 3){
      #ifdef DEBUG_UART_ENABLED
        SerialDebug.println ("Stoping UDP");
      #endif
      Udp.stop();
      statusOfProgram = -2;
    }

    #ifdef DEBUG_UART_ENABLED
      SerialDebug.print (LANStatusOld);
      SerialDebug.println (LANStatus);
    #endif
    LANStatusOld = LANStatus;

  }
  
  if (LANStatus >= 3) { //still CONNECTED
    #ifdef DEBUG_UART_ENABLED
      // SerialDebug.print(" L");
    #endif
    uint16_t sizeOfUDP = Udp.parsePacket();
    if (sizeOfUDP > 0) {
      #ifdef DEBUG_UART_ENABLED
        // SerialDebug.print(sizeOfUDP);
        // SerialDebug.println(" packets received");
      #endif

      if (sizeOfUDP < UDP_PACKET_SIZE) {
        Udp.read(packetBuffer, UDP_PACKET_SIZE);
        #ifdef DEBUG_UART_ENABLED
          // for (uint16_t i=0; i<sizeOfUDP; i++){
            // SerialDebug.print(packetBuffer [i]);
            // SerialDebug.print(" ");
          // }  
          // SerialDebug.println("");
        #endif

        if (packetBuffer [0] == 0x80 && packetBuffer [1] == 0x81 && packetBuffer [2] ==0x7f && packetBuffer [3] ==0xfe){
          lastvalideXtrackreceived = millis();

          #ifdef DEBUG_UART_ENABLED
            SerialDebug.print(packetBuffer [10]);
            SerialDebug.print(" Correct Header found. Distance = ");
          #endif
          if (packetBuffer [10] == 255){
            cross_track_error = 0;
            statusOfProgram = 0;
            #ifdef DEBUG_UART_ENABLED
              SerialDebug.println(" AUTO not active");
            #endif
          }
          else {
            cross_track_error = packetBuffer [10] - 127;
            statusOfProgram = 1;
            #ifdef DEBUG_UART_ENABLED
              SerialDebug.println(cross_track_error);
            #endif
          }
        }
      }  
    }
    if (millis() - lastvalideXtrackreceived > MAX_WAIT_TIME_UDP ) {
      statusOfProgram = -1;
    }
  }
  
  if (cross_track_error != cross_track_errorOld || statusOfProgram != statusOfProgramOld || millis () - lastLEDUpdate > LED_UPDATE_TIME){

    lastLEDUpdate = millis();

    #ifndef LED_BRIGHTNESS_CONTROL_ENABLED 
      FastLED.setBrightness(Neopixel_Brightness);
    #else
      uint16_t analog_Neopixel_Brightness = map (analogRead(ANALOG_PIN), 0, 4095, MIN_LED_BRIGHTNESS, MAX_LED_BRIGHTNESS);
      FastLED.setBrightness(analog_Neopixel_Brightness);
    #endif  

    if (statusOfProgram == -2){
      for (int i = 0; i < NUMPIXELS; i++){
        if (i == 0 || i == NUMPIXELS -1){//Left end right end of Leds
          leds[i] = CRGB::Red;
        }
        else{
          leds[i] = CRGB::Black;//Clear
        }
      }
    }
    else if (statusOfProgram == -1){
      for (int i = 0; i < NUMPIXELS; i++){
        if (i == 0 || i == NUMPIXELS -1){//Left end right end of Leds
          leds[i] = CRGB::Blue;
        }
        else{
          leds[i] = CRGB::Black;//Clear
        }
      }
    }
    else if (statusOfProgram == 0){
      for (int i = 0; i < NUMPIXELS; i++){
        if (i == 0 || i == NUMPIXELS -1){//Left end right end of Leds
          leds[i] = CRGB::Green;
        }
        else{
          leds[i] = CRGB::Black;//Clear
        }
      }
    }
    else if (statusOfProgram == 1){
      int8_t level = constrain((int8_t)(cross_track_error/cmPerLBPixel), -centerpixel, centerpixel);
      int8_t n = level + centerpixel;
      for (int i = 0; i < NUMPIXELS; i++){
        if (i == centerpixel && i == n){//Center
          leds[i] = CRGB::Yellow;
        }else if (level < 0 && i >= n && i < centerpixel){ //Right Bar
          leds[i] = CRGB::Green;
        }else if (level > 0 && i <= n && i > centerpixel){//Left Bar
          leds[i] = CRGB::Red;
        }else{
          leds[i] = CRGB::Black;//Clear
        }
      }
    }


    FastLED.show();
    statusOfProgramOld = statusOfProgram;
    cross_track_errorOld = cross_track_error;
  }

}

// End of main loop