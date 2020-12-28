
/*
* edited by Velleman / Patrick De Coninck
* Read a card using a mfrc522 reader on your SPI interface
* Pin layout should be as follows (on Arduino Uno - Velleman VMA100):
* MOSI: Pin 11 / ICSP-4
* MISO: Pin 12 / ICSP-1
* SCK: Pin 13 / ISCP-3
* SS/SDA (MSS on Velleman VMA405) : Pin 10
* RST: Pin 9
* VCC: 3,3V (DO NOT USE 5V, VMA405 WILL BE DAMAGED IF YOU DO SO)
* GND: GND on Arduino UNO / Velleman VMA100
* IRQ: not used
*/
#include <ArduinoJson.h>
#include <WiFi.h> //Arduino
#include <HTTPClient.h>
#include <SPI.h> //Arduino
#include <RFID.h> //VMA405
#include <NTPClient.h> //Getting time from NTP

#define SS_PIN 2 //SW Chip Select
#define RST_PIN 4

// ESP32: Output PWM 0.1Hz (10 second period), 50% duty-cycle on GPIO pin 27
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/mcpwm.h"

#define GPIO_PWM0A_OUT 17
RFID rfid(SS_PIN,RST_PIN);

//NETWORK CONNECTION PARAMETERS
const char* ssid = "Tfa_i_szcztery_wajfaj";
const char* password = "trinitron";

//DOMAIN ADDRESS
const char* serverName = "http://192.168.0.199:8081/addEntry";

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


//Time measured in ms will grow quickly, thus variable stored in long
unsigned long lastTime = 0; 

// Set timer to 5 second(5000)
unsigned long timerDelay = 5000;

// Set connection timeout to 15s(30);
unsigned int connectionTimeout = 30;


// the number of the LED pin
const int ledPin = 17;  // 16 corresponds to GPIO16

//backpass timer
unsigned long currentMillis;
bool readFlag = true;
unsigned int readTimeout = 4000;

// setting PWM properties
const int freq = 2;
const int ledChannel = 0;
const int resolution = 8;
//int power = 7;
//int led = 8; 
int buzzer = 16;
int serNum[5];
String formattedDate;

void buzz(int time){
  digitalWrite(buzzer, HIGH);
  delay(time);
  digitalWrite(buzzer, LOW);
}

void buzz(int time, int delayTime){
  digitalWrite(buzzer, HIGH);
  delay(time);
  digitalWrite(buzzer, LOW);
  delay(delayTime);
}

void BlinkRed(){
  mcpwm_start(MCPWM_UNIT_0,MCPWM_TIMER_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void StopBlinkRed(){
  mcpwm_stop(MCPWM_UNIT_0,MCPWM_TIMER_0);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_A);
}
String jsonString;
void setup(){

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);     // Initialise channel MCPWM0A on GPIO pin  17
    MCPWM0.clk_cfg.prescale = 199;                // Set the 160MHz clock prescaler to 199 (160MHz/(199+1)=800kHz)
    MCPWM0.timer[0].period.prescale = 199;        // Set timer 0 prescaler to 199 (800kHz/(199+1))=4kHz)
    MCPWM0.timer[0].period.period = 3999;        // Set the PWM period to 0.1Hz (4kHz/(39999+1)=0.1Hz) 
    MCPWM0.channel[0].cmpr_value[0].val = 2000;  // Set the counter compare for 50% duty-cycle
    MCPWM0.channel[0].generator[0].utez = 2;      // Set the PWM0A ouput to go high at the start of the timer period
    MCPWM0.channel[0].generator[0].utea = 1;      // Clear on compare match
    MCPWM0.timer[0].mode.mode = 1;                // Set timer 0 to increment
    MCPWM0.timer[0].mode.start = 1;               // 
    // configure LED PWM functionalitites
     //ledcSetup(ledChannel, freq, resolution);
  
    // attach the channel to the GPIO to be controlled
    //ledcAttachPin(ledPin, ledChannel);
    //ledcWrite(ledChannel, 255);
    Serial.begin(115200);
    Serial.println("SPI and RFID initialization"); //TODO some kind of check?
    SPI.begin();
    rfid.init();
    WiFi.begin(ssid, password);
    Serial.print("Connecting to network: ");
    Serial.println(ssid);
    while((WiFi.status() != WL_CONNECTED) && connectionTimeout) {
      delay(500);
      Serial.print(".");
      connectionTimeout--;
    }
    if(connectionTimeout == 0){
      Serial.print("Could not connect, exiting...");
      exit(0);
    } else {
      Serial.print("Connected to WiFi network with IP Address: ");
      Serial.println(WiFi.localIP());

      Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
    }

    // Initialize a NTPClient to get time
    timeClient.begin();
    timeClient.setTimeOffset(3600); //GMT+1
    
/*
* define VMA100 (UNO) pins 7 & 8 as outputs and put them LOW
*/
   // pinMode(led, OUTPUT);
   // pinMode (power,OUTPUT);
   // digitalWrite(led, LOW);
   // digitalWrite (power,LOW);
   pinMode (buzzer,OUTPUT);
   digitalWrite(buzzer, LOW);
}

void loop(){
  //blocking reading same card twice in a row in small amount of time
  if(millis()-currentMillis>readTimeout && readFlag == false){
    readFlag=true;
    Serial.println("Ready for read");
  } else {
    
  }
  
    if(rfid.isCard() && readFlag){
        if(rfid.readCardSerial()){
          String csn;
          for(int i =0;i<sizeof(rfid.serNum);i++){
            csn = csn+rfid.serNum[i];
          }
            Serial.print(rfid.serNum[0]);
            Serial.print(" ");
            Serial.print(rfid.serNum[1]);
            Serial.print(" ");
            Serial.print(rfid.serNum[2]);
            Serial.print(" ");
            Serial.print(rfid.serNum[3]);
            Serial.print(" ");
            Serial.print(rfid.serNum[4]);
            Serial.println("");
            buzz(200);//read everything from card
            BlinkRed();
            currentMillis = millis();
            readFlag = false;
            //mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_A);
            //delay(1000);//This delay is for blocking system from reading multiple times from one card, TODO
            
           if(WiFi.status() == WL_CONNECTED){

            
            HTTPClient http;

            http.begin(serverName);
            http.addHeader("Content-Type","application/json"); //Specify content type
            StaticJsonDocument<100> jsonBody;
            timeClient.forceUpdate();
            formattedDate = timeClient.getFormattedDate();
            jsonBody["datetime"] = formattedDate;
            jsonBody["csn"]= csn;
            serializeJson(jsonBody, jsonString);
            serializeJson(jsonBody, Serial);
            int httpResponseCode = http.POST(jsonString);

            Serial.print("\nHTTP Response code: ");
            Serial.println(httpResponseCode);
            if(httpResponseCode != 200){
              mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0,MCPWM_OPR_A);
              http.end();
              buzz(2000,200);
            } else {
              http.end();
              StopBlinkRed();
              buzz(200,200);
              buzz(200,200);
            }
            
           } else {
            Serial.println("WiFi Disconnected");
           }
        }
        
               
    }
    
    
    
    rfid.halt();

}
