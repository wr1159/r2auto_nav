/* Program used to take user input from 4x4 matrix keypad and output it to OLED using ESP32 board */
/* Also, sends the number to the database */ 
#include "Keypad.h"
#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <Keypad.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "WiFi.h"
#include <ESP32Servo.h> 
#define Servo_PWM 18 


const char* ssid       = "Saksham";
const char* password   = "saksham3004";
const char* broker_ip = "172.20.10.5";

/* instatiate wifi, mqtt clients */
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
Servo MG995_Servo; 


/*Setting up the oled*/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin

/* Setting up the keypad */
const byte ROWS = 4; //four rows
const byte COLS = 3; //four columns

String input_num; //for input

char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

/* For ESP32 Microcontroller */
byte pin_rows[ROWS] = {14, 32, 33, 26}; 
byte pin_column[COLS] = {27, 12, 25};

/*initialise oled*/
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire, -1);

/*initialise keypad*/
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROWS, COLS); // instantiating a keypad object 

/* callback function for mqtt connection- called when a new message is received
 * for a topic that we're subscribed to.*/
void mqtt_cb(char* topic, uint8_t *msg, uint32_t len){
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String msgAngle;
  
  for (int i = 0; i < len; i++) {
    Serial.print((char)msg[i]);
    msgAngle += (char)msg[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // Changes the output state according to the message
  if (String(topic) == "topic name") { //TODO: change topic name to RPi's NFC status
    Serial.print("Changing servo position...");
    if(msgAngle == "arrive"){
      Serial.println("original");
      // MG995_Servo.write(0);
      delay(200);
    }
    else if(msgAngle == "away"){
      Serial.println("open");
      // MG995_Servo.write(270);
      delay(200);
    }
  }
}

/* handle a reconnect to the broker if the connection is dropped */
void mqtt_reconnect(void){
    while(!mqtt_client.connected()){
        Serial.printf("Connecting to broker '%s'...", broker_ip);
        if(mqtt_client.connect("R&D #")){
            Serial.println("CONNECTED");
            //subscribe
            mqtt_client.subscribe("topic name"); //TODO: change topic name to RPi's NFC topic
        }
        else{
            Serial.println("FAILED:");
            Serial.print(mqtt_client.state());
            Serial.println("\r\nRetrying in 5 seconds.");
            delay(5000);
        }
    }

}

void Display(char key){
  Serial.println(key);
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println(key);
  display.display();
}

void clearDisplay(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.display();
}


void setup() {
  Serial.begin(115200);
  
  Serial.println("OLED FeatherWing test");
  /* SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally */
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  Serial.println("OLED begun");

  /* Show image buffer on the display hardware. */
  /* Since the buffer is intialized with an Adafruit splashscreen */
  /* internally, this will display the splashscreen. */
  display.display();
  delay(1000);

  /* Clear the buffer. */
  display.clearDisplay();
  display.display();

  Serial.println("IO test");

  /* text display tests */
  display.setTextSize(4);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display(); // actually display all of the above


// connect to WiFi
  Serial.printf("Connecting to '%s'...", ssid);

  WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" CONNECTED");

  /* initialize (but don't connect yet) to the MQTT broker */
  mqtt_client.setServer(broker_ip, 1883);
  mqtt_client.setCallback(mqtt_cb);
  MG995_Servo.attach(Servo_PWM, 771, 2470);
  MG995_Servo.write(30);

}

void loop() {
  char key = keypad.getKey(); //get table number
  Display(key);

  if(!mqtt_client.connected()){
        mqtt_reconnect();
  }

  /* handle/maintain the MQTT connection, including any new messages */
  mqtt_client.loop();
  
  while (key){ 
    if (key == '*'){
      clearDisplay();
      input_num = ""; //clear password
    }
    else if (key == '#'){
      Serial.print("Input number: ");
      Serial.println(input_num);

      //Sending topic to RPi
      mqtt_client.publish("ESP32/tableNumber", input_num.c_str());
      // MG995_Servo.write(70);
      MG995_Servo.write(100);
      delay(5000);
      // MG995_Servo.write(140);
      MG995_Servo.write(30);
      input_num = "";
      clearDisplay();
    }
    else{
      input_num = key;
    }
    key = keypad.getKey();
    Display(key);
  }
  
}
