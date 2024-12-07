#include <Arduino.h> 
#include <AccelStepper.h>
#include <WiFi.h> 
#include <WebServer.h>

// Pin definitions
#define STEP_PIN 26
#define DIR_PIN 25
#define BUTTON_FWD_PIN 27
#define BUTTON_MID_PIN 14
#define BUTTON_BWD_PIN 12
#define POT_PIN 34
#define CAMERA_TRIGGER_PIN 15
#define IR_LED_PIN 21

#undef IR_SEND_PIN // enable this, if you need to set send pin programmatically using uint8_t tSendPin below
#include <IRremote.hpp>


// Wi-Fi credentials
 const char* ssid = "AIVD bus 28";
 const char* password = "Welkom69420";

uint16_t nikon_command[] = {2000,27850,390,1580,410,3580,400};

// Initialize the stepper library
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
WebServer server(80);

void handleRoot() {
  server.send(200, "text/html", "<form action=\"/sweep\" method=\"GET\">"
                                "Distance (mm): <input type=\"number\" name=\"distance\"><br>"
                                "Interval (mm): <input type=\"number\" name=\"interval\"><br>"
                                "Delay (ms): <input type=\"number\" name=\"delay\"><br>"
                                "<input type=\"submit\" value=\"Start Sweep\">"
                                "</form>");
}

void handleSweep() {
  if (server.hasArg("distance") && server.hasArg("interval") && server.hasArg("delay")) {
    long distance = server.arg("distance").toDouble() * 260.0;
    long interval = server.arg("interval").toDouble() * 260.0;
    long delay_int = server.arg("delay").toInt();

    for (long i = 0; i <= distance; i += interval) {
      if(!digitalRead(BUTTON_FWD_PIN)){
        return;
      }
      stepper.moveTo(stepper.currentPosition() + interval);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }
      IrSender.sendRaw(nikon_command, sizeof(nikon_command) / sizeof(nikon_command[0]), NEC_KHZ);
      delay(delay_int);  // Wait 1 second before next move (adjust as needed)
    }
  }
  handleRoot();
}

void handle_NotFound(){
    server.send(404, "text/plain", "Not found");
}

void setup() {
  // Setup pins
  pinMode(BUTTON_FWD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BWD_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MID_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);
  pinMode(CAMERA_TRIGGER_PIN, OUTPUT);
  pinMode(IR_LED_PIN, OUTPUT);
  digitalWrite(CAMERA_TRIGGER_PIN, LOW);
  IrSender.begin(IR_LED_PIN);
  //irsend.enableIROut(38); // Set IR LED frequency to 38kHz

  Serial.begin(115200);

  // Set max speed and acceleration
  stepper.setMaxSpeed(750);
  stepper.setAcceleration(500);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && digitalRead(BUTTON_FWD_PIN)) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP address: "); 
  Serial.println(WiFi.localIP());

  // Start web server
  server.on("/", handleRoot);
  server.on("/sweep", handleSweep);
  server.onNotFound(handle_NotFound);
  server.begin();
}

void loop() {
  // Read button states
  bool forward = digitalRead(BUTTON_FWD_PIN) == LOW;
  bool backward = digitalRead(BUTTON_BWD_PIN) == LOW;
  bool middle = digitalRead(BUTTON_MID_PIN) == LOW;

  // Read potentiometer and map it to a speed range
  int potValue = analogRead(POT_PIN);
  float speed = map(potValue, 0, 4095, 50, 750);

  if (forward) {
    stepper.setSpeed(speed);
  } else if (backward) {
    stepper.setSpeed(-speed);
  } else {
    stepper.setSpeed(0);
  }

  if (middle){
      IrSender.sendRaw(nikon_command, sizeof(nikon_command) / sizeof(nikon_command[0]), NEC_KHZ);
  }

  // Move the motor
  stepper.runSpeed();
  
  // Handle web server 
  server.handleClient();
}
