#include <ArduinoJson.h>

#include <Wire.h>

//Encoder Motor
StaticJsonBuffer<256> jsonBufferOut;
StaticJsonBuffer<128> jsonBufferIn;

void parseCommand();
const char* json_str = "{\"commandType\":4, \"commandDetail\":{\"go\":true, \"direction\":true}}";


void setup() {
  Serial.begin(115200);
  

  Serial.println(json_str);
  /* parse JSON string */
  parseCommand();
  
  Serial.println(json_str);
  
}

void loop() {}

void parseCommand() {
  JsonObject &root = jsonBufferIn.parseObject(json_str);
  if (root.success()) {
    int command = root["commandType"].as<int>();
    Serial.println("Command: ");
    Serial.println(command); 
    switch (command) {
      case 0:
        {
          //LOCK
          break;
        }
      case 1:
        {
          //UNLOCK
          break;
        }
      case 2:
        {
          //KILL SWITCH
          break;
        }
      case 3:
        {
          //MODE CHANGE
          Serial.println("Command 3 processed");
          break;
        }
      case 4:
        {
          //MOVE
          bool option = root["commandDetail"]["go"].as<bool>();
          bool forward = root["commandDetail"]["direction"].as<bool>();
          Serial.println("Forward: ");
          Serial.println(forward);
          Serial.println("Option:");
          Serial.println(option);
          break;
        }
      case 5:
        {
          //ROTATE/TURN
          bool option = root["commandDetail"]["direction"].as<int>();
          bool dir = root["dir"].as<bool>();
          Serial.println("dir: "); 
          Serial.println(dir);
          Serial.println("Option: ");
          Serial.println(option);
          break;
        }
    }
  }
}


