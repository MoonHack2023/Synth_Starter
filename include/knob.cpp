#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <algorithm>
#include <ES_CAN.h>
#include <iostream>
#include <string>
#include <math.h>
#include <knob.h>

class Knob {
  private:
    int knobId;
    volatile int rotationVar = 0;
    std::string prevKnob = "00";
    char pressed = '1';
    std::string keyStrArray[7];
    bool master;

  public:
    // Constructor
    Knob(int _knobId, std::string _keyStrArray[7], bool _master): knobId(_knobId), keyStrArray(_keyStrArray), master(_master) {}
    int knobRotation = 0;

    // Get the value of the knob
    int getRotationVar(){
      return rotationVar;
    }

    // Get the value of pressed
    char getPressed(){
      return pressed;
    }

    void clip (int max, int min){
      if (knobRotation > max){
        knobRotation = max;
      }
      else if (knobRotation < min){
        knobRotation = min;
      }
    }

    // Decode the Knob
    void decode(){
      std::string currentKnob;
      switch (knobId){
        case 0:
          currentKnob = keyStrArray[4].substr(2, 4); 
          pressed = keyStrArray[6][0];
          break;
        case 1:
          currentKnob = keyStrArray[4].substr(0, 2); 
          pressed = keyStrArray[6][1];
          break;
        case 2:
          currentKnob = keyStrArray[3].substr(2, 4); 
          pressed = keyStrArray[5][0];
          break;
        case 3:
          currentKnob = keyStrArray[3].substr(0, 2); 
          pressed = keyStrArray[5][1];
          break;
      }

      if (prevKnob == "00" && currentKnob == "01"){
        rotationVar = -1;
      }
      else if (prevKnob == "01" && currentKnob == "00"){
        rotationVar = 1;
      }
      else if (prevKnob == "10" && currentKnob == "11"){
        rotationVar = 1;
      }
      else if (prevKnob == "11" && currentKnob == "10"){
        rotationVar = -1;
      }
      else{
        rotationVar = 0;
      }
      knobRotation += rotationVar;

      if (knobId == 3){
        if (!master){
          knobRotation = 0;
        }
        else{
          clip(8, 0);
        }

      }
      else if (knobId == 1){
        clip(1, 0);
      }
      else{
        clip(8, 0);
      }
      prevKnob = currentKnob;
    }
   
};