#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <algorithm>
#include <ES_CAN.h>
#include <iostream>
#include <string>
#include <math.h>
#include "knob.hpp"

extern volatile uint32_t currentStepSize;
extern volatile uint8_t keyArray[7];
extern const int NUM_ROWS = 7; // define a constant for the number of rows
extern std::string keyStrArray[7];
extern SemaphoreHandle_t keyArrayMutex;
extern SemaphoreHandle_t RXMutex;
extern SemaphoreHandle_t CAN_TX_Semaphore;
extern volatile int rotationVar = 0;
extern volatile int octaveVar = 0;
extern volatile int masVar = 0;
extern std::string prevKnob3 = "00";
extern std::string prevKnob2 = "00";
extern std::string prevKnob1 = "00";
extern std::string prevKnob0 = "00";
// int knob3Rotation = 0;
// int knob2Rotation = 0;
// int knob1Rotation = 0;
// int knob0Rotation = 0;
extern QueueHandle_t msgInQ;
extern uint8_t RX_Message[8]={0};
extern QueueHandle_t msgOutQ;
extern std::string prevKeyArray[7] = {"1111", "1111", "1111", "1111", "1111", "1111", "1111"};
// int OCTAVE = 4;
extern uint8_t GLOBAL_RX_Message[8]={0};
extern std::string keyStr = "0000";
extern volatile bool master = true;
extern std::string RX_keyStr = "0000";