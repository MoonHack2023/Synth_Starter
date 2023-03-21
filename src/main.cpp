#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <algorithm>
#include <ES_CAN.h>
#include <iostream>
#include <string>
#include <math.h>

// Define the max() macro
#define max(a, b) ((a) > (b) ? (a) : (b))

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Check current step size
volatile uint32_t currentStepSize;
volatile uint8_t keyArray[7];
const int NUM_ROWS = 7; // define a constant for the number of rows
std::string keyStrArray[7];
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t RXMutex;
SemaphoreHandle_t CAN_TX_Semaphore;
volatile int volVar = 0;
volatile int octaveVar = 0;
volatile int masVar = 0;
std::string prevKnob3 = "00";
std::string prevKnob2 = "00";
std::string prevKnob1 = "00";
std::string prevKnob0 = "00";
int knob3Rotation = 0;
int knob2Rotation = 0;
int knob1Rotation = 0;
int knob0Rotation = 0;
QueueHandle_t msgInQ;
uint8_t RX_Message[8]={0};
QueueHandle_t msgOutQ;
std::string prevKeyArray[7] = {"1111", "1111", "1111", "1111", "1111", "1111", "1111"};
int OCTAVE = 4;
uint8_t GLOBAL_RX_Message[8]={0};
std::string keyStr = "0000";
volatile bool master = true;
std::string RX_keyStr = "0000";

// volatile uint32_t localCurrentStepSize;

// class MyClass {
//     public:
//         void printGlobalVariable() {
//             Serial.println(keyStr.c_str());
//         }
// };

void clip (int& knobRotation, int max, int min){
  if (knobRotation > max){
    knobRotation = max;
  }
  else if (knobRotation < min){
    knobRotation = min;
  }
}


const std::string keyValues[NUM_ROWS][4] = {
  {"0111", "1011", "1101", "1110"},
  {"0111", "1011", "1101", "1110"},
  {"0111", "1011", "1101", "1110"}
};
const std::string noteNames[NUM_ROWS][4] = {
  {"C4", "C#4", "D4", "D#4"},
  {"E4", "F4", "F#4", "G4"},
  {"G#4", "A4", "A#4", "B4"}
};


//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

  // Function to concatenate bits
  uint8_t concatenateBits(int c0, int c1, int c2, int c3){
    uint8_t result = 0;
    result |= (c0 << 3);
    result |= (c1 << 2);
    result |= (c2 << 1);
    result |= c3;
    return result;
  }

  //Function to read the inputs from the four columns of the switch matrix (C0,1,2,3) and return the four bits concatenated together as a single byte
  uint8_t readCols(){

  int c0state = digitalRead(C0_PIN);
  int c1state = digitalRead(C1_PIN);
  int c2state = digitalRead(C2_PIN);
  int c3state = digitalRead(C3_PIN);

  // Call the concatenateBits() function with the read states
  uint8_t cols = concatenateBits(c0state, c1state, c2state, c3state);
  return cols;
}

//Select a given row of the switch matrix by setting the value of each row select address pin 
void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, rowIdx & 0b001);
  digitalWrite(RA1_PIN, rowIdx & 0b010);
  digitalWrite(RA2_PIN, rowIdx & 0b100);
  digitalWrite(REN_PIN,HIGH);
}

// Decode the rightest Knob
void decodeKnob(int knobId, std::string currentKnob, int& knobRotation, std::string& prevKnob){

  int rotationVar = 0;

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
      clip(knobRotation, 8, 0);
    }
  }
  else if (knobId == 1){
    clip(knobRotation, 1, 0);
    // master = bool(knobRotation);
  }
  else{
    clip(knobRotation, 8, 0);
  }

  prevKnob = currentKnob;


}


const uint32_t stepSizes [] = {

  51076922, //C4
      54112683, //C#4
      57330004, //D4
      60740598, //D#4
      64352275, //E4 
      68178701, //F4
      72231588, //F#4
      76528508, //G4
      81077269, //G#4
      85899345, //A4
      91006452, //A#4
      96426316, //B4
};

const int LUT_SIZE = 1024;
int32_t LUT[LUT_SIZE];

void sine_LUT() {
  const float step = 2.0 * PI / LUT_SIZE;
  for (int i = 0; i < LUT_SIZE; i++) {
    float angle = i * step;
    LUT[i] = (int32_t)(127.0f * sinf(angle)) + 128;
  }
}

// // Sawtooth wave
// void sampleISR() {
//   static uint32_t phaseAcc = 0;
//   phaseAcc += currentStepSize;
//   int32_t Vout = (phaseAcc >> 24) - 128;
//   Vout = Vout >> (8 - knob3Rotation);
//   analogWrite(OUTR_PIN, (Vout + 128));
// }

//Sine wave
void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  uint32_t index = phaseAcc >> 22; // scale the phase accumulator to fit the lookup table size (2024 = 2^10)
  int32_t sineValue = LUT[index];
  // Serial.println()
  sineValue = sineValue >> (8 - volVar);
  analogWrite(OUTR_PIN, sineValue);
}


// Create chords by summing the currentstepsize of each key 
uint32_t chords(std::string keyStr, int OCTAVE){
  int zeroCount = 0;
  uint32_t sum = 0;
  uint32_t localCurrentStepSize = 0;
  for (int i = 0; i < 12; i++){
    if (keyStr[i] == '0'){
      zeroCount++;
      localCurrentStepSize = stepSizes[i];
      if (OCTAVE < 4){
        localCurrentStepSize = localCurrentStepSize >> -(OCTAVE - 4);
      }
      else{
        localCurrentStepSize = localCurrentStepSize << (OCTAVE - 4);
      }
      
      sum += localCurrentStepSize;
    }
  }
  // if (zeroCount != 0){
  //   sum /= zeroCount;
  // }
  return sum;
}

uint32_t countZero(std::string keyStr){
  int zeroCount = 0;
  for (int i = 0; i < 12; i++){
    if (keyStr[i] == '0'){
      zeroCount++;
    }
  }
  return zeroCount;
}

// Everything that's relevant to scanning the Keys
void scanKeysTask(void * pvParameters){
  // Serial.println("SCAN");
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t TX_Message[8] = {0};
  uint8_t prevTX_Message;
    // const int NUM_ROWS = 3; // define a constant for the number of rows
    uint32_t localCurrentStepSizeT = 0;
    uint32_t localCurrentStepSizeR = 0;
    uint32_t localCurrentStepSize  = 0;
    for (int row = 0; row < NUM_ROWS; row++) {
      setRow(row);
      delayMicroseconds(3);
      uint8_t keys = readCols();
      std::bitset<4> keyBits(keys);
      std::string keyString = keyBits.to_string();
      keyStrArray[row] = keyString;
      keyArray[row] = keys;
    }
    keyStr = keyStrArray[0]+ keyStrArray[1] + keyStrArray[2] + keyStrArray[3]; // + keyStrArray[4] + keyStrArray[5] + keyStrArray[6];
    // decodeKnob(1, keyStrArray[4].substr(0, 2), knob1Rotation, prevKnob1);
    // master = bool(knob1Rotation);
    if (keyStrArray[5][3] == '1' ){ // left most or solo
      master = true;
      // Serial.println("MASTER");
    }
    else{
      master = false;
      // Serial.println("Slave");
    }
    // int zeroCount = 0;
    // uint32_t sum = 0;
    // for (int i = 0; i < 12; i++){
    //   if (keyStr[i] == '0'){
    //     zeroCount++;
    //     localCurrentStepSizeT = stepSizes[i];
    //     localCurrentStepSizeT = localCurrentStepSizeT << (OCTAVE - 4);
    //     sum += localCurrentStepSizeT;
    //   }
    // }
    // if (zeroCount != 0){
    //   sum /= zeroCount;
    // }
    
    // currentStepSize = localCurrentStepSizeT;
  // this was checkeypress
  // uint8_t TX_Message[8];

  if (!master){
    TX_Message[1] = OCTAVE ;
    for (int row = 0; row < NUM_ROWS-1 ; row++){
      for (int col = 0; col < 5; col++){
        if (keyStrArray[row][col] != prevKeyArray[row][col]){
          if (prevKeyArray[row][col] == '1'){
            TX_Message[0] = 80;
          }
          else if (prevKeyArray[row][col] == '0'){
            TX_Message[0] = 82;
          }
          TX_Message[2] = row*4 + col;
        }
      }
    }
    std::string lo_str = keyStr.substr(0, 6);  // bit 0 to bit 5
    std::string hi_str = keyStr.substr(6, 6);   // bit 6 to bit 11
    int lo_val = stoi(lo_str, nullptr, 2);
    int hi_val = stoi(hi_str, nullptr, 2);
    TX_Message[3] = lo_val;
    TX_Message[4] = hi_val;
  }
  
  uint32_t sumSlave = 0;
  if (master){
    if (keyStrArray[6][3] == '0' || keyStrArray[5][3] == '0'){
      // Serial.println("MASTER");
      xSemaphoreTake(RXMutex, portMAX_DELAY);
      // for (int i = 0; i < 4; i++){
        // detect press messages
      if (RX_Message[0] == 80){
        // Serial.println("Pressed");
        localCurrentStepSizeR = stepSizes[RX_Message[2]];
        localCurrentStepSizeR = localCurrentStepSizeR << (RX_Message[1] - 4);
      }
      // }
      std::bitset<6> binaryHigh(RX_Message[3]);
      std::string binaryHighStr = binaryHigh.to_string();
      std::bitset<6> binaryLow(RX_Message[4]);
      std::string binaryLowStr = binaryLow.to_string();
      RX_keyStr = binaryHighStr + binaryLowStr;
      
      xSemaphoreGive(RXMutex);
      
      
      if (keyStrArray[6][3] == '0'){
        sumSlave = chords(RX_keyStr,OCTAVE+1);
      }
      else{
        sumSlave = chords(RX_keyStr,OCTAVE-1);
      }
    }

    for (int i = 0; i < 7; i++){
        keyStrArray[i] = "0000";
    }
    keyStr = keyStrArray[0]+ keyStrArray[1] + keyStrArray[2] + keyStrArray[3]; // + keyStrArray[4] + keyStrArray[5] + keyStrArray[6];
    

    uint32_t sumMaster = chords(keyStr,OCTAVE);

    if (localCurrentStepSize != 0) {
      localCurrentStepSize = (sumSlave +  sumMaster) / (countZero(RX_keyStr) + countZero(keyStr));
    }
    else{
      localCurrentStepSize = (sumSlave +  sumMaster);
    }

    
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }


  std::copy(keyStrArray, keyStrArray + sizeof(keyStrArray)/sizeof(keyStrArray[0]), prevKeyArray);
  
  if (!master && TX_Message[0] == 80){
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
    TX_Message[0] = 82;
  }

  // knob3ptr -> decode();
  // knob2ptr -> decode();
  // knob1ptr -> decode();
  
  decodeKnob(3, keyStrArray[3].substr(0, 2), knob3Rotation, prevKnob3);
  volVar = knob3Rotation;
  decodeKnob(2, keyStrArray[3].substr(2, 4), knob2Rotation, prevKnob2);
  OCTAVE = knob2Rotation;
  

  
    
    // std::string currentKnob3 = keyStrArray[3].substr(0, 2); 
    // Serial.println(keyStrArray[3].substr(0,2).c_str());
    // Serial.println(knob3Rotation);
    // __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }


// Display it on the screen
void displayUpdateTask(void *  pvParameters){
  //Serial.println("DISPLAY");
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID = 0x123;

  // while(1) was here {
    // Serial.println("DISPLAY");
    //vTaskDelayUntil( &xLastWakeTime, xFrequency);
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    // u8g2.drawStr(2,10,"Hello World!"); // write something to the internal memory
    
    u8g2.drawStr(2,10, keyStr.c_str());
    u8g2.drawStr(2,20, RX_keyStr.c_str());
    
    // u8g2.setCursor(2,20);
    // xSemaphoreTake(RXMutex, portMAX_DELAY);
    // u8g2.print((char) RX_Message[0]);
    // u8g2.print(RX_Message[1]);
    // u8g2.print(RX_Message[2]);
    // xSemaphoreGive(RXMutex);
     // u8g2.setCursor(2,40);
    std::string vol = "Vol: " + std::to_string(volVar);
    u8g2.drawStr(40,30, vol.c_str());
    std::string octave = "Oct: " + std::to_string(OCTAVE);
    u8g2.drawStr(2,30, octave.c_str());
    u8g2.drawStr(90,30, master ? "M": "S");
    
    u8g2.sendBuffer();
    
  
}

void decodeTask(void *  pvParameters){
  //Serial.println("DECODE");
  uint32_t ID = 0x123;
  uint32_t localCurrentStepSize;
  uint8_t Local_RX_Message[8];
  // Serial.println("DECODE");
  // while (1){
    xSemaphoreTake(RXMutex, portMAX_DELAY);
    xQueueReceive(msgInQ, Local_RX_Message, portMAX_DELAY); // i think comment this line out to for timing for DECODE TASK
    // Serial.print("Local:");
    // Serial.println(Local_RX_Message[1]);
    memcpy(RX_Message, Local_RX_Message, sizeof(RX_Message));
    // mem(RX_Message, RX_Message + sizeof(RX_Message)/sizeof(RX_Message), Local_RX_Message);
    xSemaphoreGive(RXMutex);

    // Serial.print("Global: ");
    // Serial.println(RX_Message[1]);
  
  //} while loop bracket
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID = 0x123;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_Task (void * pvParameters) {
  //Serial.println("CAN");
	uint8_t msgOut[8];
	//while (1) { comment out the while loop
	   xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
	   xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY); 
	   CAN_TX(0x123, msgOut);
     xSemaphoreGive(CAN_TX_Semaphore); //added for timing
	//}
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

// void change_setup_Task (void * pvParameters){
//   keyStrArray
// }

void setup() {
  // put your setup code here, to run once:
  
  msgInQ = xQueueCreate(384,3);
  msgOutQ = xQueueCreate(384,3);
  keyArrayMutex = xSemaphoreCreateMutex();
  RXMutex = xSemaphoreCreateMutex();  
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);  // for timing i think we need more
  uint8_t test_send[8]={0};
  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  CAN_Init(false);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  // if (!master){
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  // }
  setCANFilter(0x123,0x7ff);
  CAN_Start();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("FROM SETUP");

  // for timing CAN_TX_TASK

  for (int iter = 0; iter < 32; iter++) {
      xQueueSend( msgOutQ, test_send, portMAX_DELAY);
    }


  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
    CAN_TX_Task(NULL);  // FOR CAN you need to go click on  CAN_TX_TASK and comment line 111 in the library
  } 
  Serial.println(micros()-startTime);

  // Serial.println("Hello World");
//   sine_LUT();

//   TIM_TypeDef *Instance = TIM1;
//   HardwareTimer *sampleTimer = new HardwareTimer(Instance);
//   sampleTimer->setOverflow(22000, HERTZ_FORMAT);
//   // if (master){
//   sampleTimer->attachInterrupt(sampleISR);
//  //}
//   sampleTimer->resume();


  
  // TaskHandle_t scanKeysHandle = NULL;
  // xTaskCreate(
  //   scanKeysTask,		/* Function that implements the task */
  //   "scanKeys",		/* Text name for the task */
  //   64,      		/* Stack size in words, not bytes */
  //   NULL,			/* Parameter passed into the task */
  //   4,			/* Task priority */
  //   &scanKeysHandle );  /* Pointer to store the task handle */
  
  // TaskHandle_t displayUpdateHandle = NULL;
  // xTaskCreate(
  //   displayUpdateTask,		/* Function that implements the task */
  //   "displayUpdate",		/* Text name for the task */
  //   256,      		/* Stack size in words, not bytes */
  //   NULL,			/* Parameter passed into the task */
  //   3,			/* Task priority */
  //   &displayUpdateHandle );  /* Pointer to store the task handle */
  
  
  

  // TaskHandle_t decodeHandle = NULL;
  // xTaskCreate(
  //   decodeTask,		/* Function that implements the task */
  //   "decode",		/* Text name for the task */
  //   32,      		/* Stack size in words, not bytes */
  //   NULL,			/* Parameter passed into the task */
  //   2,			/* Task priority */
  //   &decodeHandle );  /* Pointer to store the task handle */

    
  

    

  // TaskHandle_t CAN_TXHandle = NULL;
  // xTaskCreate(
  //   CAN_TX_Task,		/* Function that implements the task */
  //   "CAN_TX",		/* Text name for the task */
  //   32,      		/* Stack size in words, not bytes */
  //   NULL,			/* Parameter passed into the task */
  //   1,			/* Task priority */
  //   &CAN_TXHandle );  /* Pointer to store the task handle */
  
  // vTaskStartScheduler();
}

void loop() {
    // Serial.println(RX_Message[3]);
    // Serial.println(finall.c_str());
    // Serial.print(master);
    // Serial.println(knob1Rotation);
    //classptr->printGlobalVariable();
    // knob3ptr-> print();
    // Serial.print("WEST: ");
    // Serial.println(keyStrArray[5].c_str());
    // Serial.print("EAST: ");
    // Serial.println(keyStrArray[6].c_str());
}