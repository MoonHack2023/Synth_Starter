#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <algorithm>
#include <ES_CAN.h>
#include <iostream>
#include <string>
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
const int NUM_ROWS = 4; // define a constant for the number of rows
std::string keyStrArray[7];
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t RXMutex;
SemaphoreHandle_t CAN_TX_Semaphore;
volatile int rotationVar = 0;
volatile int octaveVar = 0;
std::string prevKnob3 = "00";
std::string prevKnob2 = "00";
int knob3Rotation = 0;
int knob2Rotation = 0;
QueueHandle_t msgInQ;
uint8_t RX_Message[8]={0};
QueueHandle_t msgOutQ;
std::string prevKeyArray[7] = {"1111", "1111", "1111", "1111", "1111", "1111", "1111"};
int OCTAVE = 4;
uint8_t GLOBAL_RX_Message[8]={0};
std::string keyStr = "0000";
bool master = true;
std::string RX_keyStr = "0000";

// volatile uint32_t localCurrentStepSize;

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

void decodeKnob3(){
  std::string currentKnob3 = keyStrArray[3].substr(0, 2); 
  //Serial.println(keyStrArray[3]);

  if (prevKnob3 == "00" && currentKnob3 == "01"){
    rotationVar = -1;
  }
  else if (prevKnob3 == "01" && currentKnob3 == "00"){
    rotationVar = 1;
  }
  else if (prevKnob3 == "10" && currentKnob3 == "11"){
    rotationVar = 1;
  }
  else if (prevKnob3 == "11" && currentKnob3 == "10"){
    rotationVar = -1;
  }
  else{
    rotationVar = 0;
  }
  knob3Rotation += rotationVar;

  if (knob3Rotation > 8){
    knob3Rotation = 8;
  }
  else if (knob3Rotation < 0){
    knob3Rotation = 0;
  }

  prevKnob3 = currentKnob3;
}
void decodeKnob2(){
  std::string currentKnob2 = keyStrArray[3].substr(2, 4); 
  //Serial.println(keyStrArray[3]);

  if (prevKnob2 == "00" && currentKnob2 == "01"){
    octaveVar = -1;
  }
  else if (prevKnob2 == "01" && currentKnob2 == "00"){
    octaveVar = 1;
  }
  else if (prevKnob2 == "10" && currentKnob2 == "11"){
    octaveVar = 1;
  }
  else if (prevKnob2 == "11" && currentKnob2 == "10"){
    octaveVar = -1;
  }
  else{
    octaveVar = 0;
  }

  knob2Rotation += octaveVar;

  if (knob2Rotation > 8){
    knob2Rotation = 8;
  }
  else if (knob2Rotation < 0){
    knob2Rotation = 0;
  }

  OCTAVE = knob2Rotation;
  prevKnob2 = currentKnob2;
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

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - knob3Rotation);
  analogWrite(OUTR_PIN, (Vout + 128));
}


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
  if (zeroCount != 0){
    sum /= zeroCount;
  }
  return sum;
}

void scanKeysTask(void * pvParameters){
  Serial.println("SCAN");
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t TX_Message[8] = {0};
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency);
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
    keyStr = keyStrArray[0]+ keyStrArray[1] + keyStrArray[2] + keyStrArray[3];
    
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
    std::string hi_str = keyStr.substr(6, 5);   // bit 6 to bit 11
    int lo_val = stoi(lo_str, nullptr, 2);
    int hi_val = stoi(hi_str, nullptr, 2);
    TX_Message[3] = lo_val;
    TX_Message[4] = hi_val;
  }
  
  if (master){
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
    std::bitset<5> binaryLow(RX_Message[4]);
    std::string binaryLowStr = binaryLow.to_string();
    RX_keyStr = binaryHighStr + binaryLowStr;
    
    xSemaphoreGive(RXMutex);
    
    uint32_t sumMaster = chords(keyStr,OCTAVE);
    uint32_t sumSlave = chords(RX_keyStr,RX_Message[1]);

    localCurrentStepSize = (sumSlave +  sumMaster);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }


  std::copy(keyStrArray, keyStrArray + sizeof(keyStrArray)/sizeof(keyStrArray[0]), prevKeyArray);
  
  if (!master){
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
  }

  decodeKnob3();
  decodeKnob2();
    
    // std::string currentKnob3 = keyStrArray[3].substr(0, 2); 
    // Serial.println(keyStrArray[3].substr(0,2).c_str());
    // Serial.println(knob3Rotation);
    // __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void *  pvParameters){
  // Serial.println("DISPLAY");
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID = 0x123;

  while(1){
    // Serial.println("DISPLAY");
    vTaskDelayUntil( &xLastWakeTime, xFrequency);
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
    std::string vol = "Vol: " + std::to_string(knob3Rotation);
    u8g2.drawStr(66,30, vol.c_str());
    std::string octave = "Octave: " + std::to_string(OCTAVE);
    u8g2.drawStr(2,30, octave.c_str());


    u8g2.sendBuffer();
    
  }  
}

void decodeTask(void *  pvParameters){
  Serial.println("DECODE");
  uint32_t ID = 0x123;
  uint32_t localCurrentStepSize;
  uint8_t Local_RX_Message[8];
  // Serial.println("DECODE");
  while (1){
    xSemaphoreTake(RXMutex, portMAX_DELAY);
    xQueueReceive(msgInQ, Local_RX_Message, portMAX_DELAY);
    // Serial.print("Local:");
    // Serial.println(Local_RX_Message[1]);
    memcpy(RX_Message, Local_RX_Message, sizeof(RX_Message));
    // mem(RX_Message, RX_Message + sizeof(RX_Message)/sizeof(RX_Message), Local_RX_Message);
    xSemaphoreGive(RXMutex);
    // Serial.print("Global: ");
    // Serial.println(RX_Message[1]);
  
  }  
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID = 0x123;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_Task (void * pvParameters) {
  Serial.println("CAN");
	uint8_t msgOut[8];
	while (1) {
	   xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
	   xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
	   CAN_TX(0x123, msgOut);
	}
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void setup() {
  // put your setup code here, to run once:
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
  keyArrayMutex = xSemaphoreCreateMutex();
  RXMutex = xSemaphoreCreateMutex();  
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

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
  if (!master){
    CAN_RegisterTX_ISR(CAN_TX_ISR);
  }
  setCANFilter(0x123,0x7ff);
  CAN_Start();

  //Initialise UART
  Serial.begin(9600);
  // Serial.println("Hello World");

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  if (master){
    sampleTimer->attachInterrupt(sampleISR);
  }
  sampleTimer->resume();

  

  
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    4,			/* Task priority */
    &scanKeysHandle );  /* Pointer to store the task handle */
  
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &displayUpdateHandle );  /* Pointer to store the task handle */

  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
    decodeTask,		/* Function that implements the task */
    "decode",		/* Text name for the task */
    32,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &decodeHandle );  /* Pointer to store the task handle */
  
  TaskHandle_t CAN_TXHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CAN_TX",		/* Text name for the task */
    32,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &CAN_TXHandle );  /* Pointer to store the task handle */

  


  
  vTaskStartScheduler();
}

void loop() {
    // Serial.println(RX_Message[3]);
    // Serial.println(finall.c_str());

}