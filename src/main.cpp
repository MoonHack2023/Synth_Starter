#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <algorithm>
#include <ES_CAN.h>
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
volatile int rotationVar = 0;
std::string prevKnob3 = "00";
int knob3Rotation = 0;
// QueueHandle_t msgInQ;
uint8_t TX_Message[8] = {0};




std::string prevKeyArray[7] = {"1111", "1111", "1111", "1111", "1111", "1111", "1111"};
const int OCTAVE = 4;

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
    rotationVar = 1;
  }
  else if (prevKnob3 == "01" && currentKnob3 == "00"){
    rotationVar = -1;
  }
  else if (prevKnob3 == "10" && currentKnob3 == "11"){
    rotationVar = -1;
  }
  else if (prevKnob3 == "11" && currentKnob3 == "10"){
    rotationVar = 1;
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

const uint32_t stepSizes [] = {
  /*
  1ull << 32 shift the value 1 to the left by 32 bits,setting the 33rd bit to 1. 
  This creates a 64-bit binary number of 2ˆ32
  Using this to obtain a constant that represents one full cycle of a sine wave in the phase accumulator
  use 1ull << 32 instead of 2^32 directly to ensure that the result is a 64-bit integer with the most significant bit set to 1.
  */
  (uint32_t)((1ull << 32) * 261.63 / 22000), //C4
  (uint32_t)((1ull << 32) * 277.18 / 22000), //C#4
  (uint32_t)((1ull << 32) * 293.66 / 22000), //D4
  (uint32_t)((1ull << 32) * 311.13 / 22000), //D#4
  (uint32_t)((1ull << 32) * 329.63 / 22000), //E4
  (uint32_t)((1ull << 32) * 349.23 / 22000), //F4
  (uint32_t)((1ull << 32) * 369.99 / 22000), //F#4
  (uint32_t)((1ull << 32) * 392.00 / 22000), //G4
  (uint32_t)((1ull << 32) * 415.30 / 22000), //G#4
  (uint32_t)((1ull << 32) * 440.00 / 22000), //A4
  (uint32_t)((1ull << 32) * 466.16 / 22000), //A#4
  (uint32_t)((1ull << 32) * 493.88 / 22000), //B4
};

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - knob3Rotation);
  analogWrite(OUTR_PIN, (Vout + 128));
}

void checkKeyPress(){
  // uint8_t TX_Message[8] = {0};
  // uint8_t TX_Message[8] = {0};
  TX_Message[1] = OCTAVE;
  for (int row = 0; row < NUM_ROWS ; row++){
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
  // Serial.println(TX_Message[2]);
  std::copy(keyStrArray, keyStrArray + sizeof(keyStrArray)/sizeof(keyStrArray[0]), prevKeyArray);
  CAN_TX(0x123, TX_Message);
  Serial.println(TX_Message[0]);
}

void scanKeysTask(void * pvParameters){
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency);
    // const int NUM_ROWS = 3; // define a constant for the number of rows
    uint32_t localCurrentStepSize = 0;
    for (int row = 0; row < NUM_ROWS; row++) {
      setRow(row);
      delayMicroseconds(3);
      uint8_t keys = readCols();
      std::bitset<4> keyBits(keys);
      std::string keyString = keyBits.to_string();
      keyStrArray[row] = keyString;
      keyArray[row] = keys;
      for (int col = 0; col < 4; col++) {
        xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
        if (keyStrArray[row] == keyValues[row][col]) {
          localCurrentStepSize = stepSizes[row * 4 + col];
        }
        xSemaphoreGive(keyArrayMutex);
      }
    }
    currentStepSize = localCurrentStepSize;
    checkKeyPress();
    
    decodeKnob3();
    
    
    // std::string currentKnob3 = keyStrArray[3].substr(0, 2); 
    // Serial.println(keyStrArray[3].substr(0,2).c_str());
    // Serial.println(knob3Rotation);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void *  pvParameters){
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID = 0x123;
  uint8_t RX_Message[8]={0};

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency);
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    // u8g2.drawStr(2,10,"Hello World!"); // write something to the internal memory
    std::string con = keyStrArray[0]+ keyStrArray[1] + keyStrArray[2] + keyStrArray[3];
    u8g2.drawStr(2,10, con.c_str());
    
    u8g2.setCursor(66,30);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);

    while (CAN_CheckRXLevel())
	    CAN_RX(ID, RX_Message);

    u8g2.sendBuffer();
    
  }  
}

// void decodeTask(void *  pvParameters){
//   const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
//   TickType_t xLastWakeTime = xTaskGetTickCount();
//   uint32_t ID = 0x123;
//   uint32_t localCurrentStepSize;
//   while(1){
//     xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

//     while (CAN_CheckRXLevel()){
// 	    CAN_RX(ID, RX_Message);
//     }

//     for (int i = 0; i < 4; i++){
//       // detect press messages
//       if (RX_Message[0] == 80){
//         localCurrentStepSize = stepSizes[RX_Message[2]] ;
//         localCurrentStepSize << (RX_Message[1] - 4);
//         __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
//       }
//       // detect release messages
//       else if (RX_Message[0] == 82){
//         currentStepSize = 0;
//       }
//     }

    
//   }  
// }

// void CAN_RX_ISR (void) {
// 	uint8_t RX_Message_ISR[8];
// 	uint32_t ID;
// 	CAN_RX(ID, RX_Message_ISR);
// 	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
// }


void setup() {
  // put your setup code here, to run once:

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

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_Start();

  // msgInQ = xQueueCreate(36,8);
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &scanKeysHandle );  /* Pointer to store the task handle */
  
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &displayUpdateHandle );  /* Pointer to store the task handle */

  // TaskHandle_t decodeHandle = NULL;
  // xTaskCreate(
  //   decodeTask,		/* Function that implements the task */
  //   "decode",		/* Text name for the task */
  //   256,      		/* Stack size in words, not bytes */
  //   NULL,			/* Parameter passed into the task */
  //   1,			/* Task priority */
  //   &decodeHandle );  /* Pointer to store the task handle */

  
  keyArrayMutex = xSemaphoreCreateMutex();

  
  vTaskStartScheduler();
}

void loop() {

}
