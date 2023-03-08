#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
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
uint32_t currentStepSize[12];
uint32_t no_key_pressed = 0; 
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

const int LUT_SIZE = 128;
int32_t LUT[LUT_SIZE];
const int32_t freq[12] = {262, 277, 294, 311, 330, 349, 367, 392, 415, 440, 466, 494};
void sine_LUT(){
    for (int i = 0; i < LUT_SIZE; i++)
  {
      LUT[i] = (int32_t)(127 * sinf(2.0 * PI * (float)i / LUT_SIZE)) + 128;
  }
}

std::string con_global; 

void sampleISR() {
  static uint32_t phaseAcc = 0;
  uint32_t total = 0;
  for (int i = 0 ; i < 12; i++){
    if (con_global[i] = '0'){
    phaseAcc += currentStepSize[i];
    uint32_t index = phaseAcc >> 25; // scale the phase accumulator to fit the lookup table size
    int32_t sineValue = LUT[index];
    total += sineValue;
    }
  }
  analogWrite(OUTR_PIN, total);
}


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

  sine_LUT();

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;
  uint8_t keyArray[7];
  std::string keyStrArray[7];
  if (millis() > next) {
    next += interval;

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Helllo World!"); // write something to the internal memory

    const int NUM_ROWS = 3; // define a constant for the number of rowsco
    for (int row = 0; row < NUM_ROWS; row++) {
        setRow(row);
        delayMicroseconds(3);
        uint8_t keys = readCols();
        std::bitset<4> keyBits(keys);
        std::string keyString = keyBits.to_string();
        keyStrArray[row] = keyString;
        keyArray[row] = keys;
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
    
    volatile uint32_t localCurrentStepSize[NUM_ROWS * 4] = {0}; // using a local variable for the step size and set to 0 (no output if no keys are pressed)
    std::string con = keyStrArray[0]+ keyStrArray[1] + keyStrArray[2];
    con_global = con; 
    size_t no_key_pressed = 0;
    for (int i = 0; i < sizeof(con); i++){
      if (con[i] == '0'){
        localCurrentStepSize[i] = stepSizes[i];
        no_key_pressed++;
      }
    }
    for (int i = 0; i < 12; i++){
      currentStepSize[i] = localCurrentStepSize[i];
    }


    

    // for (size_t i = 0; i < NUM_ROWS * 4; i++) {
    //   currentStepSize[i] = localCurrentStepSize[i]; // copy the final value to the global variable
    //   Serial.print("The index now is: ");
    //   Serial.print(i);
    //   Serial.print(" : ");
    //   Serial.println(currentStepSize[i]);
    // }


        

    // Serial.println(keys);
    u8g2.setCursor(2,20);

    // Serial.print(keyStrArray[0].c_str());
    // Serial.print(keyStrArray[1].c_str());
    // Serial.println(keyStrArray[2].c_str());


    // Serial.println(no_key_pressed);

    u8g2.drawStr(2,20, con.c_str());

    // transfer internal memory to the display
    u8g2.sendBuffer(); 
 
      
    
    //Toggle LED
    digitalToggle(LED_BUILTIN);
    setRow(1);
    // printFullBin(readCols());
  }

}