#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>


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
}

uint8_t readCols(){
  int c0, c1, c2, c3;
  uint8_t res;
  // digitalWrite(RA0_PIN, LOW);
  // digitalWrite(RA1_PIN, LOW);
  // digitalWrite(RA2_PIN, LOW);
  // digitalWrite(REN_PIN, HIGH);
  c0 = digitalRead(C0_PIN);
  c1 = digitalRead(C1_PIN);
  c2 = digitalRead(C2_PIN);
  c3 = digitalRead(C3_PIN);
  res = 0;
  res |= (c0 << 3);
  res |= (c1 << 2);
  res |= (c2 << 1);
  res |= c3;

  return(res);
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  if (rowIdx == 0){
    digitalWrite(RA0_PIN, LOW);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA2_PIN, LOW);
  }
  else if (rowIdx == 1){
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA2_PIN, LOW);
  }
  else if (rowIdx == 2){
    digitalWrite(RA0_PIN, LOW);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA2_PIN, LOW);
  }
  else if (rowIdx == 3){
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA2_PIN, LOW);
  }
  else if (rowIdx == 4){
    digitalWrite(RA0_PIN, LOW);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA2_PIN, HIGH);
  }
  else if (rowIdx == 5){
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, LOW);
    digitalWrite(RA2_PIN, HIGH);
  }
  else if (rowIdx == 6){
    digitalWrite(RA0_PIN, LOW);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA2_PIN, HIGH);
  }
  else if (rowIdx == 7){
    digitalWrite(RA0_PIN, HIGH);
    digitalWrite(RA1_PIN, HIGH);
    digitalWrite(RA2_PIN, HIGH);
  }
  digitalWrite(REN_PIN, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  if (millis() > next) {
    next += interval;
    uint8_t keyArray[7];
    std::string keyStrArray[7];
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory
    int rowIdx = 0;
    for (int i = 0; i < 3; i++){
      setRow(i);
      delayMicroseconds(3);
      uint8_t keys = readCols();
      std::bitset<4> myBinary(keys);
      std::string binaryString = myBinary.to_string();
      binaryString.c_str();
      keyStrArray[i] = binaryString;
      
    }
  
    // Serial.println(keys);
    u8g2.setCursor(2,20);
    // u8g2.print(keyArray[0], BIN);
    // u8g2.print(keyArray[1], BIN);
    // u8g2.print(keyArray[2], BIN);
    Serial.print(keyStrArray[0].c_str());
    Serial.print(keyStrArray[1].c_str());
    Serial.println(keyStrArray[2].c_str());

    std::string con = keyStrArray[0]+ keyStrArray[1] + keyStrArray[2];

    u8g2.drawStr(2,20, con.c_str());
    // u8g2.drawStr(10,20, keyStrArray[1].c_str());
    // u8g2.drawStr(10,20, keyStrArray[2].c_str());
    //u8g2.setCursor(2,20);
    //u8g2.print(count++);
    



    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
    
  }
}