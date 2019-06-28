#include <SPI.h>
/*
 * Interface Commands (3-bit mode)
 * 
 * 000 (0) - Move down coarse step size
 * 001 (1) - Move up coarse step size
 * 010 (2) - Move down fine step size
 * 011 (3) - Move up coarse step size
 * 100 (4) - Move lense to down to min
 * 101 (5) - Move lense up to max
 * 
 */

// Step size parameters
#define STEP_FINE          5
#define STEP_COARSE        10

// xxxset up the speed, data order and data mode
 SPISettings cameraSetting(10000, MSBFIRST, SPI_MODE2);

// DAQ Interface
const int daqToggle = 3; // DAQ Digital 2

const int daqB0     = 5; // DAQ Digital 3
const int daqB1     = 6; // DAQ Digital 4
const int daqB2     = 7; // DAQ Digital 5

// Lense position
int lens_pos = 0;

// Lens Interface
const int chipSelectPin = 10;
const int messagePin = 9;

// Buffer
uint8_t outputByte, inputByte;
const int delayTime = 200;
//const int delayTime = 100;
int inByte;
char serial_buffer[2];

void setup() 
{

  // Initialize Lens Interface
  SPI.begin(); 
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV128, MSBFIRST, SPI_MODE3));
  //SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV32, MSBFIRST, SPI_MODE3));
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, LOW);
  digitalWrite(messagePin, LOW);
  outputByte = 0x55;
  
  // Initialize DAQ interface
  pinMode(daqToggle, INPUT);
  pinMode(daqB2, INPUT);
  pinMode(daqB1, INPUT);
  pinMode(daqB0, INPUT);

  // Setup Toggle ISR
  attachInterrupt(digitalPinToInterrupt(daqToggle), commandLensISR, CHANGE);

  // Initialize USB Serial
  Serial.begin(19200);

  // Synchronize Lens
  syncLens();
  syncLens();

}

void loop() {
  
  // ISR Service Toggle Pin
  // Listen for query
  
}

void syncLens() {
  digitalWrite(messagePin, HIGH);
  
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(0x0A);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime); 
  
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, LOW);
  
  delayMicroseconds(delayTime);
  
  digitalWrite(messagePin, LOW);
}

void moveSteps(short steps) {

  // Get bytes of steps
  char b0 = steps & 0xFF;
  char b1 = (steps >> 8) & 0xFF;
  //Serial.print(b1,HEX);
  //Serial.print("\n");
  //Serial.print(b0,HEX);
  //Serial.print("\n");
  digitalWrite(messagePin, HIGH);
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(0x44);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime);
  
  // First Byte
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(b1);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime);    
  // Second Byte
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(b0);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime);

  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(0x0F);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime);
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(0x0A);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime);
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime);               
  digitalWrite(messagePin, LOW);     

}

void moveMin() {
  
  digitalWrite(messagePin, HIGH);
  
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(0x05);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime);
  
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(0x0F);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime);    
  
  digitalWrite(chipSelectPin, HIGH);
  inputByte = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(delayTime);
  
  digitalWrite(messagePin, LOW);
  
}

void moveMax() {
  
     digitalWrite(messagePin, HIGH);

    digitalWrite(chipSelectPin, HIGH);
    inputByte = SPI.transfer(0x06);
    digitalWrite(chipSelectPin, LOW);
    delayMicroseconds(delayTime);
    
    digitalWrite(chipSelectPin, HIGH);
    inputByte = SPI.transfer(0x0F);
    digitalWrite(chipSelectPin, LOW);
    delayMicroseconds(delayTime);    

    digitalWrite(chipSelectPin, HIGH);
    inputByte = SPI.transfer(0x00);
    digitalWrite(chipSelectPin, LOW);
    delayMicroseconds(delayTime);

    digitalWrite(messagePin, LOW);
}



/*
 * Interface Commands (3-bit mode)
 * 
 * 000 (0) - Move down coarse step size
 * 001 (1) - Move up coarse step size
 * 010 (2) - Move down fine step size
 * 011 (3) - Move up coarse step size
 * 100 (4) - Move lense to down to min
 * 101 (5) - Move lense up to max
 * 
 */

void commandLensISR() {

  // Read Interface
  int cmd = (PIND & B11100000) >> 5;
  
  //Serial.print(cmd,BIN);
  //Serial.print('\n');
  // Synchronize Lens
  syncLens();
  
  // Send Command
  switch(cmd) {
    case 0:
      Serial.write("Move down coarse\n");
      moveSteps(-(STEP_COARSE + 1));
      lens_pos -= STEP_COARSE;
      break;
    case 1:
      Serial.write("Move up coarse\n");
      moveSteps(STEP_COARSE);
      lens_pos += STEP_COARSE;
      break;
    case 2:
      Serial.write("Move down fine\n");
      moveSteps(-(STEP_FINE + 1));
      lens_pos -= STEP_FINE;
      break;
    case 3:
      Serial.write("Move up fine\n");
      moveSteps(STEP_FINE);
      lens_pos += STEP_FINE;
      break;
    case 4:
      Serial.write("Move min\n");
      moveMin();
      lens_pos = 0;
      break;
    case 5:
      Serial.write("Move max\n");
      moveMax();
      lens_pos = 0;
      //lens_pos += STEP_FINE;      
      break;
    case 6:
      //Serial.write("Query z position\n");
      serial_buffer[1] = lens_pos & 0xFF;
      serial_buffer[0] = (lens_pos >> 8) & 0xFF;
      Serial.write(serial_buffer,sizeof(serial_buffer));
      break;
    case 7:
      //Serial.write("Query z position\n");
      serial_buffer[1] = lens_pos & 0xFF;
      serial_buffer[0] = (lens_pos >> 8) & 0xFF;
      Serial.print(lens_pos);
      Serial.print("\n");
      break;
  }
}



/*****************************************************
 *  Old Loop Code and other camera commands
 */
void commands() {
    //delay(25);
  
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    
    // get incoming byte:
    inByte = Serial.read();

    switch(inByte){

//      case 'f': //flash camera
//        PORTD = PORTD | B00010000;
//        //digitalWrite(Flash_Fire,HIGH);   // turn off YELLOW LED and fires flash
//        //digitalWrite(Flash_Reset,HIGH); // turn on RED led
//        delay(1000);
//        digitalWrite(Flash_Fire,LOW);  //turns on YELLOW LED reset relay
//        digitalWrite(Flash_Reset,HIGH);
//        delay(1000);
//        digitalWrite(Flash_Reset,LOW);  //turns off RED LED
//        break;     
        
      case '0':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA

        8/10/15
        move focal group 10^0 1 steps toward higher focal number
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x01);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  

        break;    

      case ')':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA

        8/10/15
        move focal group 10^0 1 steps toward lower focal number        
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xFF);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xFE);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  

        break; 


      case '1':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA

        8/10/15
        move focal group 10^1 10 steps toward higher focal number
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  

        break;    

      case '!':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA

        8/10/15
        move focal group 10^1 10 steps toward lower focal number        
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xFF);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xF5);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  

        break; 

      case '2':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA

        8/10/15
        move focal group 10^2 100 steps toward higher focal number
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x64);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  

        break;    

      case '@':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA

        8/10/15
        move focal group 10^2 100 steps toward lower focal number        
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xFF);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x9B);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  

        break; 

      case '3':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA

        8/10/15
        move focal group 10^3 1000 steps toward higher focal number
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x03);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xE8);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  

        break;    

      case '#':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA

        8/10/15
        move focal group 10^3 1000 steps toward lower focal number        
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xFC);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x17);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  

        break; 


      case 'S': 
      //Sync command request 0x0A - reply 0xAA
      /* 8/6/15 The sync command needs to be sent when the lens powers up.  It may also need to be sent more often
       *  
       */      
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime); 
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(messagePin, LOW);
        
        break;        

      case 'I': //INFO command
      // body - 0x80 0x0A  reply - maybe 7 bytes, not sure 
      /*
       * 8/6/15 Body asks for Lens info, replys with 7 bytes not sure what they mean
       * The Sync command needs to be sent first, maybe twice on older (? len's)
       */
        digitalWrite(messagePin, HIGH);
   
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x80);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);                
                        
        break;

      case 'F':
      // request 0xA0 reply 2 bytes
      /*
       * 8/6/15 The body asks for current focal length in the next 2 bytes
       * 0x64 -> 100mm lens
       * Not getting a responce back yet
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xA0);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(messagePin, LOW);                  

        break;

      case 'C':
      /*
       * Request current position of "focal group"
       * 8/6/15  body 0x0C - reply is 16 bit signed int
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0C);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xC0);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(messagePin, LOW);                  

        break;

      case 'M':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x64);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  
        
        break;    

      case 'm':
      /*
       * Move focal group copied from 'buttersflybikers.cz'
       * 8/6/15
        0x000160 44 00 ;Move focus group
        0x000161 06 44 ;06 01 = 1537 steps
        0x000162 01 44 ;toward INF
        0x000163 0F 44
        0x000164 0A 0F
        0x000165 00 AA
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x44);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xFF);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x9B);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0A);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);               
  
        digitalWrite(messagePin, LOW);                  

        break; 

      case 'E':
      /*
       * Lens extension factor (?)
       * 0x00015D E4 00 ;Lens extension factor
         0x00015E 00 A1
         0x00015F 00 C6
       */
        digitalWrite(messagePin, HIGH);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xE4);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
  
        digitalWrite(messagePin, LOW);                  

        break; 

      case 'V': //INFO command
      // body - 0xCA 0x0A  reply - maybe 7 bytes, not sure 
      /*
       * 8/6/15 Body asks for Lens Version, replys with 7 bytes not sure what they mean
       * The Sync command needs to be sent first, maybe twice on older (? len's)
       */
        digitalWrite(messagePin, HIGH);
   
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0xCA);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);                
                        
        break;

      case '5': //INFO command
      // body - 0x05 0x0F reply - maybe 2 bytes, not sure 
      /*
       * 8/6/15 Body asks for to move lens to min
       */
        digitalWrite(messagePin, HIGH);
   
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x05);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

         digitalWrite(messagePin, LOW);
                                 
        break;        

      case '6': //INFO command
      // body - 0x06 0x0F reply - maybe 2 bytes, not sure 
      /*
       * 8/6/15 Body asks for to move lens to max
       */
        digitalWrite(messagePin, HIGH);
   
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x06);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);
        
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x0F);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);    
  
        digitalWrite(chipSelectPin, HIGH);
        inputByte = SPI.transfer(0x00);
        digitalWrite(chipSelectPin, LOW);
        delayMicroseconds(delayTime);

        digitalWrite(messagePin, LOW);
        break;


        
      default:
        Serial.write(inByte);
        inByte = 0;      
    }//end switch
  } // end if
  //delay(1000);  
}

