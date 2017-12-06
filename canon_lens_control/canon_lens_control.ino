#include <SPI.h>

// xxxset up the speed, data order and data mode
 SPISettings cameraSetting(10000, MSBFIRST, SPI_MODE2);

uint8_t outputByte, inputByte;
const int chipSelectPin = 10;
const int messagePin = 9;
const int delayTime = 200;
//int Flash_Reset=6;
//int Flash_Fire=4;

int inByte;

void setup() 
{

  // initialize SPI:
  SPI.begin(); 
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV128, MSBFIRST, SPI_MODE3));
  outputByte = 0x55;

  // initalize the chip select pin:
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, LOW);
  digitalWrite(messagePin, LOW);

  Serial.begin(19200);

  //Flash setup
  //pinMode(Flash_Reset, OUTPUT);
  //pinMode(Flash_Fire, OUTPUT);
  //digitalWrite(Flash_Reset,HIGH);
  //digitalWrite(Flash_Fire,LOW);
  //delay(25);

  //digitalWrite(Flash_Fire,LOW);  //turns on YELLOW LED reset relay
  //delay(1000);
  //digitalWrite(Flash_Reset,LOW);  //turns off RED LED
}

void loop() 
{
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
