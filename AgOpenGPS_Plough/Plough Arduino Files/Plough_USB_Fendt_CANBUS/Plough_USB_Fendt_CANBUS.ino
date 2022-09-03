/*

Example USB Nano code for Plough control 
Use with Nano/Uno & MCP2515 CANBUS Shield connected to Fendt K-BUS

Inputs
- Width sensor connected to Nano A0 or ADC1115 A0 for more resolution
- Allow auto width toggle switch conneced to A1
- Allow auto lift toggle switch conneced to A2

Outputs = Fendt K-BUS
- Plough IN/OUT = drive handle left hydralic rocker (Green) 
- Plough UP/DOWM = big GO/END

*/
#include <mcp_can.h>                                         
#include <SPI.h>
#include <EEPROM.h> 
#define EEP_Ident 0x5405
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115
  
//******** User Settings **********************************************

#define CAN0_INT 2        // Interrupt pin (Check CAN Board)    Standard = 2
MCP_CAN CAN0(10);         // Chip Select pin (Check CAN Board)  Standard = 10 or 9

#define ModuleSpeed MCP_16MHZ   // Big UNO shaped board
//#define ModuleSpeed MCP_8MHZ    // Small blue CAN board                                     

#define Model 0           // Model 0 = Com2&3 Fendt 100kbs K-Bus
                          // Model 1 = SCR/S4 Fendt 250kbs K-Bus  

//Switch Inputs
#define autoLiftPin A2
#define autoWidthPin A1

//Plough width input
#define ploughWidthPin A0  

bool useNanoAnalog = true;
//bool useNanoAnalog = false;

//bool invertWidth = true;
bool invertWidth = false;

#define deadBand 50 //Deadband in mm that the plough is near enough                 

bool deBug = false;                         
//bool deBug = true;        //Prints the CAN messages recived after the filters (Only button data should be recived)

//*********************************************************************

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

bool workSwitchCAN;
bool workRelayControl;
unsigned long workTriggerTime;

boolean goDown = false, endDown = false , bitState = false, bitStateOld = false;  //CAN Hitch Control

uint16_t modelID;
byte goPress[8]   = {0x15, 0x00, 0x00, 0xCA, 0x80, 0x01, 0x00, 0x00} ;    //  press big go
byte goLift[8]    = {0x15, 0x00, 0x00, 0xCA, 0x00, 0x02, 0x00, 0x00} ;    //  lift big go
byte endPress[8]  = {0x15, 0x00, 0x00, 0xCA, 0x80, 0x03, 0x00, 0x00} ;    //  press big end
byte endLift[8]   = {0x15, 0x00, 0x00, 0xCA, 0x00, 0x04, 0x00, 0x00} ;    //  lift big end

uint16_t hydLoopCounter;
uint8_t ploughMode = 0;
uint8_t lastPloughMode = 0;
byte plusPress[8]  = {0x15, 0x00, 0x00, 0xCA, 0x80, 0x01, 0x00, 0x00} ;
byte plusLift[8]   = {0x15, 0x00, 0x00, 0xCA, 0x00, 0x02, 0x00, 0x00} ;
byte minusPress[8] = {0x15, 0x00, 0x00, 0xCA, 0x80, 0x03, 0x00, 0x00} ;
byte minusLift[8]  = {0x15, 0x00, 0x00, 0xCA, 0x00, 0x04, 0x00, 0x00} ;


//Variables for config - 0 is false  
  struct Config {
      uint8_t raiseTime = 2;
      uint8_t lowerTime = 4;
      uint8_t enableToolLift = 0;
      uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

      uint8_t user1 = 0; //user defined values set in machine tab
      uint8_t user2 = 0;
      uint8_t user3 = 0;
      uint8_t user4 = 0;

      int16_t minRaw = 0; 
      int16_t minRealMM = 0; 
      int16_t maxRaw = 0; 
      int16_t maxRealMM = 0;      

  };  Config aogConfig;   //16 bytes
  
  const unsigned int LOOP_TIME = 40;      
  unsigned long lastCurrentTime = LOOP_TIME;
  unsigned long currentTime = LOOP_TIME;
  uint16_t count = 0;

  //Comm checks
  uint8_t watchdogTimer = 0; //make sure we are talking to AOG
  uint8_t serialResetTimer = 0; //if serial buffer is getting full, empty it

  bool isRaise = false;
  bool isLower = false;
  bool autoLift, autoWidth;  
  
   //Communication with AgOpenGPS
  int16_t temp, EEread = 0;

   //Parsing PGN
  bool isPGNFound = false, isHeaderFound = false;
  uint8_t pgn = 0, dataLength = 0, idx = 0;
  int16_t tempHeader = 0;
  
  uint8_t AOG[] = {0x80,0x81, 0x7f, 0xED, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };

  //The variables used for storage
  uint8_t relayHi=0, relayLo = 0, tramline = 0, uTurn = 0, hydLift = 0, geoStop = 0;
  float gpsSpeed;

  int16_t lineDistance = 0;   //MM to the line
  int16_t workingWidth = 0;   //MM working width
  int16_t targetWidth = 0;    //MM target width (Working width - line distance)  
  int16_t ploughWidth = 0;    //MM plough current working width
  int16_t ploughError = 0;    //MM error 
  
  uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;

  //steering variables
  int16_t steeringPosition = 0; //from steering sensor
  int16_t steerAngleActual = 0;
  
//reset function
void(* resetFunc) (void) = 0;

void setup()
{
  Serial.begin(38400);
  Wire.begin();

      EEPROM.get(0, EEread);     // read identifier

      if (EEread != EEP_Ident)   // check on first start and write EEPROM
      {
          EEPROM.put(0, EEP_Ident);
          EEPROM.put(6, aogConfig);
      }
      else
      {
          EEPROM.get(6, aogConfig);
      }

    if(!useNanoAnalog)
    {
    adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
    adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);
    }
      
// Fendt Com2&3 is 100kbs K-BUS & CAN ID:61F        
  if(Model == 0)  
  {
    if(CAN0.begin(MCP_STDEXT, CAN_100KBPS, ModuleSpeed) == CAN_OK)   
      Serial.println("MCP2515 Initialized Successfully!");
    else
      Serial.println("Error Initializing MCP2515...");  

  CAN0.init_Mask(0,0,0x07FFFF00);       // Init first mask...     - Makes filter look at the ID & first data byte    
  CAN0.init_Filt(0,0,0x061F1500);       // Init first filter...   - Make sure the CAN ID = 61F & first data byte = 0x15  
  CAN0.init_Filt(1,0,0x061F1500);       // Init second filter...  - As above
  
  CAN0.init_Mask(1,0,0x07FFFF00);       // Init second mask...    - As above
  CAN0.init_Filt(2,0,0x061F1500);       // Init third filter...   - As above
  CAN0.init_Filt(3,0,0x061F1500);       // Init fouth filter...   - As above
  CAN0.init_Filt(4,0,0x061F1500);       // Init fifth filter...   - As above
  CAN0.init_Filt(5,0,0x061F1500);       // Init sixth filter...   - As above  

  modelID = 0x61F;
  goPress[1]  = 0x33;       //Com2&3 Big GO
  goLift[1]   = 0x33;       //Com2&3 Big GO
  endPress[1] = 0x34;       //Com2&3 Big END
  endLift[1]  = 0x34;       //Com2&3 Big END

  goPress[2]  = 0x1E;       //Com2&3 Commands
  goLift[2]   = 0x1E; 
  endPress[2] = 0x1E; 
  endLift[2]  = 0x1E;  

   minusPress[1]  = 0x4A;   //Com2&3 Drive Handle Green 0x49 is Red
   minusLift[1]   = 0x4A;   //Com2&3 Drive Handle Green
   plusPress[1]   = 0x4A;   //Com2&3 Drive Handle Green
   plusLift[1]    = 0x4A;   //Com2&3 Drive Handle Green
   
   minusPress[2]  = 0x1E;   //Com2&3 Commands
   minusLift[2]   = 0x1E; 
   plusPress[2]   = 0x1E; 
   plusLift[2]    = 0x1E; 
  
   minusPress[4]  = 0x00; 
   minusLift[4]   = 0x00; 
   plusPress[4]   = 0x00; 
   plusLift[4]    = 0x00;  

   minusPress[5]  = 0x8B;   //Com2&3 Drive Handle Green - Press
   minusLift[5]   = 0x57;   //Com2&3 Drive Handle Green Release
   plusPress[5]   = 0x0F;   //Com2&3 Drive Handle Green + Press
   plusLift[5]    = 0x57;   //Com2&3 Drive Handle Green Release 

   minusPress[6]  = 0x39;   //Com2&3 Drive Handle Green - Press
   minusLift[6]   = 0x15;   //Com2&3 Drive Handle Green Release
   plusPress[6]   = 0x1D;   //Com2&3 Drive Handle Green + Press
   plusLift[6]    = 0x15;   //Com2&3 Drive Handle Green Release   
           
  }

// Fendt SCR/S4 is 250kbs K-BUS & CAN ID:613    
  else if(Model == 1)  
  {
    if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, ModuleSpeed) == CAN_OK)
      Serial.println("MCP2515 Initialized Successfully!");
    else
      Serial.println("Error Initializing MCP2515...");

  CAN0.init_Mask(0,0,0x07FFFF00);       // Init first mask...     - Makes filter look at the ID & first data byte    
  CAN0.init_Filt(0,0,0x06131500);       // Init first filter...   - Make sure the CAN ID = 613 & first data byte = 0x15  
  CAN0.init_Filt(1,0,0x06131500);       // Init second filter...  - As above
  
  CAN0.init_Mask(1,0,0x07FFFF00);       // Init second mask...    - As above
  CAN0.init_Filt(2,0,0x06131500);       // Init third filter...   - As above
  CAN0.init_Filt(3,0,0x06131500);       // Init fouth filter...   - As above
  CAN0.init_Filt(4,0,0x06131500);       // Init fifth filter...   - As above
  CAN0.init_Filt(5,0,0x06131500);       // Init sixth filter...   - As above  

  modelID = 0x613;
  goPress[1]  = 0x20; //SCR/S4 Big GO
  goLift[1]   = 0x20; //SCR/S4 Big GO
  endPress[1] = 0x21; //SCR/S4 Big END
  endLift[1]  = 0x21; //SCR/S4 Big END

  goPress[2]  = 0x06; //SCR/S4 Commands
  goLift[2]   = 0x06; 
  endPress[2] = 0x06; 
  endLift[2]  = 0x06;  

  minusPress[1] = 0x26; //SCR/S4 Drive Handle Green -   0x29 is Red
  minusLift[1]  = 0x26; //SCR/S4 Drive Handle Green - 
  plusPress[1]  = 0x25; //SCR/S4 Drive Handle Green +   0x28 is Red
  plusLift[1]   = 0x25; //SCR/S4 Drive Handle Green + 
  
  minusPress[2]  = 0x06;   //SCR/S4 Commands
  minusLift[2]   = 0x06; 
  plusPress[2]   = 0x06; 
  plusLift[2]    = 0x06;  
            
  }
   
  CAN0.setMode(MCP_NORMAL);                                   
  pinMode(CAN0_INT, INPUT);   
                                    
  if(Model == 0) {
    Serial.println("\r\nFendt Com2&3 100kbs K-Bus Plough Module.\r\n");
  }
  else if(Model == 1) {
    Serial.println("\r\nFendt SCR/S4 250kbs K-Bus Plough Module.\r\n");
  }
  else {
    Serial.println("\r\nNo Model, set in program and reload.\r\n");
  }
  
  pinMode(autoLiftPin, INPUT_PULLUP); 
  pinMode(autoWidthPin, INPUT_PULLUP);

  currentTime = millis();

  workingWidth = aogConfig.user1 + 200;
  workingWidth = workingWidth * 10;

}

void loop()
{
  
  currentTime = millis();
  
//Timed loop triggers every 40 msec ( Looptime setting )
  if (currentTime - lastCurrentTime >= LOOP_TIME)
  {
   lastCurrentTime = currentTime;
    
    //AOG timed loop
    if(count++ > 4) //5hz
    { 
          count = 0;
          //If connection lost to AgOpenGPS, the watchdog will count up 
          if (watchdogTimer++ > 250) watchdogTimer = 12;

          //clean out serial buffer to prevent buffer overflow
          if (serialResetTimer++ > 20)
          {
              while (Serial.available() > 0) Serial.read();
              serialResetTimer = 0;
          }

          if (watchdogTimer > 12)
          {
                relayLo = 0;
                relayHi = 0;               
          }

          int16_t cmPlough = (int16_t)(ploughWidth * 0.1);
          
          AOG[5] = cmPlough;
          AOG[6] = cmPlough >> 8;
          AOG[7] = (uint8_t)ploughMode;
          
          //add the checksum
          int16_t CK_A = 0;
          for (uint8_t i = 2; i < sizeof(AOG) - 1; i++)
          {
              CK_A = (CK_A + AOG[i]);
          }
          AOG[sizeof(AOG) - 1] = CK_A;

          if(!deBug){
            Serial.write(AOG, sizeof(AOG));
            Serial.flush();   // flush out buffer
          }

          else{
            Serial.print(workingWidth);
            Serial.print(", ");
            Serial.print(aogConfig.minRaw);
            Serial.print(", ");
            Serial.print(aogConfig.minRealMM);
            Serial.print(", ");
            Serial.print(aogConfig.maxRaw);
            Serial.print(", ");
            Serial.println(aogConfig.maxRealMM);                                  
          }         
          
    }//AOG timed loop

   //Hitch Control
    if (goDown)   liftGo();   //Lift Go button if pressed
    if (endDown)   liftEnd(); //Lift End button if pressed

    autoLift = digitalRead(autoLiftPin);
    autoWidth = digitalRead(autoWidthPin);
     
    bitState = (bitRead(relayLo, 0));

    //Only if tool lift is enabled AgOpen will press GO/END via CAN
    if (aogConfig.enableToolLift == 1 && autoLift == 0)
    {
      if (bitState  && !bitStateOld) pressGo(); //Press Go button
      if (!bitState && bitStateOld) pressEnd(); //Press End button
    }

    bitStateOld = bitState;

//******************************************************************************************

    if(!useNanoAnalog)
    {
     adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);        
     steeringPosition = adc.getConversion();    
     adc.triggerConversion();//ADS1115 Single Mode 
        
     steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
    }
    else
    {
    //Get the plogh width (Just standard Nano analog pin)
     steeringPosition = analogRead(ploughWidthPin);    
    }
     steerAngleActual = steeringPosition;
     steerAngleActual = constrain(steerAngleActual,aogConfig.minRaw,aogConfig.maxRaw);
     steerAngleActual = map(steerAngleActual,aogConfig.minRaw,aogConfig.maxRaw,aogConfig.minRealMM,aogConfig.maxRealMM); 
     
     ploughWidth = steerAngleActual;
     
     if(lineDistance > 30000) 
     {
      ploughError = 0;
      targetWidth = workingWidth;
     }
     else
     {
      targetWidth = workingWidth + lineDistance;
      ploughError = ploughWidth - targetWidth;
     }

//** We have the error of where we need to be, now we decide what to do **
     if (ploughError < -deadBand){
      ploughMode = 4; //Plus
     }
     else if (ploughError > deadBand){
      ploughMode = 6; //Minus
     }
     else{
      ploughMode = 3; //Hold, we are in the deadband
     }

//** Are we at the min or max limit? **
     if (ploughWidth >= aogConfig.maxRealMM && ploughMode == 4){
      ploughMode = 5; //We are at the max limit
     } 

     if (ploughWidth <= aogConfig.minRealMM && ploughMode == 6){
      ploughMode = 7; //We are at the min limit
     }          

//** Are switchs on to enable auto control? **
     if (aogConfig.enableToolLift != 1){
      ploughMode = 1; //Enable auto tool control is not ON in AgOpen
     } 
     
     if (autoWidth != 0){
      ploughMode = 2; //Auto width toggle switch is not ON connected to the Nano
     } 

     if (bitState != 1){
      ploughMode = 0; //Section not ON
     }   

     if (lineDistance > 30000){
      ploughMode = 8; //No field, No green steering wheel, no AB line
     }        

/*
      if(ploughMode == 0) font.DrawText(center + 10, 210, "Section Off", 1);
      else if (ploughMode == 1) font.DrawText(center + 10, 210, "AOG Auto Off", 1);
      else if (ploughMode == 2) font.DrawText(center + 10, 210, "Auto Switch Off", 1);
      else if (ploughMode == 3) font.DrawText(center + 10, 210, "Hold", 1);
      else if (ploughMode == 4) font.DrawText(center + 10, 210, "Plus", 1);
      else if (ploughMode == 5) font.DrawText(center + 10, 210, "Max", 1);
      else if (ploughMode == 6) font.DrawText(center + 10, 210, "Minus", 1);
      else if (ploughMode == 7) font.DrawText(center + 10, 210, "Min", 1);
      else if (ploughMode == 8) font.DrawText(center + 10, 210, "No Field", 1);
*/

    //Hyd spool timed loop
    if(hydLoopCounter++ > 12) //500ms
    { 
     hydLoopCounter = 0;
     
     if(bitState ==1 && aogConfig.enableToolLift == 1 && autoWidth ==0)
     {
      if(ploughMode == 4){        //Plus
        CAN0.sendMsgBuf(modelID, 0, 8, plusPress);
        lastPloughMode = 4;
       }
      else if(ploughMode == 6){   //Minus
        CAN0.sendMsgBuf(modelID, 0, 8, minusPress);
        lastPloughMode = 6;
       }
      else{                       //Not + or - so don't move anything
        if(lastPloughMode == 4) CAN0.sendMsgBuf(modelID, 0, 8, plusLift);
        else if(lastPloughMode == 6) CAN0.sendMsgBuf(modelID, 0, 8, minusLift);
        lastPloughMode = 0;
       }
      }
      
      else if(lastPloughMode != 0)
      {
        if(lastPloughMode == 4) CAN0.sendMsgBuf(modelID, 0, 8, plusLift);
        if(lastPloughMode == 6) CAN0.sendMsgBuf(modelID, 0, 8, minusLift);
        lastPloughMode = 0;
      }
      
     }     

  }//End Timed loop
  
//Read CAN Module  
  if(!digitalRead(CAN0_INT))                                    
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  
/*                              
    if(deBug)
    {
      Serial.print("ID: ");
      Serial.print(rxId, HEX);
      Serial.print(" Data: ");
      for(int i = 0; i<len; i++)      // Print each byte of the data
      {
        Serial.print(rxBuf[i], HEX);
        Serial.print(", ");
      }     
    }
*/    
//SCR/S4 Armrest Buttons     
    if(rxId == 0x613 && rxBuf[0] == 0x15 && Model == 1)                          
    {
    //Steer Switch Relay 
      if (rxBuf[1] == 0x22 && rxBuf[4] == 0x80)       //Small Go
      {
//        digitalWrite(SteerSW_PIN, relayON);
        if(deBug) Serial.print("\t\tSmall GO Pressed");                                                 
      } 
         
      if (rxBuf[1] == 0x23 && rxBuf[4] == 0x80)       //Small END
      {
//        digitalWrite(SteerSW_PIN, !relayON);
        if(deBug) Serial.print("\t\tSmall END Pressed");                                                 
      }

    //Work Switch Relay 
      if (rxBuf[1] == 0x20 && rxBuf[4] == 0x80)       //Big Go
      {
        workSwitchCAN = 1;
        workTriggerTime = millis();
        if(deBug) Serial.print("\t\tBig GO Pressed");                                                   
      } 
         
      if (rxBuf[1] == 0x21 && rxBuf[4] == 0x80)       //Big END
      {
        workSwitchCAN = 0;
        workTriggerTime = millis();
        if(deBug) Serial.print("\t\tBig END Pressed");                                                  
      }
      
    }//End if SCR/S4
    
//Com3 Armrest Buttons       
    if(rxId == 0x61F && rxBuf[0] == 0x15 && Model == 0)                        
    {
    //Steer Switch Relay 
      if (rxBuf[1] == 0x35 && rxBuf[4] == 0x80)       //Small Go
      {
//        digitalWrite(SteerSW_PIN, relayON);
        if(deBug) Serial.print("\t\tSmall GO Pressed");                                                 
      } 
         
      if (rxBuf[1] == 0x36 && rxBuf[4] == 0x80)       //Small END
      {
//        digitalWrite(SteerSW_PIN, !relayON);
        if(deBug) Serial.print("\t\tSmall END Pressed");                                                 
      }

    //Work Switch Relay 
      if (rxBuf[1] == 0x33 && rxBuf[4] == 0x80)       //Big Go
      {
        workSwitchCAN = 1;
        workTriggerTime = millis();
        if(deBug) Serial.print("\t\tBig GO Pressed");                                                 
      } 
         
      if (rxBuf[1] == 0x34 && rxBuf[4] == 0x80)       //Big END
      {
        workSwitchCAN = 0;
        workTriggerTime = millis();
        if(deBug) Serial.print("\t\tBig END Pressed");                                                 
      }
      
    }//End if Com3
        
//   if(deBug) Serial.println("");
         
  }//End Read CAN Module

// Serial Receive
      //Do we have a match with 0x8081?    
      if (Serial.available() > 4 && !isHeaderFound && !isPGNFound)
      {
          uint8_t temp = Serial.read();
          if (tempHeader == 0x80 && temp == 0x81)
          {
              isHeaderFound = true;
              tempHeader = 0;
          }
          else
          {
              tempHeader = temp;     //save for next time
              return;
          }
      }

      //Find Source, PGN, and Length
      if (Serial.available() > 2 && isHeaderFound && !isPGNFound)
      {
          Serial.read(); //The 7F or less
          pgn = Serial.read();
          dataLength = Serial.read();
          isPGNFound = true;
          idx = 0;
      }

      //The data package
      if (Serial.available() > dataLength && isHeaderFound && isPGNFound)
      {
          if (pgn == 239) // EF Machine Data
          {
              uTurn = Serial.read();
              gpsSpeed = (float)Serial.read();//actual speed times 4, single uint8_t

              hydLift = Serial.read();
              tramline = Serial.read();  //bit 0 is right bit 1 is left

              //just get the rest of bytes
              lineDistance = ((uint16_t)(Serial.read() | Serial.read() << 8));

              relayLo = Serial.read();          // read relay control from AgOpenGPS
              relayHi = Serial.read();

              if (aogConfig.isRelayActiveHigh) lineDistance *=-1;

              //Bit 13 CRC
              Serial.read();

              //reset watchdog
              watchdogTimer = 0;

              //Reset serial Watchdog  
              serialResetTimer = 0;

              //reset for next pgn sentence
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;

          }

          else if (pgn == 238) //EE Machine Settings 
          {
              aogConfig.raiseTime = Serial.read();
              aogConfig.lowerTime = Serial.read();
              Serial.read();

              //set1 
              uint8_t sett = Serial.read();  //setting0     
              if (bitRead(sett,0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;
              if (bitRead(sett,1)) aogConfig.enableToolLift = 1; else aogConfig.enableToolLift = 0;

              aogConfig.user1 = Serial.read();
              aogConfig.user2 = Serial.read();
              aogConfig.user3 = Serial.read();
              aogConfig.user4 = Serial.read();
      
              if(aogConfig.user2 == 111)
              {
               aogConfig.minRaw = steeringPosition;
               aogConfig.minRealMM = ((int16_t)(aogConfig.user3 | aogConfig.user4 << 8));
               aogConfig.minRealMM = aogConfig.minRealMM * 10;
              }
              
              if(aogConfig.user2 == 222)
              {
               aogConfig.maxRaw = steeringPosition;
               aogConfig.maxRealMM = ((int16_t)(aogConfig.user3 | aogConfig.user4 << 8));
               aogConfig.maxRealMM = aogConfig.maxRealMM * 10;                
              }
              
              //crc
              //udpData[13];        //crc
              Serial.read();

              //save in EEPROM and restart
              EEPROM.put(6, aogConfig);
              resetFunc();

              //reset for next pgn sentence
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;
          }

          else //nothing found, clean up
          {
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;
          }
      }//End Serial Data Receive
    
}//End Main Loop 

//Fendt K-Bus Buttons
  void pressGo()
  {                                     
    CAN0.sendMsgBuf(modelID, 0, 8, goPress);
    goDown = true;
  }

  void liftGo()
  {
    CAN0.sendMsgBuf(modelID, 0, 8, goLift);
    goDown = false;
  }

  void pressEnd() 
  {
    CAN0.sendMsgBuf(modelID, 0, 8, endPress);
    endDown = true;
  }

  void liftEnd()
  {
    CAN0.sendMsgBuf(modelID, 0, 8, endLift);
    endDown = false;
}
