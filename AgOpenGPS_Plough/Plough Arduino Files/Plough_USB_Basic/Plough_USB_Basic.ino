/*

Simple example USB Nano code for Plough control 

Inputs
- Width sensor connected to Nano A0 or ADC1115 A0 for more resolution
- Allow auto width toggle switch conneced to A1
- Allow auto lift toggle switch conneced to A2

Outputs
- Plough IN relay = Pin D5
- Plough OUT relay = Pin D6
- Plough UP relay = Pin D7
- Plough DOWN relay = Pin D8

*/

#include <EEPROM.h> 
#define EEP_Ident 0x5405
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115

//Relay Outputs
#define relayIn 5                                    
#define relayOut 6 
#define relayUp 7                                    
#define relayDown 8

//Active High or Low relays
#define relayON 1         // 1 = High is ON, 0 = Low is ON   

//Switch Inputs
#define autoLiftPin A2
#define autoWidthPin A1

//Plough width input
#define ploughWidthPin A0    
 
bool useNanoAnalog = true;
//bool useNanoAnalog = false;

#define deadBand 50 //Deadband in mm that the plough is near enough

//For debug printing to serial monitor
bool deBug = false;                         
//bool deBug = true;  

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

  unsigned long lastDownTime, lastUpTime;

  //Comm checks
  uint8_t watchdogTimer = 0; //make sure we are talking to AOG
  uint8_t serialResetTimer = 0; //if serial buffer is getting full, empty it

  bool isRaise = false;
  bool isLower = false;
  
  bool bitState = false;
  bool bitStateOld = false;
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

  uint16_t hydLoopCounter;
  uint8_t ploughMode = 0;
  
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
                              
  pinMode(relayIn, OUTPUT);                                           
  pinMode(relayOut, OUTPUT); 
  pinMode(relayUp, OUTPUT);                                           
  pinMode(relayDown, OUTPUT); 
   
  digitalWrite(relayIn, !relayON);
  digitalWrite(relayOut, !relayON); 
  digitalWrite(relayUp, !relayON);
  digitalWrite(relayDown, !relayON);
  
  pinMode(autoLiftPin, INPUT_PULLUP); 
  pinMode(autoWidthPin, INPUT_PULLUP);

  currentTime = millis();

  Serial.println("\r\nAgOpenGPS Plough USB Basic\r\n");

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
          
    }//End AOG timed loop

    autoLift = digitalRead(autoLiftPin);
    autoWidth = digitalRead(autoWidthPin);
     
    bitState = (bitRead(relayLo, 0));

//Lift / Lower the plough
    if (aogConfig.enableToolLift == 1 && autoLift == 0)
    {
      if (bitState  && !bitStateOld)  //Section ON, drop Plough
      {
        digitalWrite(relayUp,!relayON);
        digitalWrite(relayDown,relayON);
        lastDownTime = millis();
      }
      if (!bitState && bitStateOld)   //Section OFF, lift Plough
      {
        digitalWrite(relayDown,!relayON);
        digitalWrite(relayUp,relayON);         
        lastUpTime = millis();
      }
    }
    else
    {
        digitalWrite(relayDown,!relayON);
        digitalWrite(relayUp,!relayON);        
    }

    bitStateOld = bitState;

   //Turn up/down relays off 
    if (millis() - lastDownTime >= (aogConfig.lowerTime * 1000))
    {
      digitalWrite(relayDown,!relayON);
    }
    if (millis() - lastUpTime >= (aogConfig.raiseTime * 1000))
    {
      digitalWrite(relayUp,!relayON);
    }    
           
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
     if(aogConfig.minRaw < aogConfig.maxRaw){
      steerAngleActual = constrain(steerAngleActual,aogConfig.minRaw,aogConfig.maxRaw);
     }
     else{
      steerAngleActual = constrain(steerAngleActual,aogConfig.maxRaw,aogConfig.minRaw);
     }
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
     
     if(bitState ==1 && aogConfig.enableToolLift == 1 && autoWidth ==0)   //Section 1 ON, enableToolLift ON, autoLift toggle switch ON
     {
      if(ploughMode == 4){            //Plus
        digitalWrite(relayIn,!relayON);
        digitalWrite(relayOut,relayON);
       }
      else if(ploughMode == 6){       //Minus
        digitalWrite(relayOut,!relayON);
        digitalWrite(relayIn,relayON);
       }
      else{                           //Not + or - so don't move anything
        digitalWrite(relayOut,!relayON);
        digitalWrite(relayIn,!relayON);
       }
      }
      
      else
      {
        digitalWrite(relayOut,!relayON);
        digitalWrite(relayIn,!relayON);
      }      
     }//End in/out relay control loop     

  }//End Timed loop

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
