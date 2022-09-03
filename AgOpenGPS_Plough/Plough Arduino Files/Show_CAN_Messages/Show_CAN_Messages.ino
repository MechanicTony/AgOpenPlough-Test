
#include <mcp_can.h>
#include <SPI.h>

//******** User Settings **********************************************

#define CAN0_INT 2        // Interrupt pin (Check CAN Board)    Standard = 2
MCP_CAN CAN0(10);         // Chip Select pin (Check CAN Board)  Standard = 10 or 9

#define ModuleSpeed MCP_16MHZ   // Big UNO shaped board
//#define ModuleSpeed MCP_8MHZ    // Small blue CAN board  

#define Model 0           // Model 0 = Com2&3 Fendt 100kbs K-Bus
                          // Model 1 = SCR/S4 Fendt 250kbs K-Bus

//*********************************************************************

unsigned long Time;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

void setup()
{
  Serial.begin(115200);
  if(Model == 0)  
  {
    if(CAN0.begin(MCP_STDEXT, CAN_100KBPS, ModuleSpeed) == CAN_OK)   
      Serial.println("MCP2515 Initialized Successfully!");
    else
      Serial.println("Error Initializing MCP2515..."); 
  }
  else if(Model == 1)  
  {
    if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, ModuleSpeed) == CAN_OK)
      Serial.println("MCP2515 Initialized Successfully!");
    else
      Serial.println("Error Initializing MCP2515...");
  }
    
  pinMode(CAN0_INT, INPUT);   
  
  Serial.println("MCP2515 Library Mask & Filter Example...");
  CAN0.setMode(MCP_NORMAL);
}

void loop()
{
    if(!digitalRead(CAN0_INT))
    {
      Time = millis();
      
      CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
      
      Serial.print(Time);
      Serial.print(", ");
      Serial.print("ID: ");
      Serial.print(rxId, HEX);
      Serial.print(" Data: ");
      for(int i = 0; i<len; i++)           // Print each byte of the data
      {
        Serial.print(rxBuf[i], HEX);
        Serial.print(", ");
      }
      
  Serial.println();
  
    }

}
