/*
 * Demo ET-MEGA32U4-RS485 Hardware Board
 * MCU    : ATMEGA32U4
 *        : Arduino Leonado
 *        : Bootloader
 *        : -> .../caterina/Caterina-Leonardo.hex
 *        : Fuse Bit
 *        : -> low_fuses      = 0xFF
 *        : -> high_fuses     = 0xD8
 *        : -> extended_fuses = 0xCB(0xFB)
 *        : Lock Bit
 *        : -> 0x2F(0xEF)
 * RS485  : RS485 RXD:D0
 *        : RS485 TXD:D1
 *        : RS485 Direction(D4:LOW=Receive,HIGH=Send)
 * I2C    : SCL=D3,SDA=D2
 *        : BME280
 *        : RTC:DS3231
 *        : PCF8574(External Output Relay)
 *        : PCF8574A(External Input Opto)
 *        : I2C OLED 1.3
 * ADC    : Analog#0 : A;[0..3]
 *        : Analog#1 : A4
 *        : Analog#2 : A5
 * Output : Link(LED Active High:D13)
 *        : Relay0(Active Low:D6)
 *        : Relay1(Active Low:D7)
 * Input  : RS485 ID0:D8
 *        : RS485 ID1:D9
 *        : RS485 ID2:D10
 *        : RS485 ID3:D11
 *        : RS485 ID4:D12
 * 1-Wire : DS18B20(1-Wire:D5)
 * 
 * Demo   : RS485 Modbus RTU & Modbus RTU Relay2
 *        : Setup Slave Device ID = 1
 *        : Dip Switch[1:ON,2..5:OFF,6=ON]
 */
 
//=================================================================================================
#include "ETT_ModbusRTU.h"
//=================================================================================================
 
//=================================================================================================
// USART RS232/RS485
//=================================================================================================
#define SerialDebug Serial                                                                        // USB Serial
#define SerialRS485 Serial1                                                                       // Serial1(D1=TXD,D0=RXD)
//=================================================================================================
const int RS485_DIRECTION_PIN =     4;                                                            // RS485 TXD Enable,Disable
const int RS485_RXD_SELECT    =     LOW;
const int RS485_TXD_SELECT    =     HIGH;
//=================================================================================================
const int RS485_ID0_PIN       =     8;                                                            // Slave ID LSB
const int RS485_ID1_PIN       =     9;         
const int RS485_ID2_PIN       =     10;        
const int RS485_ID3_PIN       =     11;        
const int RS485_ID4_PIN       =     12;                                                           // Slave ID MSB
//=================================================================================================
const int LED_LINK_PIN        =     13;
const int LED_OFF             =     LOW;
const int LED_ON              =     HIGH;
//=================================================================================================
const int INTERNAL_RELAY0_PIN =     6;
//=================================================================================================
const int INTERNAL_RELAY1_PIN =     7;
//=================================================================================================
const int InternalRelayOff    =     HIGH;
const int InternalRelayOn     =     LOW;
//=================================================================================================

//=================================================================================================
// data array for modbus network sharing
//=================================================================================================
uint16_t au16dataRelay[2];      //!< data array for modbus network sharing
uint8_t u8state;                //!< machine state
uint8_t u8query;                //!< pointer to message query
//=================================================================================================

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(0,                        // node id = 0(master)
              SerialRS485,              // Serial(2)
              RS485_DIRECTION_PIN);     // RS485 Modbus

/**
 * This is an structe which contains a query to an slave device
 */
modbus_t telegram[2];                   // 2-Modbus Commans
unsigned long u32wait;
//=================================================================================================

void setup() 
{
  //===============================================================================================
  pinMode(INTERNAL_RELAY0_PIN, OUTPUT);
  digitalWrite(INTERNAL_RELAY0_PIN, InternalRelayOff);
  pinMode(INTERNAL_RELAY1_PIN, OUTPUT);
  digitalWrite(INTERNAL_RELAY1_PIN, InternalRelayOff);
  //===============================================================================================
  //===============================================================================================
  //SerialDebug.begin(115200);
  //SerialDebug.println();
  //while(!SerialDebug);                                                                            // Wait MEGA32U4 USB Serial Complete
  //===============================================================================================
  
  //===============================================================================================
  // Initial RS485
  //===============================================================================================
  SerialRS485.begin(9600);
  //===============================================================================================
  pinMode(RS485_DIRECTION_PIN, OUTPUT);
  digitalWrite(RS485_DIRECTION_PIN, RS485_RXD_SELECT);
  //===============================================================================================
  
  //===============================================================================================
  // telegram[0]: Toggle Relay1 device[1]
  //            : Modbus Relay RS485(Modbus RTU)
  //            : Device ID[1]
  // Device Addr,Function Code(06),MSB Reg Addr,LSB Reg Addr,MSB Value,LSB Value,CRC16
  //===============================================================================================
  au16dataRelay[0] = 0x0300;                                                                      // Toggle Output Command
  //===============================================================================================
  telegram[0].u8id = 1;                                                                           // slave address = 1
  telegram[0].u8fct = 6;                                                                          // function code (toggle relay1)
  telegram[0].u16RegAdd = 0x0001;                                                                 // start address in slave
  telegram[0].u16CoilsNo = 1;                                                                     // number of elements (coils or registers) to read
  telegram[0].au16reg = au16dataRelay;                                                            // pointer to a memory array in the Arduino
  //===============================================================================================
  // telegram[1]: Toggle Relay2 device[1]
  //            : Modbus Relay RS485(Modbus RTU)
  //            : Device ID[1]
  // Device Addr,Function Code(06),MSB Reg Addr,LSB Reg Addr,MSB Value,LSB Value,CRC16
  //===============================================================================================
  telegram[1].u8id = 1;                                                                          // slave address = 1
  telegram[1].u8fct = 6;                                                                         // function code (this one is registers read)
  telegram[1].u16RegAdd = 0x0002;                                                                // start address in slave
  telegram[1].u16CoilsNo = 1;                                                                    // number of elements (coils or registers) to read
  telegram[1].au16reg = au16dataRelay;                                                           // pointer to a memory array in the Arduino
  //===============================================================================================
  master.begin(SerialRS485);                                                                     // Mosbus Interface
  master.setTimeOut(2000);                                                                       // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 2000;
  u8state = u8query = 0; 
  //===============================================================================================
}

void loop() 
{
  switch( u8state ) 
  {
    case 0: 
      if (millis() > u32wait) u8state++;        // wait state
    break;
    
    case 1: 
      master.query(telegram[u8query]);          // send query (only once)
      u8state++;
      u8query++;
      if(u8query > 2) u8query = 0;              // 0,1,...,0
    break;

    
    case 2:
      master.poll();                            // check incoming messages
      if(master.getState() == COM_IDLE) 
      {
        u8state = 0;
        u32wait = millis() + 2000; 
      } 
    break;
  }
}

