//-----------------------------------------------------------------------------
// EFM8UB2_UART_Lib_Buffer.c
//-----------------------------------------------------------------------------
// Copyright 2014 Silicon Laboratories, Inc.
// http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
//
// Program Description:
//
// This program demonstrates how to configure the EFM8UB2 to use routines
// in UART peripheral driver (buffer configuration) to write to and read from
// the UART0 interface. The program reads five lower case characters,
// capitalizes the letters, then outputs the capitalized letters.
//
// Resources:
//   SYSCLK - 48 MHz HFOSC / 4
//   UART0  - 115200 baud, 8-N-1
//   Timer1 - UART0 clock source
//   P0.0   - Board Controller enable
//   P0.4   - UART0 TX
//   P0.5   - UART0 RX
//   P1.4   - Display enable
//
//-----------------------------------------------------------------------------
// How To Test: EFM8UB2 STK
//-----------------------------------------------------------------------------
// 1) Place the switch in "AEM" mode.
// 2) Connect the EFM8UB2 STK board to a PC using a mini USB cable.
// 3) Compile and download code to the EFM8UB2 STK board.
//    In Simplicity Studio IDE, select Run -> Debug from the menu bar,
//    click the Debug button in the quick menu, or press F11.
// 4) On the PC, open HyperTerminal (or any other terminal program) and connect
//    to the JLink CDC UART Port at 115200 baud rate and 8-N-1.
// 5) Run the code.
//    In Simplicity Studio IDE, select Run -> Resume from the menu bar,
//    click the Resume button in the quick menu, or press F8.
// 6) Type five lower case letters. Note: terminal will not display anything
//    until five letters are entered.
// 7) Observe the capitalized input in the terminal.
//
// Target:         EFM8UB2
// Tool chain:     Generic
//
// Release 0.1 (ST)
//    - Initial Revision
//    - 10 OCT 2014
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "bsp.h"
#include "uart_0.h"
#include "uart_1.h"
#include "InitDevice.h"
#include "SPI_0.h"
#include <stdio.h>
#include "adc_0.h"
#include <stdint.h>
#include "si_toolchain.h"


//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define BUFFER_LENGTH   2
#define MAX_BUFFER_SIZE              8 // Maximum buffer Master will send

#define  SYSCLK         12000000
#define  SMB_FREQUENCY  30000          // Target SCL clock rate
#define  WRITE          0x00           // SMBus WRITE command
#define  READ           0x01           // SMBus READ command
#define  EEPROM_ADDR    0xA0           // Device address for slave target
#define  SMB_BUFF_SIZE  0x08           // Defines the maximum number of bytes
#define  SMB_MTSTA      0xE0           // (MT) start transmitted
#define  SMB_MTDB       0xC0           // (MT) data byte transmitted
#define  SMB_MRDB       0x80           // (MR) data byte received
#define VREF_MV         (3300UL)
#define ADC_MAX_RESULT  ((1 << 10)-1) // 10 bit ADC

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
SI_SEGMENT_VARIABLE(buffer[BUFFER_LENGTH], uint8_t, SI_SEG_XDATA);
SI_SEGMENT_VARIABLE(SPI_TxBuf[MAX_BUFFER_SIZE+1], uint8_t, EFM8PDL_SPI0_TX_SEGTYPE);
SI_SEGMENT_VARIABLE(SPI_RxBuf[MAX_BUFFER_SIZE+1], uint8_t, EFM8PDL_SPI0_RX_SEGTYPE);

//-----------------------------------------------------------------------------
// Main Routine
//-----------------------------------------------------------------------------

void SPI_Array_Write(uint8_t writeData[], uint8_t size);
void SPI_Array_Read(uint8_t add,uint8_t readData[], uint8_t size);
void Delay(void);

void EEPROM_ReadArray (uint8_t* dest_addr, uint8_t src_addr,uint8_t len);
uint8_t EEPROM_ByteRead(uint8_t addr);
void EEPROM_WriteArray(uint8_t dest_addr, uint8_t* src_addr, uint8_t len);
void EEPROM_ByteWrite(uint8_t addr, uint8_t dat);
void EEPROM_Check(void);

void TEST(void);

uint8_t* pSMB_DATA_IN;           // Global pointer for SMBus data
                                       // All receive data is written here

uint8_t SMB_SINGLEBYTE_OUT;      // Global holder for single byte writes.

uint8_t* pSMB_DATA_OUT;          // Global pointer for SMBus data.
                                       // All transmit data is read from here

uint8_t SMB_DATA_LEN;            // Global holder for number of bytes
                                       // to send or receive in the current
                                       // SMBus transfer.

uint8_t WORD_ADDR;               // Global holder for the EEPROM word
                                       // address that will be accessed in
                                       // the next transfer

uint8_t TARGET;                  // Target SMBus slave address

bit SMB_BUSY = 0;                      // Software flag to indicate when the
                                       // EEPROM_ByteRead() or
                                       // EEPROM_ByteWrite()
                                       // functions have claimed the SMBus

bit SMB_RW;                            // Software flag to indicate the
                                       // direction of the current transfer

bit SMB_SENDWORDADDR;                  // When set, this flag causes the ISR
                                       // to send the 8-bit <WORD_ADDR>
                                       // after sending the slave address.

bit SMB_RANDOMREAD;                    // When set, this flag causes the ISR
                                       // to send a START signal after sending
                                       // the word address.
                                       // For the 24LC02B EEPROM, a random read
                                       // (a read from a particular address in
                                       // memory) starts as a write then
                                       // changes to a read after the repeated
                                       // start is sent. The ISR handles this
                                       // switchover if the <SMB_RANDOMREAD>
                                       // bit is set.

bit SMB_ACKPOLL;                       // When set, this flag causes the ISR
                                       // to send a repeated START until the
                                       // slave has acknowledged its address

// 16-bit SI_SFR declarations

SI_SBIT(SDA, SFR_P1, 2);                  // SW1 ='0' means switch pressed
SI_SBIT(SCL, SFR_P1, 3);                  // SW2 ='0' means switch pressed
SI_SBIT(I2C_EN, SFR_P2, 5);

extern uint8_t ADC_complete;

uint8_t SPI_buffer[8]={0x66,0x66};
//extern uint32_t mV;
void main (void)
{
   PCA0MD &= ~0x40;                    // WDTE = 0 (disable watchdog timer)
   HFO0CN |= 0x03;   //clock=SYSCLK/1

   I2C_EN=0;
   EEPROM_Check();

   enter_DefaultMode_from_RESET();
   UART0_init(UART0_RX_ENABLE, UART0_WIDTH_8, UART0_MULTIPROC_DISABLE);
   IE_EA = 1;
   while (1)
   {
	   if(ADC_complete)
	   {
		   ADC_complete=0;
	   }

      if ((UART0_rxBytesRemaining() == 0) && (UART0_txBytesRemaining() == 0))
      {
         UART0_readBuffer(buffer, BUFFER_LENGTH);
         TEST();

      }
      if ((UART1_rxBytesRemaining() == 0) && (UART1_txBytesRemaining() == 0))
      {
         UART1_readBuffer(buffer, BUFFER_LENGTH);
      }
   }
}

//-----------------------------------------------------------------------------
// UART ISR Callbacks
//-----------------------------------------------------------------------------
void UART0_receiveCompleteCb ()
{
   UART0_writeBuffer(buffer, BUFFER_LENGTH);
	//TEST();
}

void uart0_transmitCompleteCB ()
{
}

void UART1_receiveCompleteCb()
{
	UART1_writeBuffer(buffer, BUFFER_LENGTH);
}

void UART1_transmitCompleteCb()
{
}

void SPI0_transferCompleteCb()
{

}

void Delay(void)
{
   uint32_t count;

   for (count = 200; count > 0; count--);
}

void SPI_Array_Write(uint8_t writeData[], uint8_t size)
{
  uint8_t i;
  uint8_t dummy = 0xFF;

  // Wait until the SPI is free, in case it's already busy
  while(!SPI0CN0_NSSMD0);

  // Setup command buffer
  //SPI_TxBuf[0] = SPI_WRITE_BUFFER;

  // Copy write data to TX buffer
  for (i = 0; i < size; i++)
  {
    SPI_TxBuf[i] = writeData[i];
  }

  // Transfer command
  SPI0_transfer(SPI_TxBuf, NULL, SPI0_TRANSFER_TX, size);

  // Wait until the SPI transfer is complete
  while(!SPI0CN0_NSSMD0);
}


void SPI_Array_Read(uint8_t add,uint8_t readData[], uint8_t size)
{
  uint8_t i;
  uint8_t dummy = 0xFF;

  if (size > MAX_BUFFER_SIZE)
  {
     return;
  }

  // Wait until the SPI is free, in case it's already busy
  while(!SPI0CN0_NSSMD0);

  // Setup command buffer
  SPI_TxBuf[0] = add;

  // Send dummy bytes to shift in RX data
  for (i = 0; i < size; i++)
  {
     SPI_TxBuf[1 + i] = dummy;
  }

  // Transfer command
  SPI0_transfer(SPI_TxBuf, SPI_RxBuf, SPI0_TRANSFER_RXTX, size + 1);

  // Wait until the SPI transfer is complete
  while(!SPI0CN0_NSSMD0);

  // Copy RX data into user buffer
  for (i = 0; i < size; i++)
  {
      readData[i] = SPI_RxBuf[1 + i];
  }
}

//-----------------------------------------------------------------------------
// EEPROM_ByteWrite ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t addr - address to write in the EEPROM
//                        range is full range of character: 0 to 255
//
//   2) uint8_t dat - data to write to the address <addr> in the EEPROM
//                        range is full range of character: 0 to 255
//
// This function writes the value in <dat> to location <addr> in the EEPROM
// then polls the EEPROM until the write is complete.
//
void EEPROM_ByteWrite(uint8_t addr, uint8_t dat)
{
   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // Mark next transfer as a write
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 0;                 // Do not send a START signal after
                                       // the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling (The ISR
                                       // will automatically restart the
                                       // transfer if the slave does not
                                       // acknoledge its address.

   // Specify the Outgoing Data
   WORD_ADDR = addr;                   // Set the target address in the
                                       // EEPROM's internal memory space

   SMB_SINGLEBYTE_OUT = dat;           // Store <dat> (local variable) in a
                                       // global variable so the ISR can read
                                       // it after this function exits

   // The outgoing data pointer points to the <dat> variable
   pSMB_DATA_OUT = &SMB_SINGLEBYTE_OUT;

   SMB_DATA_LEN = 1;                   // Specify to ISR that the next transfer
                                       // will contain one data byte

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;

}

//-----------------------------------------------------------------------------
// EEPROM_WriteArray ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t dest_addr - beginning address to write to in the EEPROM
//                        range is full range of character: 0 to 255
//
//   2) uint8_t* src_addr - pointer to the array of data to be written
//                        range is full range of character: 0 to 255
//
//   3) uint8_t len - length of the array to be written to the EEPROM
//                        range is full range of character: 0 to 255
//
// Writes <len> data bytes to the EEPROM slave specified by the <EEPROM_ADDR>
// constant.
//
void EEPROM_WriteArray(uint8_t dest_addr, uint8_t* src_addr,
                       uint8_t len)
{
   uint8_t i;
   uint8_t* pData = (uint8_t*) src_addr;

   for( i = 0; i < len; i++ ){
      EEPROM_ByteWrite(dest_addr++, *pData++);
   }

}

//-----------------------------------------------------------------------------
// EEPROM_ByteRead ()
//-----------------------------------------------------------------------------
//
// Return Value :
//   1) uint8_t data - data read from address <addr> in the EEPROM
//                        range is full range of character: 0 to 255
//
// Parameters   :
//   1) uint8_t addr - address to read data from the EEPROM
//                        range is full range of character: 0 to 255
//
// This function returns a single byte from location <addr> in the EEPROM then
// polls the <SMB_BUSY> flag until the read is complete.
//
uint8_t EEPROM_ByteRead(uint8_t addr)
{
   uint8_t retval;               // Holds the return value

   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // A random read starts as a write
                                       // then changes to a read after
                                       // the repeated start is sent. The
                                       // ISR handles this switchover if
                                       // the <SMB_RANDOMREAD> bit is set.
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 1;                 // Send a START after the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling

   // Specify the Incoming Data
   WORD_ADDR = addr;                   // Set the target address in the
                                       // EEPROM's internal memory space

   pSMB_DATA_IN = &retval;             // The incoming data pointer points to
                                       // the <retval> variable.

   SMB_DATA_LEN = 1;                   // Specify to ISR that the next transfer
                                       // will contain one data byte

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;
   while(SMB_BUSY);                    // Wait until data is read

   return retval;

}

//-----------------------------------------------------------------------------
// EEPROM_ReadArray ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t* dest_addr - pointer to the array that will be filled
//                                 with the data from the EEPROM
//                        range is full range of character: 0 to 255
//
//   2) uint8_t src_addr - beginning address to read data from the EEPROM
//                        range is full range of character: 0 to 255
//
//   3) uint8_t len - length of the array to be read from the EEPROM
//                        range is full range of character: 0 to 255
//
// Reads up to 256 data bytes from the EEPROM slave specified by the
// <EEPROM_ADDR> constant.
//
void EEPROM_ReadArray (uint8_t* dest_addr, uint8_t src_addr,
                       uint8_t len)
{
   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // A random read starts as a write
                                       // then changes to a read after
                                       // the repeated start is sent. The
                                       // ISR handles this switchover if
                                       // the <SMB_RANDOMREAD> bit is set.
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 1;                 // Send a START after the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling

   // Specify the Incoming Data
   WORD_ADDR = src_addr;               // Set the target address in the
                                       // EEPROM's internal memory space

   // Set the the incoming data pointer
   pSMB_DATA_IN = (uint8_t*) dest_addr;

   SMB_DATA_LEN = len;                 // Specify to ISR that the next transfer
                                       // will contain <len> data bytes

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;
   while(SMB_BUSY);                    // Wait until data is read

}

void EEPROM_Check(void)
{
	uint8_t i;
	while(!SDA)
	   {
	      // Provide clock pulses to allow the slave to advance out
	      // of its current state. This will allow it to release SDA.
	      XBR1 = 0x40;                     // Enable Crossbar
	      SCL = 0;                         // Drive the clock low
	      for(i = 0; i < 255; i++);        // Hold the clock low
	      SCL = 1;                         // Release the clock
	      while(!SCL);                     // Wait for open-drain
	                                       // clock output to rise
	      for(i = 0; i < 10; i++);         // Hold the clock high
	      XBR1 = 0x00;                     // Disable Crossbar
	   }
}

void TEST()
{
   switch(*buffer)
   {
   case 1:
	   SPI_Array_Write(SPI_buffer,2);
	   UART0_writeBuffer(buffer,2);
	   break;
   case 2:
	   EEPROM_ByteWrite(0x00,0x66);
	   EEPROM_ByteRead(0x00);
	   UART0_writeBuffer(buffer,2);
	   break;
   case 0x66:
	   *buffer=0x66;
	   *(buffer+1)=0x66;
	   UART0_writeBuffer(buffer,2);
	   break;
   case 0xaa:
	   *buffer=0xaa;
	   *(buffer+1)=0xaa;
	   UART1_writeBuffer(buffer,2);
	   break;
   default:
	   break;
   }
}
