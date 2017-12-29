
// Resources:
//   SYSCLK - 48 MHz HFOSC / 4
//   UART0  - 115200 baud, 8-N-1
//   Timer1 - UART0 clock source
//   P0.0   - Board Controller enable
//   P0.4   - UART0 TX
//   P0.5   - UART0 RX

#include "bsp.h"
#include "uart_0.h"
#include "InitDevice.h"
#include "retargetserial.h"
#include "uart_1.h"
#include "spi_0.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define BUFFER_LENGTH   5
#define SYSCLK                24500000 // Internal oscillator frequency in Hz

#define SPI_CLOCK               250000 // Maximum SPI clock
                                       // The SPI clock is a maximum of 1 MHz
                                       // when this example is used with
                                       // the SPI0_Slave code example.

#define MAX_BUFFER_SIZE              8 // Maximum buffer Master will send

// Instruction Set
#define  SLAVE_LED_ON             0x01 // Turn the Slave LED on
#define  SLAVE_LED_OFF            0x02 // Turn the Slave LED off
#define  SPI_WRITE                0x04 // Send a byte from the Master to the
                                       // Slave
#define  SPI_READ                 0x08 // Send a byte from the Slave to the
                                       // Master
#define  SPI_WRITE_BUFFER         0x10 // Send a series of bytes from the
                                       // Master to the Slave
#define  SPI_READ_BUFFER          0x20 // Send a series of bytes from the Slave
                                       // to the Master
#define  ERROR_OCCURRED           0x40 // Indicator for the Slave to tell the
                                       // Master an error occurred
#define  SMB_FREQUENCY  30000          // Target SCL clock rate
                                       // This example supports between 10kHz
                                       // and 100kHz

#define  WRITE          0x00           // SMBus WRITE command
#define  READ           0x01           // SMBus READ command

// Device addresses (7 bits, lsb is a don't care)
#define  EEPROM_ADDR    0xA0           // Device address for slave target
                                       // Note: This address is specified
                                       // in the Microchip 24LC02B
                                       // datasheet.
// SMBus Buffer Size
#define  SMB_BUFF_SIZE  0x08           // Defines the maximum number of bytes
                                       // that can be sent or received in a
                                       // single transfer

// Status vector - top 4 bits only
#define  SMB_MTSTA      0xE0           // (MT) start transmitted
#define  SMB_MTDB       0xC0           // (MT) data byte transmitted
#define  SMB_MRDB       0x80           // (MR) data byte received


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
SI_SEGMENT_VARIABLE(buffer[BUFFER_LENGTH], uint8_t, SI_SEG_XDATA);
SI_SEGMENT_VARIABLE(SPI_TxBuf[MAX_BUFFER_SIZE+1], uint8_t, EFM8PDL_SPI0_TX_SEGTYPE);
SI_SEGMENT_VARIABLE(SPI_RxBuf[MAX_BUFFER_SIZE+1], uint8_t, EFM8PDL_SPI0_RX_SEGTYPE);


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

bool SMB_BUSY = 0;                      // Software flag to indicate when the
                                       // EEPROM_ByteRead() or
                                       // EEPROM_ByteWrite()
                                       // functions have claimed the SMBus

bool SMB_RW;                            // Software flag to indicate the
                                       // direction of the current transfer

bool SMB_SENDWORDADDR;                  // When set, this flag causes the ISR
                                       // to send the 8-bit <WORD_ADDR>
                                       // after sending the slave address.

bool SMB_RANDOMREAD;                    // When set, this flag causes the ISR
                                       // to send a START signal after sending
                                       // the word address.
                                       // For the 24LC02B EEPROM, a random read
                                       // (a read from a particular address in
                                       // memory) starts as a write then
                                       // changes to a read after the repeated
                                       // start is sent. The ISR handles this
                                       // switchover if the <SMB_RANDOMREAD>
                                       // bit is set.

bool SMB_ACKPOLL;                       // When set, this flag causes the ISR
                                       // to send a repeated START until the
                                       // slave has acknowledged its address

// 16-bit SI_SFR declarations

SI_SBIT(SDA, SFR_P2, 3);                  // SW1 ='0' means switch pressed
SI_SBIT(SCL, SFR_P2, 4);                  // SW2 ='0' means switch pressed
SI_SBIT(CS, SFR_P1, 1);

void SPI_Byte_Write (uint8_t writeData);
uint8_t SPI_Byte_Read(uint8_t writeData);
void Delay(void);
void EEPROM_ByteWrite(uint8_t addr, uint8_t dat);
uint8_t EEPROM_ByteRead(uint8_t addr);



//-----------------------------------------------------------------------------
// SiLabs_Startup() Routine
// ----------------------------------------------------------------------------
// This function is called immediately after reset, before the initialization
// code is run in SILABS_STARTUP.A51 (which runs before main() ). This is a
// useful place to disable the watchdog timer, which is enable by default
// and may trigger before main() in some instances.
//-----------------------------------------------------------------------------
void SiLabs_Startup (void)
{
  // Disable the watchdog here
}

//-----------------------------------------------------------------------------
// Main Routine
//-----------------------------------------------------------------------------
void main (void)
{
	uint8_t Data1,Data2;
   enter_DefaultMode_from_RESET();
   UART0_init(UART0_RX_ENABLE, UART0_WIDTH_8, UART0_MULTIPROC_DISABLE);
   SPI0_init(SPI0_CLKMODE_0,true,false);

   BSP_DISP_EN = BSP_DISP_BC_DRIVEN;   // Display not driven by EFM8

   BSP_BC_EN = BSP_BC_CONNECTED;       // Board controller connected to EFM8
                                       // UART pins
   IE_EA = 1;
   CS=0;

   while (1)
   {
      if ((UART0_rxBytesRemaining() == 0) && (UART0_txBytesRemaining() == 0))
      {
         UART0_readBuffer(buffer, BUFFER_LENGTH);
      }

      if ((UART1_rxBytesRemaining() == 0) && (UART1_txBytesRemaining() == 0))
      {
         UART1_readBuffer(buffer, BUFFER_LENGTH);
      }
      /*SPI_Byte_Write(0x80);
      SPI_Byte_Write(0x0);*/
      CS=1;
      SPI0_writeByte(0x80);
      Data1 = SPI0DAT;
      SPI0_writeByte(0x00);
      Data2 = SPI0DAT;
      while(SPI0_isBusy());
      CS=0;

      Delay();
   }
}

//-----------------------------------------------------------------------------
// UART ISR Callbacks
//-----------------------------------------------------------------------------
void UART0_receiveCompleteCb ()   //UART0 receive interrupter
{
   UART0_writeBuffer(buffer, BUFFER_LENGTH);

}

void UART0_transmitCompleteCb ()  //UART0 transmit interrupter
{

}

void UART1_receiveCompleteCb ()  //UART1 receive interrupter
{
	uint8_t data1,data2;
	if(buffer[0]==0x55&&buffer[1]==0x00)
	{
		SPI_Byte_Write(buffer[2]);
	}
	if(buffer[0]==0x55&&buffer[1]==0x01)
	{
         data1=SPI_Byte_Read(buffer[2]);
         data1=SPI_Byte_Read(0);
         UART1_write(data1);
         UART1_write(data2);
	}
	else
	{

	}
   //UART1_writeBuffer(buffer, BUFFER_LENGTH);
}

void UART1_transmitCompleteCb ()  //UART1 transmit interrupter
{
}

void SPI0_transferCompleteCb(void)    //SPI0 interrupter
{
}


void SPI_Byte_Write(uint8_t writeData)
{
      CS=1;
      SPI0_writeByte(writeData);
      while(SPI0_isBusy());
      CS=0;
}

//-----------------------------------------------------------------------------
// SPI_Byte_Read
//-----------------------------------------------------------------------------
//
// Return Value : The byte read from the slave
// Parameters   : None
//
// Reads a single byte from the SPI Slave.  The command consists of:
//
// Command = SPI_READ
// Length = 1 byte of command, 1 byte of data
//
//-----------------------------------------------------------------------------
uint8_t SPI_Byte_Read(uint8_t writeData)
{
	 uint8_t Data;

     CS=1;
     SPI0_writeByte(writeData);
     while(SPI0_isBusy());
     CS=0;
     if(SPI0CN0_SPIF)
     {
    	 Data = SPI0DAT;
     }
     return Data;
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
void Delay(void)
{
   uint32_t count;

   for (count = 200; count > 0; count--);
}
