//******************************************************************************
// rf24l01.c -- Driver for RF24L01                                             *
// Source https://github.com/ARENIBDelta/rf24l01_stm8s/blob/master             *
//               RF24L01_master/rf24l01.c                                      *
// Modified by Ed Smallenburg                                                  *
//                                                                             *
//******************************************************************************
// Use of pipes, example network name is "XXXX":                               *
// TX pipe is used to address the target node. "XXXX1".."XXX5".                *
// RX_P0 is used for the ACK. "XXXX1".."XXXX5".                                *
// RX_P1 is used for data. "XXXX1".."XXXX5".                                   *
// RX_P2..RX_P5 are not used.                                                  *
//******************************************************************************
// History:                                                                    *
// 22-01-2019, ES: First set-up.                                               *
//******************************************************************************

#include "rf24l01.h"
#include "delay.h"
#include <stm8s_spi.h>
#include <stm8s_gpio.h>
#include <stdio.h>                      // For printf(...)
#include <string.h>                     // For memcpy


//******************************************************************************
// Global data.                                                                *
//******************************************************************************
uint8_t  pipe_addr[5+1] ;                    // Saved pipe address
                                             // Byte 1..4 is the network name


//******************************************************************************
//                R F 2 4 L 0 1 _ I N I T                                      *
//******************************************************************************
// Initialize GPIO for SPI bus and CE line.                                    *
// Initialize the SPI bus.                                                     *
//******************************************************************************
void RF24L01_init(void)
{
  GPIO_Init ( GPIOC, GPIO_PIN_4,
              GPIO_MODE_OUT_PP_HIGH_FAST ) ;      // CSN (Chip select)
  GPIO_Init ( GPIOC, GPIO_PIN_3,
              GPIO_MODE_OUT_PP_LOW_FAST ) ;       // CE
  SPI_Init (                                      // Init SPI bus
      SPI_FIRSTBIT_MSB,
      SPI_BAUDRATEPRESCALER_2,                    // Clock at 8 MHz
      SPI_MODE_MASTER,
      SPI_CLOCKPOLARITY_LOW,
      SPI_CLOCKPHASE_1EDGE,
      SPI_DATADIRECTION_2LINES_FULLDUPLEX,
      SPI_NSS_SOFT,
      (uint8_t)0x07 ) ;                           // CRC polynomial
  SPI_Cmd ( ENABLE ) ;
}


//******************************************************************************
//                R F 2 4 L 0 1 _ C S _ H I G H                                *
//******************************************************************************
// Set chip select high (deactivate).                                          *
//******************************************************************************
void RF24L01_CS_HIGH(void)
{
  delay_us ( 8 ) ;                        // Wait for last clock pulse
  GPIO_WriteHigh ( GPIOC, GPIO_PIN_4 ) ;
}


//******************************************************************************
//                R F 2 4 L 0 1 _ C S _ L O W                                  *
//******************************************************************************
// Set chip select low (activate).                                             *
//******************************************************************************
void RF24L01_CS_LOW(void)
{
  GPIO_WriteLow ( GPIOC, GPIO_PIN_4 ) ;
}


//******************************************************************************
//                     R F 2 4 L 0 1 _ S P I C M D                             *
//******************************************************************************
// Send a single command over the SPI bus.  CS is assumed to be LOW.           *
//******************************************************************************
uint8_t RF24L01_SPIcmd ( uint8_t command )
{
  while ( SPI_GetFlagStatus ( SPI_FLAG_TXE ) == RESET ) ;
  SPI_SendData ( command ) ;
  while ( SPI_GetFlagStatus ( SPI_FLAG_BSY ) == SET ) ;
  while ( SPI_GetFlagStatus ( SPI_FLAG_RXNE ) == RESET ) ;
  return SPI_ReceiveData() ;
}


//******************************************************************************
//                R F 2 4 L 0 1 _ S E N D _ C O M M A N D                      *
//******************************************************************************
// Send a single byte to the SPI bus.                                          *
//******************************************************************************
void RF24L01_send_command ( uint8_t command )
{
  RF24L01_CS_LOW() ;                    // Chip select
  RF24L01_SPIcmd ( command ) ;          // Send command
  RF24L01_CS_HIGH() ;                   // Chip deselect
}


//******************************************************************************
//                R F 2 4 L 0 1 _ R E A D _ R E G I S T E R                    *
//******************************************************************************
// Read a sigle byte from the SPI bus.                                         *
//******************************************************************************
uint8_t RF24L01_read_register ( uint8_t register_addr )
{
  uint8_t result ;
  
  RF24L01_CS_LOW() ;                               // Chip select
  RF24L01_SPIcmd ( RF24L01_command_R_REGISTER |    // Send address and read command
                   register_addr ) ;
  result = RF24L01_SPIcmd ( 0x00 ) ;               // Get data
  RF24L01_CS_HIGH() ;                              // Chip deselect
  return result ;
}


//******************************************************************************
//                R F 2 4 L 0 1 _ W R I T E _ 1 _ R E G                        *
//******************************************************************************
// Write a single byte to a register.                                          *
//******************************************************************************
void RF24L01_write_1_reg ( uint8_t register_addr, uint8_t value )
{
  RF24L01_CS_LOW() ;                             // Activate chip select
  RF24L01_SPIcmd ( RF24L01_command_W_REGISTER |  // Send address and write command
                   register_addr ) ;             // reg-addr + 0x20
  RF24L01_SPIcmd ( value ) ;
  RF24L01_CS_HIGH() ;                            // Deactivate chip select
}


//******************************************************************************
//                R F 2 4 L 0 1 _ W R I T E _ R E G I S T E R                  *
//******************************************************************************
// Write a number of bytes to a register.                                      *
//******************************************************************************
void RF24L01_write_register ( uint8_t register_addr, uint8_t *value, uint8_t length )
{
  uint8_t i ;

  RF24L01_CS_LOW() ;                               // Activate chip select
  RF24L01_SPIcmd ( RF24L01_command_W_REGISTER |
                   register_addr ) ;               // reg-addr + 0x20
  for ( i = 0 ; i < length ; i++ )                 // Send data  
  {
    RF24L01_SPIcmd ( value[i] ) ;
  }
  RF24L01_CS_HIGH() ;                              // Deactivate chip select
}


//******************************************************************************
//                R F 2 4 L 0 1 _ S E T U P                                    *
//******************************************************************************
// Configure the RF24L01.                                                      *
// The network name is specified by 4 characters.  The adresses for the        *
// transmit pipe, the ack pipe (0) and the 5 receive pipes are derived from    *
// the network name.                                                           * 
//******************************************************************************
void RF24L01_setup ( uint8_t *nw_name, uint8_t channel )
{
  uint8_t i ;                                             // Loop control
  
  strcpy ( (char*)pipe_addr + 1, nw_name ) ;              // Set network name
  pipe_addr[0] = 'X' ;
  RF24L01_write_1_reg ( RF24L01_reg_SETUP_AW,   0x03 ) ;  // Address width 5 bits
  RF24L01_write_1_reg ( RF24L01_reg_SETUP_RETR, 0x78 ) ;  // 8 retries, interval 2 msec
  RF24L01_write_1_reg ( RF24L01_reg_FEATURE,    0x04 ) ;  // Enable dynamic payload length
  for ( i = 0 ; i < 6 ; i++ )                             // Prepare ACK- and 5 RX pipes
  {
    RF24L01_write_1_reg ( RF24L01_reg_RX_PW_P0 + i,       // FiFo size payload all pipes
                          0x20 ) ;
  }
  RF24L01_write_1_reg ( RF24L01_reg_EN_AA,    0x3F ) ;    // Auto acknowledgement
  RF24L01_write_1_reg ( RF24L01_reg_DYNPD,    0x3F ) ;    // Dynamic payload
  RF24L01_write_1_reg ( RF24L01_reg_EN_RXADDR,0x3F ) ;    // Enable pipe 0-5
  RF24L01_write_1_reg ( RF24L01_reg_RF_CH, channel ) ;    // Set frequency channel
  RF24L01_write_1_reg ( RF24L01_reg_RF_SETUP, 0x26 ) ;    // 250 kbps, 0dbm
  RF24L01_write_1_reg ( RF24L01_reg_CONFIG,   0x7D ) ;    // No intrpts, 2 byte CRC,
                                                          // Pwrdwn, RX
  RF24L01_write_1_reg ( RF24L01_reg_STATUS,   0x70 ) ;    // Clear status bits
}


//******************************************************************************
//                R F 2 4 L 0 1 _ A U T O _ A C K                              *
//******************************************************************************
// Enable or disable the auto_ack feature.                                     *
//******************************************************************************
void RF24L01_auto_ack ( bool enable )
{
  uint8_t ackbits = 0x00 ;                              // Assume disable all
  
  if ( enable )
  {
    ackbits = 0x3F ;                                    // Enable all
  }
  RF24L01_write_1_reg ( RF24L01_reg_EN_AA, ackbits ) ;  // Auto acknowledgement
}


//******************************************************************************
//                R F 2 4 L 0 1 _ S E T _ M O D E _ T X                        *
//******************************************************************************
// Set the RF24L01 to transmit mode.  Parameter is the node number 1..5.       *
// Set Enable intrpts, 2 byte CRC, Powerup and TX in CONFIG register.          *
// The RX_ADDR of ACK FiFo 0 must contain the address of the remote station.   *
//******************************************************************************
void RF24L01_set_mode_TX ( uint8_t node )
{
  RF24L01_send_command ( RF24L01_command_FLUSH_TX ) ;  // Flush TX FiFo
  GPIO_WriteLow ( GPIOC, GPIO_PIN_3 ) ;
  RF24L01_write_1_reg ( RF24L01_reg_CONFIG, 0x7E ) ;
  pipe_addr[0] = '0' + node ;                          // Address for pipe
  RF24L01_write_register ( RF24L01_reg_TX_ADDR,        // 5 byte TX address
                            pipe_addr, 5 ) ;
  RF24L01_write_register ( RF24L01_reg_RX_ADDR_P0,     // Set listen address
                           pipe_addr, 5 ) ;            // For ACK
  // printf ( "Set mode TX to %s\n",
  //          (char*)pipe_addr ) ;
}


//******************************************************************************
//                R F 2 4 L 0 1 _ S E T _ M O D E _ R X                        *
//******************************************************************************
// Set the RF24L01 to receive mode.  Parameter is the node number 1..5.        *
// The RX mode is an active mode where the nRF24L01+ radio is used as a        *
// receiver. To enter this mode, the nRF24L01+ must have the PWR_UP bit,       *
// PRIM_RX bit and the CE pin set high.                                        *
//******************************************************************************
void RF24L01_set_mode_RX ( uint8_t node )
{
  pipe_addr[0] = '0' + node ;
  RF24L01_write_1_reg ( RF24L01_reg_CONFIG, 0x7F ) ;  // No intrpts, 2 byte CRC,
                                                      // Powerup, RX
  RF24L01_write_1_reg ( RF24L01_reg_STATUS, 0x07 ) ;  // Clear DR, DS and RT
  RF24L01_write_register ( RF24L01_reg_RX_ADDR_P1,    // 5 byte RX address
                           pipe_addr, 5 ) ;
  RF24L01_send_command ( RF24L01_command_FLUSH_RX ) ; // Flush FiFo (0xE2)
  GPIO_WriteHigh ( GPIOC, GPIO_PIN_3 ) ;              // CE must be HIGH during receive
  //printf ( "Set mode RX from %s\n",
  //         (char*)pipe_addr ) ;
}


//******************************************************************************
//                R F 2 4 L 0 1 _ G E T _ S T A T U S                          *
//******************************************************************************
// Read the RF24L01 status register. Most important bits are:                  *
// Bit 0   : TX FiFo full                                                      *
// Bit 1-3 : Data pipe number holding RX data. "111" if FiFo empty             *
// Bit 4   : Max number of retransmits reached.                                *
// Bit 5   : Data sent                                                         *
// Bit 6   : RX data ready                                                     *
//******************************************************************************
uint8_t RF24L01_get_status(void)
{
  uint8_t status ;
  
  RF24L01_CS_LOW() ;                                      // Chip select
  status = RF24L01_SPIcmd ( RF24L01_command_NOP ) ;       // Send command
  RF24L01_CS_HIGH() ;                                     // Chip deselect
  return status ;
}


//******************************************************************************
//                R F 2 4 L 0 1 _ W R I T E _ P A Y L O A D                    *
//******************************************************************************
// Write the payload to the FiFo and start sending.                            *
//******************************************************************************
void RF24L01_write_payload ( uint8_t *data, uint8_t length )
{
  uint8_t    i ;

  RF24L01_CS_LOW() ;                                        // Chip select
  RF24L01_SPIcmd ( RF24L01_command_W_TX_PAYLOAD ) ;         // (0xA0)
  for ( i = 0 ; i < length ; i++ )                          // Send data
  {
    RF24L01_SPIcmd ( data[i] ) ;
  }
  RF24L01_CS_HIGH() ;                                       // Chip deselect
  GPIO_WriteHigh ( GPIOC, GPIO_PIN_3 ) ;                    // Pulse CE to send the data
  delay_ms ( 1 ) ;                                          // Should at least 10 usec
  GPIO_WriteLow ( GPIOC, GPIO_PIN_3 ) ;
}


//******************************************************************************
//                R F 2 4 L 0 1 _ R E A D _ P A Y L O A D                      *
//******************************************************************************
// Read the payload from the FiFo.                                             *
//******************************************************************************
void RF24L01_read_payload ( uint8_t *data, uint8_t length )
{
  uint8_t i ;
  
  RF24L01_CS_LOW() ;                                        // Chip select
  RF24L01_SPIcmd ( RF24L01_command_R_RX_PAYLOAD ) ;         // Send RX command
  for ( i = 0 ; i < length ; i++ )                          // Get data
  {
    while ( SPI_GetFlagStatus ( SPI_FLAG_TXE ) == RESET ) ;
    SPI_SendData ( 0x00 ) ;
    while ( SPI_GetFlagStatus ( SPI_FLAG_BSY ) == SET ) ;
    *(data++) = SPI_ReceiveData() ;
  }
  RF24L01_CS_HIGH() ;                                       // Chip deselect
  RF24L01_send_command ( RF24L01_command_FLUSH_RX ) ;       // Flush rest of input
}


//******************************************************************************
//                R F 2 4 L 0 1 _ C L E A R _ I N T E R R U P T S              *
//******************************************************************************
// Clear interrupts and return the status                                      *
//******************************************************************************
uint8_t RF24L01_clear_interrupts(void)
{
  uint8_t a ;
  
  a = RF24L01_get_status() ;
  RF24L01_write_1_reg ( RF24L01_reg_STATUS, a ) ;
  return a ;
}