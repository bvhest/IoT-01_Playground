//******************************************************************************
// main.c voor stoflamp.                                                       *
// By Ed Smallenburg.                                                          *
//******************************************************************************
// De functie van dit programma is om een LED strip aan te sturen met een      *
// kleur die afhankelijk is van de inhoud van een bericht dat via RF is op-    *
// gestuurd door het meetkastje.                                               *
// Input kan ook afkomstig zijn van een IR receiver (VS1838B), maar dat is in  *
// dit geval niet van toepassing.                                              *
//                                                                             *
// VS1838B pins (front view):                                                  *
// Pin 1  OUT                                                                  *
// Pin 2  GND                                                                  *
// Pin 3  Signal, normally HIGH                                                *
//                                                                             *
//                                                                             *
// STM8 Connections:                                                           *
// D5 -> Serial debug output                                                   *
// D6 -> Serial input                                                          *
// A1 -> IR receiver VS1838B                                                   *
// D4 -> LED strip output                                                      *
// A3 -> Power hold output.                                                    *
//                                                                             *
//******************************************************************************
// History:                                                                    *
// 26-03-2019, ES: First set-up.                                               *
// 01-05-2019, ES: Uitschakelen bij kleur = { 0, 0, 0 }.                       *
//******************************************************************************
//
#include "stm8s.h"
#include <stdio.h>                      // For printf(...)
#include <stdlib.h>
#include "delay.h"
#include "rf24l01.h"                    // For RF24L01 routines

#define  RECVSIZ  3                     // Size of receive buffer
#define  RECVPORT GPIOA                 // Port number of VS1838B module
#define  RECVPIN  GPIO_PIN_1            // Pin number used
#define  RECVINT  ITC_IRQ_PORTA         // IRQ used for the port
#define  RECVEXTI EXTI_PORT_GPIOA       // EXTI_PORT used

#define  NETWORK    "STFL"              // Network name
#define  RF_CHN     12                  // RF channel to use
#define  MYNODE     2                   // Node in receive mode

#define  NB_LEDS    14                  // Aantal LEDs op de strip

// Common data for LED strip
uint8_t   Led[NB_LEDS * 3] ;            // Array with colors for all LEDs
uint16_t  nLedBytes = sizeof(Led) ;     // Number of bytes in Led array 
uint8_t   recvbuf[RECVSIZ+1] ;          // Receive buffer IR receiver
uint8_t   rf24buf[RECVSIZ] ;            // Idem voor RF24 bericht
uint16_t  recvres = 0 ;                 // Data received from RF receiver
bool      newrecvres = FALSE ;          // Signal for new result
// For Serial I/O
uint8_t   txbuf[64] ;                   // Output buffer
uint8_t   rxbuf[64] ;                   // Input buffer
uint8_t   txinx = 0 ;                   // Index in output buffer
uint8_t   rxinx = 0 ;                   // Index in input buffer
bool      f_rxrdy = FALSE ;             // Complete message received
bool      f_txrdy = TRUE ;              // Complete message sent

uint8_t   pwm_red ;                     // PWM waarde voor rode LED (0..255)
uint8_t   pwm_blue ;                    // PWM waarde voor blauwe LED (0..255)
uint8_t   pwm_green ;                   // PWM waarde voor groene LED (0..255)

//******************************************************************************
//                   C L K _ A N D _ G P I O _ I N I T                         *
//******************************************************************************
// Initialize Clock and GPIO registers.                                        *
//******************************************************************************
void clk_and_gpio_init(void)
{
  CLK_HSIPrescalerConfig ( CLK_PRESCALER_HSIDIV1 ) ;   // Normal clock
  GPIO_DeInit ( GPIOA ) ;                              // Init ALL GPIO
  GPIO_DeInit ( GPIOB ) ;
  GPIO_DeInit ( GPIOC ) ;
  GPIO_DeInit ( GPIOD ) ;
  ITC_DeInit() ;                                       // Interrupt setup
  EXTI_DeInit() ;                                      // Sensivity setup
  //
  GPIO_Init ( GPIOA, GPIO_PIN_3,
              GPIO_MODE_OUT_PP_HIGH_FAST ) ;           // A3 Power hold circuit
  GPIO_Init ( GPIOB, GPIO_PIN_5,
              GPIO_MODE_OUT_PP_HIGH_SLOW ) ;           // B5 Build-in LED
  GPIO_Init ( GPIOD, GPIO_PIN_4,
              GPIO_MODE_OUT_PP_LOW_FAST ) ;            // D4 RGB strip out
  GPIO_Init ( RECVPORT, RECVPIN,
              GPIO_MODE_IN_FL_IT ) ;                   // INPUT, INTERRUPT
  ITC_SetSoftwarePriority ( RECVINT,                   // Priority for port is 0
                            ITC_PRIORITYLEVEL_0 ) ;
  EXTI_SetExtIntSensitivity ( RECVEXTI,                // Rising/falling edge interrupt
                              EXTI_SENSITIVITY_RISE_FALL ) ;
  // Init Timer 4
  TIM4->PSCR = TIM4_PRESCALER_128 ;                    // Scaler is 128, run on 0.125 MHz
  TIM4->ARR = 0xFF ;                                   // Reload value
  TIM4->CR1 |= TIM4_CR1_CEN ;                          // Enable TIM4
}


//******************************************************************************
// Interrupt handling for UART 1.                                              *
//******************************************************************************

//******************************************************************************
//                        U A R T 1 _ R X _ I R Q H A N D L E R                *
//******************************************************************************
// Handle the interrupts of incoming messages.                                 *
//******************************************************************************
INTERRUPT void UART1_RX_IRQHandler(void)
{
  uint8_t c ;                                   // Character from uart
  
  c = UART1_ReceiveData8() ;                    // Read the input character
  if ( ! f_rxrdy )                              // Ignore if message is complete
  {
    if ( c < ' ' )                              // Control character?
    {
      if ( rxinx > 0 )                          // Yes, any input yet?
      {
        if ( c == '\r' )                        // Yes, is it a CR?
        {
          f_rxrdy = TRUE ;                      // Yes, set flag, stop future input
          rxbuf[rxinx] = '\0' ;                 // Store delimeter in buffer
          rxinx = 0 ;                           // Prepare for next message
        }
        return ;                                // Ignore other control characters
      }
    }
    else
    {
      // A normal input character received
      if ( rxinx < ( sizeof(rxbuf) - 1 ) )      // Protect against overflow
      {
        rxbuf[rxinx++] = c ;                    // Store received character and store
      }
    }
  }
}


//******************************************************************************
//                        U A R T 1 _ T X _ I R Q H A N D L E R                *
//******************************************************************************
// Handle the interrupts of outgoing messages.                                 *
//******************************************************************************
INTERRUPT void UART1_TX_IRQHandler(void)
{
  uint8_t c ;                                   // Character to send

  c = txbuf[txinx++] ;                          // Get next character from buffer
  if ( c < ' ' )                                // End of message?
  {
    UART1_ITConfig ( UART1_IT_TXE, DISABLE ) ;  // Yes, disable further interrupts
    c = '\n' ;                                  // Force send of linefeed
    f_txrdy = TRUE ;
    txinx = 0 ;                                 // Reset index
  }
  UART1_SendData8 ( c ) ;                       // Send the character or delimeter
}



//******************************************************************************
//                           S E R I A L _ I N I T                             *
//******************************************************************************
// Initialize Uart1 for serial I/O.                                            *
//******************************************************************************
void serial_init(void)
{
  UART1_DeInit() ;
  UART1_Init ( (uint32_t)115200, UART1_WORDLENGTH_8D,
               UART1_STOPBITS_1, UART1_PARITY_NO,
               UART1_SYNCMODE_CLOCK_DISABLE,
               UART1_MODE_TXRX_ENABLE ) ;
  UART1_ITConfig ( UART1_IT_RXNE_OR, ENABLE ) ;   // Enable receive interrupts
  UART1_Cmd ( ENABLE ) ;                          // Enable UART
}  


//******************************************************************************
//                           P U T C H A R                                     *
//******************************************************************************
// Send a byte to the serial output buffer.                                    *
// The result is equal to the character sent.                                  *
//******************************************************************************
char putchar ( char c )
{
  while ( !f_txrdy )                          // Wait for previous message
  {
    delay_ms ( 1 ) ;
  }
  if ( txinx < sizeof(txbuf) )                // Buffer space available?
  {
    txbuf[txinx++] = c ;                      // Yes, store the byte
  }
  if ( c < ' ' )                              // Control character?
  {
    f_txrdy = FALSE ;                         // Not ready now
    txinx = 0 ;                               // Start transfer from index 0
    UART1_ITConfig ( UART1_IT_TXE, ENABLE ) ; // Enable transmit interrupts
  }
  return c ;                                  // Return the data sent
}


//******************************************************************************
//                           S E N D M S G                                     *
//******************************************************************************
// Send a message to the serial output.                                        *
// D4 is set to LOW to enable the RS485 output.                                *
//******************************************************************************
void sendmsg ( const char* str )
{
  while ( putchar ( *str++ ) >= ' ' ) ;       // Send until delimeter
  while ( !f_txrdy )                          // Wait for message to complete
  {
    delay_ms ( 1 ) ;
  }
}


//******************************************************************************
//                           T X R D Y                                         *
//******************************************************************************
// Check if transmitter is finished.                                           *
//******************************************************************************
bool txrdy(void)
{
  return f_txrdy ;
}


//******************************************************************************
//                           R X R D Y                                         *
//******************************************************************************
// Check if receiver is finished.                                              *
//******************************************************************************
bool rxrdy(void)
{
  return f_rxrdy ;
}


//******************************************************************************
//                       R E S E T _ R X R D Y                                 *
//******************************************************************************
// Reset the receiver ready flag.                                              *
//******************************************************************************
void reset_rxrdy(void)
{
  f_rxrdy = FALSE ;
}


//******************************************************************************
//                  E X T I _ P O R T A _ I R Q H A N D L E R                  *
//******************************************************************************
// Handle the interrupts of digital inputs for A port.                         *
// This is input from the IR receiver.                                         *
//******************************************************************************
INTERRUPT void EXTI_PORTA_IRQHandler(void)
{
  static uint8_t  lasttime ;                  // Time in micros of last interrupt
  uint8_t         newtime ;                   // Time in micros of new interrupt
  uint8_t         delta8 ;
  static uint8_t  nsync = 0 ;                 // number of syncs seen
  static uint8_t  recvinx = 0 ;               // Index in recvbuf
  static uint8_t  bytefound ;                 // Detected input code
  static uint8_t  bitsfound ;                 // Number of input bits seen
  uint16_t        delta ;                     // Pulse length

  newtime = TIM4->CNTR ;                      // Read new timestamp (microseconds)
  // Read new status.  Check if it is the rising edge of the pulse
  if ( ( GPIO_ReadInputData ( RECVPORT ) & RECVPIN ) == 0 )
  {
    lasttime = newtime ;                      // Falling, init timer
    return ;                                  // Leading edge, no further actions
  }
  // We have received the rising edge
  delta8 = newtime - lasttime ;               // Compute delta in ticks
  delta = (uint16_t)delta8 * 8 ;              // Compute delta in microseconds
  lasttime = newtime ;                        // For next measurement.  Necessary?
  if ( newrecvres )                           // Pending result?
  {
    return ;                                  // Yes, ignore input
  }
  if ( delta > 50 && delta <= 350 )          // Is it a "0"?
  {
    if ( nsync > 5 )                          // Syncs seen?
    {
      bytefound = bytefound << 1 ;            // Shift zero bit in
      bitsfound++ ;                           // Count number of bits seen
    }
  }
  else if ( delta > 400 && delta <= 700 )     // Is it a "1"?
  {
    if ( nsync > 5 )                          // Syncs seen?
    {
      bytefound = ( bytefound << 1 ) + 1 ;    // Shift one bit in
      bitsfound++ ;                           // Count number of bits seen
    }
  }
  else if ( delta > 750 && delta < 1250 )     // Is it a sync?
  {
    ++nsync ;                                 // Yes, count
    if ( recvinx )                            // Data seen?
    {
      if ( nsync > 10 )
      {
        newrecvres = TRUE ;                   // Yes message complete
        nsync = 0 ;                           // For next message
        recvinx = 0 ;
      }
    }
    else
    {                                         // Begin sync
      bytefound = 0 ;                         // No code yet
      bitsfound = 0 ;                         // No bits found
      recvinx = 0 ;
    }
  }
  else                                        // Length too short or too long?
  {
    nsync = 0 ;                               // Reset sync count
  }
  if ( ( nsync > 5 ) && ( bitsfound == 8 ) )  // 8 bits seen?
  {
    bitsfound = 0 ;                           // Get ready for next byte
    recvbuf[recvinx] = bytefound ;            // Store new byte
    if ( recvinx < RECVSIZ )                  // Space left?
    {
      recvinx++ ;                             // Yes
    }
  }
}


//******************************************************************************
//                      C L E A R _ R E C V B U F                              *
//******************************************************************************
// Clear recvbuf, 10 bytes data, 1 byte checksum.                              *
//******************************************************************************
void clear_recvbuf(void)
{
  int      i ;                                      // Loop control

  for ( i = 0 ; i < RECVSIZ ; i++ )                 // Clear whole buffer
  {
    recvbuf[i] = 0xFF  ;
  }
}


//******************************************************************************
//                      C H E C K _ C H E C K S U M                            *
//******************************************************************************
// Check the checksum in recvbuf, RECVSIZ bytes data, 1 byte checksum          *
//******************************************************************************
bool check_checksum(void)
{
  int     i ;                               // Index in recvbuf
  uint8_t sum = 0 ;

  for ( i = 0 ; i <= RECVSIZ ; i++ )
  {
    sum ^= recvbuf[i] ;                     // Add byte to checksum
  }
  return ( sum == 0 ) ;
}


//******************************************************************************
//                            S E T L E D C O L O R                            *
//******************************************************************************
// Fill the array with color information.                                      *
// For this aplication all the leds get the same color.                        *
//******************************************************************************
void SetLedColor ( unsigned char R, unsigned char G, unsigned char B )
{
  int j ;
  
  j = NB_LEDS * 3 - 1 ; 
  while ( j > 0 )
  {
    Led[j--] = G ;                          // Order is GRB, not RGB
    Led[j--] = R ;
    Led[j--] = B ;
  }
}


//******************************************************************************
//                            S E N D T O S K 6 8 1 2                          *
//******************************************************************************
// Sens all bits for all the leds to the Din pin of the strip.                 *
// Timing:                                                                     *
// 0-bit: 300 ns hoog, 900 ns laag.                                            *
// 1-bit: 600 ns hoog, 600 ns laag.                                            *
//******************************************************************************
void SendToSK6812(void)
{
  #asm
        LDW     X, _nLedBytes
		lb_byteloop:
        DECW	  X 
        JRMI    lb_exit      
				LD      A, (_Led,X)             ; Get next byte value
				LDW			Y,	#0x8                ; 8 bits per color
    lb_bitloop:   
				BSET    0x500F, #0x4            ; Output HIGH
				SLL			A                       ; Shift next bit in Carry
        JRNC    lb_bit_0                ; Jump for a shorter zero bit
				NOP
				NOP
        NOP
				NOP
				NOP
        NOP
    lb_bit_0: 
        NOP
        BRES    0x500F, #0x4            ; Output LOW
        JRC     lb_bit_1                ; Jump for a shorter 1 bit
				NOP
				NOP
        NOP
        NOP
        NOP
    lb_bit_1:
        NOP
        NOP
        NOP
				DECW	  Y                       ; Count number of bits per byte
				JRNE    lb_bitloop
				JRA     lb_byteloop
      lb_exit:
#endasm
}


//******************************************************************************
//                      H O O F D P R O G R A M M A                            *
//******************************************************************************
void main(void)
{
  int16_t i ;
  char    C ;                                       // Color in serial command
  uint8_t RF_status ;                               // Status reg of RF24L01
  uint8_t dleng ;                                   // Data length received
  bool    RF24_okay ;                               // RF24L01 aangesloten of niet
  int16_t sw_off_timer = 0 ;                        // Timer voor meten inactiviteit

  clk_and_gpio_init() ; 
  serial_init() ;                                   // Initialize serial port
  RF24L01_init() ;                                  // Init SPI and RF24L01
  RF24L01_setup ( NETWORK, RF_CHN ) ;               // Init communication channel
  RF24L01_auto_ack ( TRUE ) ;                       // Auto ack
  enableInterrupts() ;                              // Enable interrupts  
  printf ( "Starting...\n" ) ;                      // Yes, show it
  GPIO_WriteHigh ( GPIOA, GPIO_PIN_3 ) ;            // A3 HIGH, laat voeding aan
  RF24L01_set_mode_RX ( MYNODE ) ;                  // Start receiving as node "MYNODE"
  RF24_okay = ( RF24L01_get_status() != 0xFF ) ;    // Test if RF24L01 is aangesloten
  SetLedColor ( 0, 100, 0 ) ;                       // Test pattern
  SendToSK6812() ;                                  // Write to LED strip
  delay_ms ( 200 ) ;
  SetLedColor ( 0, 0, 0 ) ;                         // Blank
  SendToSK6812() ;                                  // Write to LED strip
  while ( TRUE )
  {
    if ( RF24_okay &&                               // NRF24L01 aangesloten?
         ( ( RF24L01_get_status() & 0x40 ) != 0 ) ) // New RX data from NRF24L01?
    {
      RF_status = RF24L01_clear_interrupts() ;      // Read and clear status
      //printf ( "Status is now 0x%02X\n",          // Bit 6 = data ready
      //         (int)RF_status ) ;                 // Pipe number in bit 3-1
      dleng = RF24L01_read_register ( RF24L01_command_R_RX_PL_WID ) ;
      GPIO_WriteLow ( GPIOB, GPIO_PIN_5 ) ;         // Temp. received, show activity
      printf ( "RF24L01 data available, "           // Show for debugging
               "message length is %d bytes\n",
               (int)dleng ) ;
      if ( dleng > RECVSIZ )                        // Bescherm tegen overflow
      {
        dleng = RECVSIZ ;
      }
      RF24L01_read_payload ( rf24buf, dleng ) ;     // Lees de payload
      GPIO_WriteLow ( GPIOB, GPIO_PIN_5 ) ;         // Bericht ontvangen, toon activiteit
      printf ( "Data is R = %d, G = %d, B = %d\n",
               (int)rf24buf[0],
               (int)rf24buf[1],
               (int)rf24buf[2] ) ;
      SetLedColor ( rf24buf[0],                     // PWM waarde rood
                    rf24buf[1],                     // PWM waarde groen
                    rf24buf[2] ) ;                  // PWM waarde blauw
      if ( rf24buf[0] | rf24buf[1] | rf24buf[2] )   // Tenminste iets verlicht?
      {
        sw_off_timer = 0 ;                          // Ja, herstart inactiviteit timer
      }
      delay_ms ( 200 ) ;                            // Wait for end of interrupts UART1
      SendToSK6812() ;                              // Schrijf naar LED strip
      GPIO_WriteHigh ( GPIOB, GPIO_PIN_5 ) ;        // Einde activiteit
    }
    if ( newrecvres )                               // Nieuw bericht van VS1838B?
    {
      printf ( "IR Buf is " ) ;                     // Ja, toon voor debug
      for ( i = 0 ; i < RECVSIZ ; i++ )
      {
        printf ( "%02X ", (int)recvbuf[i] ) ;
      }
      printf ( "\n" ) ;
      if ( check_checksum() )                       // Yes, check de checksum
      {
        GPIO_WriteLow ( GPIOB, GPIO_PIN_5 ) ;       // Bericht ontvangen, toon activiteit
        SetLedColor ( recvbuf[0],                   // PWM waarde rood
                      recvbuf[1],                   // PWM waarde groen
                      recvbuf[2] ) ;                // PWM waarde blauw
        if ( recvbuf[0] | recvbuf[1] | recvbuf[2] ) // Tenminste iets verlicht?
        {
          sw_off_timer = 0 ;                        // Ja, herstart inactiviteit timer
        }
        SendToSK6812() ;                            // Write to LED strip
        clear_recvbuf() ;
        delay_ms ( 200 ) ;
        GPIO_WriteHigh ( GPIOB, GPIO_PIN_5 ) ;      // Einde activiteit
      }
      else
      {
        printf ( "Checksum fout\n" ) ;
      }
      newrecvres = FALSE ;
    }
    if ( rxrdy() )                                  // Serial commando (voor test)?
    {
      i = atoi ( rxbuf + 1 ) ;                      // Ja, voorbeeld: "R300" voor rood 30%
      C = rxbuf[0] ;
      printf ( "Input %c = %d\n", C,
               (int)i ) ;
      reset_rxrdy() ;                               // Reset input ready flag
      if ( C == 'R' )
      {
        SetLedColor ( i, 0, 0 ) ;
      }
      else if ( C == 'G' )
      {
        SetLedColor ( 0, i, 0 ) ;
      }
      else if ( C == 'B' )
      {
        SetLedColor ( 0, 0, i ) ;
      }
      SendToSK6812() ;                              // Write to LED strip
    }
    if ( sw_off_timer++ > 12000 )                   // 2 minuten zonder bericht?
    {
      SetLedColor ( 0, 0, 0 ) ;                     // LEDs uit
      SendToSK6812() ;                              // Write to LED strip
      GPIO_WriteLow ( GPIOA, GPIO_PIN_3 ) ;         // A3 LOW, voeding uit 
    }
    delay_ms ( 10 ) ;
  }
}
