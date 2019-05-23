//******************************************************************************
// main.c for stoflamp button.                                                 *
// By Ed Smallenburg.                                                          *
//******************************************************************************
// The function of this unit is to drive a three color LED trough PWM signals. *
// Input is anIR receiver (VS1838B)                                            *
//                                                                             *
// STM8 Connections:                                                           *
// D5 -> Serial debug output                                                   *
// D6 -> Serial input                                                          *
// A1 -> IR receiver VS1838B                                                   *
// D4 -> GREEN LED                                                             *
// D3 -> BLUE LED                                                              *
// A3 -> RED LED                                                               *
//                                                                             *
//******************************************************************************
// History:                                                                    *
// 26-03-2019, ES: First set-up.                                               *
// 01-05-2019, ES: LEDs uit als er 2 minuten geen bericht binnenkomt.          *
//******************************************************************************
//
#include "stm8s.h"
#include <stdio.h>                      // For printf(...)
#include <stdlib.h>
#include "delay.h"

#define  RECVSIZ  3                     // Size of receive buffer
#define  RECVPORT GPIOA                 // Port number of VS1838B module
#define  RECVPIN  GPIO_PIN_1            // Pin number used
#define  RECVINT  ITC_IRQ_PORTA         // IRQ used for the port
#define  RECVEXTI EXTI_PORT_GPIOA       // EXTI_PORT used

uint8_t   recvbuf[RECVSIZ+1] ;          // Receive buffer IR receiver
uint16_t  recvres = 0 ;                 // Data received from RF receiver
bool      newrecvres = FALSE ;          // Signal for new result
// For Serial I/O
uint8_t   txbuf[64] ;                   // Output buffer
uint8_t   rxbuf[64] ;                   // Input buffer
uint8_t   txinx = 0 ;                   // Index in output buffer
uint8_t   rxinx = 0 ;                   // Index in input buffer
bool      f_rxrdy = FALSE ;             // Complete message received
bool      f_txrdy = TRUE ;              // Complete message sent
uint16_t  pwm_red ;                     // PWM waarde voor rode LED (0..1000)
uint16_t  pwm_blue ;                    // PWM waarde voor blauwe LED (0..1000)
uint16_t  pwm_green ;                   // PWM waarde voor groene LED (0..1000)

//******************************************************************************
//                   C L K _ A N D _ G P I O _ I N I T                         *
//******************************************************************************
// Initialize Clock and GPIO registers.                                        *
// Port B5 and D4 are set to OUT.                                              *
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
  GPIO_Init ( GPIOB, GPIO_PIN_5,
              GPIO_MODE_OUT_PP_HIGH_FAST ) ;           // Build-in LED
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


void InitTimer2_PWM(void)
{
  
  TIM2->PSCR = 0x03;   
  TIM2->ARRH = (unsigned char)(999 >> 8); 
  TIM2->ARRL = (unsigned char)999;   

  TIM2->CCER1 = 0x00;  // Disable the Channels 1-2
  TIM2->CCER1 = 0x33;  // Enable the Channel 1-2 & Low Polarity

  TIM2->CCER2 = 0x00;  // Disable the Channels 3
  TIM2->CCER2 = 0x03;  // Enable the Channel 3 & Low Polarity

  TIM2->CCMR1 = 0x78;  // PWM Mode2(CH1) - Preload  Enabled
  TIM2->CCMR2 = 0x78;  // PWM Mode2(CH2) - Preload  Enabled
  TIM2->CCMR3 = 0x78;  // PWM Mode2(CH3) - Preload  Enabled

  TIM2->CCR1H = (unsigned char)((1000-5) >> 8) ; // 5 % on D4 GREEN
  TIM2->CCR1L = (unsigned char)(1000-5) & 0xFF ;    

  TIM2->CCR2H = (unsigned char)((1000-5) >> 8) ; // 5 % on D3 BLUE
  TIM2->CCR2L = (unsigned char)(1000-5) & 0xFF ;

  TIM2->CCR3H = (unsigned char)((1000-5) >> 8) ; // 5 % on A3 RED
  TIM2->CCR3L = (unsigned char)(1000-5) & 0xFF ;

  TIM2->CR1  |= 0x80;  // AutoReload
  TIM2->CR1  |= 0x01;  // Timer
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
  //UART1_HalfDuplexCmd(ENABLE);
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
  if ( delta > 100 && delta <= 450 )          // Is it a "0"?
  {
    if ( nsync > 5 )                          // Syncs seen?
    {
      bytefound = bytefound << 1 ;            // Shift zero bit in
      bitsfound++ ;                           // Count number of bits seen
    }
  }
  else if ( delta > 450 && delta <= 800 )     // Is it a "1"?
  {
    if ( nsync > 5 )                          // Syncs seen?
    {
      bytefound = ( bytefound << 1 ) + 1 ;    // Shift one bit in
      bitsfound++ ;                           // Count number of bits seen
    }
  }
  else if ( delta > 800 && delta < 1500 )     // Is it a sync?
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


void do_red ( uint16_t val )
{
  val = ( 255 - val ) * 4 ;
  if ( val > 1000 )
  {
    val = 1000 ;
  }
  TIM2->CCR3H = (uint8_t)(val >> 8) ;
  TIM2->CCR3L = (uint8_t)(val & 0xFF) ;
}
 

void do_blue ( uint16_t val )
{
  val = ( 255 - val ) * 4 ;
  if ( val > 1000 )
  {
    val = 1000 ;
  }
  TIM2->CCR2H = (uint8_t)(val >> 8) ;
  TIM2->CCR2L = (uint8_t)(val & 0xFF) ;
}


void do_green ( uint16_t val )
{
  val = ( 255 - val ) * 4 ;
  if ( val > 1000 )
  {
    val = 1000 ;
  }
  TIM2->CCR1H = (uint8_t)(val >> 8) ;
  TIM2->CCR1L = (uint8_t)(val & 0xFF) ;
}


//******************************************************************************
//                              D O _ L E D S                                  *
//******************************************************************************
// Stuur de drie kleuren aan volgens PWM waarden uit bericht van IR of RF.     *
//******************************************************************************
void do_leds(void)
{
  do_red   ( pwm_red ) ;
  do_green ( pwm_green ) ;
  do_blue  ( pwm_blue ) ;
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
//                      H O O F D P R O G R A M M A                            *
//******************************************************************************
void main(void)
{
  int16_t i ;
  char    C ;                                       // Color in serial command
  uint8_t  dleng ;                                  // Data length received
  int16_t sw_off_timer = 0 ;                        // Timer voor meten inactiviteit

  clk_and_gpio_init() ; 
  InitTimer2_PWM() ;
  serial_init() ;                                   // Initialize serial port
  enableInterrupts() ;                              // Enable interrupts  
  printf ( "Starting...\n" ) ;                      // Yes, show it
  while ( TRUE )
  {
    if ( newrecvres )                               // New data seen from VS1838B?
    {
      printf ( "IR Buf is " ) ;                     // Ja, toon voor debug
      for ( i = 0 ; i < RECVSIZ ; i++ )
      {
        printf ( "%02X ", (int)recvbuf[i] ) ;
      }
      printf ( "\n" ) ;
      if ( check_checksum() )                       // Yes, check the checksum
      {
        pwm_red   = recvbuf[0] ;                    // Pak PWM waarde rood
        pwm_green = recvbuf[1] ;                    // Pak PWM waarde groen
        pwm_blue  = recvbuf[2] ;                    // Pak PWM waarde blauw
        if ( pwm_red | pwm_green | pwm_blue )       // Tenminste iets verlicht?
        {
          sw_off_timer = 0 ;                          // Ja, herstart inactiviteit timer
        }
        clear_recvbuf() ;
        GPIO_WriteLow ( GPIOB, GPIO_PIN_5 ) ;       // Message received, show activity
        do_leds() ;                                 // Stuur LEDs aan
        delay_ms ( 200 ) ;
        GPIO_WriteHigh ( GPIOB, GPIO_PIN_5 ) ;      // Message received, show activity
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
        do_red ( i ) ;
      }
      else if ( C == 'G' )
      {
        do_green ( i ) ;
      }
      else if ( C == 'B' )
      {
        do_blue ( i ) ;
      }
    }
    if ( sw_off_timer++ > 12000 )                   // 2 minuten zonder bericht?
    {
      pwm_red   = 0 ;                               // LEDs uit
      pwm_green = 0 ;
      pwm_blue  = 0 ;
      do_leds() ;                                   // Stuur LEDs aan
      sw_off_timer = 0 ;
    }
    delay_ms ( 10 ) ;
  }
}
