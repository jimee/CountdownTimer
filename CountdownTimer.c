//******************************************************************************
// MSP430F20xx Demo - LFXT1 Oscillator Fault Detection
//
// Description: System runs normally in LPM3 with WDT timer clocked by
// 32kHz ACLK with a 1x4 second interrupt. P1.0 is normally pulsed every
// second inside WDT interrupt. If an LFXT1 oscillator fault occurs,
// NMI is requested forcing exit from LPM3. P1.0 is toggled rapidly by software
// as long as LFXT1 oscillator fault is present. Assumed only LFXT1 as NMI
// source - code does not check for other NMI sources.
// ACLK = LFXT1 = 32768, MCLK = SMCLK = Default DCO
//
// //*External watch crystal on XIN XOUT is required for ACLK*//
//
//
// MSP430F20xx
// ---------------
// /|\| XIN|-
// | | | 32kHz
// --|RST XOUT|-
// | |
// | P1.0|-->LED
//
// M. Buccini / L. Westlund
// Texas Instruments Inc.
// September 2005
// Built with CCE Version: 3.2.0 and IAR Embedded Workbench Version: 3.40A
//******************************************************************************
//
// Countdown timer with multiplexed up-down-go/stop button selectors
// probably better with interrupts than polled
// 2 buttons
// off/off: no action
// on /off: down
// off/on:  up
// on /on:  go/stop/off
// 
// quacking alarm, version 2
// initiated by output low to base of PNP transistor

#include "msp430g2231.h"

#ifdef MSP430
#else
  #define TASSEL__ACLK	TASSEL_1
  #define MC__UP	MC_1 
  typedef unsigned char	uint8_t;
  typedef unsigned int	uint16_t;
  typedef int  		int16_t;
#endif

#define SR_CLOCK  BIT0
#define SR_DATA   BIT1
#define SR_POWER  BIT2	// output high turns on display, pullup high turns off
#define BUTTON1   BIT1	// the only problem with aliasing this with SR_DATA is that the display turns off when you press it
#define BUTTON2   BIT3
#define DIGIT_0_P1	BIT4
#define DIGIT_1_P1	BIT5
#define DIGIT_2_P1	BIT6
#define DIGIT_3_P1	BIT7
#define QUACK     BIT2 // output low initiates alarm

#define SEG_A_SR	BIT2
#define SEG_B_SR	BIT0
#define SEG_C_SR	BIT6
#define SEG_D_SR	BIT4
#define SEG_E_SR	BIT3
#define SEG_F_SR	BIT1
#define SEG_G_SR	BIT7
#define SEG_d_SR	BIT5

// calcuate number of segments on individual digits, letters show
// will use to decide how long a digit / letter stays "on"
#define SEGS_STAY(v) \
  (((v & (1<<6)) ? 1 : 0) +\
   ((v & (1<<5)) ? 1 : 0) +\
   ((v & (1<<4)) ? 1 : 0) +\
   ((v & (1<<3)) ? 1 : 0) +\
   ((v & (1<<2)) ? 1 : 0) +\
   ((v & (1<<1)) ? 1 : 0) +\
   ((v & (1<<0)) ? 1 : 0))
#define SEGS_STAY_ALT(v) ((SEGS_STAY(v)-1)>>1)
  
// macro magic
// what the shift register outputs for individual digits / letters
// we do this at compile time so that we don't need to use runtime cycles
// to map segment and port pins during runtime
#define SEGS_SR_DET(v) \
  (((v & (1<<6)) ? SEG_A_SR : 0) |	\
   ((v & (1<<5)) ? SEG_B_SR : 0) |	\
   ((v & (1<<4)) ? SEG_C_SR : 0) |	\
   ((v & (1<<3)) ? SEG_D_SR : 0) |	\
   ((v & (1<<2)) ? SEG_E_SR : 0) |	\
   ((v & (1<<1)) ? SEG_F_SR : 0) |	\
   ((v & (1<<0)) ? SEG_G_SR : 0))
#define SEGS_SR(v)	{(uint8_t)SEGS_STAY(v),(uint8_t)SEGS_SR_DET(v)}

// shift register pins used to turn segments on, led anodes
#define SEGS (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G)
  
// port 1 pins used turn digits on, led cathodes
#define DIGITS_P1 ((uint8_t)DIGIT_0_P1|(uint8_t)DIGIT_1_P1|(uint8_t)DIGIT_2_P1|(uint8_t)DIGIT_3_P1)

// port 1 pins used for buttons (pullup high, activate low)
#define BUTTONS ((uint8_t)BUTTON1|(uint8_t)BUTTON2)
#define STOP_GO BUTTONS // BUTTONS
  
static const uint8_t digit2P1[] = 
{ 
  (uint8_t)DIGIT_0_P1, (uint8_t)DIGIT_1_P1, (uint8_t)DIGIT_2_P1, (uint8_t)DIGIT_3_P1
};

// composition of digits and letters we need
//_____________________ abc defg
#define LTR_0 0x7e	// 0111 1110
#define LTR_1 0x30	// 0011 0000
#define LTR_2 0x6d	// 0110 1101
#define LTR_3 0x79	// 0111 1001
#define LTR_4 0x33	// 0011 0011
#define LTR_5 0x5b	// 0101 1011
#define LTR_6 0x5f	// 0101 1111
#define LTR_7 0x70	// 0111 0000
#define LTR_8 0x7f	// 0111 1111
#define LTR_9 0x7b	// 0111 1011
#define LTR_g 0x5e	// 0101 1110
#define LTR_r 0x05	// 0000 0101
#define LTR_u 0x1c	// 0001 1100
#define LTR_b 0x1f	// 0001 1111
#define LTR_p 0x67	// 0110 0111
#define BLANK 0x00	// 0000 0000

// 1st byte cycles to stay
// 2nd byte shift register value
static const uint8_t digit2SR[][2] = 
{ 
  SEGS_SR(LTR_0), SEGS_SR(LTR_1), SEGS_SR(LTR_2), SEGS_SR(LTR_3),
  SEGS_SR(LTR_4), SEGS_SR(LTR_5), SEGS_SR(LTR_6), SEGS_SR(LTR_7),
  SEGS_SR(LTR_8), SEGS_SR(LTR_9), SEGS_SR(BLANK)
};

// digits / letters we are using
enum 
{
  POS_0, POS_1, POS_2, POS_3, POS_4, POS_5, POS_6, POS_7,
  POS_8, POS_9, POS__
};

#define HOURS 2
#define MINS  1
#define SECS  0

#define TPS 512 // ticks per second
#define DEBOUNCE_TIME_INITIAL 80 // 152 ms 
#define DEBOUNCE_TIME_SUBSEQUENT 40 // 76 ms 
#define DEBOUNCE_TIME_STOP_GO 240 // 456 ms 
#define TIMEOUT 600 // 10 minutes (600 seconds) to set timer then sleep

#define ABS(x) (((x)>=0)?(x):(-x))
#define SIGN(x)(((x)>0)?(+1):(((x)<0)?(-1):0))

volatile uint16_t i;
volatile uint16_t ticks;
volatile uint16_t timeout;
volatile uint8_t time[ 3 ]; // hours/mins/secs
volatile uint8_t mins;
volatile uint8_t hours;
volatile uint8_t alarm;
volatile uint8_t buffer[ 4 ];
volatile uint8_t digit;
volatile uint8_t stays;
volatile uint8_t button_press;
volatile uint8_t debounce;
volatile uint8_t debounce_time;

// Function Definitions
void port1_init ( void );
void timer_reset ( void );
void timer_wind_up ( void );
void update_buffer ( uint8_t dot );
uint8_t set_timer ( void );
void update_display ( void );
void send_SR ( uint8_t byte );
void P1_setif ( uint8_t boolean, uint8_t mask );
void poll_buttons ( void );
void timer_wind_down ( void );
void update_time ( uint8_t unit, int offset );
uint8_t count_down ( void );
uint8_t time0 ( void );
void blank_buffer ( void );

void main ( void )
{
  // crystal Cload capacitor adjustment - only one line of next four should be uncommented
     BCSCTL3 = LFXT1S_0 + XCAP_3; // 32768KHz crystal, 12.5 pF
  // BCSCTL3 = LFXT1S_0 + XCAP_2; // 32768KHz crystal, 10 pF
  // BCSCTL3 = LFXT1S_0 + XCAP_1; // 32768KHz crystal, 6 pF
  // BCSCTL3 = LFXT1S_0 + XCAP_0; // 32768KHz crystal, 1 pF  
  
  // Set up interrupt on Port 1 for button press to wake CPU
  port1_init();
  timer_reset();
  do
  { 
    timer_wind_up();
    if ( set_timer() || count_down() )
    {  
      // timers and timer interrupts off      
      timer_reset();
      timer_wind_down();
      port1_init();
      __bis_SR_register( CPUOFF + GIE ); // sleep in LPM0 until button press interrupt
    }  
  }  
  while ( 1 );
}

void port1_init ( void ) // set up port 1 for interrupt on button press
{  
  P1IE &= ~BUTTONS; // disable port 1 interrupt for button
  P1DIR = (uint8_t) ~( BUTTONS + QUACK ); // Port 1 button input, others output
  P1OUT = BUTTONS + QUACK; // Port 1 button reset high, others low
  P1REN = BUTTONS + QUACK; // pullup resistors enabled on button
  P1IES = BUTTONS; // high-to-low transition on port 1 interrupt 
  P1IFG &= (uint8_t) ~BUTTONS; // interrupt flags cleared
  P1IE = BUTTONS; // enable port 1 interrupt for buttons
}    

void timer_reset ( void )
{
  time[ SECS ] = 00; // max 59
  time[ MINS ] = 10; // max 59
  time[ HOURS ] = 00; // max 99
}  

void timer_wind_up ( void ) // initialize timer, interrupts etc
{
  // disable port 1 interrupt for button
  // we'll poll it here
  P1IE &= ~BUTTONS; 
  
  // WDT timer for display refresh
  WDTCTL = WDT_ADLY_1_9; // WDT 1.9ms interval timer for display refresh
  IE1 |= WDTIE; // Enable WDT interrupt 

  // initialize timer and display
  update_buffer( 1 );
  button_press = 0;
  debounce = 0;
  debounce_time = DEBOUNCE_TIME_INITIAL;
  digit = 1;
  timeout = 0;
  
  P1DIR |= SR_POWER; // SR power on (output)
  P1OUT |= SR_POWER; // SR power on (high)
  P1REN &= ~SR_POWER; // SR power on (pullup resistor disabled)
}

void update_buffer ( uint8_t dot ) // time[0]=secs, time[1]=mins, time[2]=hours
{
  uint16_t tens;
  if ( time[ HOURS ] == 0 && time[ MINS ] == 0 ) // time[ MINS ] < 10
  {
    buffer[ 0 ] = digit2SR[ POS__ ][ 1 ]; 
    buffer[ 1 ] = ( time[ MINS ] == 0 ) ? digit2SR[ POS__ ][ 1 ] : digit2SR[ time[ MINS ] ][ 1 ]; 
    tens = ( ( ( time[ SECS ] + 1 ) * 51 ) >> 9 ); // secs div 10 using dirty maths    
    buffer[ 2 ] = digit2SR[ (uint8_t) tens ][ 1 ];
    buffer[ 3 ] = digit2SR[ time[ SECS ] - (uint8_t) ( 10 * tens ) ][ 1 ];
  } else
  {
    tens = ( ( ( time[ HOURS ] + 1 ) * 51 ) >> 9 ); // hours div 10 using dirty maths
    buffer[ 0 ] = ( tens == 0 ) ? digit2SR[ POS__ ][ 1 ] : digit2SR[ (uint8_t) tens ][ 1 ];
    buffer[ 1 ] = digit2SR[ time[ HOURS ] - (uint8_t) ( 10 * tens ) ][ 1 ];  
    tens = ( ( ( time[ MINS ] + 1 ) * 51 ) >> 9 ); // mins div 10 using dirty maths
    buffer[ 2 ] = digit2SR[ (uint8_t) tens ][ 1 ];
    buffer[ 3 ] = digit2SR[ time[ MINS ] - (uint8_t) ( 10 * tens ) ][ 1 ];
  }
  if ( dot ) 
  {
    buffer[ 1 ] |= SEG_d_SR; // show dot
  } else
  {
    buffer[ 1 ] &= ~SEG_d_SR; // don't show dot
  }  
  /*
  if ( debounce == 0 ) 
  {
    buffer[ 0 ] |= SEG_d_SR; // show dot
  } else
  {
    buffer[ 0 ] &= ~SEG_d_SR; // don't show dot
  } 
  */
}  

uint8_t set_timer ( void )
{  
  uint8_t go = 0;
  do // set timer
  {    
    if ( !ticks )
    {
      ticks = TPS;
      timeout++;
    }  
    update_display();    
    poll_buttons();
    if ( button_press == STOP_GO ) // go
    {
      go++;
      /*
      button_press = 0;
      update_time( 2, -1 );
      update_buffer( 1 );
      continue;
      */
    } else if ( button_press == BUTTON1 ) // down
    {
      update_time( MINS, -1 );
      button_press = 0;
      debounce_time = DEBOUNCE_TIME_SUBSEQUENT;
      update_buffer( 1 );
      continue;
    } else if ( button_press == BUTTON2 ) // up
    {
      update_time( MINS, +1 );
      button_press = 0;
      debounce_time = DEBOUNCE_TIME_SUBSEQUENT;
      update_buffer( 1 );
      continue;
    } else if ( !debounce )
    {
      button_press = 0;
      debounce_time = DEBOUNCE_TIME_INITIAL;
      //update_buffer( 1 );
    }  
    _BIS_SR( LPM3_bits + GIE ); // Enter LPM3 w/interrupt
  }
  while ( ( timeout < TIMEOUT ) && !( go && !debounce ) ); // set timer
  return !go; // 0 = start count down, 1 = timed out
}

void update_display ( void ) // hh.mm
{
  if ( stays ) 
  {
    stays--;
  } else
  {      
    uint8_t temp1, temp2;  
    temp1 = digit2P1[ digit ];
    P1OUT |= temp1; // current digit off
    P1DIR &= (uint8_t) ~temp1; // current digit output disabled
    P1REN |= temp1; // current digit pullup resistor enabled  
    digit = ( digit - 1 ) & 0x03; // next digit
    temp1 = digit2P1[ digit ];
    temp2 = buffer[ digit ];    
    send_SR( temp2 ); // Send digit to SR for display  
    P1REN &= (uint8_t) ~temp1; // current digit pullup resistor disabled
    P1DIR |= temp1; // enable output for current digit
    P1OUT &= (uint8_t) ~temp1; // output digit
    stays = digit2SR[ digit ][ 0 ] + ( temp2 & SEG_d_SR ) ? 1 : 0;   
    stays = ( stays <= 2 ) ? 0 : ( stays >> 1 ); // else flickers    
  }  
}  
             
// send byte to SR LSB first
void send_SR ( uint8_t byte )
{
  uint8_t bits = 8;
  P1DIR |= (uint8_t) SR_DATA; // output
  P1REN &= ~SR_DATA; // pullup resistors disabled 
  do
  {  
    P1OUT &= ~SR_CLOCK; 
    P1_setif( byte & 0x80, SR_DATA ); // get next bit
    byte <<= 1;    
    P1OUT |= SR_CLOCK; // send bit  
  }
  while ( --bits );
}

inline void P1_setif ( uint8_t boolean, uint8_t mask )
{
  if ( boolean )
  {
    P1OUT |= mask;
  } else 
  {
    P1OUT &= (uint8_t) ~mask;
  }  
}

void poll_buttons ( void )
{
  if ( !debounce )
  {
    P1DIR &= (uint8_t) ~BUTTONS; // input  
    P1OUT |= BUTTONS; // pullup resistors
    P1REN |= BUTTONS; // pullup resistors enabled
    button_press = (uint8_t) ~P1IN & BUTTONS;
    if ( button_press == STOP_GO )
    {  
      debounce = DEBOUNCE_TIME_STOP_GO;
    } else if ( button_press )
    {
      debounce = debounce_time;
    }
  }
}

void update_time ( uint8_t unit, int offset ) // offset = +/- 1
{
  do
  {  
    if ( unit <= MINS ) // secs or mins
    {      
      if ( time[ unit ] == 0 && offset == -1 )
      {
        time[ unit ] = 59;
        unit++;
        continue;
      } else if ( time[ unit ] == 59 && offset == +1 )
      {
        time[ unit ] = 0;
        unit++;
        continue;
      } else
      {
        time[ unit ] += offset;
        break;
      }  
    } else if ( unit == HOURS ) // hours
    {
      if ( time[ HOURS ] == 0 && offset == -1 ) 
      {
        time[ SECS ] = 0;
        time[ MINS ] = 0;
        time[ HOURS ] = 0;
      } else if ( time[ HOURS ] == 99 && offset == +1 ) // max time 99h 00m 00s
      {
        time[ SECS ] = 0;
        time[ MINS ] = 0;
        time[ HOURS ] = 99;
      } else
      {
        time[ HOURS ] += offset;
      }  
      break;
    }
  }
  while ( 1 );  
}

void timer_wind_down ( void ) // shut down timer and prepare for sleep
{
  // WDT timer for display refresh
  WDTCTL = WDT_ADLY_1_9; // WDT 1.9ms interval timer for display refresh
  IE1 &= ~WDTIE; // Disable WDT interrupt
  
  // shut off display
  P1DIR &= ~SR_POWER; // SR power off (input)
  P1OUT |= SR_POWER; // SR power off (pulled up so as not to set off alarm)
  P1REN |= SR_POWER; // SR power off (pullup resistor enabled)
}

uint8_t count_down ( void )
{
  if ( time0() )
  {
    return 0; // back to time setting mode
  }  
  alarm = 0;
  ticks = TPS;
  update_buffer( 1 );
  uint8_t stop = 0;
  uint8_t flash;
  do // count down
  {
    // poll buttons
    // return to timer setting mode if stop/go button pressed 
    poll_buttons();
    if ( button_press == STOP_GO ) // stop
    {
      stop++; // go back to time setting mode
    } 
    update_display(); 
    if ( alarm == 0 )
    {
      if ( ticks == 0 )
      {
        ticks = TPS;
        update_time( 0, -1 );
        update_buffer( 1 ); 
        if ( time0() )
        {
          alarm++;  // set alarm flag
          P1DIR |= QUACK; // initiate alarm signal
          P1OUT &= (uint8_t) ~QUACK; // alarm signal on (0V) for 250 ms
          flash = 12;           
        }   
        continue;        
      } else if ( ticks == ( TPS >> 1 ) )
      {
        update_buffer( 0 ); // toggle dot every 500 ms
        continue;
      }  
    } else if ( alarm == 1 ) 
    {  
      if ( ticks == 0 )
      {
        ticks = TPS;
        if ( flash-- == 0 )
        {
          alarm++; // now go to sleep
        }        
        update_buffer( 1 );
        continue;
      } else if ( ticks == ( TPS >> 1 ) ) // flash .00 during quack
      {  
        blank_buffer();
        continue;
      } else if ( !( P1OUT & QUACK ) && ( ticks == ( TPS >> 1 ) + ( TPS >> 2 ) ) )
      {  
        P1OUT |= QUACK; // Alarm signal off (3V)	
        P1DIR &= (uint8_t) ~QUACK; 
        continue;
      } 
    } 
    _BIS_SR( LPM3_bits + GIE ); // Enter LPM3 w/interrupt
  }
  while( ( alarm < 2 ) && !( stop && !debounce ) ); // count down 
  if ( alarm == 2 ) // finished
  {
    blank_buffer();
    return 1; // enter LPM0 sleep and wait for button press
  } else
  {  
    return 0; // return to time setting mode
  }  
} 

uint8_t time0 ( void )
{
  return time[ SECS ] == 0 && time[ MINS ] == 0 && time[ HOURS ] == 0;
}  

void blank_buffer( void )
{
  uint8_t j;
  for ( j = 4; j; )
  {  
    buffer[ --j ] = digit2SR[ POS__ ][ 1 ]; // blank out display         
  }  
}  

// Watchdog Timer interrupt
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer ( void )
{ 
  ticks--;
  if ( debounce ) 
  {
    debounce--;
  }  
  _BIC_SR_IRQ( LPM3_bits ); // wake up CPU from LPM3
}

// Port interrupt for button press
// Wakes up CPU to enter main loop
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR ( void )
{  
  P1IE &= (uint8_t) ~BUTTONS; // Turn off Port1 interrupt during debounce
  button_press = P1IFG;
  P1IFG &= (uint8_t) ~BUTTONS; // Clear port interrupt flag
  //P1REN &= (uint8_t) ~BUTTONS; // Disable pullup resistor
  debounce = DEBOUNCE_TIME_STOP_GO; // Wait before next button press
  _BIC_SR_IRQ( CPUOFF + GIE ); // wake up CPU from LPM3
}
