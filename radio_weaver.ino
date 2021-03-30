/*
 *   Radio using qrp-labs receiver module, arduino shield, bandpass and lowpass filters,  and a chipkit uc32
 *   
 *   Running I2C at 5 volts because of the way the qrp-labs SI5351 module is wired. (Could be modified if desired)
 */


// I2C OLED 128x64 display
#include <OLED1306_Basic.h>

#define ROW0 0          // text based rows for the 128x64 OLED
#define ROW1 8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40
#define ROW6  48
#define ROW7  56

OLED1306 LCD;            // a modified version of LCD_BASIC by Rinky-Dink Electronics

extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];

// I2C
#define I2BUFSIZE 256
unsigned int i2buf[I2BUFSIZE];
int i2in, i2out;

// SI5351
#define CLOCK_FREQ   2700449800LL   // *100 for setting fractional frequency
uint32_t freq = 7100000L;
uint32_t divider = 28;              // divider for 4 x freq
#define SI5351   0x60    // i2c address
#define PLLA 26          // register address offsets for PLL's
#define PLLB 34
#define CLK0_EN   1
#define CLK1_EN   2
#define CLK2_EN   4


void setup() {
int i;

  i2init();

  LCD.InitLCD();                        // using a modified Nokia library for the OLED
  LCD.setFont(SmallFont);
  LCD.clrScr();
  LCD.print("Weaver mode SDR",0,ROW0);
  i2flush();
  delay(2000);


  // set up SI5351 
  i2cd( SI5351, 3, 0xff );  // disable outputs
  for( i = 16; i < 24; ++i ) i2cd( SI5351, i, 0x80 );  // power down output drivers
  i2flush();

  i2cd(SI5351,16,0x4f);   // clock 0, PLLA
  i2cd(SI5351,17,0x6f);   // clock 1, PLLB
  i2cd(SI5351,18,0x6f);   // clock 2, PLLB

  // set some divider registers that will never change
  for(i = 0; i < 3; ++i ){
    i2cd(SI5351,42+8*i,0);
    i2cd(SI5351,43+8*i,1);
    i2cd(SI5351,47+8*i,0);
    i2cd(SI5351,48+8*i,0);
    i2cd(SI5351,49+8*i,0);
  }


  si_pll_x(PLLA,4*freq,divider,0);    // receiver 4x clock on clock 0
  si_load_divider(divider,0,0,1);
  si_pll_x(PLLB,4*freq,divider,0);     // tx on clocks 1 and 2
  si_load_divider(divider,1,0,4);     // at 1/4 of the rx freq using the R divider as 4
 // phase 0 register 165, phase 1 register 166, phase 2 167
  i2cd(SI5351,166,0);
  i2cd(SI5351,167,divider*4);           // ?? 90 degreee phase offset 
  si_load_divider(divider,2,1,4);    // load divider for clock 1 and reset pll's
  
  // i2cd(SI5351,3,0xff ^ ( CLK0_EN) );   // turn on receiver clock
   i2cd(SI5351,3,0xff ^ (CLK0_EN + CLK1_EN + CLK2_EN));   // testing only, all on
   i2flush();
   delay(1000);
   i2cd( SI5351, 177, 0xAC );         // reset again as sometimes do not start correctly


}

void loop() {

  i2poll();

}



/*  I2C write only implementation using polling of the hardware registers */
/*  functions do not block */
/*  call i2poll() in loop() to keep everything going */

// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200

void i2init(){
    // start up our local I2C routines
  I2C1CONSET = 0x8000;      // I2C on bit
  I2C1BRG = 90;             // 400 khz
  //i2stop();                 // clear any extraneous info that devices may have detected on powerup
}

void i2start( unsigned char adr ){
unsigned int dat;
  // shift the address over and add the start flag
  dat = ( adr << 1 ) | ISTART;
  i2send( dat );
}

void i2send( unsigned int data ){   // just save stuff in the buffer
int next;

  next = (i2in + 1) & (I2BUFSIZE - 1);
  while( i2out == next ) i2poll();
  i2buf[i2in++] = data;
  i2in &= (I2BUFSIZE - 1);
  i2poll();
}

void i2stop( ){
   i2send( ISTOP );   // que a stop condition
}


void i2flush(){  // needed during setup, call poll to empty out the buffer.  This one does block.

  while( i2poll() ); 
}

int i2poll(){    // everything happens here.  Call this from loop.
static int state = 0;
static int data;
//static int delay_counter;

//   if( delay_counter ){   // the library code has a delay after loading the transmit buffer and before the status bits are tested for transmit active
//     --delay_counter;
//     return (16 + delay_counter);
//   }
   
   switch( state ){    
      case 0:      // idle state or between characters
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2BUFSIZE - 1 );
           
           if( data & ISTART ){   // start
              data &= 0xff;
              // set start condition
              I2C1CONSET = 1;
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              I2C1CONSET = 4;
              state = 3;
           }
           else{   // just data to send
              I2C1TRN = data;
              //delay_counter = 1;   // delay for transmit active to come true
              state = 2;
           }
        }
      break; 
      case 1:  // wait for start to clear, send saved data which has the address
         if( (I2C1CON & 1) == 0  ){
            state = 2;
            I2C1TRN = data;
            //delay_counter = 1;
         }
      break;
      case 2:  // wait for ack/nack done and tbuffer empty, blind to success or fail
         if( (I2C1STAT & 0x4001 ) == 0){  
            state = 0;
         }
      break;
      case 3:  // wait for stop to clear
         if( (I2C1CON & 4 ) == 0 ){
            state = 0;
            //delay_counter = 1;  // a little delay just for fun at the end of a sequence
         }
      break;    
   }
   
   if( i2in != i2out ) return (state + 8);
   else return state;
}


/******   SI5351  functions   ******/

void i2cd( unsigned char addr, unsigned char reg, unsigned char dat ){
  // direct register writes.  A possible speed up could be realized if one were
  // to use the auto register inc feature of the SI5351

   i2start(addr);
   i2send(reg);
   i2send(dat);
   i2stop();
}

void  si_pll_x(unsigned char pll, uint32_t freq, uint32_t out_divider, uint32_t fraction ){
 uint64_t a,b,c;
 uint64_t bc128;             // floor 128 * b/c term of equations
 uint64_t pll_freq;
 uint64_t cl_freq;

 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3
 uint64_t r;

   cl_freq = CLOCK_FREQ;
   
   //if( pll == PLLA ) cl_freq += drift;               // drift not applied to 3 mhz calibrate freq
   //cl_freq += drift;                                 // drift applied to 3 mhz.  Pick one of these.
   
   c = 1000000;     // max 1048575
   pll_freq = 100ULL * (uint64_t)freq + fraction;    // allow fractional frequency for wspr
   pll_freq = pll_freq * out_divider;
   a = pll_freq / cl_freq ;
   r = pll_freq - a * cl_freq ;
   b = ( c * r ) / cl_freq;
   bc128 =  (128 * r)/ cl_freq;
   P1 = 128 * a + bc128 - 512;
   P2 = 128 * b - c * bc128;
   if( P2 > c ) P2 = 0;        // avoid negative numbers 
   P3 = c;

   i2cd(SI5351, pll + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 1, (P3 & 0x000000FF));
   i2cd(SI5351, pll + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, pll + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 4, (P1 & 0x000000FF));
   i2cd(SI5351, pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, pll + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 7, (P2 & 0x000000FF));
   
 //  i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset  
}


// load new divider for specified clock, reset PLLA and PLLB if desired
void si_load_divider( uint32_t val, uint8_t clk , uint8_t rst, uint8_t Rdiv){
uint8_t R;

   R = 0;
   Rdiv >>= 1;      // calc what goes in the R divisor field
   while( Rdiv ){
      ++R;
      Rdiv >>= 1;
   }
   R <<= 4;     
   
   val = 128 * val - 512;
   i2cd( SI5351, 44+8*clk, ((val >> 16 ) & 3) | R );
   i2cd( SI5351, 45+8*clk, ( val >> 8 ) & 0xff );
   i2cd( SI5351, 46+8*clk, val & 0xff );   
   if( rst ) i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset needed
}
