// This is a modified and simplified version of wiring.c, only for an ATMega328P 
// running at 8 Mhz, to make millis() be accurate to 1 msec instead of 2+ msec.
// (This breaks analogWrite() on pins 5 and 6, because that also uses timer0.)
//   L. Shustek, 4/29/2020

#if F_CPU != 8000000
   This is only for 8 Mhz
#endif

#ifndef __AVR_ATmega328P__
   This is only for AtMega328P
#endif

/*
   wiring.c - Partial implementation of the Wiring API for the ATmega8.
   Part of Arduino - http://www.arduino.cc/

   Copyright (c) 2005-2006 David A. Mellis

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General
   Public License along with this library; if not, write to the
   Free Software Foundation, Inc., 59 Temple Place, Suite 330,
   Boston, MA  02111-1307  USA
*/

#include "wiring_private.h"

// the prescaler is set so that timer0 ticks every 64 8 Mhz clock cycles, and the
// overflow handler is now called every 125 ticks, or precisely every 1 msec.

#define TOP_LIMIT 124  // 0..124 is 125 states

volatile unsigned long timer0_micros = 0;
volatile unsigned long timer0_millis = 0;

 //  extern bool starting_test; // TEMP
ISR(TIMER0_COMPA_vect) { // Timer0 compare match A interrupt
   timer0_millis += 1; // register one more msec and 1000 more usecs
   timer0_micros += 1000; 
   //PINB = 0x04; // TEMP toggle debug transmit
   }

unsigned long millis() {
   unsigned long m;
   uint8_t oldSREG = SREG;
   // disable interrupts while we read timer0_millis or we might get an
   // inconsistent value (e.g. in the middle of a write to timer0_millis)
   cli();
   m = timer0_millis;
   SREG = oldSREG;
   return m; }

unsigned long micros() {
   unsigned long m;
   uint8_t oldSREG = SREG, t;
   cli(); // disable interrupts
   m = timer0_micros;
   t = TCNT0; // tick progress, each 8 usec, towards the next overflow
   if ((TIFR0 & _BV(OCF0A)) && (t < TOP_LIMIT))
      m += 1000; // an interrupt is pending but not yet taken
   SREG = oldSREG; // restore interrupts
   return m + ((unsigned int)t << 3); }

void delay(unsigned long ms) {
   #if 1 //the good (accurate) code
   uint32_t start = micros();
   while (ms > 0) {
      yield();
      while ( ms > 0 && (micros() - start) >= 1000) {
         ms--;
         start += 1000; } } }
   #else //the crude code (off by as much as 1 msec) that doesn't depend on micros()
   uint32_t start = millis();
   while (ms > 0) {
      yield();
      while (ms > 0 && (millis() - start) >= 1) {
         ms--;
         start += 1; } } }
   #endif

/* Delay for the given number of microseconds.  Assumes an 8 MHz clock. */
void delayMicroseconds(unsigned int us) {
   // call = 4 cycles + 2 to 4 cycles to init us(2 for constant delay, 4 for variable)

   // calling avrlib's delay_us() function with low values (e.g. 1 or
   // 2 microseconds) gives delays longer than desired.
   //delay_us(us);

   // for an 8 MHz clock

   // for a 1 and 2 microsecond delay, simply return.  the overhead
   // of the function call takes 14 (16) cycles, which is 2us
   if (us <= 2) return; //  = 3 cycles, (4 when true)

   // the following loop takes 1/2 of a microsecond (4 cycles)
   // per iteration, so execute it twice for each microsecond of
   // delay requested.
   us <<= 1; //x2 us, = 2 cycles

   // account for the time taken in the preceeding commands.
   // we just burned 17 (19) cycles above, remove 4, (4*4=16)
   // us is at least 6 so we can substract 4
   us -= 4; // = 2 cycles

   // busy wait
   __asm__ __volatile__ (
      "1: sbiw %0,1" "\n\t" // 2 cycles
      "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
   );
   // return = 4 cycles
}

void init() {   // this needs to be called before setup()

   cli(); // disable interrupts

   // setup Timer0 for millis() interrupts at exactly 1.000 msec
   TCCR0B = 0;
   TCCR0A = 0;
   TCNT0 = 0; // zero count to avoid bogus wraparound
   OCR0A = TOP_LIMIT;  // count to 125, then restart and interrupt
   TIMSK0 = 0b00000010;  // enable OCRA interrupt
   TCCR0A = 0b00000010;  // no OCRA/B pin output; mode 2: CTC
   TCCR0B = 0b00000011;  // mode 2: CTC; clk/64 prescaling; GO!

   TCCR1B = 0; // turn off Timer1 and Timer2
   TCCR1A = 0;
   TCCR2B = 0;
   TCCR2A = 0;

   sbi(ADCSRA, ADPS2);  // set a2d prescaler so we are inside the desired 50-200 KHz range.
   sbi(ADCSRA, ADPS1);
   cbi(ADCSRA, ADPS0);
   sbi(ADCSRA, ADEN);    // enable a2d conversions

   // the bootloader connects pins 0 and 1 to the USART; disconnect them
   // here so they can be used as normal digital i/o; they will be
   // reconnected in Serial.begin()
   #if defined(UCSRB)
   UCSRB = 0;
   #elif defined(UCSR0B)
   UCSR0B = 0;
   #endif

   sei(); // enable interrupts
}
