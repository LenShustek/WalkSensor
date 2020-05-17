/***************************************************************************************************

    Battery-powered walk path sensor

   This is the software for a small (4.5" x 3" 1") battery-powered wireless sensor
   that, when used in a pair on opposite sides of a walking path, detects someone
   walking through an IR beam between the sensors. It then sends a Bluetooth alert
   to a mobile phone or tablet.

   This is for version 3, which uses SMD parts.

   The custom-built sensor hardware consist of:
   - AtMega 328P processor, similar to what is in an Arduino UNO
   - Microchip RN4870 Bluetooth LE module
   - 940nm IR LED transmitter powered by a constant-current driver
   - 940nm 38 Khz tuned IR receiver
   - red/green battery status light
   - blue and yellow operational status lights
   - power on/off pushbutton
   - program-controlled power supply circuit
   - battery condition sensor
   - 9V alkaline battery (500 mAh)

   approximate power utilization:
    - when idle: 10 uA, so 4+ years shelf life
    - as an IR transmitter: 6.7 ma, so 60 hours of continuous use (of 80% capacity)
    - as an IR receiver: 6.4 ma, so 62 hours of continuous use (of 80% capacity)

   Operation
    power:
      push the button to turn it on
        all three lights flash
        if battery is marginal, the red light flashes every 5 seconds
        if the battery is too low, all lights flash 5 times and the unit shuts off
      push the button to turn it off
        all three lights flash once, and the unit shuts off
      after 1 hour of no activity, the unit turns off automatically
   mode: when the unit turns on, it tries to detect an opposing IR transmitter
    if no IR signal detected, it becomes the IR transmitter
      - the yellow light flashes every 2 seconds
    if an IR signal is detected: it becomes the IR receiver
      - the blue light flashes every 2 seconds
      - we beacon advertise with the name "retrotope-timer(0)nnnn", where
        nnnn is our timestamp in milliseconds, updated every 250 msec
      - when the IR signal is interrupted,
        - the blue light turns on for 5 seconds
        - we advertise with the name "retrotope-timer(1)nnnn", where
          nnnn is the timestamp in milliseconds at the time the beam broke
      -if the beam break lasts more than 5 seconds,
        - continue the same "retrotope-timer(1)nnnn" advertising
        - flash yellow and blue lights obnoxiously

   special startup modes: hold the button for more than 5 seconds
     - all three lights flash in rotation for a couple of seconds
     - the red light flashes N=1,2,3... times, and then waits for the button
       - if the button is pushed after N flashes, it enters mode N:
         - 1: normal mode, except no power down after one hour
         - 2: normal mode, except flash software version and battery voltage first
         - 3: normal mode, except do a factory reset on the Bluetooth module
              (generally requires a power off/on to recover)
         - 4: special timer sync mode: link two units to compare their clocks
              (needs work for V3)

   The design of the hardware and software for the sensors and the corresponding
   receiver is open-source.  See https://github.com/LenShustek/walksensor.

   Tips for programming the "bare" ATMega328P on our PC board:
    - There is only 2K of RAM, so be frugal allocating array variables!
    - A 2x3 pin header mates to the Pololu USB AVR Programmer v2.1,
      configured with their utility program to supply 3.3V to the target board.
      The red cable stripe goes next to the dot.
    - Select the board "Arduino/Genuino Uno".
    - The boards.txt file in C:\Users\len\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.6.21\...
      has to be changed to allow the frequency to be set to other than 16 Mhz.
        #uno.build.f_cpu=1000000L
        ...
        menu.speed=CPU Speed
        uno.menu.speed.1=1 MHz
        uno.menu.speed.2=2 MHz
        uno.menu.speed.4=4 MHz
        uno.menu.speed.8=8 MHz
        uno.menu.speed.16=16 MHz
        uno.menu.speed.1.build.f_cpu=1000000L
        uno.menu.speed.2.build.f_cpu=2000000L
        uno.menu.speed.4.build.f_cpu=4000000L
        uno.menu.speed.8.build.f_cpu=8000000L
        uno.menu.speed.16.build.f_cpu=16000000L
     (may need to uncheck "aggressively cache compiled core" in files/preferences after changing it?)
     (See https://tttapa.github.io/Pages/Arduino/Bootloaders/ATmega328P-custom-frequency.html)
   - Select the "Atmel STK500 development board" programmer in the Aduino IDE tools/programmer menu.
       Then "Upload using programmer"; it only takes about 10 seconds.
   - Use AVRDUDE to check and modify fuse settings to change the clock.
       avrdude -c stk500v2 -P COM7 -p m328p  (show signature and current fuses)
          avrdude: Device signature = 0x1e950f (probably m328p)
          avrdude: safemode: Fuses OK (E:FF, H:D9, L:62)
      The ATMega328p comes with the low fuse preprogrammed as 0x62, which uses the
      internal RC 8 Mhz ocillator divided by 8, producing a system clock of 1 Mhz.
      We change it to use a more accurate external low-power 8 Mz ceramic resonator, divided by 1,
      with startup time of 1K clocks + 4.1 msec, producing a system clock of 8 Mhz.
      So the low fuse should be FE: 1 (no div by 8), 1 (no ext clk), 11 (startup time), 1110 (clk select)
         avrdude -c stk500v2 -P COM7 -p m328p -U lfuse:w:0xfe:m

   ------------------------------------------------------------------------------------------------------
   Copyright (c) 2016,2020 Len Shustek

                                    The MIT License (MIT)
   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
   associated documentation files (the "Software"), to deal in the Software without restriction,
   including without limitation the rights to use, copy, modify, merge, publish, distribute,
   sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or
   substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
   DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
   ------------------------------------------------------------------------------------------------------

  **** Change log ****

   31 Oct 2016, V1.0, L. Shustek, First version.
   16 Nov 2017, V1.1, L. Shustek, various enhancements on the way to production
   16 Dec 2017, V1.2, L. Shustek, experiment with transmitting and receiving simultaneously
   27 Dec 2017, V2.0, L. Shustek, experiment with GAP advertisements
   29 Jan 2017, V2.1, L. Shustek, implement GAP advertising protocol;
    1 Feb 2017, V2.2, L. Shustek, Turn off verbose mode in BLE library.
                                  Add special startup modes if button pushed for more than 5 seconds:
                                    #1: normal mode but with no power-down timeout
                                    #2: special timing mode that links two units to compare clock speed
    4 Feb 2017, V2.3, L. Shustek, Look at additional test input (D0/RX) which, when low, is treated
                                  as a beam break.
   29 Feb 2017, V2.4, L. Shustek, Cosmetic changes. (We abandoned dynamically changing from IR receiver
                                  to IR transmitter or vice versa if we think the other unit is the same.)
   10 May 2020, V3.0, L. Shustek, Version 3, with AtMega328 and Microchip RN4870 Bluetooth module

   TODO:
    - experiment with a lower duty cycle for transmit; can it be as reliable?
 *****************************************************************************************************/
#define MAJOR_VERSION 3
#define MINOR_VERSION 0

#define DEBUG false           // output debugging data on the software serial port?
#define BT_SHOWCMDS false     // if debugging, show we show BT commands and responses?
#define MARKLOCATION false    // mark execution locations for scope tracing?
#define EVENTS false          // record events in a circular buffer for debugging?
#define BLUETOOTH true        // is the the Bluetooth module installed?
#define STOP_IF_BAT_LOW true  // refuse to run with really low battery?
#define USE_SLEEP true        // use power-saving sleep during delay?

#include <Arduino.h>
// This also depends on our modified version of wiring.c that fixes millis() to be exact.
// It should be in this sketch directory.

#if DEBUG
   #include "SoftwareSerial.h" // our local version, which doesn't take over all interrupt vectors,
   // so that we can take over port B pin change interrupt at PCINT0 for the IR receiver.
   // The modified versions of SoftwareSerial.cpp and .h should be in this sketch directory.
   /* Because we might issue BT commands when debugging that produce multiple lines of output,
   we also increase the hardware serial receive buffer from 64 to 256 bytes.
   That requires creating the following file in the directory of the platform.txt that is used:
   C:\Users\len\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.1\platform.local.txt
   with these two lines:
   compiler.c.extra_flags=-DSERIAL_RX_BUFFER_SIZE=256
   compiler.cpp.extra_flags=-DSERIAL_RX_BUFFER_SIZE=256
   And then restarting the Arduino IDE.  */
   #if SERIAL_RX_BUFFER_SIZE != 256
      buffer size has not been increased!
   #endif
#endif

#define stringify(x) stringifier(x)  // for turning #defined macros into strings
#define stringifier(x) #x

//  timing and other parameters
// The parameters marked * affect the quality of our beam recognition algorithm

#define MINOR_CYCLE 5             //* the minor cycle time in msec
#define IR_XMT_PERIOD 5           //* IR transmit cycle time in milliseconds between bursts
#define IR_RCV_PERIOD 50          //* IR receive sample period in milliseconds
#define IR_ON_USEC 850            //* IR burst transmit time in microseconds
// For the Vishay TSOP364 (new kind), the max burst is 35 cycles, or 921 usec
// For the Vishay TSOP362 (legacy kind), the max burst is 70 cycles, or 1.94 msec
#define IR_RCV_WIDTH_MIN_USEC (IR_ON_USEC-132-30)  //* the minimum pulse width we count (30 usec margin)
#define MISSING_PULSE_THRESHOLD 2 //* how many missing pulses constitute a beam break
#define CHK_RCV_TIME 200          //  how long to wait to see if the other guy is sending, in msec
/* With those values the transmitter sends one 0.85 msec burst every 5 msec, and 
   the receiver recognizes a pulse if it's at least 0.68 msec wide. 
   When initializing we look for 200 msec, during which time we expect to see 40 pulses, 
   and we decide that we're a receiver if we receive at least 20.
   When operating we look for 50 msec, during which time we expect to see 10 pulses,
   and we record a break if we don't see at least 8. */

#define RCV_NAMECHANGE_TIME 250   // how often to change our name timestamp when beam is seen, in msec
#define RCV_BREAK_TIME 5000       // how long to stretch the beam break time to, in msec
#define ADVERTISING_INTERVAL_HEX_MSEC "0032" // 50 msec between advertisements

#define XLED_ON 50                // transmit status LED on time in milliseconds
#define XLED_PERIOD 2000          // transmit status LED cycle time in milliseconds
#define RLED_ON 50                // receive status LED on time in milliseconds
#define RLED_PERIOD 2000          // receive status LED cycle time in milliseconds
#define LLED_ON 300               // lost beaam LED on time in milliseconds
#define LLED_PERIOD 600           // lost beam LED cycle time in milliseconds
#define BLED_ON 50                // battery LED on time in milliseconds
#define BLED_PERIOD 5000          // battery LED cycle time in millliseconds
#define BATTERY_CHK_PERIOD 5000   // battery check time in milliseconds

#define IR_FREQ 38000L            // tuned IR receiver frequency
#define POWERDOWN_MINUTES 60      // after how many minutes to power down automatically
#define DEBOUNCE_DELAY 25         // debounce delay in msec

#define BATTERY_WEAK 8000         // millivolt level (out of 9000) to warn about low battery 
#define BATTERY_FAIL 7000         // millivolt level (out of 9000) to treat as battery failure

// hardware configuration

// Note that some of these pins have their digital input buffers disabled
// in the setup code to save power.

/* The multiple schemes for pin numbering is confusing!
   IDE  port  TQFP pin  use
   A0  PC0      23     battery monitor
   A1  PC1      24     yellow LED
   A2  PC2      25     blue LED
   A3  PC3      26     green LED
   A4  PC4      27     red LED
   A5  PC5      28     debug serial receive (must be PC, or PD, not PB)
   --  ADC6     19     (unusable for digital I/O)
   --  ADC7     22     (unusable for digital I/O)
   0   PD0      30     RxD from Bluetooth module
   1   PD1      31     TxD to Bluetooth module
   2   PD2      32     RTS from Bluetooth module  (unused presently)
   3   PD3      1      CTS from Bluetooth module  (unused presently)
   4   PD4      2      Config to Bluetooth module (unused presently)
   5   PD5      9      "button not pushed" input (also OC0B)
   6   PD6      10     "keep power on" output (also OC0A)
   7   PD7      11     - RESET to Bluetooth module
   8   PB0      12     IR receiver input (causes PB0 pin change interrupt)
   9   PB1      13     IR LED transmit (output of Counter / Timer 1)
   10  PB2      14     debug serial transmit
   11  PB3      15     ISP MOSI
   12  PB4      16     ISP MISO
   13  PB5      17     ISP SCK
   --  PC6      29     ISP CPU reset  */

#define POWERUP_FET 6       // output pin high keeps us powered up by turning the MOSFET on
#define POWERDOWN_SW 5      // this input pin (needs internal pullup) is low if the on/off button is pushed

#define LED_YELLOW A1       // LED pins: active high
#define LED_BLUE A2
#define LED_GREEN A3
#define LED_RED A4

#define IR_TRANSMIT 9      // active low output pin for IR transmit through constant-current LED driver
//.                           This is PB1, but more importantly, the OC1A output for Timer/Counter1.
#define IR_RECEIVE 8       // active low input pin if IR receiver sees 10 or more pulses at 38 Khz
//.                           This must be on port B, C, or D to allow pin-change interrupts.

// Microchip RN4870 Bluetooth LE module pin assignments
#define BT_RX 0            // receive data from BT (its transmit)
#define BT_TX 1            // PD1: xmit data to BT (its receive)
#define BT_TX_PORT PORTD          // data port D using normal address for load/store
#define BT_TX_MASK 0x02           // mask for bit 1
#define BT_RESET 7         // active low reset 
#define BT_RTS 2           // (not used)
#define BT_CTS 3           // (not used)
#define BT_CONFIG 4        // (not used)

#define DEBUG_RX A5        // software serial receive line; must allow pin-change interrupts
#define DEBUG_TX 10        // software serial transmit line

#if MARKLOCATION && !DEBUG          // pin for scope or logic analyzer tracing
   #define LOCATION_MARKER_PIN 10      // PB2, and CAN'T BE USED WITH DEBUG ON!
   #define LOCATION_MARKER_PORT 0x05   // data port B using the 0-based address for SBI/CBI instructions
   #define LOCATION_MARKER_BIT 2       // bit number (not mask) of the scope pin
#endif
//#define TEST_INPUT 2        // active low test input pin to simulate a beam break

#define BATTERY_VOLTAGE A0    // 0..1023 from A-to-D converter from battery voltage divider
#define BATTERY_R_TOP 330     // top battery resistor divider, in Kohms
#define BATTERY_R_BOT 180     // bottom battery resistor divider, in Kohms
#define BATTERY_VDROP_MV 330  // millivolts of drop before the divider: 0.26V Schottky diode + 0.07V MOSFET
#define AREF_MV  3300         // internal analog reference voltage, in millivolts, is VDD

// global variables

bool receiver = false;            // are we the receiver?
bool no_powerdown = false;        // should we power down after a while?
bool factory_reset = false;       // start Bluetooth module with factory reset?
int ir_rcv_time = 0;              // count milliseconds of IR receive cycle
int ir_xmt_time = 0;              // count milliseconds of IR transmit cycle
int battery_time = 0;             // count milliseconds of battery check time
uint16_t voltage;                 // the battery voltage we last read
unsigned long starttime_millis;   // starting time of the polling in millis()
unsigned long cycle_time;         // when we did the last polling cycle
unsigned long name_change_time;   // when we last changed our advertising name
unsigned long beam_lost_time;     // when we first noticed the beam was lost

// If the following 2 become bigger than a byte, interrupts must be disabled around access
volatile byte falling_pulses = 0; // how many good IR pulses we received
volatile byte short_pulses = 0;   // how many IR pulses were too short

static enum { // receiver state
   RCVSTATE_BEAMSEEN,             // we are seeing the beam
   RCVSTATE_BEAMBROKEN,           // the beam is broken, stretched to 5 seconds
   RCVSTATE_BEAMWAIT }            // waiting for beam to return after that
rcv_state = RCVSTATE_BEAMSEEN;

struct led_status { // LED state
   byte led;    // pin number
   bool led_on; // currently on?
   int on_time; // msec
   int period;  // msec
   int cycle_msec; } // how many msec into the cycle are we
led_battery = {LED_RED, false, BLED_ON, BLED_PERIOD, 0 },
led_receiving = {LED_BLUE, false, RLED_ON, RLED_PERIOD, 0 },
led_transmitting = {LED_YELLOW, false, XLED_ON, XLED_PERIOD, 0 },
led_beamlost_1 = {LED_BLUE, false, LLED_ON, LLED_PERIOD, 0 },
led_beamlost_2 = {LED_YELLOW, false, LLED_ON, LLED_PERIOD, 0 };

#if DEBUG
   SoftwareSerial debug_serial(DEBUG_RX, DEBUG_TX);
#endif

//----------------------------------------------------------------------------------------
//  Interrupt routine for changes on the IR receiver signal
//----------------------------------------------------------------------------------------

// In order to compile the interrupt routine with DEBUG on, we use a local version of
// SoftwareSerial that only takes over the interrupt vector for the pin-change interrupt
// on the receive pin it actually uses, which is PC5, interrupt PCINT1. That way we
// get to take over PCINT0 for the IR receiver's pin change interrupt.

static unsigned long pulse_start_time;

ISR(PCINT0_vect) { // interrupt for falling or rising edge of IR receiver
   // Count only those low pulses that are wide enough
   // The pulse from TSOP receivers can be 132 usc less wide that what was sent, or 158 usec wider!
   if (digitalRead(IR_RECEIVE) == LOW) {  // falling (leading) edge
      pulse_start_time = micros(); }
   else {  // rising (trailing) edge
      if (micros() - pulse_start_time > IR_RCV_WIDTH_MIN_USEC
            //&& digitalRead(TEST_INPUT) == HIGH // and test input is not telling us to fake a beam break
         ) {
         if (falling_pulses < 255) ++falling_pulses; }
      else {
         if (short_pulses < 255) ++short_pulses; } } }

//----------------------------------------------------------------------------------------
//  Routines that manipulate the lights
//----------------------------------------------------------------------------------------

void flashlights(int duration, int times) {
   for (int i = 0; i < times; ++i) {
      digitalWrite(LED_YELLOW, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      //  digitalWrite(LED_RED, HIGH);
      delay(duration);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
      delay(duration); } }

void flashlight(int light, int duration, int times) {
   for (int i = 0; i < times; ++i) {
      digitalWrite(light, HIGH);
      delay(duration);
      digitalWrite(light, LOW);
      delay(duration); } }

void rotatelights(int duration, int times) {
   for (int i = 0; i < times; ++i) {
      digitalWrite(LED_YELLOW, HIGH);
      delay(duration);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_BLUE, HIGH);
      delay(duration);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, HIGH);
      delay(duration);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, HIGH);
      delay(duration);
      digitalWrite(LED_GREEN, LOW); } }

void update_led(struct led_status *p, int add_msec) {
   if (p->cycle_msec == 0) { // just starting the cycle
      digitalWrite(p->led, HIGH); // turn on
      p->led_on = true; }
   if (p->led_on && p->cycle_msec >= p->on_time) {
      digitalWrite(p->led, LOW);  // end of on time: turn off
      p->led_on = false; }
   if ((p->cycle_msec += add_msec) > p->period) // update where we are
      p->cycle_msec = 0; }       // end of off time; set up to restart

void stop_led(struct led_status *p) {
   if (p->led_on) digitalWrite(p->led, LOW);
   p->cycle_msec = 0; }

//----------------------------------------------------------------------------------------
// Utility routines
//----------------------------------------------------------------------------------------

void shutdown(int flashcount) {
   #if DEBUG
   debug_serial.println("shutdown!");
   #endif
   flashlights(500, flashcount); // 500 msec flashes
   digitalWrite(POWERUP_FET, LOW);  // turn off our power
   while (1) {
      PRR = 0xff; // turn off everything in the processor
      asm("sleep\n"); } // wait for oblivion as the power dies
}
enum {ERR_1, ERR_2, ERR_3, ERR_4, ERR_5, ERR_6, ERR_7, ERR_8, ERR_9, ERR_10, ERR_11, ERR_12, ERR_13 };

void fatal(int code) { // report a fatal error
   #if DEBUG
   debug_serial.print("Fatal:");
   debug_serial.println(code + 1);
   #endif
   while (1) { // keep flashing until the button is pushed
      for (int i = 0; i < 3; ++i) { // flash red 3 times
         if (digitalRead(POWERDOWN_SW) == LOW) goto shutdown;
         digitalWrite(LED_RED, HIGH);
         delay(300);
         digitalWrite(LED_RED, LOW);
         delay(300); }
      for (int i = 0; i < code + 1; ++i) { // then flash green with the code
         if (digitalRead(POWERDOWN_SW) == LOW) goto shutdown;
         digitalWrite(LED_GREEN, HIGH);
         delay(300);
         digitalWrite(LED_GREEN, LOW);
         delay(300); }
      delay(500); }
shutdown: shutdown(1); }

void lowpower_delay(int wait_msec) {
   // Do a delay with minimum power consumption. Only use this after initialization is complete.
   if (wait_msec > 0) {
      PRR = (1 << PRTWI) + (1 << PRTIM2) // power down TWI and Timer 2,
            + (receiver ? (1 << PRTIM1) : (1 << PRUSART0)); // and Timer1 if receiver, USART if transmitter
      // Timer0 is used by delay() and millis()
      // Timer1 is used by us for IR transmitter pulses
      // Timer2 is not used
      // USART is used to talk to the Bluetooth module only when we are an IR receiver
      ADCSRA &= ~(1 << ADEN); // turn off A-to-D converter
      ACSR |= (1 << ACD);   // turn off analog comparator

      #if USE_SLEEP
      // This only saves about 0.5 mA, unfortunately, because we have to leave Timer0 running,
      // and hence the global system clock. But at least the CPU shuts down between Timer0 interrupts.
      int interrupt_count = 0;
      unsigned long delay_start = millis(); // time now
      do { // go into "idle" sleep mode
         SMCR = 0x01 ; // sleep mode control: idle (need timer0 running!), sleep enable
         asm("sleep\n");
         SMCR = 0;
         ++interrupt_count; }
      while (millis() - delay_start < wait_msec); // repeat until the delay is over
      #if DEBUG
      if (0) {
         debug_serial.print(interrupt_count);
         debug_serial.println(" wait ints"); }
      #endif

      #else // not USE_SLEEP
      delay(wait_msec);
      #endif
   } }

#if MARKLOCATION
inline void MARKLOC(byte cnt) { // coded pin output to observe locations on a scope or logic analyzer
   do {
      __asm__ ("sbi %0,%1\n"::"I"(LOCATION_MARKER_PORT), "I"(LOCATION_MARKER_BIT));
      __asm__ ("nop\n");
      __asm__ ("cbi %0,%1\n"::"I"(LOCATION_MARKER_PORT), "I"(LOCATION_MARKER_BIT)); }
   while (--cnt); }
#else
#define MARKLOC(x)
#endif

#if EVENTS && DEBUG // a buffered event log for debugging
#define EVENTS_MAX 15
struct {
   byte code;
   unsigned long timestamp; } events[EVENTS_MAX];
int event_oldest = 0;
int event_newest = 0;
int event_count = 0;

void EVENT(byte code) { // record an event
   if (event_count == 0) event_count = 1;
   else {
      if (++event_newest >= EVENTS_MAX) event_newest = 0;
      if (event_count >= EVENTS_MAX) {
         if (++event_oldest >= EVENTS_MAX) event_oldest = 0; }
      else ++event_count; }
   events[event_newest].code = code;
   events[event_newest].timestamp = millis(); }

void output_events(void) { // show all recent events
   if (event_count == 0) debug_serial.println("no events");
   else {
      debug_serial.print(event_count); debug_serial.println(" events");
      for (int ndx = event_oldest; ;) {
         debug_serial.print(events[ndx].code);
         debug_serial.print(" at "); debug_serial.println(events[ndx].timestamp);
         if (ndx == event_newest) break;
         if (++ndx >= EVENTS_MAX) ndx = 0; } } }
#else
#define EVENT(x)
#endif //EVENTS

//----------------------------------------------------------------------------------------
// Battery routines
//----------------------------------------------------------------------------------------

uint16_t read_battery_voltage (void) { // return voltage in millivolts
   uint16_t aval, Vmv;
   ACSR &= ~(1 << ACD);   // turn on the analog comparator
   ADCSRA |= (1 << ADEN); // turn on the A-to-D converter
   aval = analogRead(BATTERY_VOLTAGE);
   // aval = (V * (B/(T+B)) / Vref) * 1024; solve for V
   Vmv = ((unsigned long)aval * AREF_MV * (BATTERY_R_TOP + BATTERY_R_BOT)) / (1024UL * BATTERY_R_BOT);
   #if 0
   debug_serial.print("bat aval="); debug_serial.print(aval);
   debug_serial.print(", mV="); debug_serial.println(Vmv);
   #endif
   Vmv += BATTERY_VDROP_MV; // compensate for diode and MOSFET voltage drops, approximately
   return Vmv; }

#if DEBUG
void show_battery_voltage(uint16_t v) { // in tenths of a volt, rounded up
   debug_serial.print("B="); debug_serial.print((v + 50) / 1000);
   debug_serial.print('.'); debug_serial.print(((v + 50) % 1000) / 100);
   debug_serial.println('V'); }
#endif

void flash_battery_voltage(uint16_t v) { // flash battery voltage in tenths of a volt
   flashlight(LED_BLUE, 500, 1);
   flashlight(LED_YELLOW, 250, (v + 50) / 1000);
   delay(500);
   flashlight(LED_BLUE, 250, 1);
   flashlight(LED_YELLOW, 250, ((v + 50) % 1000) / 100);
   delay(500);
   flashlight(LED_BLUE, 500, 1); }

//----------------------------------------------------------------------------------------
// Microchip RN4871 Bluetooth LE module routines
//----------------------------------------------------------------------------------------

#define BT_TIMEOUT_MSEC 250

/* typical timing when communicating at 38,400 baud:
     to execute Y (stop advertising): 8 msec
     to send a 60-character name string with for IA: 16 msec (UART time)
     to execute IA,Z (clear advertising): 8 msec
     to execute IA,09 (set name): 10 msec
     to execute A,0032 (start advertising): 10 msec
     to completely change the device name: 68-74 msec
     But it's better now, since we don't start/stop advertising; so about 50 msec
*/
/*  An ATMega328 running at 8 Mhz cannot reliably do USART serial communication
    at 115200 baud, which is the default speed of the virgin Microchip Bluetooth
    module, because (see Table 20-6 in the CPU manual) the clock error rate is 3.5%.

    So to initialize a virgin Bluetooth module, we use special highspeed_serial_xxx
    routines that can write reliably at 115200 by disabling interrupts and carefully
    controlling the instruction loops to get the right timing. We use it to issue the
    command to change the baud rate to 38400, which takes effect after a reboot.

    We actually only need to start at 115200 the first time, since it writes the
    new speed into non-volatile configuration memory. But rather than having two
    versions of the software, or trying to identify a virgin chip, we just let
    an initialized module ignore the beginning stuff we send at the wrong speed. */

// The following delays target 8.68 usec per bit, or 69.44 clock cycles,
// or 23.1 iterations of the 3-cycle _delay_loop_1() in delay_basic.h.
// But we reduce them experimentally and perhaps add individual NOP
// instructions to compensate for other instruction overhead.
#define HIGHSPEED_START_DELAY 20  // length of the start bit
#define HIGHSPEED_DATA_DELAY 19   // length of the data bits
#define HIGHSPEED_STOP_DELAY 23   // length of the stop bit (this can be arbitrarily long)

void highspeed_serial_begin(void) {
   digitalWrite(BT_TX, 1); // put serial transmit into idle ("mark", 1) before resetting
   pinMode(BT_TX, OUTPUT); // to avoid garbage characters being sent
}
void highspeed_serial_write(uint8_t ch) { // write one serial character at 115200 baud
   uint8_t oldSREG = SREG;
   cli(); // turn off interrupts so we have exact timing
   BT_TX_PORT = BT_TX_PORT & ~BT_TX_MASK;  // start transmitting the start bit (0)
   _delay_loop_1(HIGHSPEED_START_DELAY); // the width is 8.63 usec, vs 8.68 usec nominal
   for (uint8_t i = 8; i > 0; --i) { // now write each data bit
      // This code uses no conditionals so that the execution time is constant for any data.
      uint8_t nextbit = ch & 1; // the next bit to transmit
      nextbit = nextbit - (nextbit << 1); // replicate it throughout the byte
      BT_TX_PORT = (BT_TX_PORT & ~BT_TX_MASK) | (nextbit & BT_TX_MASK); // copy it to the data register bit
      _delay_loop_1(HIGHSPEED_DATA_DELAY); // delay for the bit time
      __asm__ ("nop\n");  // delay one more cycle; the total is 8.76 usec, vs 8.68 usec nominal
      ch >>= 1; }
   _delay_loop_1(2); // an additional delay to compensate for early loop exit on the last bit
   // The total for those 9 bits is 78.60 usec vs 78.12 nominal, or just 0.6% slow.
   // That is much better than the UART hardware, which is 3.5% to 3.9% slow!
   BT_TX_PORT = BT_TX_PORT | BT_TX_MASK;  // start transmitting the stop bit (1)
   SREG = oldSREG; // restore interrupts
   _delay_loop_1(HIGHSPEED_STOP_DELAY); }

void highspeed_serial_print(const char *cmd) {
   for (const char *ptr = cmd; *ptr; ++ptr) {
      highspeed_serial_write(*ptr);
      delayMicroseconds(20); } } // space out the characters -- maybe we don't still need to do this

void BT_hard_reset(void) {
   digitalWrite(BT_RESET, 1); // do a hard reset of the Bluetooth module
   pinMode(BT_RESET, OUTPUT);
   digitalWrite(BT_RESET, LOW);
   delay(2);
   digitalWrite(BT_RESET, HIGH);
   delay(200); // 68 msec after reset pulse, 100 msec before $$$
}

bool BT_initialize(void) {
   for (int i = 0; i < 3; ++i) { // try 3 times
      digitalWrite(BT_TX, 1); // put serial transmit into idle ("mark") before resetting
      pinMode(BT_TX, OUTPUT); // to avoid garbage characters being sent
      BT_hard_reset();  // reset the module

      if (factory_reset) {   // special mode: reset the chip to factory defaults, for testing
         highspeed_serial_begin();
         highspeed_serial_print("$$$");  // try to enter command mode at 115200
         delay(50); // ignore response
         highspeed_serial_print("SF,2\r"); // factory reset
         delay(500);
         Serial.begin(38400); // now try again at low speed
         Serial.print("$$$");  // enter command mode
         delay(50); // ignore the response
         Serial.print("SF,2\r"); // factory reset
         delay(500);
         while (Serial.available()) Serial.read(); // empty any garbage in the serial queue
         Serial.end();
         // This doesn't go on to initialize the module correctly, and I don't know why.
         // You apparently must cycle the power to get it to work after a factory reset.
      }

      #if DEBUG
      debug_serial.println("$$$");
      #endif
      highspeed_serial_begin();
      highspeed_serial_print("$$$");  // try to enter command mode at 115200
      delay(50); // let it respond, but don't try to read it
      highspeed_serial_print("SB,05\r");  // set the baud rate to 38,400
      delay(50); // let it respond
      highspeed_serial_print("R,1\r"); // issue a reboot command to make it permanent
      delay(500); // wait long enough for it to happen

      Serial.begin(38400); // start hardware serial with the slower (new?) baud rate
      Serial.setTimeout(BT_TIMEOUT_MSEC); // and a 250 msec timeout
      while (Serial.available()) Serial.read(); // empty any garbage in the serial queue
      Serial.print("$$$"); // enter command mode at the new speed
      char buffer[10];
      int nchars = Serial.readBytes(buffer, 4); // check the response
      buffer[nchars] = 0;
      if (strcmp(buffer, "CMD>") == 0) {
         #if DEBUG
         debug_serial.println("BT ok");
         #endif
         return true; }
      #if DEBUG
      debug_serial.print("bad ");
      debug_serial.print(nchars);
      debug_serial.print(':');
      debug_serial.println(buffer);
      #endif
      return false; } }

int Serial_timedPeek() {  // this exists in Stream.cpp but is not public!
   unsigned long starttime = millis();
   do {
      int ch = Serial.peek();
      if (ch >= 0) return ch; }
   while (millis() - starttime < BT_TIMEOUT_MSEC);
   return -1; // timeout
}

void BT_sendcommand(const char *cmd) {
   // send a command that might produce no output, or a lot for debugging
   #if DEBUG
   debug_serial.print("cmd ");
   debug_serial.println(cmd);
   #endif
   Serial.println(cmd);
   delay(250); // wait for all output to be generated and buffered before doing debug_serial,
   // because that uses a software UART during transmit which disables interrupts.
   while (Serial.available()) {
      char ch = Serial.read();
      #if DEBUG
      debug_serial.write(ch);
      #endif
   }
   #if DEBUG
   debug_serial.println();
   #endif
}

bool BT_sendcommand_chkok(const char *cmd) {
   // send a command that should elicit an "AOK' response.
   #if DEBUG && BT_SHOWCMDS
   debug_serial.print("cmd ");
   debug_serial.println(cmd);
   unsigned long starttime;
   starttime = millis();
   #endif
   Serial.println(cmd); // send it to the Bluetooth module
   while (Serial_timedPeek() == ' ') Serial.read(); // skip leading blanks
   char response[15];
   int nchars = Serial.readBytes(response, 9);
   response[nchars] = 0;
   bool ok =  strcmp(response, "AOK\r\nCMD>") == 0;
   #if DEBUG && BT_SHOWCMDS
   if (ok) {
      debug_serial.print(millis() - starttime);
      debug_serial.println(",ok"); }
   else {
      debug_serial.print("bad, len ");
      debug_serial.print(nchars);
      debug_serial.print(':');
      debug_serial.println(response);
      for (int i = 0; i < nchars; ++i) {
         debug_serial.print(response[i], HEX);
         debug_serial.print(' '); }
      debug_serial.println(); }
   #endif
   return ok; }

void BT_set_devname(char beamstate, unsigned long int time) {
   // change our GAP beacon advertising device name
   #if DEBUG
   unsigned long start_set_devname = millis();
   #endif
   //if (!BT_sendcommand_chkok("Y")) fatal(ERR_x); // stop advertising (Do we need to??)
   // We have to construct the advertising packet manually, because the Microchip interface
   // limits the name that can be set with "SN" to 20 characters. (17, actually)
   if (!BT_sendcommand_chkok("IB,Z")) fatal(ERR_3); // clear all beacon advertising data
   if (!BT_sendcommand_chkok("IB,01,06")) fatal(ERR_4); // add flags: generally discoverable, not BR/EDR
   Serial.print("IB,09,");  // prepare to add complete local name, encoded in hex
   char buffer[40];
   sprintf(buffer, "retrotope-timer(%c)%lu", beamstate, time);
   #if DEBUG && BT_SHOWCMDS
   debug_serial.print("snd IB,09,\"");
   debug_serial.print(buffer);
   debug_serial.print("\" ");
   unsigned long starttime;
   starttime = millis();
   #endif
   for (char *bufptr = buffer; *bufptr; ++bufptr) {
      char hex[3];
      sprintf(hex, "%02X", *bufptr);
      Serial.print(hex);
      #if DEBUG && BT_SHOWCMDS
      debug_serial.print(hex);
      #endif
   }
   #if DEBUG && BT_SHOWCMDS
   debug_serial.print(", t="); debug_serial.println(millis() - starttime);
   #endif
   if (!BT_sendcommand_chkok("")) fatal(ERR_5); // add the final CR and check the response

   //sprintf(buffer, "SN,retrotope-timer(%c)%lu", beamstate, time); // fails: too long
   //sprintf(buffer, "SN,timer%ct%lu", beamstate, time); // would work, but not backward compatible with V2
   //if (!BT_sendcommand_chkok(buffer)) fatal(ERR_6); // change our name

   //if (!BT_sendcommand_chkok("A," ADVERTISING_INTERVAL_HEX_MSEC)) fatal(ERR_7); // restart advertising
   #if DEBUG
   debug_serial.print(millis() - start_set_devname); debug_serial.print(" ms setname ");
   debug_serial.println(beamstate);
   #endif
}

//----------------------------------------------------------------------------------------
//  startup code
//----------------------------------------------------------------------------------------

void do_clock_sync(void) {
   /* This is a special mode that compares the clock frequency of two units and displays it via the serial monitor.
       The LED driver outputs should be connected to the IR receiver inputs of each other,
       plus a ground lead. Both units transmit and time the receive pulses from the other, although
       typically only one is connected to the serial monitor.  */
   #if DEBUG
   byte old_falling_pulses;
   PRR &= ~(1 << PRTIM1);      // make sure Timer/Counter1 is powered up
   TCCR1B = 0;             // but turned off
   TCCR1A = 0;
   TCNT1 = 0;              // zero the counter to avoid bogus wraparound
   TIMSK1 = 0;             // don't generate interrupts
   OCR1A = (unsigned int) 46874; // (clk/(freq*2*prescale)) - 1
   // make this provide a period which is exactly an integer number of seconds, here 12 seconds.
   TCCR1A = 0b01000000;    // toggle OC1A on match, CTC mode
   TCCR1B = 0b00001101;    // CTC mode, clk/1024 prescaling, GO!
   PCICR  = 0x01;       // enable pin-change interrupts
   PCMSK0 = 0x01;       // enable on Arduino pin 8 (PCINT0, package pin 12)
   //while (!Serial) ;
   delay(2000);
   debug_serial.begin(115200);
   debug_serial.println("starting sync code");
   delay(500);
   old_falling_pulses = falling_pulses;
   while (1) {  // look for input pulses to our receiver
      if (falling_pulses != old_falling_pulses) {
         old_falling_pulses = falling_pulses;
         debug_serial.println(pulse_start_time); // wraps in 70 minutes
      }
      if (digitalRead(POWERDOWN_SW) == LOW) {
         shutdown(1); } }
   #endif
}

bool startup_mode (int mode) { // choose a special startup mode
   unsigned long wait_start;
   delay(1000);
   flashlight(LED_RED, 300, mode); // flash red light to indicate which mode, 1..ny
   wait_start = millis();
   while (millis() - wait_start < 2000) // wait up to two seconds
      if (digitalRead(POWERDOWN_SW) == LOW) { // for button to be pushed
         delay(DEBOUNCE_DELAY);
         while (digitalRead(POWERDOWN_SW) == LOW) ;  // and then released
         delay (DEBOUNCE_DELAY);
         flashlight(LED_GREEN, 300, mode);  // flash green to signal selection of this mode
         return true; }
   return false; // this mode was not selected
}

void special_startup(void) {
   rotatelights(100, 8);          // signal special startup
   while (digitalRead(POWERDOWN_SW) == LOW) ;  // wait for button to finally be released
   delay (DEBOUNCE_DELAY);
   if (startup_mode(1))           // mode 1: normal operation, but
      no_powerdown = true;          // never power down
   if (startup_mode(2)) {         // mode 2: normal operation, but
      flashlight(LED_YELLOW, 300, MAJOR_VERSION); // flash out version number first
      flashlight(LED_BLUE, 300, MINOR_VERSION);
      delay(1000);
      flash_battery_voltage(read_battery_voltage()); // and battery voltage
      delay(1000); }
   else if (startup_mode(3))      // mode 3: normal operation, but
      factory_reset = true;          // do Bluetooth factory reset first
   else if (startup_mode(4))      // mode 4: special wired clock sync to second unit
      do_clock_sync();               // (this never returns)
}

void testir(void) { //TEMP for testing hardware
   for (int i = 0; i < 10; ++i) {
      digitalWrite(IR_TRANSMIT, LOW);
      delay(500);
      digitalWrite(IR_TRANSMIT, HIGH);
      delay(500); }
   PRR &= ~(1 << PRTIM1);  // make sure Timer/Counter1 is powered up
   TCCR1B = 0;             // but turned off
   TCCR1A = TCCR1C = 0;
   TCNT1 = 0;              // zero the counter to avoid bogus wraparound
   TIMSK1 = 0;             // don't generate interrupts
   OCR1A = F_CPU / IR_FREQ / 2 - 1;  // the compare limit is twice the frequency
   TCCR1A = 0b01000000;    // toggle OC1A on match, CTC mode
   TCCR1B = 0b00001001;    // CTC mode, no prescaling, clk/1, GO!
   while (1) ; // END TEMP
}

void setup() {
   unsigned long startwait_millis;

   pinMode(POWERUP_FET, OUTPUT);     // so that we stay alive after the on/off button push is over,
   digitalWrite(POWERUP_FET, HIGH);  // turn on the power MOSFET as quickly as possible
   startwait_millis = millis();

   pinMode(POWERDOWN_SW, INPUT_PULLUP);
   pinMode(BATTERY_VOLTAGE, INPUT); // analog input
   pinMode(IR_RECEIVE, INPUT_PULLUP);
   pinMode(IR_TRANSMIT, OUTPUT);
   //pinMode(TEST_INPUT, INPUT_PULLUP);
   digitalWrite(IR_TRANSMIT, HIGH); // disable IR transmitter

   #if BLUETOOTH
   digitalWrite(BT_RESET, 1); // don't reset Bluetooth module (yet)
   pinMode(BT_RESET, OUTPUT);
   #endif

   pinMode(LED_YELLOW, OUTPUT);
   pinMode(LED_BLUE, OUTPUT);
   pinMode(LED_GREEN, OUTPUT);
   pinMode(LED_RED, OUTPUT);
   if (DEBUG)
      rotatelights(200, 4);
   else
      flashlights(100, 3); // 200 msec flash, 3 times

   // to save power, turn off digital inputs on PD6/7 (Arduiino pins 6/7, TQFP pins 10/11)
   DIDR1 = (1 << AIN1D) | (1 << AIN0D);
   // similarly for PC0-4 (A0-A4), and for PC5 (A5) if it is not used as input for the debug port
   DIDR0 = (1 << ADC4D) | (1 << ADC3D) | (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D)
           | (DEBUG ? 0 : (1 << ADC5D));

   #if DEBUG
   debug_serial.begin(38400);
   delay(500);
   debug_serial.println("\r\nstarted V" stringify(MAJOR_VERSION) "." stringify(MINOR_VERSION));
   voltage = read_battery_voltage();
   show_battery_voltage(voltage);
   flash_battery_voltage(voltage);
   delay(500);
   #endif

   #if 0 // TEMP test code for the new millis() timer
   debug_serial.println("1 min monotonicity test");
   unsigned long lastmicros = micros();
   for (long i = 0; i < 116280L * 60; ++i) { // make sure micros() is monotonic
      unsigned long thismicros;
      long delta;
      thismicros = micros();
      delta = (long)(thismicros - lastmicros);
      if (delta < 0 || delta > 50) { // (remember timer0 interrupts every 1 msec)
         debug_serial.print(i);
         debug_serial.print(": delta="); debug_serial.print(delta);
         debug_serial.print(", thismicros="); debug_serial.println(thismicros);
         thismicros = micros(); }
      lastmicros = thismicros; }
   debug_serial.println("done");
#define SLOTS 15
   unsigned long times[SLOTS];
   for (int i = 0; i < SLOTS; ++i) { // check millis() increases
      times[i] = millis();
      delay(20); }
   debug_serial.println("millis, by 20");
   for (int i = 0; i < SLOTS; ++i) {
      debug_serial.print(times[i]);
      if (i > 0) {
         debug_serial.print(", ");
         debug_serial.print(times[i] - times[i - 1]); }
      debug_serial.println(); }
   for (int i = 0; i < SLOTS; ++i) { // check micros() increases
      times[i] = micros();
      delayMicroseconds(20); }
   debug_serial.println("micros, by 20");
   for (int i = 0; i < SLOTS; ++i) {
      debug_serial.print(times[i]);
      if (i > 0) {
         debug_serial.print(", ");
         debug_serial.print(times[i] - times[i - 1]); }
      debug_serial.println(); }
   for (int i = 0; i < 10; ++i)  { // check micros() to millis() alignment
      unsigned long startmicros = micros();
      delay(20);
      debug_serial.print(micros() - startmicros); debug_serial.println(" usec in 20 msec"); }
   for (int i = 0; i < 10; ++i)  {
      unsigned long startmillis = millis();
      delayMicroseconds(20000);
      debug_serial.print(millis() - startmillis); debug_serial.println(" millis in 20000 usec"); }
   #endif // end new millis() test code TEMP

   #if 0 // TEMP code to measure our millis() interrupt rate on a scope
   pinMode(DEBUG_TX, OUTPUT);
   while (1) ;
   #endif

   #if MARKLOCATION
   digitalWrite(LOCATION_MARKER_PIN, LOW);
   pinMode(LOCATION_MARKER_PIN, OUTPUT);
   #endif

   starttime_millis = millis();                 // record our starting time
   while (digitalRead(POWERDOWN_SW) == LOW) {   // wait for the power button to be released
      if (starttime_millis - startwait_millis > 5000) {  // if it is pushed for 5 seconds or more
         special_startup();                     // choose a special startup mode
         break; }
      starttime_millis = millis();              // record another starting time
   }
   delay(DEBOUNCE_DELAY);

   // see if we are receiving the beam from the other unit, and configure accordingly
   falling_pulses = short_pulses = 0;
   PCICR  |= 0x01;      // enable pin-change interrupts PCINT0 for PCINT[7:0], PB7:0
   PCMSK0 = 0x01;       // enable only on PB0, Arduino pin 8 (PCINT0, TQFP-32 package pin 12)
   delay(CHK_RCV_TIME); // wait to see if we receive IR pulses
   #if DEBUG
   debug_serial.print("rcvd:"); debug_serial.print(falling_pulses);
   debug_serial.print(" short:"); debug_serial.println(short_pulses);
   #endif
   if (falling_pulses > (CHK_RCV_TIME / IR_XMT_PERIOD / 2)) // become the receiver if we saw at least half the pulses transmitted
      receiver = true;

   if (receiver) {  // we are a receiver
      #if BLUETOOTH
      if (!BT_initialize()) fatal(ERR_1); // initialize the Bluetooth module
      if (!BT_sendcommand_chkok("Y")) fatal(ERR_8); // make sure advertising is off
      if (!BT_sendcommand_chkok("SC,1")) fatal(ERR_9); // set "unconnectable beacon" mode
      if (!BT_sendcommand_chkok("SR,0004")) fatal(ERR_10); // make non-connectable on power-on/off
      // (See Firmware 1.40 Release Note, July 2019)
      if (!BT_sendcommand_chkok("SGA,0")) fatal(ERR_11); // set highest power for advertising
      if (!BT_sendcommand_chkok("STB," ADVERTISING_INTERVAL_HEX_MSEC )) fatal(ERR_12); // beacon advertising interval
      #if DEBUG
      BT_sendcommand("D"); // show various info, just for grins
      #endif
      BT_set_devname('0', name_change_time);     // set the timestamp in our name to "now"
      name_change_time = millis() - starttime_millis; // the last time we changed our name
      #if DEBUG
      debug_serial.println("rcvr!");
      #endif
      falling_pulses = 0;
      ir_rcv_time = 0;
      #endif
   }
   else { // we are a transmitter
      PCICR &= 0xfe; // disable pin-change interrupts for the IR receiver on PCINT 0
      if (!BT_initialize()) fatal(ERR_2); // initialize the Bluetooth module
      BT_sendcommand("O,0"); // then put it into into deep sleep
      //digitalWrite(BT_RESET, 0); // which takes 0.30 ma less power than keeping it reset
      #if DEBUG
      debug_serial.println("xmtr!");
      ir_xmt_time = 0;
      #endif
   }

   cycle_time = millis() - starttime_millis; // invent a time at which we did the previous cycle
   battery_time = BATTERY_CHK_PERIOD;        // force a battery check the first time

} //end setup()

//----------------------------------------------------------------------------------------
//  main repeat loop
//----------------------------------------------------------------------------------------

void loop() {
   unsigned long timenow;
   unsigned int cycle_msec;

   //**** delay for a minor cycle time

   EVENT(1);
   lowpower_delay(MINOR_CYCLE);
   timenow = millis() - starttime_millis;
   cycle_msec = (unsigned int) (timenow - cycle_time); // how long since the last cycle
   cycle_time = timenow;
   EVENT(2);

   //**** check for auto-powerdown

   if (!no_powerdown && timenow > (unsigned long)POWERDOWN_MINUTES * 60 * 1000) {
      #if DEBUG
      debug_serial.println("auto shutdown");
      #endif
      shutdown(3); }

   //*** check the battery voltage every so often

   if ((battery_time += cycle_msec) >= BATTERY_CHK_PERIOD) {
      EVENT(3);
      voltage = read_battery_voltage();
      #if DEBUG
      show_battery_voltage(voltage);
      #endif
      if (STOP_IF_BAT_LOW && voltage < BATTERY_FAIL) {  // battery failing?
         #if DEBUG
         debug_serial.println("bat fail");
         flash_battery_voltage(voltage);
         #endif
         shutdown(5); }
      battery_time = 0; }

   //**** check for status light changes due

   if (voltage < BATTERY_WEAK) // flash if battery is weak
      update_led (&led_battery, cycle_msec);
   else if (led_battery.led_on) // we were flashing, but it's good now
      stop_led (&led_battery);

   if (receiver) {
      if (rcv_state == RCVSTATE_BEAMSEEN)
         update_led(&led_receiving, cycle_msec);
      else if (rcv_state == RCVSTATE_BEAMWAIT) {
         update_led(&led_beamlost_1, cycle_msec);
         update_led(&led_beamlost_2, cycle_msec); }
      // else RCVSTATE_BEAMBROKEN: do nothing; light is constant blue
   }
   else  update_led(&led_transmitting, cycle_msec);

   //**** check the powerdown pushbutton

   if (digitalRead(POWERDOWN_SW) == LOW) {
      shutdown(1); }

   //**** check for receiver or transmitter IR activity

   if (receiver) {
      EVENT(5);
      ir_rcv_time += cycle_msec;
      if (ir_rcv_time >= IR_RCV_PERIOD) { // end of a receive sampling period
         int last_ir_rcv_time = ir_rcv_time; // remember how long it was
         int last_falling_pulses = falling_pulses; // and how many pulses we saw
         // We start a new sampling period NOW because if we do BT name changes,
         // it takes a long time and we must sample the next period in the meantime.
         falling_pulses = 0;  // start a new sampling period
         ir_rcv_time = 0;
         switch (rcv_state) {  // process the result from the last period
            case RCVSTATE_BEAMSEEN: // we think we're seeing the beam
               if (last_falling_pulses <= last_ir_rcv_time / IR_XMT_PERIOD - MISSING_PULSE_THRESHOLD) { // if we missed at least 2 or so pulses
                  rcv_state = RCVSTATE_BEAMBROKEN;  // "beam break": start a constant timestamp now
                  beam_lost_time = timenow;
                  #if DEBUG
                  debug_serial.print("rcv_time:"); debug_serial.print(last_ir_rcv_time);
                  debug_serial.print(" fall:"); debug_serial.println(last_falling_pulses);
                  #endif
                  BT_set_devname('1', beam_lost_time);
                  digitalWrite(LED_BLUE, HIGH);     // constant blue light
               }
               else { // we are still seeing the beam
                  if (timenow - name_change_time >= RCV_NAMECHANGE_TIME) {
                     name_change_time = timenow;
                     BT_set_devname('0', name_change_time); // change the timestamp in the name
                  } }
               break;
            case RCVSTATE_BEAMBROKEN:  // beam broken: stretch it out to 5 seconds
               if (timenow - beam_lost_time < RCV_BREAK_TIME)  // not end of "stretched" beam lost time
                  break; // just stay in this state
               digitalWrite(LED_BLUE, LOW);    // end of break time: turn off the constant blue light
               rcv_state = RCVSTATE_BEAMWAIT;  // fall into "wait for beam to return" state below
            case RCVSTATE_BEAMWAIT:  // wait for beam to return
               if (last_falling_pulses <= last_ir_rcv_time / IR_XMT_PERIOD - MISSING_PULSE_THRESHOLD) { // if we missed at least 2 or so pulses
                  // beam is still broken, so must be "lost": we'll do special blue/yellow flashing lights
               }
               else { // we've seen the beam return
                  digitalWrite(LED_BLUE, LOW);   // turn off the blue/yellow flashing lights
                  digitalWrite(LED_YELLOW, LOW);
                  rcv_state = RCVSTATE_BEAMSEEN;
                  name_change_time = timenow + RCV_NAMECHANGE_TIME; // setup to change to '0' name soon
               }
               break;
            default: fatal(ERR_13); } } }

   else { // transmitter
      ir_xmt_time += cycle_msec;
      if (ir_xmt_time >= IR_XMT_PERIOD) { // time to output a 38Khz burst from the IR transmitter
         EVENT(7);
         PRR &= ~(1 << PRTIM1);  // make sure Timer/Counter1 is powered up
         TCCR1B = 0;             // but turned off
         TCCR1A = TCCR1C = 0;
         TCNT1 = 0;              // zero the counter to avoid bogus wraparound
         TIMSK1 = 0;             // don't generate interrupts
         OCR1A = F_CPU / IR_FREQ / 2 - 1;  // the compare limit is twice the frequency
         TCCR1A = 0b01000000;    // toggle OC1A on match, CTC mode
         TCCR1B = 0b00001001;    // CTC mode, no prescaling, clk/1, GO!
         delayMicroseconds(IR_ON_USEC);
         TCCR1B = 0;             // turn off the timer
         TCCR1A = 0;
         digitalWrite(IR_TRANSMIT, HIGH); // leave with the transmitter disabled
         EVENT(8);
         ir_xmt_time = 0; } } }
//*
