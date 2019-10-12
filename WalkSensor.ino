/***************************************************************************************************

    Battery-powered walk path sensor

   This is the software for a small (4" x 4" 1.6") battery-powered wireless sensor
   that, when used in a pair on opposite sides of a walking path, detects someone
   walking thorugh an IR beam between the sensors. It then sends a Bluetooth alert
   to a mobile phone or tablet.

   The custom-built sensor hardware consist of:
   - Adafruit Bluefruit Feather 32U4 AVR microcontroller with Bluetooth LE module
   - 940nm IR LED transmitter powered by a constant-current driver
   - 940nm 38 Khz tuned IR receiver
   - red/green battery status light
   - blue and yellow operational status lights
   - power on/off pushbutton
   - program-controlled power supply circuit
   - 4-cell AA battery holder

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
      - we GAP advertise with the name "retrotope-timer(0)nnnn", where
        nnnn is our timestamp in milliseconds, updated every 250 msec
      when the IR signal is interrupted,
        - the blue light turns on for 5 seconds
        - we GAP advertise with the name "retrotope-timer(1)nnnn", where
          nnnn is or timestamp in milliseconds at the time the beam broke
      if the beam break lasts more than 5 seconds,
        - continue the same "retrotope-timer(1)nnnn" advertising
        - flash yellow and blue lights obnoxiously

   special startup modes: hold the button for more than 5 seconds
     - all three lights flash in rotation for a couple of seconds
     - the red light flashes N times, and then waits for the button
       - if the button is pushed after N flashes, it enters mode N:
         - 1: normal mode, except no powerdown after one hour
         - 2: special timer sync mode: link two units to compare their clocks
         - 3: not implemented yet


   The design of the hardware and software for the sensors and the corresponding
   receiver is open-source.  See https://github.com/LenShustek/walksensor.

   ------------------------------------------------------------------------------------------------------
   Copyright (c) 2016, Len Shustek

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

   TODO:
   - experiment with a lower duty cycle for transmit; can it be as reliable?
   - work on better sleep/hibernate mode to reduce current?
     (Snoozelib doesn't work; maybe try https://github.com/adafruit/Adafruit_SleepyDog, but
     the watchdog timer only goes down to 15 msec.)

*****************************************************************************************************/
#define VERSION "2.4"

#define DEBUG false
#define BLUETOOTH true
#define STOP_IF_BATTERY_LOW true
#define USE_SLEEP false
#define BOTH_XMT_RCV false

/* Here's the measured power utilization without going into sleep mode:
   off: about 10 ua, so several years with 2000 mAH AA batteries
   transmitter: 15 ma average, so 130 hours with 2000 maH AA batteries
   receiver: 14 ma average, so 142 hours with 2000 maH AA batteries
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

// configuration parameters

// those marked * affect the quality of our beam recognition algorithm

#define MINOR_CYCLE 5     //* minor cycle time in msec, of which all times below should be a multiple
#define IR_XMT_PERIOD 5           //* IR transmit cycle time in milliseconds between bursts
#define IR_RCV_PERIOD 50          //* IR receive sample period in milliseconds
#define XLED_ON 50                // transmit status LED on time in milliseconds
#define XLED_PERIOD 2000          // transmit status LED cycle time in milliseconds
#define RLED_ON 50                // receive status LED on time in milliseconds
#define RLED_PERIOD 2000          // receive status LED cycle time in milliseconds
#define LLED_ON 300               // lost beaam LED on time in milliseoncds
#define LLED_PERIOD 600           // lost beam LED cycle time in milliseconds
#define BLED_ON 50                // battery LED on time in milliseconds
#define BLED_PERIOD 5000          // battery LED cycle time in millliseconds
#define BATTERY_CHK_PERIOD 5000   // battery check time in milliseconds

#define IR_ON 1                   //* IR burst transmit time in milliseconds
#define PULSE_WIDTH_PERCENT 97    //* what percent of an IR pulse we need to see to count it
#define MISSING_PULSE_THRESHOLD 2 //* how many missing pulses constitute a beam break
#define CHK_RCV_TIME 200          // how long to wait to see if the other guy is sending, in msec
#define RCV_NAMECHANGE_TIME 250   // how often to change our name timestamp when beam is seen
#define RCV_BREAK_TIME 5000       // how long to stretch the beam break time to, in msec

#define IR_FREQ 38000L            // tuned IR receiver frequency
#define POWERDOWN_MINUTES 60      // after how many minutes to power down automatically
#define DEBOUNCE_DELAY 25         // debounce delay in msec

#define BATTERY_WEAK 5000         // millivolt level (out of 6000) to warn about low battery 
#define BATTERY_FAIL 3900         // millivolt level (out of 6000) to treat as battery failure

static enum {
   RCVSTATE_BEAMSEEN,         // we are seeing the beam
   RCVSTATE_BEAMBROKEN,       // the beam is broken, stretched to 5 seconds
   RCVSTATE_BEAMWAIT }        // waiting for beam to return after that
rcv_state = RCVSTATE_BEAMSEEN;

// hardware configuration

// Beware: analog pins A0..A5 cannot be programmed as digital pins on an AVR 32u4!

#define POWERUP_FET 13      // output pin high to keep us powered up by turning the MOSFET on
#define POWERDOWN_SW 10     // this input pin is low if the on/off button is pushed

#define LED_YELLOW 5        // LED pins: active high
#define LED_BLUE 6
#define LED_GREEN 2
#define LED_RED 12

#define IR_TRANSMIT 9       // active low output pin for IR transmit through constant-current LED driver
//.                            this is PB5, but more importantly, OC1A for Timer/Counter1
#define IR_RECEIVE 11       // active low input pin if IR receiver sees 10 or more pulses at 38 Khz (must be port B!)
#define TEST_INPUT 0        // active low test input pin to simulate a beam break
#define BATTERY_VOLTAGE A4  // 0..1023 from A-to-D converter from battery voltage divider
#define BATTERY_R_TOP 205   // top battery resistor divider, in Kohms
#define BATTERY_R_BOT 205   // bottom battery resistor divider, in Kohms
#define AREF_MV  3300       // internal analog reference voltage, in millivolts

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// global variables

bool receiver = false;            // are we the receiver?
bool transmitter = false;         // are we the transmitter?
bool no_powerdown = false;        // should we power down after a while?
volatile int falling_pulses = 0;  // how many IR pulses have we received
int ir_rcv_time = 0;              // count milliseconds of IR receive cycle
int ir_xmt_time = 0;              // count milliseconds of IR transmit cycle
int battery_time = 0;             // count milliseconds of battery check time
uint16_t voltage;                 // the battery voltage we last read
unsigned long starttime_millis;   // starting time of the processor
unsigned long name_change_time;   // when we last changed our advertising name
unsigned long beam_lost_time;     // when we first noticed the beam was lost


struct led_status { // LED state
   int led;
   int on_time; // msec
   int period;  // msec
   int cycle_count; }
led_battery = {LED_GREEN, BLED_ON, BLED_PERIOD, 0 },
led_receiving = {LED_BLUE, RLED_ON, RLED_PERIOD, 0 },
led_transmitting = {LED_YELLOW, XLED_ON, XLED_PERIOD, 0 },
led_beamlost_1 = {LED_BLUE, LLED_ON, LLED_PERIOD, 0 },
led_beamlost_2 = {LED_YELLOW, LLED_ON, LLED_PERIOD, 0 };

//----------------------------------------------------------------------------------------
//  Random byte generator
//
// This is an 8-bit version of the 2003 George Marsaglia XOR pseudo-random number
// generator. It has a full period of 255 before repeating.
// See http://www.arklyffe.com/main/2010/08/29/xorshift-pseudorandom-number-generator/
//----------------------------------------------------------------------------------------
static byte seed = 23;
byte random_byte(void) {
   seed ^= (byte)(seed << 7); // The casts are silly, but are needed to keep the compiler
   seed ^= (byte)(seed >> 5); // from generating "mul 128" for "<< 7"! Shifts are faster.
   seed ^= (byte)(seed << 3);
   return seed; }

//----------------------------------------------------------------------------------------
//  Interrupt routine for changes on the IR receiver signal
//----------------------------------------------------------------------------------------

// In order to compile the interrupt routine, you must comment out line 43 in Adafruit_BluefruitLE_UART.h
// located in c:\projects\Arduino\libraries\Adafruit_BluefruitLE_nRF51-master
// See https://forums.adafruit.com/viewtopic.php?f=24&t=83256&start=15 about this library bug,
// and also https://forums.adafruit.com/viewtopic.php?f=57&t=105935

static unsigned long pulse_start_time;
ISR(PCINT0_vect) { // interrupt for falling or rising edge of IR receiver
   // count only those low pulses that are "almost" as wide as what the transmitter sends
   if (digitalRead(IR_RECEIVE) == LOW) {  // falling edge
      pulse_start_time = micros(); }
   else {  // rising edge
      if (micros() - pulse_start_time > (unsigned long)IR_ON * 1000 * PULSE_WIDTH_PERCENT / 100 // "almost" == 95% or so
            && digitalRead(TEST_INPUT) == HIGH) // and test input is not telling us to fake a beam break
         ++falling_pulses; } }

//----------------------------------------------------------------------------------------
// Utility routines
//----------------------------------------------------------------------------------------

enum {ERR_1, ERR_2, ERR_3, ERR_4, ERR_5, ERR_6 };
void fatal_error(int code) {
   while (1) {
      for (int i = 0; i < 3; ++i) {
         digitalWrite(LED_RED, HIGH);
         delay(500);
         digitalWrite(LED_RED, LOW);
         delay(500); }
      for (int i = 0; i < code + 1; ++i) {
         digitalWrite(LED_GREEN, HIGH);
         delay(500);
         digitalWrite(LED_GREEN, LOW);
         delay(500); } } }

void shutdown(int flashcount) {
   if (DEBUG) Serial.println("shutdown!");
   flashlights(500, flashcount); // 500 msec flashes
   digitalWrite(POWERUP_FET, LOW);  // turn off our power
   while (1) ; // wait for oblivion as the power dies
}

uint16_t read_battery_voltage (void) { // return voltage in millivolts
   uint16_t aval, Vmv;
   aval = analogRead(BATTERY_VOLTAGE);
   // aval = (V * (B/(T+B)) / Vref) * 1024; solve for V
   Vmv = ((unsigned long)aval * AREF_MV * (BATTERY_R_TOP + BATTERY_R_BOT)) / (1024UL * BATTERY_R_BOT);
   if (DEBUG) {
      Serial.print("battery: aval="); Serial.print(aval);
      Serial.print(", mV="); Serial.println(Vmv); }
   return Vmv; }

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
      digitalWrite(LED_GREEN, HIGH);
      delay(duration);
      digitalWrite(LED_GREEN, LOW); } }

void set_devname(char beamstate, unsigned long int time) {
   // change our GAP advertising device name
   if (DEBUG) {
      Serial.print("set devname time "); Serial.println(time); }
   ble.print(F("AT+GAPDEVNAME=retrotope-timer("));
   ble.print(beamstate);
   ble.print(F(")"));
   char buffer[20];
   sprintf(buffer, "%lu", time);
   ble.println(buffer);
   if (!ble.waitForOK()) fatal_error(ERR_4); }

void do_clock_sync(void) {
   /* This is a special mode that compares the clock frequency of two units and displays it via the serial monitor.
       The LED driver output (D9-R11) should be connected to the IR receiver input (D11) of each other,
       plus a ground lead. Both units transmit and time the receive pulsesd from the other, although
       typically only one is connected to the serial monitor.
   */
   int old_falling_pulses;
   PRR1 &= ~PRTIM1;        // make sure Timer/Counter1 is powered up
   TCCR1B = 0;             // but turned off
   TCCR1A = 0;
   TCNT1 = 0;              // zero the counter to avoid bogus wraparound
   TIMSK1 = 0;             // don't generate interrupts
   OCR1A = (unsigned int) 46874; // (clk/(freq*2*prescale)) - 1
   // make this provide a period which is exactly an integer number of seconds, here 12 seconds.
   TCCR1A = 0b01000000;    // toggle OC1A on match, CTC mode
   TCCR1B = 0b00001101;    // CTC mode, clk/1024 prescaling, GO!
   PCICR  = 0x01;       // enable pin-change interrupts
   PCMSK0 = 0x80;       // enable on pin 11 (PCINT7)
   //while (!Serial) ;
   delay(2000);
   Serial.begin(115200);
   Serial.println("starting sync code");
   delay(500);
   old_falling_pulses = falling_pulses;
   while (1) {  // look for input pulses to our receiver
      if (falling_pulses != old_falling_pulses) {
         old_falling_pulses = falling_pulses;
         Serial.println(pulse_start_time); // wraps in 70 minutes
      }
      if (digitalRead(POWERDOWN_SW) == LOW) {
         Serial.println("power switch pushed");
         shutdown(1); } } }

void do_delay(int wait_msec) {
   if (wait_msec > 0) {
      #if USE_SLEEP // this code has issues...
      // we can't use the watchdog timer b/c minimum sleep time in 15 msec
      int interrupt_count = 0;
      unsigned long delay_start = millis(); // time now
      do { // HIBERNATE!
         PRR0 = 0b10001000 ; // power down TWI and Timer/Counter 1
         PRR1 = DEBUG ? 0b00011001 : 0b10011001 ; // power down Timer/Counters 3 and 4, USART1, and if not DEBUG, USB
         ADCSRA &= 0x7f;   // turn off A-to-D converter
         ACSR |= 0x80;     // turn off analog comparator
         SMCR = 0b00000001 ; // sleep mode control: idle (need timer0 running!)
         asm("sleep\n");
         SMCR = 0;
         ++interrupt_count; }
      while (millis() - delay_start < wait_msec);
      if (0) {
         Serial.print(interrupt_count);
         Serial.println(" interrupts"); }

      #else // not USE_SLEEP
      PRR1 = DEBUG ? 0b00011001 : 0b10011001 ; // power down Timer/Counters 3 and 4, USART1, and if not DEBUG, USB
      ADCSRA &= 0x7f;   // turn off A-to-D converter
      ACSR |= 0x80;     // turn off analog comparator
      delay(wait_msec);
      #endif
   } }

void update_led(struct led_status * p, bool light) {
   if (p->cycle_count == 0 && light)
      digitalWrite(p->led, HIGH); // turn on
   if (p->cycle_count == p->on_time)
      digitalWrite(p->led, LOW);  // end of on time: turn off
   if ((p->cycle_count += MINOR_CYCLE) > p->period)
      p->cycle_count = 0; }       // end of off time

//----------------------------------------------------------------------------------------
//  startup code
//----------------------------------------------------------------------------------------

bool startup_mode (int mode) { // choose a special startup mode
   unsigned long wait_start;
   delay(1000);
   flashlight(LED_RED, 300, mode); // flash red light to indicate which mode, 1..n
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
   else if (startup_mode(2))      // mode 2: special wired clock sync to second unit
      do_clock_sync();               // (this never returns)
   else if (startup_mode(3))      // mode 3: nonce error
      flashlight(LED_RED, 300, 3); }

void setup() {
   unsigned long startwait_millis;

   pinMode(POWERUP_FET, OUTPUT);     // so that we stay alive after the on/off button push is over,
   digitalWrite(POWERUP_FET, HIGH);  // turn on the power MOSFET as quickly as possible
   startwait_millis = millis();

   pinMode(POWERDOWN_SW, INPUT_PULLUP);
   pinMode(BATTERY_VOLTAGE, INPUT); // analog input
   pinMode(IR_RECEIVE, INPUT_PULLUP);
   pinMode(IR_TRANSMIT, OUTPUT);
   pinMode(TEST_INPUT, INPUT_PULLUP);
   digitalWrite(IR_TRANSMIT, HIGH); // disable IR transmitterx
   pinMode(LED_YELLOW, OUTPUT);
   pinMode(LED_BLUE, OUTPUT);
   pinMode(LED_GREEN, OUTPUT);
   pinMode(LED_RED, OUTPUT);
   flashlights(100, 3); // 100 msec flash, 3 times

   if (DEBUG) {
      //while (!Serial) ;
      delay(2000);
      Serial.begin(115200);
      Serial.println("WalkSensor started");
      delay(500); }

   #if BLUETOOTH
   // Reset and disable the Bluetooth module until we figure out whether we are a receiver or a transmitter
   if (!ble.begin(VERBOSE_MODE)) fatal_error(ERR_1);
   if (!ble.factoryReset()) fatal_error(ERR_2);
   ble.sendCommandCheckOK("AT+HWModeLED=DISABLE");  // turn off mode LED to save power
   ble.sendCommandCheckOK("AT+GAPSTOPADV");         // stop advertising
   #endif

   starttime_millis = millis();                 // record our starting time
   while (digitalRead(POWERDOWN_SW) == LOW) {   // wait for the power button to be released
      if (starttime_millis - startwait_millis > 5000) {  // if it is pushed for 5 seconds or more
         special_startup();                     // choose a special startup mode
         break; }
      starttime_millis = millis();              // record another starting time
   }
   delay(DEBOUNCE_DELAY);

   #if BOTH_XMT_RCV
   transmitter = true;
   receiver = true;
   #else     // see if we are receiving the beam from the other unit, and configure accordingly
   PCICR  = 0x01;       // enable pin-change interrupts
   PCMSK0 = 0x80;       // enable on pin 11 (PCINT7)
   delay(CHK_RCV_TIME); // wait to see if we receive any IR pulses
   if (DEBUG) {
      Serial.print("rcv interrupts: ");
      Serial.println(falling_pulses); }
   if (falling_pulses > (CHK_RCV_TIME / IR_XMT_PERIOD / 2)) // become the receiver if we saw at least half the pulses transmitted
      receiver = true;
   else transmitter = true; // otherwise become the transmitter
   #endif
   falling_pulses = 0;

   if (receiver) {  // configure the Bluetooth module
      #if BLUETOOTH
      ble.echo(false);
      if (DEBUG) ble.info();
      ble.sendCommandCheckOK("AT+BLEPOWERLEVEL=4");   //max transmit power
      ble.sendCommandCheckOK("AT+GAPINTERVALS=,,50,,50"); // 50 msec advertising interval forever
      set_devname('0', name_change_time);             // set the timestamp in our name to "now"
      name_change_time = millis() - starttime_millis; // the last time we changed our name
      ble.sendCommandCheckOK("AT+GAPSTARTADV");       // start advertising
      #endif
   }
   else { // not receiver
      PCICR  = 0x00; // disable pin-change interrupts
   }

   if (DEBUG) {
      Serial.print("initialization is done; we are ");
      if (receiver) Serial.print("receiver ");
      if (transmitter) Serial.print("transmitter");
      Serial.println(); }

   if (0) { // test code to flash the battery voltage; used to verify calibration without USB power
      uint16_t v = read_battery_voltage();
      digitalWrite(LED_RED, HIGH); delay(1000);
      for (uint16_t i = 0; i < v / 1000; ++i) { // units
         digitalWrite(LED_BLUE, HIGH); delay(500);
         digitalWrite(LED_BLUE, LOW); delay(500); }
      for (uint16_t i = 0; i < (v % 1000) / 100; ++i) { // tenths
         digitalWrite(LED_YELLOW, HIGH); delay(500);
         digitalWrite(LED_YELLOW, LOW); delay(500); }
      delay(1000);
      digitalWrite(LED_RED, LOW); }

} //end setup()

//----------------------------------------------------------------------------------------
//  main repeat loop
//----------------------------------------------------------------------------------------

void loop() {
   unsigned long timenow;

   //**** delay for a minor cycle time

   do_delay(MINOR_CYCLE);
do_checks:
   timenow = millis() - starttime_millis;

   //**** check for auto-powerdown

   if (!no_powerdown && timenow > (unsigned long)POWERDOWN_MINUTES * 60 * 1000) {
      if (DEBUG) Serial.println("Auto power down");
      shutdown(3); }

   //*** check the battery voltage every so often

   if (battery_time == 0) {
      ADCSRA |= 0x80;   // turn on A-to-D converter
      ACSR &= 0x7f;     // turn on analog comparator
      voltage = read_battery_voltage();
      if (STOP_IF_BATTERY_LOW && voltage < BATTERY_FAIL) {  // battery failing?
         if (DEBUG) Serial.println("battery failure");
         shutdown(5); } }
   if ((battery_time += MINOR_CYCLE) > BATTERY_CHK_PERIOD)
      battery_time = 0;

   //**** check for status light changes due

   if (led_battery.cycle_count == 0) {
      led_battery.led = (voltage < BATTERY_WEAK) ? LED_RED : LED_GREEN;
      if (DEBUG) {
         Serial.print("battery_led = "); Serial.print(led_battery.led);
         Serial.print(" voltage="); Serial.println(voltage); } }
   update_led (&led_battery, voltage < BATTERY_WEAK); // new: only flash if battery is weak, not if it is good
   if (transmitter)
      update_led(&led_transmitting, true);
   if (receiver) {
      if (rcv_state == RCVSTATE_BEAMSEEN)
         update_led(&led_receiving, true);
      else if (rcv_state == RCVSTATE_BEAMWAIT) {
         update_led(&led_beamlost_1, true);
         update_led(&led_beamlost_2, true); } }

   //**** check the powerdown pushbutton

   if (digitalRead(POWERDOWN_SW) == LOW) {
      if (DEBUG) Serial.println("power switch pushed");
      shutdown(1); }

   //**** check for receiver or transmitter IR activity

   if (receiver) {
      ir_rcv_time += MINOR_CYCLE;
      if (ir_rcv_time >= IR_RCV_PERIOD) { // end of receive sampling period
         switch (rcv_state) {
            case RCVSTATE_BEAMSEEN: // we think we're seeing the beam
               if (falling_pulses <= IR_RCV_PERIOD / IR_XMT_PERIOD - MISSING_PULSE_THRESHOLD) { // if we missed at least 2 or so pulses
                  rcv_state = RCVSTATE_BEAMBROKEN;  // "beam break": start a constant timestamp now
                  beam_lost_time = timenow;
                  set_devname('1', beam_lost_time);
                  digitalWrite(LED_BLUE, HIGH);     // constant blue light
               }
               else { // we are still seeing the beam
                  if (timenow - name_change_time >= RCV_NAMECHANGE_TIME) {
                     name_change_time = timenow;
                     set_devname('0', name_change_time); // change the timestamp in the name
                  } }
               break;
            case RCVSTATE_BEAMBROKEN:  // beam broken: stretch it out to 5 seconds
               if (timenow - beam_lost_time >= RCV_BREAK_TIME) { // end of "stretched" beam lost time
                  digitalWrite(LED_BLUE, LOW);   // turn off the constant blue light
                  rcv_state = RCVSTATE_BEAMWAIT;  // enter "wait for beam to return" state
               }
               break;
            case RCVSTATE_BEAMWAIT:  // wait for beam to return
               if (falling_pulses <= IR_RCV_PERIOD / IR_XMT_PERIOD - MISSING_PULSE_THRESHOLD) { // if we missed at least 2 or so pulses
                  // beam is still broken, so must be "lost": we'll do special blue/yellow flashing lights
               }
               else { // we've seen the beam return
                  digitalWrite(LED_BLUE, LOW);   // turn off the blue/yellow flashing lights
                  digitalWrite(LED_YELLOW, LOW);
                  rcv_state = RCVSTATE_BEAMSEEN;
                  name_change_time = timenow + RCV_NAMECHANGE_TIME; // setup to change to '0' name soon
               }
               break;
            default: fatal_error(ERR_3); }
         falling_pulses = 0;  // start new sampling period
         ir_rcv_time = 0; } }

   if (transmitter) { // transmitter
      if (ir_xmt_time == 0) { // time to output a 38Khz burst from the IR transmitter
         PRR1 &= ~PRTIM1;        // make sure Timer/Counter1 is powered up
         TCCR1B = 0;             // but turned off
         TCCR1A = 0;
         TCNT1 = 0;              // zero the counter to avoid bogus wraparound
         TIMSK1 = 0;             // don't generate interrupts
         OCR1A = F_CPU / IR_FREQ / 2 - 1;  // the compare limit is twice the frequency
         TCCR1A = 0b01000000;    // toggle OC1A on match, CTC mode
         TCCR1B = 0b00001001;    // CTC mode, no prescaling, clk/1, GO!
         do_delay(IR_ON);
         TCCR1B = 0;             // turn off the timer
         TCCR1A = 0;
         digitalWrite(IR_TRANSMIT, HIGH); // leave with the transmitter disabled
         do_delay(MINOR_CYCLE - IR_ON);  // delay for the rest of a cycle to get back in sync
         if (IR_XMT_PERIOD > MINOR_CYCLE) ir_xmt_time = MINOR_CYCLE; // account for one cycle out of 2 or more; else leave at 0 to do again
         goto do_checks; }
      ir_xmt_time += MINOR_CYCLE;
      if (ir_xmt_time >= IR_XMT_PERIOD)
         ir_xmt_time = 0; } }
//*

