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
      the yellow light blinks every 2 seconds
    if an IR signal is detected: it becomes the IR receiver
      the blue light blinks every 2 seocnds
      when the IR signal is interrupted,
        the blue light turns on for 2 seconds
        it sends a Bluetooth "beambreak" message
        the blue light turns off until IR beam returns
      if the beam break lasts more than 10 seconds,
        it send a Bluetooth "beamlost" message and flashes the blue and yellow lights 3 times

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

   TODO:
   - experiment with a lower duty cycle for transmit; can it be as reliable?
   - work on better sleep/hibernate mode to reduce current?
     (Snoozelib doesn't work; try https://github.com/adafruit/Adafruit_SleepyDog)

*****************************************************************************************************/
#define VERSION "1.1"

#define DEBUG false
#define BLUETOOTH true
#define USE_INTERRUPT true
#define STOP_IF_BATTERY_LOW true
#define USE_SLEEP false

/* power utilization without going into sleep mode:
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
#define IR_RCV_PERIOD 50          //* IR receive sample period
#define XLED_ON 50                // transmit status LED on time in milliseconds
#define XLED_PERIOD 2000          // transmit status LED cycle time in milliseconds
#define RLED_ON 50                // receive status LED on time in milliseconds
#define RLED_PERIOD 2000          // receive status LED cycle time in milliseconds
#define BLED_ON 50                // battery LED on time in milliseconds
#define BLED_PERIOD 5000          // battery LED cycle time in millliseconds
#define BATTERY_CHK_PERIOD 5000   // battery check time in milliseconds

#define IR_ON 1                   //* IR burst transmit time in milliseconds
#define PULSE_WIDTH_PERCENT 97    //* what percent of an IR pulse we need to see to count it
#define MISSING_PULSE_THRESHOLD 2 //* how many missing pulses constitute a beam break
#define CHK_RCV_TIME 500          // how long to wait to see if the other guy is sending, in msec
#define RCV_RECOVER_TIME 1500     // how long to wait for beam to be recovered after break, in msec

#define IR_FREQ 38000L            // tuned IR receiver frequency
#define BEAMLOST_SECONDS 10       // after how many seconds to send "beam lost" message
#define POWERDOWN_MINUTES 60      // after how many minutes to power down automatically
#define DEBOUNCE_DELAY 25         // debounce delay in msec
#define MSG_DUPS 3                // how many duplicate messages to send
#define MSG_DELAY 50              // how many milliseconds between duplicate messages

#define BATTERY_WEAK 5000         // millivolt level (out of 6000) to warn about low battery 
#define BATTERY_FAIL 3900         // millivolt level (out of 6000) to treat as battery failure

// hardware configuration

// Beware: analog pins A0..A5 cannot be programmed as digital pins on an AVR 32u4!

#define POWERUP_FET 13      // output pin high to keep us powered up by turning the MOSFET on
#define POWERDOWN_SW 10     // this input pin is low if the on/off button is pushed

#define LED_YELLOW 5        // LED pins: are active high
#define LED_BLUE 6
#define LED_GREEN 2
#define LED_RED 12

#define IR_TRANSMIT 9       // active low output pin for IR transmit through constant-current LED driver
//.                            this is PB5, but more importantly, OC1A for Timer/Counter1
#define IR_RECEIVE 11       // active low input pin if IR receiver sees 10 or more pulses at 38 Khz (must be port B!)

#define BATTERY_VOLTAGE A4  // 0..1023 from A-to-D converter from battery voltage divider
#define BATTERY_R_TOP 205   // top battery resistor divider, in Kohms
#define BATTERY_R_BOT 205   // bottom battery resistor divider, in Kohms
#define AREF_MV  3300       // internal analog reference voltage, in millivolts

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// global variables

bool receiver = false;            // are we the receiver?
volatile int falling_pulses = 0;  // how many IR pulses have we received
int ir_time = 0;                  // count milliseconds of IR transmit cycle
int battery_time = 0;             // count milliseconds of battery check time
unsigned long starttime_millis;   // starting time, or time of last activity

struct led_status { // LED state
   int led;
   int on_time; // msec
   int period;  // msec
   int cycle_count; }
led_battery = {LED_GREEN, BLED_ON, BLED_PERIOD, 0 },
led_receiving = {LED_BLUE, RLED_ON, RLED_PERIOD, 0 },
led_transmitting = {LED_YELLOW, XLED_ON, XLED_PERIOD, 0 };

#if USE_INTERRUPT
// In order to compile the interrupt routine, you must comment out line 43 in Adafruit_BluefruitLE_UART.h
// located in Len\Documents\Arduino\libraries\Adafruit_BluefruitLE_nRF51-master
// See https://forums.adafruit.com/viewtopic.php?f=24&t=83256&start=15 about this library bug,
// and also https://forums.adafruit.com/viewtopic.php?f=57&t=105935

ISR(PCINT0_vect) { // interrupt for falling or rising edge of IR receiver
   // count only those low pulses that are "almost" as wide as what the transmitter sends
   static unsigned long pulse_start_time;
   if (digitalRead(IR_RECEIVE) == LOW) {  // falling edge
      pulse_start_time = micros(); }
   else {  // rising edge
      if (micros() - pulse_start_time > (unsigned long)IR_ON * 1000 * PULSE_WIDTH_PERCENT / 100) // "almost" == 95% or so
         ++falling_pulses; } }
#else
void simulate_interrupt(void) {
   // polling hack if we can't get the pin change interrupt to work on the 32u4
   static int last_IR_RECEIVE = HIGH;
   int current_IR_RECEIVE = digitalRead(IR_RECEIVE);
   if (current_IR_RECEIVE != last_IR_RECEIVE) {
      if (current_IR_RECEIVE == LOW) ++falling_pulses;
      last_IR_RECEIVE = current_IR_RECEIVE; } }
#endif

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

uint16_t read_battery_voltage (void) { // return voltage in millivolts
   uint16_t aval, Vmv;
   aval = analogRead(BATTERY_VOLTAGE);
   // aval = (V * (B/(T+B)) / Vref) * 1024; solve for V
   Vmv = ((unsigned long)aval * AREF_MV * (BATTERY_R_TOP + BATTERY_R_BOT)) / (1024UL * BATTERY_R_BOT);
   if (DEBUG) {
      Serial.print("battery: aval="); Serial.print(aval);
      Serial.print(", mV="); Serial.println(Vmv); }
   return Vmv; }

bool chk_ble_connection(void) {
   static bool ble_connected = false;
   bool conn = ble.isConnected();
   if (DEBUG && conn != ble_connected) { // if it changed
      Serial.print ("Bluetooth ");
      Serial.println(conn ? "connected" : "disconnected"); }
   ble_connected = conn;
   return ble_connected; }

void flashlights(int duration, int times) {
   for (int i = 0; i < times; ++i) {  // flash lights to show we're alive
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

void setup() {
   pinMode(POWERUP_FET, OUTPUT);     // so that we stay alive after the on/off button push is over,
   digitalWrite(POWERUP_FET, HIGH);  // turn on the power MOSFET as quickly as possible

   pinMode(POWERDOWN_SW, INPUT_PULLUP);
   pinMode(BATTERY_VOLTAGE, INPUT); // analog input
   pinMode(IR_RECEIVE, INPUT_PULLUP);
   pinMode(IR_TRANSMIT, OUTPUT);
   digitalWrite(IR_TRANSMIT, HIGH); // disable IR transmitter
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

   delay(DEBOUNCE_DELAY); // wait for button push debounce
   while (digitalRead(POWERDOWN_SW) == LOW) ;  // wait for the power button to be released
   delay(DEBOUNCE_DELAY);
   starttime_millis = millis(); // record start time

   #if USE_INTERRUPT
   PCICR  = 0x01; // enable pin-change interrupts
   PCMSK0 = 0x80; // enable on pin 11 (PCINT7)
   delay(CHK_RCV_TIME);  // wait a bit to see if we receive any IR pulses from the other guy
   #else
   unsigned long timenow = millis();
   while (millis() - timenow < CHK_RCV_TIME) simulate_interrupt(); // wait a bit to see if we receive any IR pulses
   #endif

   if (DEBUG) {
      Serial.print("rcv interrupts: ");
      Serial.println(falling_pulses); }
   receiver = falling_pulses > (CHK_RCV_TIME / IR_XMT_PERIOD / 2); // become the receiver if we saw at least half the pulses transmitted
   falling_pulses = 0;

   #if BLUETOOTH
   // We reset the Bluetooth module even if we're a transmitter, so that it doesn't broadcast our previous beacon name
   if (!ble.begin(VERBOSE_MODE)) fatal_error(ERR_1);
   if (!ble.factoryReset()) fatal_error(ERR_2);
   #endif

   if (receiver) {
      #if BLUETOOTH
      digitalWrite(LED_BLUE, HIGH); // signal "setting up Bluetooth"
      ble.echo(false);
      if (DEBUG) ble.info();
      ble.println("AT+BLEGETADDR"); // get our 32-bit unique address
      ble.readline();  // "AA:BB:CC:DD:EE:FF"
      char addr[20];
      strcpy(addr, ble.buffer);
      if (!ble.waitForOK()) fatal_error(ERR_3);
      ble.print(F("AT+GAPDEVNAME=WalkSensor."));
      ble.println(addr); // append our unique address to our name
      if (!ble.waitForOK()) fatal_error(ERR_4);
      ble.sendCommandCheckOK("AT+HWModeLED=DISABLE"); // turn off mode LED to save power
      chk_ble_connection();
      digitalWrite(LED_BLUE, LOW);
      #endif
   }
   else { // transmitter
      PCICR  = 0x00; // disable pin-change interrupts
   }

   if (DEBUG) {
      Serial.print("initialization is done; we are the ");
      Serial.println(receiver ? "receiver" : "transmitter"); }

   if (0) { // test code flash the battery voltage, to verify calibration without USB power
      uint16_t v = read_battery_voltage();
      digitalWrite(LED_RED, HIGH);
      delay(1000);
      for (uint16_t i = 0; i < v / 1000; ++i) { // units
         digitalWrite(LED_BLUE, HIGH);
         delay(500);
         digitalWrite(LED_BLUE, LOW);
         delay(500); }
      for (uint16_t i = 0; i < (v % 1000) / 100; ++i) { // tenths
         digitalWrite(LED_YELLOW, HIGH);
         delay(500);
         digitalWrite(LED_YELLOW, LOW);
         delay(500); }
      delay(1000);
      digitalWrite(LED_RED, LOW); }

} //end setup()

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

void shutdown(int flashcount) {
   if (DEBUG) Serial.println("shutdown!");
   flashlights(500, flashcount); // 500 msec flashes
   digitalWrite(POWERUP_FET, LOW);  // turn off our power
   while (1) ; // wait for oblivion
}

void send_msg(const char *msg) { // send a bluetooth message several times
   static int msg_number = 0;
   char buffer[40];
   if (DEBUG) Serial.println(msg);
   if (chk_ble_connection()) {  // we are connected
      unsigned long starttime = millis();
      ++msg_number;
      for (int cnt = 1; cnt <= MSG_DUPS; ++cnt) {
         ble.print("AT+BLEUARTTX=");
         sprintf(buffer, "%s,%d,%lu. ", msg, msg_number, millis() - starttime);
         ble.println(buffer);
         bool ok = ble.waitForOK();
         if (DEBUG) {
            Serial.print("sent message, status=");
            Serial.println(ok); }
         do_delay(MSG_DELAY); } }
   else if (DEBUG) Serial.println("not connected"); }

void update_led(struct led_status *p, bool light) {
   if (p->cycle_count == 0 && light)
      digitalWrite(p->led, HIGH);
   if (p->cycle_count == p->on_time)
      digitalWrite(p->led, LOW);
   if ((p->cycle_count += MINOR_CYCLE) > p->period)
      p->cycle_count = 0; }


void loop() {
   static uint16_t voltage; // the battery voltage we last read
   static bool beam_broken = false; // if receiver, was the beam recently interrupted?
   static bool beam_lost_msg_sent = false;
   static unsigned long beam_lost_time; // when we first noticed the beam was lost

   //**** delay for a minor cycle time

   do_delay(MINOR_CYCLE);
   #if !USE_INTERRUPT
   simulate_interrupt();   // hack if we can't get interrupts to work
   #endif
do_checks:

   //**** check for auto-powerdown

   if (millis() - starttime_millis > (unsigned long)POWERDOWN_MINUTES * 60 * 1000) {
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
   update_led (&led_battery, voltage < BATTERY_WEAK);
   if (receiver) update_led(&led_receiving, !beam_broken);
   else update_led(&led_transmitting, true);

   //**** check the powerdown pushbutton

   if (digitalRead(POWERDOWN_SW) == LOW) {
      if (DEBUG) Serial.println("power switch pushed");
      shutdown(1); }

   //**** check for receiver or transmitter IR activity

   if (receiver) {
      if (beam_broken) {
         if (falling_pulses > RCV_RECOVER_TIME / IR_XMT_PERIOD) { // if we've seen at least a second or so of beam
            digitalWrite(LED_BLUE, LOW);
            if (DEBUG) {
               Serial.print("resuming after pulses: ");
               Serial.println(falling_pulses); }
            beam_lost_msg_sent = beam_broken = false;  // restart looking for a break
            ir_time = 0;
            falling_pulses = 0; }
         else if (millis() - beam_lost_time > BEAMLOST_SECONDS * 1000) { // we never recovered the beam
            if (!beam_lost_msg_sent) {
               send_msg("beamlost ");
               for (int i = 0; i < 3; ++i) {  // flash blue and yellow lights
                  digitalWrite(LED_YELLOW, HIGH);
                  digitalWrite(LED_BLUE, HIGH);
                  delay(250);
                  digitalWrite(LED_YELLOW, LOW);
                  digitalWrite(LED_BLUE, LOW);
                  delay(250); } }
            beam_lost_msg_sent = true;
            digitalWrite(LED_BLUE, LOW); } }
      else { // beam wasn't recently interrupted
         ir_time += MINOR_CYCLE;
         if (ir_time >= IR_RCV_PERIOD) { // end of sampling period
            if (DEBUG) {
               Serial.print("end sample period, pulses: ");
               Serial.println(falling_pulses); }
            if (falling_pulses <= IR_RCV_PERIOD / IR_XMT_PERIOD - MISSING_PULSE_THRESHOLD) { // if we missed at least 2 or so pulses
               digitalWrite(LED_BLUE, HIGH);
               send_msg("beambreak ");        // the beam must have been broken
               beam_broken = true;
               beam_lost_time = starttime_millis = millis(); } // record "last activity" and "beam lost" times
            ir_time = 0;
            falling_pulses = 0;    // restart sample period
         } } }
   else { // transmitter
      if (ir_time == 0) { // time to output a 38Khz burst from the IR transmitter
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
         if (IR_XMT_PERIOD > MINOR_CYCLE) ir_time = MINOR_CYCLE; // account for one cycle out of 2 or more; else leave at 0 to do again
         goto do_checks; }
      ir_time += MINOR_CYCLE;
      if (ir_time >= IR_XMT_PERIOD)
         ir_time = 0; } }
//*

