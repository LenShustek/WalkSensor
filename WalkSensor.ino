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
        battery ok: light continually blinks green (.05s every 5s)
        battery marginal: light continually blinks red (.05s every 5s)
        battery too low: light blinks long red (5x: .5s every 1s), then unit shuts off
      push the button to turn it off
        light blinks red once (.5s), then shuts off
      after 8 hours
        unit turns off automatically
  mode: when the unit turns on, it tries to detect an opposing IR transmitter
    if no IR signal detected, it becomes the IR transmitter
      yellow status light blinks continually (.05s every 1s)
    if an IR signal is detected: it becomes the IR receiver
      when the IR signal is interrupted for >.2s
        blue light turns on until IR signal returns
        send a Bluetooth message (format TBD)


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

  TODO:
   - verify IR LED current
   - measure power current
   - work on better sleep/hibernate mode to reduce current
   - change to hardware timer to generate IR LED modulation 
   - experiment with lower duty cycle for transmit: can it be as reliable?
   - change shutdown to: 1 hour after last event
   - factory reset the Bluetooth module if we're the transmiter so that our beacon name is erased

*****************************************************************************************************/
#define VERSION "1.0"

#define DEBUG true
#define BLUETOOTH true
#define USE_INTERRUPT true
#define STOP_IF_BATTERY_LOW true
#define USE_SLEEP false

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
#define XLED_PERIOD 1000          // transmit status LED cycle time in milliseconds
#define BLED_ON 50                // battery LED on time in milliseconds
#define BLED_PERIOD 5000          // battery LED cycle time in millliseconds
#define BATTERY_CHK_PERIOD 5000   // battery check time in milliseconds

#define IR_ON 1                   //* IR burst transmit time in milliseconds
#define PULSE_WIDTH_PERCENT 97    //* what percent of an IR pulse we need to see to count it
#define MISSING_PULSE_THRESHOLD 2 //* how many missing pulses constitute a beam break
#define CHK_RCV_TIME 500          // how long to wait to see if the other guy is sending, in msec

#define IR_FREQ 38000L            // tuned IR receiver frequency
#define POWERDOWN_HOURS 8         // after how many hours to power down automatically
#define DEBOUNCE_DELAY 25         // debounce delay in msec

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
#define IR_RECEIVE 11       // active low input pin if IR receiver sees 10 or more pulses at 38 Khz (must be port B!)

#define BATTERY_VOLTAGE A4  // 0..1023 from A-to-D converter from battery voltage divider
#define BATTERY_R_TOP 470   // top battery resistor divider, in Kohms
#define BATTERY_R_BOT 470   // bottom battery resistor divider, in Kohms
#define AREF_MV  3300       // internal analog reference voltage, in millivolts

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// global variables

bool receiver = false;    // are we the receiver?
volatile int falling_pulses = 0;   // how many IR pulses have we received
int ir_time = 0;          // count milliseconds of IR transmit cycle
int battery_time = 0;     // count milliseconds of battery check time

struct led_status { // LED state
  int led;
  int on_time; // msec
  int period;  // msec
  int cycle_count;
}
led_battery = {LED_GREEN, BLED_ON, BLED_PERIOD, 0},
led_transmitting = {LED_YELLOW, XLED_ON, XLED_PERIOD, 0};

#if USE_INTERRUPT
// In order to compile the interrupt routine, you must comment out line 43 in Adafruit_BluefruitLE_UART.h
// located in Len\Documents\Arduino\libraries\Adafruit_BluefruitLE_nRF51-master
// See https://forums.adafruit.com/viewtopic.php?f=24&t=83256&start=15 about this library bug.

ISR(PCINT0_vect) { // interrupt for falling or rising edge of IR receiver
  // count only those low pulses that are "almost" as wide as what the transmitter sends
  static unsigned long pulse_start_time;
  if (digitalRead(IR_RECEIVE) == LOW) {  // falling edge
    pulse_start_time = micros();
  }
  else {  // rising edge
    if (micros() - pulse_start_time > (unsigned long)IR_ON * 1000 * PULSE_WIDTH_PERCENT / 100) // "almost" == 95% or so
      ++falling_pulses;
  }
}

#else
void simulate_interrupt(void) {
  // polling hack if we can't get the pin change interrupt to work on the 32u4
  static int last_IR_RECEIVE = HIGH;
  int current_IR_RECEIVE = digitalRead(IR_RECEIVE);
  if (current_IR_RECEIVE != last_IR_RECEIVE) {
    if (current_IR_RECEIVE == LOW) ++falling_pulses;
    last_IR_RECEIVE = current_IR_RECEIVE;
  }
}
#endif

enum {ERR_1, ERR_2, ERR_3, ERR_4, ERR_5, ERR_6};
void fatal_error(int code) {
  while (1) {
    for (int i = 0; i < 3; ++i) {
      digitalWrite(LED_RED, HIGH);
      delay(500);
      digitalWrite(LED_RED, LOW);
      delay(500);
    }
    for (int i = 0; i < code + 1; ++i) {
      digitalWrite(LED_GREEN, HIGH);
      delay(500);
      digitalWrite(LED_GREEN, LOW);
      delay(500);
    }
  }
}

uint16_t read_battery_voltage (void) { // return voltage in millivolts
  uint16_t aval, Vmv;
  aval = analogRead(BATTERY_VOLTAGE);
  // aval = (V * (B/(T+B)) / Vref) * 1024; solve for V
  Vmv = ((unsigned long)aval * AREF_MV * (BATTERY_R_TOP + BATTERY_R_BOT)) / (1024UL * BATTERY_R_BOT);
  if (DEBUG) {
    Serial.print("battery: aval="); Serial.print(aval);
    Serial.print(", mV="); Serial.println(Vmv);
  }
  return Vmv;
}

bool chk_ble_connection(void) {
  static bool ble_connected = false;
  bool conn = ble.isConnected();
  if (DEBUG && conn != ble_connected) { // if it changed
    Serial.print ("Bluetooth ");
    Serial.println(conn ? "connected" : "disconnected");
  }
  ble_connected = conn;
  return ble_connected;
}


void setup() {

  pinMode(POWERUP_FET, OUTPUT);     // so that we stay alive after the on/off button push is over,
  digitalWrite(POWERUP_FET, HIGH);  // turn on the power MOSFET as quickly as possible

  pinMode(POWERDOWN_SW, INPUT_PULLUP);
  pinMode(BATTERY_VOLTAGE, INPUT); // analog input

  pinMode(IR_RECEIVE, INPUT_PULLUP);
  pinMode(IR_TRANSMIT, OUTPUT);
  digitalWrite(IR_TRANSMIT, HIGH); // transmitter disabled

  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  for (int i = 0; i < 3; ++i) {  // flash lights to show we're alive
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    //  digitalWrite(LED_RED, HIGH);
    delay(100);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    delay(100);
  }

  if (DEBUG) {
    //while (!Serial) ;
    delay(2000);
    Serial.begin(115200);
    Serial.println("WalkSensor started");
  }
  delay(500);

  delay(DEBOUNCE_DELAY); // wait for button push debounce
  while (digitalRead(POWERDOWN_SW) == LOW) ;  // wait for the power button to be released
  delay(DEBOUNCE_DELAY);

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
    Serial.println(falling_pulses);
  }
  receiver = falling_pulses > (CHK_RCV_TIME / IR_XMT_PERIOD / 2); // become the receiver if we saw at least half the pulses transmitted
  falling_pulses = 0;

  if (receiver) {
#if BLUETOOTH
    digitalWrite(LED_BLUE, HIGH); // signal "setting up Bluetooth"
    if (!ble.begin(VERBOSE_MODE)) fatal_error(ERR_1);
    if (!ble.factoryReset()) fatal_error(ERR_2);
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
    PCICR  = 0x00; // disable enable pin-change interrupts
  }

  if (DEBUG) {
    Serial.print("initialization is done; we are the ");
    Serial.println(receiver ? "receiver" : "transmitter");
  }

  if (0) { // flash the battery voltage to verify calibration without USB power
    uint16_t v = read_battery_voltage();
    digitalWrite(LED_RED, HIGH);
    delay(1000);
    for (uint16_t i = 0; i < v / 1000; ++i) { // units
      digitalWrite(LED_BLUE, HIGH);
      delay(500);
      digitalWrite(LED_BLUE, LOW);
      delay(500);
    }
    for (uint16_t i = 0; i < (v % 1000) / 100; ++i) { // tenths
      digitalWrite(LED_YELLOW, HIGH);
      delay(500);
      digitalWrite(LED_YELLOW, LOW);
      delay(500);
    }
    delay(1000);
    digitalWrite(LED_RED, LOW);
  }

} //end setup()

void shutdown(int flashlights) {
  if (DEBUG) Serial.println("shutdown!");
  for (int i = 0; i < flashlights; ++i) {
    digitalWrite(LED_RED, HIGH);
    delay(500);
    digitalWrite(LED_RED, LOW);
    delay(500);
  }
  digitalWrite(POWERUP_FET, LOW);  // turn off our power
  while (1) ; // wait for oblivion
}

void beam_break(void) {
  if (DEBUG) Serial.println("beam broken");
  if (chk_ble_connection()) {
    ble.println("AT+BLEUARTTX=beambreak ");
    bool ok = ble.waitForOK();
    if (DEBUG) {
      Serial.print("sent message, status=");
      Serial.println(ok);
    }
  }
  else if (DEBUG) Serial.println("not connected");
}

void update_led(struct led_status *p) {
  if (p->cycle_count == 0)
    digitalWrite(p->led, HIGH);
  if (p->cycle_count == p->on_time)
    digitalWrite(p->led, LOW);
  if ((p->cycle_count += MINOR_CYCLE) > p->period)
    p->cycle_count = 0;
}



void loop() {

  static uint16_t voltage; // the battery voltage we last read
  static bool beam_broken = false; // if receiver, was the beam recently interrupted?

#if USE_SLEEP
  int interrupt_count = 0;
  unsigned long delay_start = millis(); // time now
  do {                  // HIBERNATE!
    PRR0 = 0b10001000 ; // power down TWI and Timer/Counter 1
    PRR1 = 0b00011000 ; // power down Timer/Counters 3 and 4
    SMCR = 0b00000001 ; // sleep mode control: idle
    asm("sleep\n");
    SMCR = 0;
    ++interrupt_count;
  } while (millis() - delay_start < MINOR_CYCLE);
  if (0) {
    Serial.print(interrupt_count);
    Serial.println(" interrupts");
  }
#else
  delay(MINOR_CYCLE);
#endif

#if !USE_INTERRUPT
  simulate_interrupt();   // hack if we can't get interrupts to work
#endif

do_checks:

  //**** check for auto-powerdown

  if (millis() > (unsigned long)POWERDOWN_HOURS * 60 * 60 * 1000) {
    if (DEBUG) Serial.println("Auto power down");
    shutdown(3);
  }

  //*** check battery voltage every so often

  if (battery_time == 0) {
    voltage = read_battery_voltage();
    if (STOP_IF_BATTERY_LOW && voltage < BATTERY_FAIL) {  // battery failing?
      if (DEBUG) Serial.println("battery failure");
      shutdown(5);
    }
  }
  if ((battery_time += MINOR_CYCLE) > BATTERY_CHK_PERIOD)
    battery_time = 0;

  //**** check for status light changes

  if (led_battery.cycle_count == 0) {
    led_battery.led = (voltage < BATTERY_WEAK) ? LED_RED : LED_GREEN;
    if (DEBUG) {
      Serial.print("battery_led = "); Serial.print(led_battery.led);
      Serial.print(" voltage="); Serial.println(voltage);
    }
  }
  update_led (&led_battery);
  if (!receiver)
    update_led(&led_transmitting);

  //**** check for powerdown pushbutton

  if (digitalRead(POWERDOWN_SW) == LOW) {
    if (DEBUG) Serial.println("power switch pushed");
    shutdown(1);
  }

  //**** check for receiver or transmitter IR activity

  if (receiver) {
    if (beam_broken) {
      if (falling_pulses > 1000 / IR_XMT_PERIOD) { // if we've seen at least 1 second of beam
        digitalWrite(LED_BLUE, LOW);
        if (DEBUG) {
          Serial.print("resuming after pulses: ");
          Serial.println(falling_pulses);
        }
        beam_broken = false;  // restart looking for a break
        ir_time = 0;
        falling_pulses = 0;
      }
    }
    else { // beam wasn't recently interrupted
      ir_time += MINOR_CYCLE;
      if (ir_time >= IR_RCV_PERIOD) { // end of sampling period
        if (DEBUG) {
          Serial.print("end sample period, pulses: ");
          Serial.println(falling_pulses);
        }
        if (falling_pulses <= IR_RCV_PERIOD / IR_XMT_PERIOD - MISSING_PULSE_THRESHOLD) { // if we missed at least 2 or so pulses
          digitalWrite(LED_BLUE, HIGH);
          beam_break();        // the beam must have been broken
          beam_broken = true;
        }
        ir_time = 0;
        falling_pulses = 0;    // restart sample period
      }
    }
  }
  else { // transmitter
    if (ir_time == 0) {
      //tone(IR_TRANSMIT, IR_FREQ, IR_ON); // 1-2 msec 38 khz burst;  DOESN'T WORK!?!
      //digitalWrite(IR_TRANSMIT, HIGH); // stop with transmitter disabled
      for (int i = 0; i < (int)((IR_ON * IR_FREQ) / 1000); ++i) {
        // This loop has some jitter because timer0 interrupts occur every so often. It doesn't seem to
        // bother the tuned IR receiver, but we should fix that by using Timer1 as an oscillator instead.
        digitalWrite(IR_TRANSMIT, LOW);
        delayMicroseconds(1); // these delays are set experimentally because of digitslWrite() delays
        digitalWrite(IR_TRANSMIT, HIGH);
        delayMicroseconds(1);
      }
      delay(MINOR_CYCLE - IR_ON); // delay for the rest of a cycle to get back in sync
      if (IR_XMT_PERIOD > MINOR_CYCLE) ir_time = MINOR_CYCLE;
      goto do_checks;
    }
    ir_time += MINOR_CYCLE;
    if (ir_time >= IR_XMT_PERIOD)
      ir_time = 0;
  }


}
