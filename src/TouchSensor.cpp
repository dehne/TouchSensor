/****
 * This file is a part of the TouchSensor library. See README.md for details
 * 
 *****
 * 
 * TouchSensor V2.0.3, November 2025
 * Copyright (C) 2025 D.L. Ehnebuske
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 * 
 ****/
#include <TouchSensor.h>

namespace ts_isr {
constexpr uint8_t NONPIN = NUM_DIGITAL_PINS;                        // Value in clientList indicating pin number not stored in that element
static TouchSensor* volatile client[NUM_DIGITAL_PINS] = {nullptr};  // Pointers to active clients (or nullptr if no active client in a slot)
                                                                    //   indexed by the Arduino digital pin number to which they are attached
static volatile bool clientPending[NUM_DIGITAL_PINS] = {false};     // Whether a client's discharge is pending; indexed as per client[]
static volatile unsigned long startMicros[NUM_DIGITAL_PINS] = {0};  // micros() when the client's latest sampling cycle started
static volatile uint8_t clientList[NUM_DIGITAL_PINS];               // Compacted list of clients. Each entry is the pin number of an active
                                                                    //   clients.  if clientList[i] == NOPIN, that's the end of list
static uint8_t irqToPin[3][8];                                      // Map from pin-change IRQ number to the Arduino pins that IRQ serves
static struct InitTsISR {                                           // Initialize non-zero statics (Kinda hacky, no?)
  InitTsISR() {
    for(uint8_t i = 0; i < NUM_DIGITAL_PINS; i++) {                 //   clientList full of NONPIN
      clientList[i] = NONPIN;
    }
    uint8_t irqPinCount[3] = { 0 };                                 //   irqToPin[port][0..8] lists the pins served by specified irq
    for (uint8_t pin = 0; pin < NUM_DIGITAL_PINS; pin++) {          //     Entry == NONPIN means end of list of pins for this irq
      uint8_t irq = digitalPinToPCMSK(pin) == &PCMSK0 ? 0 : digitalPinToPCMSK(pin) == &PCMSK1 ? 1 : 2;
      irqToPin[irq][irqPinCount[irq]++] = pin;
    }
    for (uint8_t irq = 0; irq < 3; irq++) {
      for (; irqPinCount[irq] < 8 ; irqPinCount[irq]++) {
        irqToPin[irq][irqPinCount[irq]] = NONPIN;
      }
    }
  };
} initTsIsr;
static volatile uint8_t clientListIx = 0;                           // The index into clientList of last client to start a measurement cycle
static bool doClassInit = true;                                     // Set false by first invocation of begin() after doing class init work

/**
 * @brief   Interrupt service routine for Timer/Counter1
 * 
 * @details Timer/Counter1 is set up in TouchSensor::begin the first time it is called. Timer/Counter1 interrupts 
 *          on a regular basis. The ISR's purpose is to initiate a measurement cycle for clients. It goes through 
 *          the clients round-robin fashion, one each time it's invoked. It switches the chosen client's sensor 
 *          pin from OUTPUT in a HIGH state (sensor pins are set that way by capISR at the end of each 
 *          measurement cycle) to INPUT and records the client's startMicros.
 * 
 */
void timerISR() {
  #ifdef TS_SCOPE
  digitalWrite(TS_TIMER_ISR_PIN, HIGH);
  #endif

  // Increment clientListIx by 1 modulo the number of *actual* clients. 
  clientListIx = (clientListIx + 1) % NUM_DIGITAL_PINS;
  if (clientList[clientListIx] == NONPIN) {
    clientListIx = 0;
  }

  // curClient = the index into client[] for the client (if any) we'll start a measurement cycle for
  uint8_t curClient = clientList[clientListIx];

  // If there are no clients, nothing to do
  if (curClient >= NUM_DIGITAL_PINS) {
    return;
  }

  if (client[curClient]) {
    startMicros[curClient] = micros();
    if (clientPending[curClient]) {
      // The client still hasn't discharged! Force it's pin to go low (and so cause an interrupt)
      pinMode(curClient, OUTPUT);
      digitalWrite(curClient, LOW);
      clientPending[curClient] = false; // Show it was forced, not a real measurement
    } else {
      // The client is ready to go. Start the next measurement cycle
      pinMode(curClient, INPUT);
      clientPending[curClient] = true;
    }
  }
  #ifdef TS_SCOPE
  digitalWrite(TS_TIMER_ISR_PIN, LOW);
  #endif

}

/**
 * @brief Interrupt serivce routine for all the pin change interrupts.
 * 
 * @param vector    The interrupt vector (0..2) that caused the interrupt.
 */
void capISR(uint8_t irqNo) {
  #ifdef TS_SCOPE
  digitalWrite(TS_CAP_ISR_PIN, HIGH);                   // Mark entry to ISR
  #endif

  // Iterate through the pins that could possibly have caused this interrupt and process the one(s) that did. Which of the
  // 3 pin-change IRQs that caused the interrupt is in irqNo. The array irqToPin[][] for a given IRQ gives the Arduino pin 
  // numbers that port serves, up to a max of 8 pins. If the IRQ serves less than 8, the list is termnated by a NONPIN entry.
  for (uint8_t pinIx = 0; pinIx < 8 && irqToPin[irqNo][pinIx] != NONPIN; pinIx++) {
    // Okay, this is a little complicated and I'm bound to forget. So here's what's going on with each client. We're only 
    // interested in client pins that are LOW. HIGH client pins either caused the interrupt because they changed to HIGH 
    //(as a side effect of the timer kicking off a measurement cycle) or are HIGH because they haven't yet discharged (and 
    // so aren't involved in this interrupt). If the client isn't still pending, that means it was forced LOW during the 
    // timer ISR. In that case, by convention, the update specifies TS_MAX_MICROS as the discharge time to mark it as 
    // (probably) bogus. Unreasonably long (> TS_MAX_MICROS) are also reported as TS_MAX_MICROS.
    uint8_t pin = irqToPin[irqNo][pinIx];
    if (client[pin] && digitalRead(pin) == LOW) {
      unsigned long curDeltaT = micros() - startMicros[pin];  // How many micros() the sensor took to discharge this cycle
      client[pin]->updateState(clientPending[pin] && curDeltaT < TS_MAX_MICROS ? curDeltaT : TS_MAX_MICROS);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
      clientPending[pin] = false;
    }
  }
  
  #ifdef TS_SCOPE
  digitalWrite(TS_CAP_ISR_PIN, LOW);                        // Mark exit from ISR
  #endif
}
ISR(TIMER1_COMPA_vect) {
  timerISR();
}
ISR(PCINT0_vect) {
  capISR(0);
}
ISR(PCINT1_vect) {
  capISR(1);
}
ISR(PCINT2_vect) {
  capISR(2);
}
} // namespace ts_isr

// TouchSensor static public member functions

void TouchSensor::run() {
  for (uint8_t cIx = 0; cIx < NUM_DIGITAL_PINS && ts_isr::clientList[cIx] != ts_isr::NONPIN; cIx++) {
    uint8_t pin = ts_isr::clientList[cIx];
    if (ts_isr::client[pin]) {
      ts_isr::client[pin]->doRun();
    }
  }
}

// TouchSensor public member functions

TouchSensor::TouchSensor(byte pinNo) {
  pinNumber = pinNo;
}

bool TouchSensor::begin() {
  ts_stateData_t defaultState;
  defaultState.prediction = defaultState.measure = TS_ASSUMED_MEASURE;
  defaultState.noise = TS_ASSUMED_NOISE;
  defaultState.measInvFactor = TS_MEAS_INV_FACTOR;
  defaultState.noiseFactor = TS_NOISE_FACTOR;
  defaultState.predictionShift = TS_PREDICTION_SHIFT;
  defaultState.noiseShift = TS_NOISE_SHIFT;
  defaultState.assumedMeas = TS_ASSUMED_MEASURE;
  defaultState.assumedNoise = TS_ASSUMED_NOISE;
  return begin(defaultState);
}

bool TouchSensor::begin(ts_stateData_t initState) {
  // Reject goofy pin numbers and requests for second sensor on same pin
  if (pinNumber >= NUM_DIGITAL_PINS || ts_isr::client[pinNumber]) {
    return false;
  }

  if (ts_isr::doClassInit) {
    #ifdef TS_SCOPE
    // Initialize the 'scope pins used for debugging
    pinMode(TS_CAP_ISR_PIN, OUTPUT);
    digitalWrite(TS_CAP_ISR_PIN, LOW);
    pinMode(TS_TIMER_ISR_PIN, OUTPUT);
    digitalWrite(TS_TIMER_ISR_PIN, LOW);
    #endif

    noInterrupts();
    // Do pin-change interrupt initialization
    PCMSK0 = 0; // Disable all the individual pins from producing pin-change interrupts
    PCMSK1 = 0;
    PCMSK2 = 0;
    
    PCICR = _BV(PCIE2) + _BV(PCIE1) + _BV(PCIE0); // Enable the three pin-change interrupt ports. 
    // NB: Still no actual pin-change interrupts because the masks were just cleared.

    /****
     * Set up Timer/Counter1 so that the ISR that kicks off a measurement of the sensor is run periodically.
     * 
     * See Section 15 of
     *   https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
     * 
     * Timer/Counter1 functionality is set by turning bits two "timer control" registers, TCCR1A and TCCR1B on or off.
     * Four of those bits named WGM13 ... WGM10 set its "Waveform Generation Mode." We want "mode 4," i.e., CTC (Clear 
     * Timer on Compare match) with OCR1A as the compare register. So, WGM13 == 0, WGM12 == 1 and WGM11 and WGM10 == 0. 
     * WGM13 and WGM12 are TCCR1B bits 4 and 3 respectively. WGM11 and WGM10 are TCCR1A bits 1 and 0 respectively. 
     * 
     * Other than WGA11 and WGA10, the bits in TCCR1A are either "reserved" or have to to with output control 
     * pins, functionality we don't need, so they should be 0.
     * 
     * The net result is that we want TCCR1A to be 0.
     * 
     * TCCR1B bits 7 through 5 are either reserved or control the stuff we're not interested in controlling and 
     * so should be 0. As noted, WGM13 should be 0 and WGM12 should be 1. Bits 2 through 0 are the Clock select 
     * bits, CS12, CS11 and CS10. We want an interrupt just a few times a second. To get that we need to make the 
     * timer/counter count relatively slowly. So, we'll use clock / 1024. That's clock mode 5, so we need to set 
     * bits CS12 and CS10.
     * 
     * The net is we need TCCR1B to have bits WGM12, CS10 and CS12 set and no others.
     * 
     * With a clock tick rate of 8MHz and the clock scaled by 1024, letting the timer count from 0..999 (i.e., 
     * 1000 counts) will give us an interrupt every 128ms which seems about right. So, OCR1A, the compare register, 
     * needs to be 999.
     * 
     * Finally, TIMSK1, the Timer/Counter1 Interrupt Mask Register, controls if/how interrupts happen for Timer/
     * Counter1. We want an interrupt only on "Output Compare A Match," which is enabled by setting the bit named 
     * OCIE1A. So we set that one and the others to zero.
     */
    TCCR1A = 0;                                     // CTC, scale to clock / 1024
    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS12); 
    OCR1A =  199;                                   // Compare A register value (200 * ticks / 1024), 12.8ms
    TIMSK1 = bit(OCIE1A);                           // Interrupt on Compare A Match

    ts_isr::doClassInit = false;
    interrupts();
  }


// Get things going for this instance
  setStateData(initState);
  pinMode(pinNumber, OUTPUT);                               // Start charging our sensor
  digitalWrite(pinNumber, HIGH);
  ts_isr::clientPending[pinNumber] = true;

  noInterrupts();

  for (uint8_t i = 0; i < NUM_DIGITAL_PINS; i++) {          // Put us in the client list at the end
    if (ts_isr::clientList[i] == ts_isr::NONPIN) {
      ts_isr::clientList[i] = pinNumber;
      clientIx = i;
      break;
    }
  }
  ts_isr::client[pinNumber] = this;                         // Register us as a new client
  volatile uint8_t *reg = digitalPinToPCMSK(pinNumber);
  uint8_t bit = digitalPinToBitMask(pinNumber);
  *reg |= bit;                                              // Turn on pin-change interrupts for our GPIO pin
  isrMeasure = state.measure;                               // Start with the assumed measurement
  ts_isr::startMicros[pinNumber] = micros();                // Record when the cycle started

  interrupts();

  #ifdef TS_DEBUG
  pinMode(TS_S0_LED + clientIx, OUTPUT);
  #endif

  return true;
}

void TouchSensor::end() {
  // If the pin number is whacky or we're not in service, there's nothing to do
  if (pinNumber >= NUM_DIGITAL_PINS || !ts_isr::client[pinNumber]) {
    return;
  }

  noInterrupts();

  volatile uint8_t *reg = digitalPinToPCMSK(pinNumber);
  uint8_t bit = digitalPinToBitMask(pinNumber);
  *reg &= ~bit;                                                                   // Turn off pin-change interrupts for our GPIO pin

  // Remove our entry from clientList
  uint8_t ix = 0;
  while (ix < NUM_DIGITAL_PINS && ts_isr::clientList[ix] != pinNumber) {          // Find ix for our entry
    ix++;
  }
  if (ts_isr::clientListIx == ix) {                                               // Fix up clientListIx to reflect our exit as a client
    ts_isr::clientListIx = ix > 0 ? ix - 1 : NUM_DIGITAL_PINS - 1;
  } else if (ts_isr::clientListIx > ix) {
    ts_isr::clientListIx = ts_isr::clientListIx - 1;
  }
  while (ts_isr::clientList[ix] != ts_isr::NONPIN) {                              // Scootch entries after ours up in the list
    ts_isr::clientList[ix] = ix < NUM_DIGITAL_PINS - 1 ? ts_isr::clientList[ix + 1] : ts_isr::NONPIN;
    if (ts_isr::clientList[ix] == ts_isr::NONPIN) {
      break;
    }
    ix++;
  }
  ts_isr::client[pinNumber] = nullptr;                                            // Deregister us as a client

  interrupts();
}

TouchSensor::~TouchSensor()  {
  if (ts_isr::client[pinNumber]) {
    end();
  }
}

void TouchSensor::setTouchedHandler(ts_handler_t handler, void* client) {
  touchedHandler = handler;
  touchedClient = client;
}

void TouchSensor::setReleasedHandler(ts_handler_t handler, void* client) {
  releasedHandler = handler;
  releasedClient = client;
}

bool TouchSensor::wasTouched() {
  bool answer = touchHappened;
  checked = true;
  return answer;
}

bool TouchSensor::beingTouched() {
  return touching;
}

ts_stateData_t TouchSensor::getStateData() {
  ts_stateData_t answer;
  memcpy(&answer, &state, sizeof(answer));
  return answer;
}

void TouchSensor::setStateData(ts_stateData_t newState) {
  memcpy(&state, &newState, sizeof(state));
  shiftedPrediction = state.prediction << state.predictionShift;
  shiftedNoise = state.noise << state.noiseShift;
}

uint8_t TouchSensor::getPin() {
  return pinNumber;
}

// TouchSensor private member functions

void TouchSensor::doRun() {
  // Return if nothing to do
  if (!updateAvailable) {
    return;
  }

  noInterrupts();
  unsigned long curMeasure = isrMeasure;
  updateAvailable = false;
  interrupts();

  // Return if the measurement is bogus
  if (curMeasure >= TS_MAX_MICROS) {
    return;
  }

  // Update the estimate of the absolute value of the measurement noise
  unsigned long curNoise    // curNoise is abs(curMeasure - state.measure)
    = (curMeasure > state.measure ? curMeasure - state.measure : state.measure - curMeasure);
  shiftedNoise = shiftedNoise - state.noise + curNoise;
  state.noise = shiftedNoise >> state.noiseShift;

  // Update the actual measurement
  state.measure = curMeasure;

  // Decide whether the new measurement is different enough that whether we're being touched has changed
  bool nowTouching = touching;
  if(state.measure > state.prediction + state.measure / state.measInvFactor + state.noiseFactor * state.noise) {
    nowTouching = true;
  } else if (state.prediction > state.measure + state.measure / state.measInvFactor + state.noiseFactor * state.noise) {
    nowTouching = false;
  }

  // Update the prediction of what we'll see next 
  if (nowTouching == touching) {
    // If the state didn't change, update the prediction to be a little closer to this time's measurement
    shiftedPrediction = shiftedPrediction - state.prediction + state.measure;
    state.prediction = shiftedPrediction >> state.predictionShift;
  } else {
    // Otherwise update the prediction to say it'll be the same as this time's measurement
    state.prediction = state.measure;
    shiftedPrediction = state.prediction << state.predictionShift;
  }

  // Decide whether a touch that the client hasn't asked about has happened
  if (nowTouching && !touching) {           // If the sensor just went from being not touched to being touched, it happened
    touchHappened = true;
  } else if (touchHappened && checked) {    // else if an earlier one was checked, it certainly didn't happen
    touchHappened = false;
  }
  checked = false;

  bool gotTouched = nowTouching && !touching;
  bool gotUntouched = !nowTouching && touching;
  touching = nowTouching;
  // If there's a touched handler and we just got touched, invoke it.
  if (touchedHandler && gotTouched) {
    touchedHandler(pinNumber, touchedClient);
  }

  // If there's a released handler and we just stopped being touched, invoke it.
  if (releasedHandler && gotUntouched) {
    releasedHandler(pinNumber, releasedClient);
  }

  #ifdef TS_DEBUG
  digitalWrite(TS_S0_LED + clientIx, touching);
  #endif
}

void TouchSensor::updateState(unsigned long newMeasure) {
  isrMeasure = newMeasure;
  updateAvailable = true;
}
