/****
 * This file is a part of the TouchSensor library. See README.md for details
 * 
 *****
 * 
 * TouchSensor V2.1.0, November 2025
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

  /**
   * Atmelavr micro processors vary in which and how many GPIO pins they support for pin-change interrupts. How 
   * Arduino pin numbers get mapped to the pins that are supported also varies. But the general architecure is 
   * consistent from implementation to implementation. There are some number of IRQ vectors, each of whidh 
   * supports pin-change interrupts for up to eight GPIO pins. The number of IRQ vectors varies from 
   * implementation to implementation, but, so far anyway, there are at most four pin-change of them. 
   * 
   * We can deduce the number of vectors by whether symbols for them exist in the code environment -- they're 
   * defined in the file pins_arduino.h for the specific implementation being used.  I've chosen PCMSKx as the 
   * symbol to use.
   * 
   * If there are, for example, three vectors defined, there can't be more than 24 pins that can generate 
   * pin-change interrupts. There might be fewer. NUM_PC_PINS is the upper bound for the number of pin-change 
   * interrupt pins supported. It's used to size arrays indexed by something we call the client index -- a slot 
   * for eight possible clients on each of the IRQ vectors. See pinToClentIx(), below.
   */
  #if defined(PCMSK3)
    constexpr uint8_t NUM_PC_PINS = 32;
    constexpr uint8_t NUM_IRQ = 4;
  #elif defined(PCMSK2)
      constexpr uint8_t NUM_PC_PINS = 24;
      constexpr uint8_t NUM_IRQ = 3;
  #elif defined(PCMSK1)
        constexpr uint8_t NUM_PC_PINS = 16;
        constexpr uint8_t NUM_IRQ = 2;
  #else
        constexpr uint8_t NUM_PC_PINS = 8;
        constexpr uint8_t NUM_IRQ = 1;
  #endif

  constexpr uint8_t NONPIN = NUM_PC_PINS;                             // Value in clientList indicating a client index not stored in that element
  static TouchSensor* volatile client[NUM_PC_PINS] = {nullptr};       // Pointers to active clients (or nullptr if no active client in a slot)
                                                                      //   indexed by pinToClientIx() of the pin to which they are attached
  static volatile bool clientPending[NUM_PC_PINS] = {false};          // Whether a client's discharge is pending; indexed as per client[]
  static volatile unsigned long startMicros[NUM_PC_PINS] = {0};       // micros() when the client's latest sampling cycle started
  static volatile uint8_t clientIx = 0;                               // The index into client of last client to start a measurement cycle
  static bool doClassInit = true;                                     // Set false by first invocation of begin() after doing class init work

/**
 * @brief Convert an Arduino digital pin number to its client index equivalent
 * 
 * @param pin       The Arduino pin number to be converted
 * @return uint8_t  The client index equivalent or NUM_PC_PINS if the specified pin does not support pin-change interrupts
 */
uint8_t inline pinToClientIx (uint8_t pin) {
  if (digitalPinToPCICR(pin) != 0) {
    return 8 * digitalPinToPCICRbit(pin) + digitalPinToPCMSKbit(pin);
  }
  return NUM_PC_PINS;
}

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
  // Find the next client to work on 
  for (uint8_t ix = (clientIx + 1) % NUM_PC_PINS; ix != clientIx; ix = (ix + 1) % NUM_PC_PINS) {
    if (client[ix]) {
      clientIx = ix;
      break;
    }
  }
  if (client[clientIx]) {
    uint8_t pin = client[clientIx]->getPin();
    if (clientPending[clientIx]) {
      // The client still hasn't discharged! Force its pin to go low (and so cause an interrupt)
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
      clientPending[clientIx] = false; // Show it was forced, not a real measurement
    } else {
      // The client is ready to go. Start the next measurement cycle
      pinMode(pin, INPUT);
      clientPending[clientIx] = true;
    }
    startMicros[clientIx] = micros();
  }

  #ifdef TS_SCOPE
  digitalWrite(TS_TIMER_ISR_PIN, LOW);
  #endif
}

/**
 * @brief Interrupt serivce routine for all the pin change interrupts.
 * 
 * @param vector    The interrupt vector (0..NUM_IRQ - 1) that caused the interrupt.
 */
void capISR(uint8_t irqNo) {
  #ifdef TS_SCOPE
  digitalWrite(TS_CAP_ISR_PIN, HIGH);                   // Mark entry to ISR
  #endif

  // Iterate through the clients whose pin(s) could have caused this interrupt. The IRQ vector for the interrupt 
  // comes in as irqNo, so the clients that could have caused it are client[8 * irq + i] where 0 <= i < 8.
  unsigned long curMicros = micros();
  for (uint8_t cIx = 8 * irqNo; cIx < 8 * irqNo + 8; cIx++) {
    if (client[cIx]) {
      // Okay, this is a little complicated and I'm bound to forget. So here's what's going on with each client. 
      // We're only interested in clients whose pin is LOW. A client with a HIGH pin either caused the interrupt 
      // because it changed to HIGH (as a side effect of the timer kicking off a measurement cycle) or is HIGH 
      // because it hasn't yet discharged (and so isn't involved in this interrupt). If the client isn't still 
      // pending, that means it was forced LOW during the timer ISR. In that case, by convention, the update 
      // specifies TS_MAX_MICROS as the discharge time to mark it as (probably) bogus. Unreasonably long 
      // (> TS_MAX_MICROS) are also reported as TS_MAX_MICROS.
      uint8_t pin = client[cIx]->getPin();
      if (digitalRead(pin) == LOW) {
        unsigned long curDeltaT = curMicros - startMicros[cIx];   // How many micros() the sensor took to discharge this cycle
        client[cIx]->updateState(clientPending[cIx] && curDeltaT < TS_MAX_MICROS ? curDeltaT : TS_MAX_MICROS);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        clientPending[cIx] = false;
      }
    }
  }
  
  #ifdef TS_SCOPE
  digitalWrite(TS_CAP_ISR_PIN, LOW);                        // Mark exit from ISR
  #endif
}

ISR(TIMER1_COMPA_vect) {          // Actual ISR for Timer/Counter1
  timerISR();
}
ISR(PCINT0_vect) {                // Actual ISR for pin-change interrupt 0
  capISR(0);
}
#ifdef PCMSK1
ISR(PCINT1_vect) {                // Actual ISR for pin-change interrupt 1
  capISR(1);
}
#endif
#ifdef PCMSK2
ISR(PCINT2_vect) {                // Actual ISR for pin-change interrupt 2
  capISR(2);
}
#endif
#ifdef PCMSK3
ISR(PCINT3_vect) {                // Actual ISR for pin-change interrupt 3
  capISR(3);
}
#endif
} // namespace ts_isr

// TouchSensor static public member functions

void TouchSensor::run() {
  for (uint8_t cIx = 0; cIx < ts_isr::NUM_PC_PINS; cIx++) {
    if (ts_isr::client[cIx]) {
      ts_isr::client[cIx]->doRun();
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
  // Reject bad pin numbers and requests for second sensor on same pin
  if (digitalPinToPCICR(pinNumber) == 0 || ts_isr::client[ts_isr::pinToClientIx(pinNumber)]) {
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

    #ifdef TS_DEBUG
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    #endif

    noInterrupts();
    // Do pin-change interrupt initialization

    // Disable all the individual pins from producing pin-change interrupts and nable the pin-change interrupt vectors
    #ifdef PCMSK0
    PCMSK0 = 0;
    PCICR |= _BV(PCIE0);
    #endif
    #ifdef PCMSK1
    PCMSK1 = 0;
    PCICR |= _BV(PCIE1);
    #endif
    #ifdef PCMSK2
    PCMSK2 = 0;
    PCICR |= _BV(PCIE2);
    #endif
    #ifdef PCMSK3
    PCMSK3 = 0;
    PCICR |= _BV(PCIE3);
    #endif

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
    TCCR1A = 0;                                     // CTC, scale to clock / 1024. i.e., tick = 1024 / F_CPU sec
    TCCR1B = bit(WGM12) | bit(CS10) | bit(CS12); 
    OCR1A =  ((16000000L / F_CPU) * 200L) - 1L;     // We want an interrupt every 12.8ms, so 200 ticks at F_CPU = 16Mhz
    TIMSK1 = bit(OCIE1A);                           // Interrupt on Compare A Match

    ts_isr::doClassInit = false;
    interrupts();
  }

  // Get things going for this instance
  uint8_t cIx = ts_isr::pinToClientIx(pinNumber);

  #ifdef TS_DEBUG
  Serial.print(F("\nStarting sensor with pin number "));
  Serial.print(pinNumber);
  Serial.print(F(", client index "));
  Serial.print(cIx);
  Serial.println('.');
  #endif
  
  setStateData(initState);
  pinMode(pinNumber, OUTPUT);                               // Start charging our sensor
  digitalWrite(pinNumber, HIGH);
  ts_isr::clientPending[cIx] = true;

  noInterrupts();

  ts_isr::client[cIx] = this;                               // Register us as a new client
  volatile uint8_t *reg = digitalPinToPCMSK(pinNumber);
  uint8_t bit = digitalPinToBitMask(pinNumber);
  *reg |= bit;                                              // Turn on pin-change interrupts for our GPIO pin
  isrMeasure = state.measure;                               // Start with the assumed measurement
  ts_isr::startMicros[cIx] = micros();                      // Record when the cycle started

  interrupts();

  return true;
}

void TouchSensor::end() {
  // If the pin number is whacky or we're not in service, there's nothing to do
  if (pinNumber >= NUM_DIGITAL_PINS || !ts_isr::client[ts_isr::pinToClientIx(pinNumber)]) {
    return;
  }

  noInterrupts();

  volatile uint8_t *reg = digitalPinToPCMSK(pinNumber);
  uint8_t bit = digitalPinToBitMask(pinNumber);
  *reg &= ~bit;                                                                   // Turn off pin-change interrupts for our GPIO pin

  ts_isr::client[ts_isr::pinToClientIx(pinNumber)] = nullptr;                     // Deregister us as a client

  interrupts();
}

TouchSensor::~TouchSensor()  {
  if (ts_isr::client[ts_isr::pinToClientIx(pinNumber)]) {
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
}

void TouchSensor::updateState(unsigned long newMeasure) {
  isrMeasure = newMeasure;
  updateAvailable = true;
}
