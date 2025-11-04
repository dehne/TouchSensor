/****
 * @file    main.cpp
 * @author  D. L. Ehnebuske (dle.com@ehnebuske.net)
 * @brief   Example of using TouchSensor, a capacitive touch sensor library for Arduino AVR MPUs, with callbacks
 * @version 0.1
 * @date    2025-10-31
 * 
 ****
 * Copyright (C) 2025 D. L. Ehnebuske
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
#include <Arduino.h>
#include "TouchSensor.h"

TouchSensor sensor[] {2, 3, 4, 5};                                  // The sensors themselves

constexpr unsigned long SERIAL_DELAY = 500;                         // The time in millis() to delay to wait for Serial to come up
constexpr unsigned long DUMP_MILLIS = 250;                          // Sensor dump interval in millis()
constexpr size_t N_SENSORS = (sizeof(sensor) / sizeof(sensor[0]));  // The number of sensors

void onChange(uint8_t pin) {
  Serial.print('\r');
  for (uint8_t sNo = 0; sNo < N_SENSORS; sNo++) {
    Serial.print(sensor[sNo].getPin() == pin ? 
      (sensor[sNo].beingTouched() ? F("t ") : F("n ")) : 
      (sensor[sNo].beingTouched() ? F("* ") : F("  ")));
  }
    Serial.print(F(" | "));
  for (uint8_t sNo = 0; sNo < N_SENSORS; sNo++) {
    ts_stateData_t state = sensor[sNo].getStateData();
    Serial.print(state.measure);
    Serial.print(F(" "));
  }
}

void setup() {
  Serial.begin(9600);
  delay(SERIAL_DELAY);

  for (uint8_t sNo = 0; sNo < N_SENSORS; sNo++) {
    Serial.print(F("sensor["));
    Serial.print(sNo);
    Serial.print(F("]: "));
    Serial.println(sensor[sNo].begin() ? F("succeeded") : F("failed"));
    sensor[sNo].setTouchedHandler(onChange);
    sensor[sNo].setReleasedHandler(onChange);
  }
  for (uint8_t sNo = 0; sNo < N_SENSORS; sNo++) {
    Serial.print(sNo);
    Serial.print(' ');
  }
  Serial.print('\n');
}

void loop() {
  TouchSensor::run();
}
