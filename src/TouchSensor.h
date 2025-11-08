/****
 * This file is a part of the TouchSensor Arduino library for AVR architecture MPUs. It gives Adruino sketches a 
 * simple-to-use interface to self-capacitance capacitive touch sensors. Because of how things work, you can't 
 * use TouchSensor and the Arduino tone() function in the same sketch.
 * 
 * There are many ways to construct such sensors. For experimenting, you can construct one from a small 
 * (~225mm**2) chunk of PCB with copper on one side and covered with thin plastic -- think packing tape. A signal 
 * lead is connected directly to the copper; a ground lead is also connected to the copper, but through a ~2 
 * megohm resistor. Signal is connected to a digital GPIO pin. Ground attaches to, well, signal ground. A more 
 * finished sensor can be made from a copper pad designed into a PCB with signal and ground traces connected as 
 * above. Like "experimental" sensors, it's important that the touch pad's copper is covered with a thin 
 * dielectric of some sort -- maybe solder mask or plastic tape -- so that the copper is not touched directly.
 * 
 * Microchip's Capacitive Touch Sensor Design Guide:
 * 
 *      https://ww1.microchip.com/downloads/en/AppNotes/Capacitive-Touch-Sensor-Design-Guide-DS00002934-B.pdf
 * 
 * is a good starting point for information on designing and building capacative touch sensors of various sorts.
 * 
 * How to use the library
 * ======================
 * 
 * The intended usage pattern is this. 
 * 
 * For each sensor you have (you can have as many as there are digital GPIO pins available), instantiate a 
 * TouchSensor object giving the ctor the Arduino digital pin number to which you've attached the sensor. 
 * Often, these are global variables. Then invoke begin() once on each object to get it going, usually in 
 * setup(). 
 * 
 * Then in loop(), call static member function TouchSensor::run(). This will get the aactive TouchSensor objects 
 * to do what they need to do to keep themselves up to date. (There is also work done in a couple of interrupt-
 * service routines, but it's kept to an absolute minimum.) If TouchSensor::run() isn't called often enough, the 
 * TouchSensors' state won't be kept up to date with what's going on in the real world. 
 * 
 * There are two basic approaches for your sketch to deal with people touching the sensors. First, you can call 
 * the member functions wasTouched() and beingTouched() as needed to find out what the current state of a sensor 
 * is. Alternatively, you can write a callback function with type ts_handler_t and have it called (during the 
 * execution of TouchSensor::run()) whenever the sensor's state changes from not touched to touched or from being 
 * touched to being not touched. Or both. You can register your callback function(s) by using the member 
 * functions setTouched() and setReleased(), respectively. Registration is often done in setup().
 * 
 * If you don't need a TouchSensor for a while, call end(). This will take it out of service by stopping the 
 * interrupt-based and state-tracking work associated with the TouchSensor. You can always call begin() again to 
 * put the sensor back in service. If you take a TouchSensor out of service, any registered callback functions 
 * will start be called once again if you put it back in service.
 * 
 * To make a TouchSensor go away completely, call its dtor. 
 * 
 * How it works
 * ============
 * 
 * The basic idea behind TouchSensor is to look for the increase in capacitance that happens when a finger is 
 * placed on a sensor's plastic-covered copper. We do this by measuring how long the sensor takes to discharge to 
 * ground after having been charged to the logic voltage. I.e., we periodically go through a cycle of charging 
 * the sensor by making the pin it's attached to an OUTPUT pin set to HIGH, and then seeing how long the voltage 
 * takes to drop to logic level LOW when the pin is made to be an INPUT pin. When the discharge time suddenly 
 * increases (or decreases) sufficiently, we know the sensor is now being touched (or no longer being touched).
 * 
 * There are three tricky parts. The first is fitting the work to the AVR architecture. The second is interfacing 
 * between instances of TouchSensor objects and interrupt service routines. The third is dealing with the 
 * variability in how the sensors behave, both from sensor to sensor and over time as the environment changes. 
 * 
 * The AVR architecture supports interrupts when any of up to 24 digital GPIO pins change state. It does this 
 * through three sets of pin-change related registers. Each set responsible for as many as 8 GPIO pins. Each of 
 * the three sets has its own ISR vector and therefore its own ISR. In our case, the work is the same for all 
 * three, so the ISRs just delegate to a common function which is passed the set number. Further, the AVR 
 * architecture has three timer/counters. We take over Timer/Counter 1 to perodically kick off the measurement 
 * cycles. Timer/Counter 1 is the timer the Arduino tone() function uses, which is why you can't use tone() and 
 * TouchSensor in the same sketch.
 * 
 * When a timer interrupt happens, we start a measurement cycle for the next active TouchSensor in the list of 
 * active sensors: We record the current value of micros(), and change the GPIO pin for the sensor (which will be 
 * in OUTPUT mode at logic level HIGH) to INPUT mode. This causes it to begin discharging through its ground 
 * resistor and through the high-impedance GPIO pin. When the charge voltage crosses the threshold from logic 
 * HIGH to LOW, the transition causes a pin-change interrupt. This invokes the common ISR function where we work 
 * out how many micros() it was since the measurement cycle began. We then invoke the (private) updateState() 
 * member function for the appropriate TouchSensor, telling it how long the cycle took. The TouchSensor merely 
 * records the value it was passed and sets a flag marking the existence of the new data. Later, when 
 * TouchSensor::run() is invoked (in the sketch's loop()), the TouchSensor's doRun() private method is invoked. 
 * When doRun notices the new data, it updates the state of things (deciding, for example, whether the sensor was 
 * just touched) for that TouchSensor.
 * 
 * The active sensors are put through measurement cycles in a round-robin fashion, one after another as timer 
 * interrupts happen. Doing it this way spreads out the work done in an ISR context across time, keeping the 
 * servicing of any given interrupt short. 
 * 
 * To be able to invoke the updateState() and doRun() for the appropriate instance of the TouchSensor, there's a 
 * static array of pointers to the TouchSensors, maintained by the instances in their begin() and end() member 
 * functions.
 * 
 * As mentioned, keeping track of what the incoming data about a given sensor means is the job of each instance's 
 * doRun() private member function. The doRun() member function for each active TouchSensor instance is invoked 
 * from the static memberfunction TouchSensor::run(), which should be invoked often during the  sketch's 
 * execution. 
 * 
 * From the series of incoming measurements, doRun() calculates two quantities, a predicted value for the next 
 * measurement, and a measure of the current level of noise on the incoming measurements. Both are defined by a 
 * computationally efficient "moving average": If a(k) is the k-th average, d(k) is the k-th datum, and s is 
 * a constant, a(k) = (a(k-1) * 2**s + d(k)) / 2**s. 
 * 
 * For p(k), the k-th prediction, s = state.predictionShift, d(k) = m(k), where m(k) is the k-th measurement.
 * 
 * For n(k), the k-th noise estimation, s = state.noiseShift and d(k) = abs(m(k-1) - m(k)) where m(k) is as above.
 * 
 * The state of the sensor -- touched or not-touched -- at time k is determined by whether m(k) is sufficiently 
 * different than p(k), given the values of n(k) and and m(k). 
 * 
 * Specifically, if 
 * 
 *      m(k) > p(k) + m(k) / state.measInvFactor + state.noiseFactor * n(k), 
 * 
 * the sensor is being touched and if 
 * 
 *      p(k) > m(k) + m(k) / state.measInvFactor + state.noiseFactor * n(k), 
 * 
 * the sensor is not being touched. If it's neither of those, its state is what it was at time k-1.
 * 
 *****
 * 
 * TouchSensor V1.1.0, November 2025
 * Copyright (C) 2024-2025 D.L. Ehnebuske
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
#pragma once
#ifndef Arduino_h
    #include <Arduino.h>                                // Arduino goop
#endif
#ifndef TouchSensor_h
#define TouchSensor_h
#endif
constexpr unsigned long TS_MEAS_INV_FACTOR  = 9;        // Hysteresis measuerement inverse factor
constexpr unsigned long TS_NOISE_FACTOR     = 2;        // Hysteresis noise factor
constexpr unsigned long TS_PREDICTION_SHIFT = 6;        // Bitshift value for the calculation of the prediction
constexpr unsigned long TS_NOISE_SHIFT      = 2;        // Bitshift value for the calculation of the noise
constexpr unsigned long TS_ASSUMED_MEASURE  = 84;       // The assumed measured discharge time in micros()
constexpr unsigned long TS_ASSUMED_NOISE    = 40;       // The assumed measurement noise in micros() (quite high, initially)
constexpr unsigned long TS_MAX_MICROS       = 10000;    // The "timed out" value for a discharge time in micros()

#define TS_DEBUG                                        // Uncomment to enable general debug output; comment to disable
#ifdef TS_DEBUG
constexpr uint8_t TS_S0_LED                 = 6;        // Sensor state LED pin base. A sensor's LED pin is TS_S0_LED + clientIx
#endif
//#define TS_SCOPE                                        // Uncomment to enable debugging scope pins
#ifdef TS_SCOPE
constexpr uint8_t TS_CAP_ISR_PIN                = 10;   // 'Scope pin for capISR entry/exit: HIGH means in the ISR
constexpr uint8_t TS_TIMER_ISR_PIN              = 11;   // 'Scope pin for timer ISR entry/exit: HIGH means in the ISR
#endif

/**
 * @brief The type definition for a TouchSensor's state data.
 * 
 */
struct ts_stateData_t {
    // The current measurement-related state. Changes measurement by measuement
    unsigned long prediction;       // The predicted value, in micros(), for what measure will be at the next measurement
    unsigned long measure;          // The current measured discharge time in micros()
    unsigned long noise;            // The current estimate, in micros(), of the amount of noise in measure

    // The touched-state algorithm's tuning-related data. Used to tune behavior of a given sensor. Once set, stable over time
    unsigned long measInvFactor;    // The algorithm's "measurement inverse factor"
    unsigned long noiseFactor;      // The algorithm's "noise factor"
    unsigned long predictionShift;  // The algorithm's "prediction shift"
    unsigned long noiseShift;       // The algorithm's "noise shift"
    unsigned long assumedMeas;      // The initially assumed measurement
    unsigned long assumedNoise;     // The amount of noise initially assumed to be present

};

/**
 * @brief The function called by pin-change interrupts. Needed so we can make it a friend.
 * 
 */
namespace ts_isr {
    void capISR(uint8_t portNo) noexcept;
}


class TouchSensor {
    /**
     * @brief A friend function -- the ISR for pin change interrupts -- that invokes updateState()
     * 
     */
    friend void ts_isr::capISR(uint8_t vector) noexcept;

public:
    /**
     * @brief   Do the bulk of the work to keep the sensor state up to date for all the TouchSensors. Put calls to it 
     *          in loop() so it gets called often!
     * 
     * @details This member function calls the private member function doRun() on each of the active TouchSensors.
     * 
     *          The apparent capacitance of an untouched self-capacitance sensor varies quite a bit depending on many 
     *          details of the sensor design and its environment. And it can change (slowly) over time. Similarly, 
     *          the jump in apparent capacitance when the sensor is touched varies from sensor to sensor and over 
     *          time. Plus there's noise in the measurements. Everything varies by sensor, environment, and over time. 
     *          The goal of the algorithm implemented here is to work despite the variabilities and changes over time 
     *          and to do so without having to calibrate each sensor. 
     * 
     *          The basic idea is to create and maintain a characterization of the measurement noise and a prediction 
     *          of what the next measurement will be each time the function is called with a new measurement. If, 
     *          given the noise, a new measurement is enough smaller than the prediction, we make the state "not 
     *          touched". If it's enough larger than the prediction, we make the state "touched". If the change is 
     *          neither much larger or smaller, we don't update the state.
     * 
     */
    static void run();

    /**
     * @brief Construct a new TouchSensor object.
     * 
     * @param pinNo The GPIO pin number to which the touch sensor capacitor is attached.
     */
    TouchSensor(byte pinNo);

    /**
     * @brief Initialize the TouchSensor with the specified state data. (For special cases.)
     * 
     * @param initState The state data to initialize the TouchSensor with
     * @return true     Initialization succeeded
     * @return false    Initialization failed
     */
    bool begin(ts_stateData_t initState);

    /**
     * @brief Initialize the TouchSensor with default state data. (The usual way to do it.)
     * 
     * @return true     Sensor successfully initialized,
     * @return false    Sensor failed to initialize.
     */
    bool begin();

    /**
     * @brief Deinitialize the TouchSensor.
     * 
     */
    void end();

    /**
     * @brief Destroy the Touch Sensor object.
     * 
     */
    ~TouchSensor();

    /**
     * @brief   The type a client-provided "touched handler" function must have. Write a function with this shape,
     *          register it with setTouchedHandler(), and it'll be called (from run()) whenever the sensor gets 
     *          touched. Register it with setReleasedHandler() and it'll be called whenever the sensor stops being 
     *          touched. Register it with both, and it'll be called for both changes.
     * 
     * @param   pin     The pin to which the TouchSensor is attached.
     * @param   client  Pointer to client data; exactly what was passed during registration
     * 
     */
    using ts_handler_t = void (*)(uint8_t pin, void* client);

    /**
     * @brief Set the function to call (from run()) when the sensor goes from being not touched to 
     *        touched. Specify nullptr as the function to stop the calling.
     * 
     * @param touchedHandler    The function to call
     * @param client            Pointer to client data or nullptr if none
     */
    void setTouchedHandler(ts_handler_t touchedHandler, void* client);

    /**
     * @brief   Set the function to call (from run()) when the sensor goes from being touched to not 
     *          touched. Specify nullptr as the function to stop the calling.
     * 
     * @param releasedHandler   The function to cll
     * @param client            Pointer to client data or nullptr if none
     */
    void setReleasedHandler(ts_handler_t releasedHandler, void* client);

    /**
     * @brief True if the sensor was touched since the last invocation of wasTouched().
     * 
     * @return true     Yes, the sensor was touched since the last time wasTouched() was called,
     * @return false    No, the sensor hasn't been touched.
     */
    bool wasTouched();

    /**
     * @brief True if the sensor is currently being touched.
     * 
     * @return true     Somebody's touching the sensor,
     * @return false    No one is touching the sensor.
     */
    bool beingTouched();

    /**
     * @brief Get the TouchSensor's state data.
     * 
     * @return ts_calData_t     The current values of the state data.
     */
    ts_stateData_t getStateData();

    /**
     * @brief Set the TouchSensor's state data
     * 
     * @param newState 
     */
    void setStateData(ts_stateData_t newState);

    /**
     * @brief           Get the number of the GPIO pin to which this TouchSensor is connected.
     * 
     * @return uint8_t  The number of the GPIO pin to which this TouchSensor is connected
     */
    uint8_t getPin();

    private:
    /**
     * @brief   Private method used to do the bulk of the updating of the state data -- not during an interrupt.
     *          invoked from the static (i.e., class) member function, run().
     * 
     */
    void doRun();

    /**
     * @brief   Private method used to update the state of the sensor. Invoked by our free-function friend capISR()
     *          during each measurement cycle.
     * 
     * @param deltaMicros   The number of micros() it took for the sensor to discharge enough to switch logic states.
     */
    void updateState(unsigned long deltaMicros) noexcept;

    ts_handler_t touchedHandler = nullptr;      // The function, if any, to call when the sensor gets touched
    void* touchedClient = nullptr;              // What the client passed when setTouchedHandler() was called
    ts_handler_t releasedHandler = nullptr;     // The function, if any, to call when the sensor stops being touched
    void* releasedClient = nullptr;             // What the client passed when setReleasedHandler() was called
    volatile unsigned long isrMeasure;          // The most recent sensor discharge time in micros() per the ISR
    ts_stateData_t state;                       // The current central state data
    unsigned long shiftedPrediction;            // Always equal to state.prediction * 2**TS_PREDICTION_SHIFT
    unsigned long shiftedNoise;                 // Always equal to state.noise * 2**TS_NOISE_SHIFT
    uint8_t pinNumber;                          // The Arduino-style pin number of the GPIO pin the sensor is attached to
    uint8_t clientIx;                           // This sensor's index in ts_isr::clientList
    volatile bool updateAvailable = false;      // Set true when curMeasure is updated
    bool touchHappened = false;                 // True if the sensor was touched since last invocation of wasTouched()
    bool checked = true;                        // True if wasTouched() has been invoked since the last ISR invocation
    bool touching = false;                      // True if the sensor is now being touched. Check with beingTouched()
};
