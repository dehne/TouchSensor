
# TouchSensor Library

## Introduction

This is TouchSensor Arduino library for AVR architecture MPUs. It gives Arduino sketches a simple-to-use interface to self-capacitance capacitive touch sensors. Because of how things work, you can't use TouchSensor and the Arduino tone() function in the same sketch.

There are many ways to construct such sensors. For experimenting, you can construct them from small (~225mm**2) adjacent areas of copper on some sort of non-conductive substrate and then covered with thin plastic -- think copper tape, perfboard and packing tape. Signal and ground leads are connected to each piece of copper; the signal leads directly and the ground leads through ~2 megohm resistors. The other ends of the signal leads are connected pairwise to digital GPIO pins. The ground leads all attach to, well, signal ground.

A more finished sensor can be made from copper pads designed into a PCB with signal and ground traces connected as above. Like "experimental" sensors, it's important that the copper areas are covered with a thin dielectric of some sort -- maybe solder mask or plastic tape -- so that the copper can't be touched directly.

[Microchip's Capacitive Touch Sensor Design Guide](https://ww1.microchip.com/downloads/en/AppNotes/Capacitive-Touch-Sensor-Design-Guide-DS00002934-B.pdf) is a good starting point for information on designing and building capacitive touch sensors of various sorts.

## How to use the library

The intended usage pattern is this.

For each sensor you have (you can have as many as there are digital GPIO pins available), instantiate a TouchSensor object giving the ctor the Arduino digital pin number to which you've attached the sensor. Often, these are global variables. Then invoke begin() once on each object to get it going, usually in setup().

Then in loop(), call static member function TouchSensor::run(). This will get the active TouchSensor objects to do what they need to do to keep themselves up to date. (There is also work done in a couple of interrupt-service routines, but it's kept to an absolute minimum.) If TouchSensor::run() isn't called often enough, the TouchSensors' state won't be kept up to date with what's going on in the real world.

There are two basic approaches for your sketch to deal with people touching the sensors. First, you can call the member functions wasTouched() and beingTouched() as needed to find out what the current state of a sensor is. Alternatively, you can write a callback function with type ts_handler_t and have it called (during the execution of TouchSensor::run()) whenever the sensor's state changes from not touched to touched or from being touched to being not touched. Or both. You can register your callback function(s) by using the member functions setTouched() and setReleased(), respectively. Registration is often done in setup().

If you don't need a TouchSensor for a while, call end(). This will take it out of service by stopping the interrupt-based and state-tracking work associated with the TouchSensor. You can always call begin() again to put the sensor back in service. If you take a TouchSensor out of service, any registered callback functions will start be called once again if you put it back in service.

To make a TouchSensor go away completely, call its dtor.

## How it works

The basic idea behind TouchSensor is to look for the increase in capacitance that happens when a finger is placed on a sensor's plastic-covered copper. We do this by measuring how long the sensor takes to discharge to logic LOW through its grounding resistor after having been charged to the logic HIGH. I.e., we periodically go through a cycle of charging the sensor by making the pin it's attached to an OUTPUT pin set to HIGH, and then seeing how long the voltage takes to drop to logic level LOW when the pin is made to be a high-impedance INPUT pin. When the discharge time suddenly increases (or decreases) sufficiently, we know the sensor is now being touched (or no longer being touched).

There are three tricky parts. The first is fitting the work to the AVR architecture. The second is interfacing between instances of TouchSensor objects and interrupt service routines. The third is dealing with the variability in how the sensors behave, both from sensor to sensor, over time and as the physical environment around the sensor changes.

At an architectural level, the AVR architecture supports interrupts when any of up to 24 digital GPIO pins change state. (Implementations typically support fewer.) It does this through three sets of pin-change related registers. Each set is responsible for as many as 8 GPIO pins. Each of the three sets has its own ISR vector and therefore its own ISR. In our case, the work is the same for all three, so the ISRs just delegate to a common function which is passed the set number. Further, the AVR architecture has three timer/counters. We take over Timer/Counter 1 to periodically kick off the measurement cycles. Timer/Counter 1 is the timer the Arduino tone() function uses, which is why you can't use tone() and TouchSensor in the same sketch.

When a timer interrupt happens, we start a measurement cycle for the next active TouchSensor in the list of active sensors: We record the current value of micros(), and change the GPIO pin for the sensor (which will be in OUTPUT mode at logic level HIGH) to INPUT mode. This causes it to begin discharging through its ground resistor and through the high-impedance GPIO pin. When the charge voltage crosses the threshold from logic HIGH to LOW, the transition causes a pin-change interrupt. This invokes the common ISR function where we work out how many micros() it was since the measurement cycle began. We then invoke the (private) updateState() member function for the appropriate TouchSensor, telling it how long the cycle took. The TouchSensor merely records the value it was passed and sets a flag marking the existence of the new data. Later, when TouchSensor::run() is invoked (in the sketch's loop()), the TouchSensor's doRun() private method is invoked.
When doRun notices the new data, it updates the state of things (deciding, for example, whether the sensor was just touched) for that TouchSensor.

The active sensors are put through measurement cycles in a round-robin fashion, one after another as timer interrupts happen. Doing it this way spreads out the work done in an ISR context across time, keeping the servicing of any given interrupt short.

To be able to invoke the updateState() and doRun() for the appropriate instance of the TouchSensor, there's a static array of pointers to the TouchSensors, maintained by the instances in their begin() and end() member
functions.

As mentioned, keeping track of what the incoming data about a given sensor means is the job of each instance's doRun() private member function. The doRun() member function for each active TouchSensor instance is invoked from the static member function TouchSensor::run(), which should be invoked often during the  sketch's execution.

From the series of incoming measurements, doRun() calculates two quantities, a predicted value for the next measurement, and a measure of the current level of noise in the incoming measurements. Both are defined by a computationally efficient "moving average":

Let a(k) be the k<sup>th</sup> average, d(k) the k<sup>th</sup> datum, and s a constant, then a(k) = (a(k-1) * 2<sup>s</sup> + d(k)) / 2<sup>s</sup>.

For p(k), the k<sup>th</sup> prediction, s = TS_PREDICTION_SHIFT, d(k) = m(k), where m(k) is the k<sup>th</sup> measurement.

For n(k), the k<sup>th</sup> noise estimation, s = TS_NOISE_SHIFT and d(k) = abs(m(k-1) - m(k)) where m(k) is as above.

The state of the sensor -- touched or not-touched -- at time k is determined by whether m(k) is sufficiently different than p(k), given the values of n(k) and and m(k). 

Specifically, if

&nbsp;&nbsp;&nbsp;&nbsp;m(k) > p(k) + m(k) / state.measInvFactor + state.noiseFactor * n(k), 

the sensor is being touched and if

&nbsp;&nbsp;&nbsp;&nbsp;p(k) > m(k) + m(k) / state.measInvFactor + state.noiseFactor * n(k), 

the sensor is not being touched. If it's neither of those, its state is what it was at time k-1.
