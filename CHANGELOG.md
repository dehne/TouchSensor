# Change Log for TouchSensor Library

## v1.0.0

First stable version.

## v1.1.0

1. Tweak the algorithm used to decide whether a TouchSensor is being touched or not to accommodate a wider range of sensor implementations. Specifically, the amount by which the measured value must differ from the predicted value now depends partly on the measured value and partly on the noise. Previously, it depended only on a constant and the noise.

2. Add the ability to set the parameters used in the decision algorithm both initially and on the fly so that a sensor that doesn't work using the defaults can be accommodated.

## v2.0.0

A backward-incompatible change already! Hope this is the last one.

Change the definition of the callback function from

    using ts_handler_t = void (*)(uint8_t pin);

to

    using ts_handler_t = void (*)(uint8_t pin, void* client);

The motivation was to enable member functions to (indirectly) be callbacks by going through a static adapter member function, but you can use the client parameter to pass a pointer to whatever you'd like at registration time. Whatever you passed is passed into the callback.

## V2.0.1

Prep for publishing to Platformio registry
