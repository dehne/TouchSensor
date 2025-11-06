# Change Log for TouchSensor Library

## v1.0.0

First stable version.

## v1.1.0

1. Tweak the algorithm used to decide whether a TouchSensor is being touched or not to accommodate a wider range of sensor implementations. Specifically, the amount by which the measured value must differ from the predicted value now depends partly on the measured value and partly on the noise. Previously, it depended only on a constant and the noise.

2. Add the ability to set the parameters used in the decision algorithm both initially and on the fly so that a sensor that doesn't work using the defaults can be accommodated.
