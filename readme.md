# Library for talking to ADXL362 accelerometer and motion sensor

## Status

The autonomous motion switch method works, as well as all the functionality it
depends on. This is the primary focus of the author. All other functionality
is untested.

FIFO functionality is not implemented. Methods for setting some of the FIFO
registers are implemented, but no there's no code to actually read data.

## Bugs

autonomousMotionSwitch() initially turns INT1/INT2 high, even if no motion was
present

## Also see

If this library doesn't cover your needs, check out https://github.com/annem/ADXL362
