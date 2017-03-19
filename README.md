# yafg - Yet Another Frequency Generator
This project implements a very simple frequency generator for testing CNC Motor/Driver/Axis when putting in charge.
It is written in C using the arduino IDE and applies the Arduino Nano platform.
The implementation provides a potentiometer and six push buttons for reading the requested signal to be generated at the specified output/axis.
YAFG is useful when putting a CNC mechanics, stepper motors or stepper driver in charge.

### Basic Information:
* applies the Arduino Nano platform (ATMega328P)
* generates pulses for up to three stepper driver
* two press buttons per driver (left/right motor turn direction)
* potentiometer for adjusting speed/signal freqeuncy (approx 100Hz to 6KHz).

### Example Usage
![example usage](https://github.com/rubienr/yafg/blob/master/meta/yafg-sketch.jpg)
![hardware implementation](https://github.com/rubienr/yafg/blob/master/meta/hw-example.jpg)
