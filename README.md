# GiggleBot by Dexter Industries
This library provides blocks to control the Gigglebot, its motors, neopixels, sensors and servos.
For more information: https://www.gigglebot.io/pages/program-the-gigglebot-robot

## Example Usage

### Driving around
To drive forward and back for 2 seconds each:

![Makecode for forward and back](https://raw.githubusercontent.com/DexterInd/pxt-giggle/master/images/forward_backward_2sec.png)

### Turning, Spinning and Steering
Turning happens around a wheel, it's a nice way of going around an obstacle.
Spinning happens on the spot, useful when drawing
Steering is up to you, you can use it to orbit around a "sun"

![Turning, Spinning and Steering](https://raw.githubusercontent.com/DexterInd/pxt-giggle/master/images/turning_spinning_steering.png)

### Following a line
The GiggleBot comes with two line sensors that allows it to follow either a thin line or a thick line. The thin line is thin enough to fit between the two sensors, while the thick line is thick enough that both sensors will fit within the line.

![Follow a thick line](https://raw.githubusercontent.com/DexterInd/pxt-gigglebot/master/images/follow_line.png)

### Reading the line sensors

You can also access the line sensors values directly. This allows you to write a line follower logic that is tailored to your specific needs.

![Read Line Sensor Value](https://raw.githubusercontent.com/DexterInd/pxt-gigglebot/master/images/line_sensor_value.png)

### Following a light

The GiggleBot comes with two light sensors that allows it to follow a light, a little bit like a cat would.  Shine a flashlight onto one eye will get the GiggleBot to turn in that direction.

![Follow Light](https://raw.githubusercontent.com/DexterInd/pxt-gigglebot/master/images/follow_light.png)

### Reading the light sensor values

You can also read the light sensors values directly in order to implement a different behaviour, like having the GiggleBot fall asleep when it gets dark, and wake up when there's light.

![Falls Asleep when dark](https://raw.githubusercontent.com/DexterInd/pxt-gigglebot/master/images/light_sensor_falls_asleep.png)


### Using the distance sensor
This piece of code will start the Gigglebot forward when button A is pressed. At all times, the distance sensor displays the distance to an obstacle onto the smile and will stop the Gigglebot if the obstacle is closer than 10 cm.

![Distance sensor](https://raw.githubusercontent.com/DexterInd/pxt-giggle/master/images/distance_sensor.png)

### To use a second micro:bit as a remote control

Use a second micro:bit as a remote control for your Gigglebot. On this micro:bit, put the following code:
![Remote Control](https://raw.githubusercontent.com/DexterInd/pxt-giggle/master/images/microbit_controller.png)

On your gigglebot, put this code:

![Remote Controlled Gigglebot](https://raw.githubusercontent.com/DexterInd/pxt-giggle/master/images/gigglebot_controlled.png)

## Supported targets

* for PXT/microbit

## License

MIT License

Copyright 2018 Dexter Industries

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Code of Conduct

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/). For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.