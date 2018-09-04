# GiggleBot by Dexter Industries
This library provides blocks to control the Gigglebot, its motors, neopixels, sensors and servos.
For more information: https://www.gigglebot.io/pages/program-the-gigglebot-robot

## Example Usage

### Driving around  #driveMillisec
To drive forward and back for 2 seconds each:

```
input.onButtonPressed(Button.A, () => {
    gigglebot.driveMillisec(gigglebotWhichDriveDirection.Forward, 2000)
    gigglebot.driveMillisec(gigglebotWhichDriveDirection.Backward, 2000)
})
```

### Turning, Spinning and Steering
#### Turning happens around a wheel, it's a nice way of going around an obstacle. #turnMillisec
Turning will have the GiggleBot make a sharp turn around one of its wheel. 
This block will get the GiggleBot to turn right for a full second.
```
gigglebot.turnMillisec(gigglebotWhichTurnDirection.Right, 1000)
```
#### Spinning happens on the spot, useful when drawing. #spinMillisec
Using the spin block is useful when drawing because the turns will be around the pen, instead of around a wheel. That way, you can get good corners when drawing.

```
gigglebot.SpinMillisec(gigglebotWhichTurnDirection.Left, 1000)
```

#### Steering is up to you, you can use it to orbit around a "sun". #steerMillisec
With steering you control how much turning each wheel can do. In this example, the robot will halfway to the right, doing a curve around an object. The first number is percent-based. With a value of 0, the robot will not turn at all. With a value of 100, you will get the same behavior as the turn block.
```
gigglebot.steerMillisec(50, gigglebotWhichTurnDirection.Right, 1000)
```
### Moving without time constraint #drivestraight #turn #gigglebotspin #steer
You can have blocks to drive, turn, spin, and steer, which are not time limited. That way, you can decide when to interrupt the GiggleBot. Maybe it gets dark, or there's an obstacle ahead.

```
gigglebot.driveStraight(gigglebotWhichDriveDirection.Forward)
```
```
gigglebot.turn(gigglebotWhichTurnDirection.Right)
```
```
gigglebot.gigglebotSpin(gigglebotWhichTurnDirection.Right)
```
```
gigglebot.steer(20, gigglebotWhichTurnDirection.Right)
```

### STOP #stop
When using blocks that do not have a time limit to them, you will need to stop the robot yourself.
```
gigglebot.stop()
```

### Setting the speed #setspeed
You can set the motors to five different speeds, from slowest to fastest. You can either set one motor, or both at the same time. Be careful, if you set the motors to different speeds, it will not drive straight.

```
gigglebot.setSpeed(gigglebotWhichMotor.Both, gigglebotWhichSpeed.Slowest)
```

### Following a line #linefollow
The GiggleBot comes with two line sensors that allows it to follow either a thick line or a thin line. The thick line is thick enough that both sensors will attempt to be over the line , while the thin line is thin enough to fit between the two sensors,and each sensor will attempt to avoid the line.

```
gigglebot.lineFollow(gigglebotLineType.Thick)
```

### Reading the line sensors #linereadsensor

You can also access the line sensors values directly. This allows you to write a line follower logic that is tailored to your specific needs.

```
basic.showNumber(gigglebot.lineReadSensor(gigglebotWhichTurnDirection.Right)) 
```

You can use the following code as the basis for your own line follower. First, detect what are good values for detecting your line, then code the appropriate movements.
```
basic.forever(() => {
    if (gigglebot.lineReadSensor(gigglebotWhichTurnDirection.Right) < 200) {
        basic.showIcon(IconNames.Yes)
    } else {
        basic.showIcon(IconNames.Asleep)
    }
})
```

### Following a light

The GiggleBot comes with two light sensors that allows it to follow a spotlight, a little bit like a cat would.  Shining a flashlight onto one eye will get the GiggleBot to turn in that direction.
```
input.onButtonPressed(Button.A, () => {
    gigglebot.lightFollow()
})

```
### Reading the light sensor values

You can also read the light sensors values directly in order to implement a different behaviour, like having the GiggleBot fall asleep when it gets dark, and wake up when there's light.

```
basic.forever(() => {
    if (gigglebot.lightReadSensor(gigglebotWhichTurnDirection.Right) < 200) {
        basic.showIcon(IconNames.Asleep)
    } else {
        basic.showIcon(IconNames.Happy)
    }
})
```


## SERVO MOTORS
The GiggleBot comes with two servo connectors. What will you do with them? You can control them one by one, both together (in synchro), or both mirrored, depending on how they are positioned on the robot.
```
gigglebot.servoMove(gigglebotServoAction.Right, 90)
```

### Using the distance sensor

You can add a distance sensor to the robot. This distance sensor is not ultrasonic, but laser-based. 

The following code will start the Gigglebot forward when button A is pressed. At all times, the distance sensor displays the distance to an obstacle onto the smile and will stop the Gigglebot if the obstacle is closer than 10 cm.
'''
input.onButtonPressed(Button.A, function () {
    gigglebot.driveStraight(gigglebotWhichDriveDirection.Forward)
})
basic.forever(function () {
    lights.smileShowGraph(gigglebot.distanceSensorReadRangeContinuous(), 300)
    if (gigglebot.distanceSensorTestForObstacle(gigglebotInequality.Closer, 100)) {
        gigglebot.stop()
    }
})
'''

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