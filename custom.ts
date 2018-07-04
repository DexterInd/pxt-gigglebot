
/**
 * Use this file to define custom functions and blocks.
 * Read more at https://makecode.microbit.org/blocks/custom
 */

enum gigglebotWhichUniqueMotor {
    //% block="right motor"
    Right,
    //% block="left motor"
    Left
}
enum gigglebotWhichMotor {
    //% block="both motors"
    Both,
    //% block="right motor"
    Right,
    //% block="left motor"
    Left

}

enum gigglebotWhichDriveDirection {
    //% block="forward"
    Forward,
    //% block="backward"
    Backward
}

enum gigglebotWhichTurnDirection {
    //% block="right"
    Right,
    //% block="left"
    Left
}

enum gigglebotWhichUnitSystem {
    //% block="mm"
    mm,
    //% block="inches"
    inches
}

enum gigglebotWhichSpeed {
    //% block="slowest"
    Slowest = 25,
    //% block="slower"
    Slower = 35,
    //% block="normal"
    Normal = 50,
    //% block="faster"
    Faster = 75,
    //% block="fastest"
    Fastest = 90
}

enum gigglebotI2CCommands {
    GET_FIRMWARE_VERSION = 1,
    GET_MANUFACTURER,
    GET_BOARD,
    GET_VOLTAGE_BATTERY,
    GET_LINE_SENSORS,
    GET_LIGHT_SENSORS,
    GET_MOTOR_STATUS_RIGHT,
    GET_MOTOR_STATUS_LEFT,
    SET_MOTOR_POWER,
    SET_MOTOR_POWERS
}

enum gigglebotLineType {
    //% block="thin"
    Thin,
    //% block="thick"
    Thick
}

enum gigglebotLineColor {
    //% block="black"
    Black,
    //% block="white"
    White
}

enum gigglebotWhichEye {
    //% block="both eyes"
    Both,
    //% block="left eye"
    Left,
    //% block="right eye"
    Right
}

enum gigglebotEyeAction {
    //% block="open"
    Open,
    //% block="close"
    Close
}

enum gigglebotGigglePixels {
    Right,
    Left,
    SmileOne,
    SmileTwo,
    SmileThree,
    SmileFour,
    SmileFive,
    SmileSix,
    SmileSeven
}

enum gigglebotServoAction {
    //% block="right"
    Right,
    //% block="left"
    Left,
    //% block="both in synchro"
    Both,
    //% block="both in mirror"
    Mirror
}

enum gigglebotInequality {
    //% block="closer than"
    Closer,
    //% block="farther than"
    Farther
}


/**
 * Custom blocks
 */

//% weight=99 color=#46BFB1 icon="\uf0d1"
namespace gigglebot {
    /**
     */

    let LINE_FOLLOWER_THRESHOLD = 100
    let MOTOR_LEFT = 0x01
    let MOTOR_RIGHT = 0x02
    let ADDR = 0x04

    let init_done = false;

    let motorDegreesPerSecondLeft = gigglebotWhichSpeed.Normal
    let motorDegreesPerSecondRight = gigglebotWhichSpeed.Normal
    let directionLeft = gigglebotWhichDriveDirection.Forward
    let DirectionLeft = gigglebotWhichDriveDirection.Forward
    let lineSensors = [0, 0]
    let lightSensors = [0, 0]

    let defaultMotorPower = 50;
    let trimLeft = 0
    let trimRight = 0
    let motorPowerLeft = (defaultMotorPower - trimLeft)
    let motorPowerRight = (defaultMotorPower - trimRight)

    let stripNeopixel = neopixel.create(DigitalPin.P8, 9, NeoPixelMode.RGB)
    let eyeNeopixelBoth = stripNeopixel.range(0, 2)
    let eyeNeopixelLeft = stripNeopixel.range(1, 1)
    let eyeNeopixelRight = stripNeopixel.range(0, 1)
    let eyeColorLeft = neopixel.colors(NeoPixelColors.Blue)
    let eyeColorRight = neopixel.colors(NeoPixelColors.Blue)
    let smileNeopixel = stripNeopixel.range(2, 7)
    eyeNeopixelBoth.setBrightness(10)
    eyeNeopixelLeft.setBrightness(10)
    eyeNeopixelRight.setBrightness(10)
    smileNeopixel.setBrightness(40)
    for (let _i = 0; _i < gigglebotGigglePixels.SmileSeven; _i++) {
        stripNeopixel.setPixelColor(_i, neopixel.colors(NeoPixelColors.Black))
    }
    stripNeopixel.show()
    if (voltageRead() < 3400) {
        eyeColorLeft = neopixel.colors(NeoPixelColors.Red)
        eyeColorRight = neopixel.colors(NeoPixelColors.Red)
    }
    eyeNeopixelLeft.setPixelColor(0, eyeColorLeft)
    eyeNeopixelRight.setPixelColor(0, eyeColorRight)
    eyeNeopixelBoth.show()


    function init() {
        if (init_done == false) {
        }
        init_done = true;
        // serial.writeLine("INIT")
    }

    function followThinLine() {
        let all_black = false
        gigglebot.driveStraight(gigglebotWhichDriveDirection.Forward)
        while (!(all_black)) {
            lineSensors = gigglebot.lineSensorsRaw()
            if (gigglebot.lineTest(gigglebotLineColor.Black)) {
                all_black = true
                stripNeopixel.setPixelColor(0, neopixel.colors(NeoPixelColors.Black))
                stripNeopixel.setPixelColor(1, neopixel.colors(NeoPixelColors.Black))
                gigglebot.stop()
            } else if (gigglebot.lineTest(gigglebotLineColor.White)) {
                gigglebot.driveStraight(gigglebotWhichDriveDirection.Forward)
            } else if (lineSensors[0] < LINE_FOLLOWER_THRESHOLD) {
                stripNeopixel.setPixelColor(0, neopixel.colors(NeoPixelColors.Blue))
                gigglebot.stop()
                motorPowerSet(gigglebotWhichMotor.Left, motorPowerLeft + 5)
            } else if (lineSensors[1] < LINE_FOLLOWER_THRESHOLD) {
                stripNeopixel.setPixelColor(1, neopixel.colors(NeoPixelColors.Blue))
                gigglebot.stop()
                motorPowerSet(gigglebotWhichMotor.Right, motorPowerRight + 5)
            } else {
                stripNeopixel.setPixelColor(0, neopixel.colors(NeoPixelColors.Green))
                stripNeopixel.setPixelColor(1, neopixel.colors(NeoPixelColors.Green))
            }
            stripNeopixel.show()
        }
    }

    function followThickLine() {
        let all_white = false
        gigglebot.driveStraight(gigglebotWhichDriveDirection.Forward)
        while (!(all_white)) {
            lineSensors = gigglebot.lineSensorsRaw()
            if (gigglebot.lineTest(gigglebotLineColor.White)) {
                all_white = true
                stripNeopixel.setPixelColor(0, neopixel.colors(NeoPixelColors.Black))
                stripNeopixel.setPixelColor(1, neopixel.colors(NeoPixelColors.Black))
                gigglebot.stop()
            } else if (gigglebot.lineTest(gigglebotLineColor.Black)) {
                gigglebot.driveStraight(gigglebotWhichDriveDirection.Forward)
            } else if (lineSensors[0] > LINE_FOLLOWER_THRESHOLD) {
                stripNeopixel.setPixelColor(0, neopixel.colors(NeoPixelColors.Blue))
                gigglebot.stop()
                gigglebot.turn(gigglebotWhichTurnDirection.Right)
            } else if (lineSensors[1] > LINE_FOLLOWER_THRESHOLD) {
                stripNeopixel.setPixelColor(1, neopixel.colors(NeoPixelColors.Blue))
                gigglebot.stop()
                gigglebot.turn(gigglebotWhichTurnDirection.Left)
            } else {
                stripNeopixel.setPixelColor(0, neopixel.colors(NeoPixelColors.Green))
                stripNeopixel.setPixelColor(1, neopixel.colors(NeoPixelColors.Green))
            }
            stripNeopixel.show()
        }
    }

    ////////// BLOCKS

    /**
     * Will let gigglebot move forward or backward for a number of milliseconds.
     * Distance covered during that time is related to the freshness of the batteries.
     */
    //% blockId="gigglebot_drive_x_millisec" block="drive %dir|for %delay|ms"
    export function driveMillisec(dir: gigglebotWhichDriveDirection, delay: number) {
        let dir_factor = 1
        if (dir == gigglebotWhichDriveDirection.Backward) {
            dir_factor = -1
        }
        if (dir == gigglebotWhichDriveDirection.Forward) {
            dir_factor = 1
        }
        motorPowersSetBoth(motorPowerLeft * dir_factor, motorPowerRight * dir_factor)
        basic.pause(delay)
        motorPowerSet(gigglebotWhichMotor.Both, 0)
    }

    /**
     * Will make gigglebot turn left and right for a number of milliseconds. How far it turns depends on the freshness of the batteries.
     */
    //% blockId="gigglebot_turn_X_millisec" block="turn %turn_dir|for %delay|ms"
    export function turnMillisec(turn_dir: gigglebotWhichTurnDirection, delay: number) {
        if (turn_dir == gigglebotWhichTurnDirection.Left) {
            motorPowersSetBoth(0, motorPowerRight)
        }
        else {
            motorPowersSetBoth(motorPowerLeft, 0)
        }
        basic.pause(delay)
        motorPowerSet(gigglebotWhichMotor.Both, 0)
    }

    /**
     * Will let gigglebot move forward or backward until told otherwise (either by a stop block or a turn block).
     */
    //% blockId="gigglebot_drive_straight" block="drive %dir"
    export function driveStraight(dir: gigglebotWhichDriveDirection) {
        let dir_factor = 1
        if (dir == gigglebotWhichDriveDirection.Backward) {
            dir_factor = -1
        }
        if (dir == gigglebotWhichDriveDirection.Forward) {
            dir_factor = 1
        }
        motorPowersSetBoth(motorPowerLeft * dir_factor, motorPowerRight * dir_factor)
    }

    /**
     * Will make gigglebot turn left or right until told otherwise (by a stop block or a drive block).
     */
    //% blockId="gigglebot_turn" block="turn %turn_dir"
    export function turn(turn_dir: gigglebotWhichTurnDirection) {
        if (turn_dir == gigglebotWhichTurnDirection.Left) {
            motorPowersSetBoth(0, motorPowerRight)
        }
        else {
            motorPowersSetBoth(motorPowerLeft, 0)
        }
    }

    /**
    * stops the robot.
    */
    //% blockId="gigglebot_stop" block="stop"
    export function stop() {
        init()
        motorPowerSet(gigglebotWhichMotor.Both, 0)
    }

    /**
     * You can set the speed for each individual motor or both together. The higher the speed the less control the robot has.
     * You may need to correct the robot (see block in "more..." section).  A faster robot needs more correction than a slower one.
     * If you want to follow a line,  it will work best at a lower speed.
     * Actual speed is dependent on the freshness of the batteries.
     */
    //% blockId="gigglebot_set_speed" block="set %motor | speed to %speed"
    //% blockGap=32
    export function setSpeed(motor: gigglebotWhichMotor, speed: gigglebotWhichSpeed) {
        if (motor != gigglebotWhichMotor.Left) {
            motorPowerRight = speed - trimRight;
        }
        if (motor != gigglebotWhichMotor.Right) {
            motorPowerLeft = speed - trimLeft;
        }
        motorPowersSetBoth(motorPowerLeft, motorPowerRight)
    }


    /**
     * Use this block to turn a second Micro:bit into a remote controller.
     * Easiest approach is to put this block inside a "Forever" block.
     * You will need to use the "remote receiver mode" block on the GiggleBot itself.
     * @param radioBlock eg: 1
     */
    //% blockId="gigglebot_remote_control"
    //% block="external remote control, group %radio_block"
    export function remoteControl(radioBlock: number): void {
        let powerLeft = motorPowerLeft
        let powerRight = motorPowerRight
        radio.setGroup(radioBlock)
        powerLeft = ((motorPowerLeft * -1 * input.acceleration(Dimension.Y)) / 512) + ((50 * input.acceleration(Dimension.X)) / 512)
        powerRight = ((motorPowerRight * -1 * input.acceleration(Dimension.Y)) / 512) - ((50 * input.acceleration(Dimension.X)) / 512)
        // limit those values from -100 to 100
        powerLeft = Math.min(Math.max(powerLeft, 100), -100)
        powerRight = Math.min(Math.max(powerRight, 100), -100)
        if (Math.abs(powerLeft) < 2 && Math.abs(powerRight) < 2) {
            basic.showIcon(IconNames.No)
        } else if (powerLeft > 0 && powerRight > 0) {
            if (Math.abs(powerLeft - powerRight) < 10) {
                basic.showArrow(ArrowNames.North)
            } else if (powerLeft > powerRight) {
                basic.showArrow(ArrowNames.NorthEast)
            } else {
                basic.showArrow(ArrowNames.NorthWest)
            }
        } else if (powerLeft < 0 && powerRight < 0) {
            if (Math.abs(powerLeft - powerRight) < 10) {
                basic.showArrow(ArrowNames.South)
            } else if (powerLeft > powerRight) {
                basic.showArrow(ArrowNames.SouthWest)
            } else {
                basic.showArrow(ArrowNames.SouthEast)
            }
        } else if (powerLeft - powerRight < 0) {
            basic.showArrow(ArrowNames.West)
        } else if (powerLeft - powerRight > 0) {
            basic.showArrow(ArrowNames.East)
        }
        radio.sendValue(powerLeft+"", powerRight)
    }

    const packet = new radio.Packet();
    /**
     * Use this block on the GiggleBot to control it with a second micro:bit
     * @param radioBlock eg:1
     *
     */
    //% mutate=objectdestructuring
    //% mutateText=Packet
    //% mutateDefaults="radio_block"
    //% blockId=gigglebot_remote block="on received remote control, group %radio_block"
    export function onRemoteControl(radioBlock: number, cb: (packet: radio.Packet) => void) {
        radio.setGroup(radioBlock)
        radio.onDataReceived(() => {
            radio.receiveNumber();
            packet.receivedNumber = radio.receivedNumber();
            packet.time = radio.receivedTime();
            packet.serial = radio.receivedSerial();
            packet.receivedString = radio.receivedString();
            packet.receivedBuffer = radio.receivedBuffer();
            packet.signal = radio.receivedSignalStrength();
            cb(packet)
        });
    }

    /**
     * @param
     */
    //% blockId="gigglebot_remote_control_action"
    //% block="do remote control action"
    export function remoteControlAction(): void {
        motorPowerLeft = parseInt(packet.receivedString)
        motorPowerRight = packet.receivedNumber
        motorPowersSetBoth(motorPowerLeft, motorPowerRight)
    }

    //////////  NEOPIXEL BLOCKS

    /**
     * Lets you use the blocks in the neopixel category for better control over the eyes.
     */
    //% blockId="gigglebot_eye" block="%which"
    //% subcategory=Lights
    export function whichEye(which: gigglebotWhichEye): neopixel.Strip {

        if (which == gigglebotWhichEye.Left)
            return eyeNeopixelLeft
        else if (which == gigglebotWhichEye.Right)
            return eyeNeopixelRight
        else
            return eyeNeopixelBoth
    }

    /**
     * Lets you use the blocks in the neopixel category for better control over the smile/rainbow.
     */
    //% subcategory=Lights
    //% blockId="gigglebot_get_smile" block="smile"
    export function smile(): neopixel.Strip {
        return smileNeopixel
    }

    //% subcategory=Lights
    //% blockId="gigglebot_smile" block="display a  %smile_color|smile"
    export function smileShow(smile_color: NeoPixelColors) {
        smileNeopixel.showColor(neopixel.colors(smile_color))
    }

    /**
     * Will display a rainbow of colors on the smile lights
     */
    //% subcategory=Lights
    //% blockId="gigglebot_rainbow_smile" block="display a rainbow smile"
    export function smileRainbow() {
        smileNeopixel.showRainbow(1, 315)
    }

    /**
     * Displays the colors of the rainbow on the lights and cycles through them
     * @param nbcycles how many times the rainbow will do a full cycle; eg: 3, 5, 10
     */
    //% subcategory=Lights
    //% blockId="gigglebot_rainbow_cycle" block="cycle rainbow %nbcycles| times "
    //% blockSetVariable=smile
    export function smileCycleRainbow(nbcycles: number = 3) {
        smileNeopixel.showRainbow(1, 315)
        for (let _i = 0; _i < (nbcycles * 7); _i++) {
            basic.pause(100)
            smileNeopixel.rotate(1)
            smileNeopixel.show()
        }
    }

    /**
     * Displays the colors of the rainbow on the lights and cycles through them based on times
     * @param delay how long to wait(in ms) before cycling; eg: 100, 200
     * @param cycle_length how long (in ms) the cycling will last for: eg: 3000
     */
    //% subcategory=Lights
    //% blockSetVariable=smile
    //% blockId="gigglebot_rainbow_cycle_time" block="cycle rainbow every %delay| ms for %cycle_length| ms "
    export function smileCycleRainbowTime(delay: number = 100, cycle_length: number = 3000) {
        smileNeopixel.showRainbow(1, 315)
        for (let _i = 0; _i < (cycle_length / delay); _i++) {
            basic.pause(delay)
            smileNeopixel.rotate(1)
            smileNeopixel.show()
        }
    }

    /**
     * Use the smile lights to display a line graph of a certain value on a graph of 0 to Max value
     */

    //% subcategory=Lights
    //% blockSetVariable=smile
    //% blockId="gigglebot_line_graph" block="display graph of %graph_value| with a max of %graph_max"
    export function smileShowGraph(graph_value: number, graph_max: number) {
        smileNeopixel.showBarGraph(graph_value, graph_max)
    }


    /////////// LINE FOLLOWER BLOCKS
    /**
     * A thin black line would fall between the two sensors. The gigglebot will stop when both sensors are reading black.
     * A thick black line would have the two sensors on top of it at all times. The gigglebot will stop when both sensors are reading white.
    */
    //% blockId="gigglebot_follow_line" block="follow a %type_of_line| black line"
    //% subcategory=Sensors
    //% group=LineFollower
    export function lineFollow(type_of_line: gigglebotLineType) {
        stripNeopixel.setBrightness(10)

        if (type_of_line == gigglebotLineType.Thin) {
            followThinLine()
        }
        else {
            followThickLine()
        }
    }

    /**
     * Will return true if the whole line sensor is reading either black or white.
    */
    //% blockId="gigglebot_test_line" block="%which|line is detected"
    //% subcategory=Sensors
    //% group=LineFollower
    export function lineTest(color: gigglebotLineColor): boolean {
        lineSensorsRaw()
        for (let _i = 0; _i < lineSensors.length; _i++) {
            if (color == gigglebotLineColor.Black && lineSensors[_i] > LINE_FOLLOWER_THRESHOLD) {
                return false
            }
            if (color == gigglebotLineColor.White && lineSensors[_i] < LINE_FOLLOWER_THRESHOLD) {
                return false
            }
        }
        return true
    }


    /**
    * Reads left or right line sensor
    */
    //% blockId="gigglebot_read_line_sensors" block="%which|line sensor"
    //% subcategory=Sensors
    //% group=LineFollower
    //% blockGap=32
    export function lineReadSensor(which: gigglebotWhichTurnDirection): number {
        lineSensorsRaw()
        return lineSensors[which]
    }

    /**
     * Will follow a spotlight shone on its eyes. If the spotlight disappears the gigglebot will stop.
     */
    //% blockId="gigglebot_follow_light" block="follow light"
    //% subcategory=Sensors
    //% group=LightSensor

    export function lightFollow() {
        let diff = 0
        let current_lights = lightSensorsRaw()
        diff = Math.abs((current_lights[0] - current_lights[1])) / 10;
        if (current_lights[0] > current_lights[1]) {
            // it's brighter to the right
            motorPowersSetBoth(motorPowerLeft, motorPowerRight - diff)
        }
        else {
            // it's brighter to the left
            motorPowersSetBoth(motorPowerLeft - diff, motorPowerRight)
        }
    }

    /**
    * Reads left or right light sensor. 
    * The light sensors are placed in front of each eye neopixel, they're tiny! 
    * The range is 0 through 1023, although in reality rarely above ~950.
    */
    //% blockId="gigglebot_read_light_sensors" block="%which|light sensor"
    //% subcategory=Sensors
    //% group=LightSensor
    //% blockGap=32
    export function lightReadSensor(which: gigglebotWhichTurnDirection): number {
        lightSensorsRaw()
        return lightSensors[which]
    }

    /**
    * Configures the Distance Sensor.
    * must be called before doing any distance sensor readings.
    */
    //% blockId="DSconfigure" block="configure distance sensor"
    //% subcategory=Sensors
    //% group=LightSensor
    export function distanceSensorConfigure() {
        distanceSensor.init()
        // set to long range (about 2.3 meters)
        // set final range signal rate limit to 0.1 MCPS (million counts per second)
        distanceSensor.setSignalRateLimitRaw(12) // 0.1 * (1 << 7) = 12.8
        distanceSensor.setVcselPulsePeriod(distanceSensor.vcselPeriodPreRange(), 18)
        distanceSensor.setVcselPulsePeriod(distanceSensor.vcselPeriodFinalRange(), 14)
        distanceSensor.startContinuous(0)
    }

    /**
     * Get a reading of how far an obstacle is from the distanse sensor.
     */
    //% blockId="DSreadRangeContinuous" block="distance to obstacle (mm)"
    //% subcategory=Sensors
    //% group=LightSensor
    export function distanceSensorReadRangeContinuous(): number {
        return distanceSensor.readRangeContinuousMillimeters()
    }

    /**
     * Test for the presence of an obstacle.
     */
    //% subcategory=Sensors
    //% group=LightSensor
    //% blockId="DStestForObstacle" block="obstacle is %inequality| %dist| mm"
    //% blockGap=32
    export function distanceSensorTestForObstacle(inequality: gigglebotInequality, dist: number): boolean {
        if (inequality == gigglebotInequality.Closer) {
            if (distanceSensor.readRangeContinuousMillimeters() < dist) {
                return true
            }
            else {
                return false
            }
        }
        else if (inequality == gigglebotInequality.Farther) {
            if (distanceSensor.readRangeContinuousMillimeters() > dist) {
                return true
            }
            else {
                return false
            }
        }
        return false
    }

    export function distanceSensorReadRangeSingle(): number {
        return distanceSensor.readRangeSingleMillimeters()
    }

    /////////// SERVO BLOCKS

    //% blockId="gigglebot_servo" block="set %which|servo to |%degree"
    //% subcategory=Servos
    export function servoMove(which: gigglebotServoAction, degree: number) {
        if (which == gigglebotServoAction.Right) {
            pins.servoWritePin(AnalogPin.P13, degree)
        }
        else if (which == gigglebotServoAction.Left) {
            pins.servoWritePin(AnalogPin.P14, degree)
        }
        else if (which == gigglebotServoAction.Both) {
            pins.servoWritePin(AnalogPin.P13, degree)
            pins.servoWritePin(AnalogPin.P14, degree)
        }
        else if (which == gigglebotServoAction.Mirror) {
            pins.servoWritePin(AnalogPin.P13, degree)
            pins.servoWritePin(AnalogPin.P14, 180 - degree)
        }
    }

    /////////// MORE BLOCKS

    //% blockId="gigglebot_trim" block="correct towards %dir|by %trim_value"
    //% advanced=true
    export function motorTrimSet(dir: gigglebotWhichTurnDirection, trim_value: number) {
        init()

        if (dir == gigglebotWhichTurnDirection.Left) {
            trimLeft = trim_value
            motorPowerLeft = defaultMotorPower - trimLeft
        }
        if (dir == gigglebotWhichTurnDirection.Right) {
            trimRight = trim_value
            motorPowerRight = defaultMotorPower - trimRight
        }
    }

    //% blockId="gigglebot_set_motor" block="set power on %motor| to | %power"
    //% advanced=true
    export function motorPowerSet(motor: gigglebotWhichMotor, power: number) {
        init()
        let buf = pins.createBuffer(3)
        buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.SET_MOTOR_POWER)
        buf.setNumber(NumberFormat.UInt8BE, 2, power)
        // activate right motor
        if (motor == gigglebotWhichMotor.Right) {
            buf.setNumber(NumberFormat.UInt8BE, 1, 0x01)
        }
        // activate left motor
        else if (motor == gigglebotWhichMotor.Left) {
            buf.setNumber(NumberFormat.UInt8BE, 1, 0x02)
        }
        // activate both motors
        else if (motor == gigglebotWhichMotor.Both) {
            buf.setNumber(NumberFormat.UInt8BE, 1, 0x03)
        }
        pins.i2cWriteBuffer(ADDR, buf, false);
    }

    //% blockId="gigglebot_set_motors" block="set left power to %left_power|and right to | %right_power"
    //% advanced=true
    export function motorPowersSetBoth(left_power: number, right_power: number) {
        init()
        let buf = pins.createBuffer(3)
        buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.SET_MOTOR_POWERS)
        buf.setNumber(NumberFormat.UInt8BE, 1, right_power)
        buf.setNumber(NumberFormat.UInt8BE, 2, left_power)
        pins.i2cWriteBuffer(ADDR, buf, false);
    }

    /**
     * Displays the current battery voltage. Anything lower than 3.4 is too low to run the motors
     */
    //% blockId="gigglebot_show_voltage" block="show battery voltage (mv)"
    //% advanced=true
    export function voltageShow() {
        let voltage = voltageRead()
        basic.showNumber(voltage)
    }

    //% blockId="gigglebot_get_firmware" block="firmware version number"
    //% advanced=true
    export function firmwareRead(): number {
        /**
         * TODO: describe your function here
         * @param value describe value here, eg: 5
         */
        init()
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.GET_FIRMWARE_VERSION)
        pins.i2cWriteBuffer(ADDR, buf)
        let val = pins.i2cReadBuffer(ADDR, 2)
        return val.getNumber(NumberFormat.UInt16BE, 0);
    }

    //% blockId="gigglebot_get_voltage" block="battery voltage (mv)"
    //% advanced=true
    export function voltageRead(): number {
        /**
         * TODO: describe your function here
         * @param value describe value here, eg: 5
         */
        init()
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.GET_VOLTAGE_BATTERY)
        pins.i2cWriteBuffer(ADDR, buf)
        let val = pins.i2cReadBuffer(ADDR, 2)
        return val.getNumber(NumberFormat.UInt16BE, 0);
    }


    /**
    * Reads the two line sensors
    */
    //% blockId="gigglebot_read_raw_line_sensors" block="raw line sensors (x2)"
    //% advanced=true
    export function lineSensorsRaw(): number[] {
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.GET_LINE_SENSORS)
        pins.i2cWriteBuffer(ADDR, buf)
        let raw_buffer = pins.i2cReadBuffer(ADDR, 3)
        for (let _i = 0; _i < 2; _i++) {
            lineSensors[_i] = (raw_buffer.getNumber(NumberFormat.UInt8BE, _i) << 2)
            lineSensors[_i] |= (((raw_buffer.getNumber(NumberFormat.UInt8BE, 2) << (_i * 2)) & 0xC0) >> 6)
            lineSensors[_i] = 1023 - lineSensors[_i]
        }
        return lineSensors
    }


    //% blockId="gigglebot_read_raw_light_sensors" block="raw light sensors (x2)"
    //% advanced=true
    export function lightSensorsRaw(): number[] {
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.GET_LIGHT_SENSORS)
        pins.i2cWriteBuffer(ADDR, buf)
        let raw_buffer = pins.i2cReadBuffer(ADDR, 3)
        for (let _i = 0; _i < 2; _i++) {
            lightSensors[_i] = (raw_buffer.getNumber(NumberFormat.UInt8BE, _i) << 2)
            lightSensors[_i] |= (((raw_buffer.getNumber(NumberFormat.UInt8BE, 2) << (_i * 2)) & 0xC0) >> 6)
            lightSensors[_i] = 1023 - lightSensors[_i]
        }
        // serial.writeNumbers(light_sensor)
        return lightSensors
    }
}
