
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
  Slowest = 30,
  //% block="slower"
  Slower = 45,
  //% block="normal"
  Normal = 60,
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
  //% block="thick"
  Thick,
  //% block="thin"
  Thin
}

enum gigglebotLineColor {
  //% block="black"
  Black,
  //% block="white"
  White
}

enum gigglebotLightLevel {
  //% block="brightness"
  Brightness,
  //% block="darkness"
  Darkness
}

enum gigglebotTowardsAway {
  //% block="towards"
  Towards,
  //% block="away from"
  Away
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

enum gigglebotLightFollowMode {
  //% block="follow"
  Follow,
  //% block="avoid"
  Avoid
}

enum gigglebotTempScale {
  //% block="Celsius"
  Celsius,
  //% block="Farhenheit",
  Farhenheit
}

enum gigglebotPressureScale {
  //% block="Pascal"
  Pascal,
  //% block="inches"
  inches
}

/**
* Custom blocks
*/



//% weight=99 color=#46BFB1 icon="\uf0d1"
//% groups='["other", "Line Follower", "Light Sensors", "Servo", "Distance Sensor (Add-On)", "Temperature Humidity Pressure (Add-On)", "On Board Sensors", "Voltage", "Firmware"]'
namespace gigglebot {
  /**
   * Basic drive and sensor functionalities for GiggleBot
   * No radio, no neopixels functionalities here in order to be compatible with Bluetooth.
   * Load pxt-giggle for radio and neopixels
   */
  let ADDR = 0x04
  let line_follower_threshold = 175
  let light_level_threshold = 850
  let currentMotorPower = gigglebotWhichSpeed.Normal;
  let trimLeft = 0
  let trimRight = 0
  let motorPowerLeft = currentMotorPower
  let motorPowerRight = currentMotorPower
  let distanceSensorInitDone = false;
  let thpSensorInitDone = false;
  let line_follow_in_action = false;
  let light_follow_in_action = false;
  let lineSensors = [0, 0]
  let lightSensors = [0, 0]
  // turn motor power off
  stop()


  /**
   * return current power setting of the left motor
   */
  export function leftPower() {
      return motorPowerLeft
  }

  /**
   * return current power setting of the right motor
   */
  export function rightPower() {
      return motorPowerRight
  }

  /**
  * Reads left or right motor power
  * @param which left or right
  */
  //% blockId="gigglebot_read_motor_power" block="%which| power"
  //% advanced=true
  //% weight=80
  export function readMotorPower(which: gigglebotWhichUniqueMotor): number {
    if (which == gigglebotWhichUniqueMotor.Left)
      return motorPowerLeft
    if (which == gigglebotWhichUniqueMotor.Right)
      return motorPowerRight
    return 0
}

  /**
   * Assigns a new power value to the left motor
   * Values from 101 through 127, and -128 through -101 are used to float the  motor.
   * @param leftpower new value for the power setting of the left motor (-100 < leftpower < 100)
   */
  export function setLeftPower(leftpower: number){
      motorPowerLeft = leftpower
  }
  /**
   * Assigns a new power value to the right motor
   * Values from 101 through 127, and -128 through -101 are used to float the  motor.
   * @param rightpower new value for the power setting of the right motor. (-100 < rightpower < 100)
   */
  //% rightpower.min= -100 rightpower.max = 100
  export function setRightPower(rightpower: number) {
      motorPowerRight = rightpower
  }

  /**
   * Follows a line that is thicker than the space between the two sensors
   * The robot will stop when both of its sensors will detect white
   */
  function followLine(type_of_line: gigglebotLineType, max_attempts: number = 3) {
      let all_white = false
      let attempt = 0
      driveStraight(gigglebotWhichDriveDirection.Forward)
      while (!(all_white) && line_follow_in_action) {
        lineSensors = lineSensorsRaw()

        // test if robot is over white
        if (lineTest(gigglebotLineColor.White)) {
          attempt = attempt + 1
          if (attempt >= max_attempts) {
            all_white = true // gets us out of the loop
            stop()  // stops and sets line_follow_in_action to false
          } else {
            // serial.writeLine("move around a bit")
            // attempt to get out of bright spot
            driveStraight(gigglebotWhichDriveDirection.Forward)
            basic.pause(5)
          }
        // robot is seeing at least one black reading
        } else {
          attempt = 0
          // test if two black readings
          if (lineTest(gigglebotLineColor.Black)) {
            driveStraight(gigglebotWhichDriveDirection.Forward)

          } else if (lineSensors[0] < line_follower_threshold) {
            /* left sensor reads black, right sensor reads white */
            if (type_of_line == gigglebotLineType.Thick){
                turn(gigglebotWhichTurnDirection.Right)
            } else {
                turn(gigglebotWhichTurnDirection.Left)
            }

          } else if (lineSensors[1] < line_follower_threshold) {
            /* right sensor reads black, left sensor reads white */
            if (type_of_line == gigglebotLineType.Thick){
                turn(gigglebotWhichTurnDirection.Left)
            } else {
                turn(gigglebotWhichTurnDirection.Right)
            }
          }

          // play well with others
          basic.pause(20)
        }
      }
   }


  /**
  * Configures the Distance Sensor.
  * must be called before doing any distance sensor readings.
  * Called automatically when calling the distance sensor blocks
  */
  function distanceSensorConfigure() {
      distanceSensor.init()
      // set to long range (about 2.3 meters)
      // set final range signal rate limit to 0.1 MCPS (million counts per second)
      distanceSensor.setSignalRateLimitRaw(12) // 0.1 * (1 << 7) = 12.8
      distanceSensor.setVcselPulsePeriod(distanceSensor.vcselPeriodPreRange(), 18)
      distanceSensor.setVcselPulsePeriod(distanceSensor.vcselPeriodFinalRange(), 14)
      distanceSensor.startContinuous(0)
      distanceSensorInitDone = true
  }

  ////////////////////////////////////////////////////////////////////////
  ////////// BLOCKS
  ///////////////////////////////////////////////////////////////////////

  /**
   * Will let GiggleBot move forward or backward for a number of milliseconds.
   * Distance covered during that time is related to the freshness of the batteries.
   * @param dir forward or backward;
   * @param delay for how many milliseconds; eg: 1000
   */
  //% blockId="gigglebotDriveMillisec" block="drive %dir|for %delay|ms"
  //% weight=100
  //% delay.min=0
  export function driveMillisec(dir: gigglebotWhichDriveDirection, delay: number) {
      if (delay < 0) delay = 0
      driveStraight(dir)
      basic.pause(delay)
      stop()
  }

  /**
   * Will make GiggleBot turn left and right for a number of milliseconds. How far it turns depends on the freshness of the batteries.
   * @param turn_dir turning left or right
   * @param delay for how many milliseconds; eg: 1000
   */
  //% blockId="gigglebotTurnMillisec" block="turn %turn_dir|for %delay|ms"
  //% weight=99
  //% delay.min=0
  export function turnMillisec(turn_dir: gigglebotWhichTurnDirection, delay: number) {
      if (delay < 0) delay = 0
      turn(turn_dir)
      basic.pause(delay)
      stop()
  }

  /**
   * GiggleBot will spin on itself for the provided number of milliseconds, like a turn but staying in the same spot. Especially useful when drawing
   * @param turn_dir turning left or right
   * @param delay how many milliseconds; eg: 1000
   */
  //% blockId="gigglebotSpinMillisec" block="spin %turn_dir|for %delay|ms"
  //% weight=98
  //% delay.min=0
  export function spinMillisec(turn_dir: gigglebotWhichTurnDirection, delay: number) {
      if (delay < 0) delay = 0
      gigglebotSpin(turn_dir)
      basic.pause(delay)
      stop()
  }

  /**
   * GiggleBot will drive forward while steering to one side for the provided number of milliseconds.
   * Useful when it needs to go around an obstacle, or orbit around an object.
   * 0% means no steering, the same as the 'drive' block. 100% is the same as the 'turn' block.
   * @param percent the variation in power between left and right; eg: 0, 20, 50, 100
   * @param dir which direction to steer, left or right
   * @param delay for how many milliseconds; eg: 1000
   *      */
  //% blockId="gigglebotSteerMillisec" block="steer %percent| towards the %dir| for %delay| ms"
  //% percent.min=0 percent.max=100
  //% weight=97
  export function steerMillisec(percent: number, dir: gigglebotWhichTurnDirection, delay: number) {
      if (delay < 0) delay = 0
      if (percent < 0) percent = 0
      if (percent > 100) percent = 100
      steer(percent, dir)
      basic.pause(delay)
      stop()
  }

  /**
   * Will let GiggleBot move forward or backward until told otherwise (either by a stop block or a turn block).
   * @param dir forward or backward
   */
  //% blockId="gigglebot_drive_straight" block="drive %dir"
  //% weight=89
  export function driveStraight(dir: gigglebotWhichDriveDirection) {
      let dir_factor = 1
      if (dir == gigglebotWhichDriveDirection.Backward) {
          dir_factor = -1
      }
      if (dir == gigglebotWhichDriveDirection.Forward) {
          dir_factor = 1
      }
      motorPowerAssignBoth(motorPowerLeft * dir_factor, motorPowerRight * dir_factor)
  }

  /**
   * Will make GiggleBot turn left or right until told otherwise (by a stop block or a drive block).
   */
  //% blockId="gigglebotTurn" block="turn %turn_dir"
  //% weight=88
  export function turn(turn_dir: gigglebotWhichTurnDirection) {
      if (turn_dir == gigglebotWhichTurnDirection.Left) {
          motorPowerAssignBoth(0, motorPowerRight)
      }
      else {
          motorPowerAssignBoth(motorPowerLeft, 0)
      }
  }

  /**
   * GiggleBot will spin on itself until told otherwise, like a turn but staying in the same spot. Especially useful when drawing.
   * @param turn_dir left or right;
   */
  //% blockId="gigglebotSpin" block="spin %turn_dir"
  //% weight=87
  export function gigglebotSpin(turn_dir: gigglebotWhichTurnDirection) {
      if (turn_dir == gigglebotWhichTurnDirection.Left) {
          motorPowerAssignBoth(-1 * motorPowerLeft, motorPowerRight)
      }
      else {
          motorPowerAssignBoth(motorPowerLeft, -1 * motorPowerRight)
      }
  }

  /**
   * GiggleBot will drive forward while steering to one side.
   * Useful when it needs to go around an obstacle, or orbit around an object.
   * 0% means no steering, the same as the 'drive' block. 100% is the same as the 'turn' block.
   * @param percent value between 0 and 100 to control the amount of steering
   * @param dir to the left or to the right
   */
  //% blockId="gigglebotSteer" block="steer %percent| towards the %dir"
  //% percent.min=0 percent.max=100
  //% weight=86
  export function steer(percent: number, dir: gigglebotWhichTurnDirection) {
      percent = Math.min(Math.max(percent, 0), 100)
      let correctedMotorPowerLeft = motorPowerLeft
      let correctedMotorPowerRight = motorPowerRight
      if (dir == gigglebotWhichTurnDirection.Left) {
          correctedMotorPowerLeft = motorPowerLeft - Math.idiv(motorPowerLeft * percent, 100)
          correctedMotorPowerRight = motorPowerRight + Math.idiv(motorPowerRight * percent, 100)
      } else {
          correctedMotorPowerLeft = motorPowerLeft + Math.idiv(motorPowerLeft * percent, 100)
          correctedMotorPowerRight = motorPowerRight - Math.idiv(motorPowerRight * percent, 100)
      }
      motorPowerAssignBoth(correctedMotorPowerLeft, correctedMotorPowerRight)
  }

  /**
  * stops the robot.
  */
  //% blockId="gigglebot_stop" block="stop"
  //% weight=70
  export function stop() {
      motorPowerAssign(gigglebotWhichMotor.Both, 0)
      light_follow_in_action = false
      line_follow_in_action = false
  }


  /**
   * This allows the user to correct the motors on the GiggleBot if it's not driving straight
   * @param dir: if the GiggleBot drives to the left, then correct to the right. Vice versa.
   * @param trim_value: a correction value between 0 and 100, but most likely below 10
   */
  //% blockId="gigglebot_trim_main" block="correct towards %dir|by %trim_value"
  //% weight=60
  export function motorTrimSetMain(dir: gigglebotWhichTurnDirection, trim_value: number) {
    motorTrimSet(dir, trim_value)
  }

  /**
   * You can set the speed for each individual motor or both together. The higher the speed the less control the robot has.
   * You may need to correct the robot (see block in "more..." section).  A faster robot needs more correction than a slower one.
   * Note that any drive correction done previously gets applied here.
   * If you want to follow a line,  it will work best at a lower speed.
   * Actual speed is dependent on the freshness of the batteries.
   * @param motor: left, right or both motors
   * @param speed: how fast the robot goes.
   */
  //% blockId="gigglebot_set_speed" block="set %motor | speed to %speed"
  //% speed.min=-100 speed.max=100
  //% weight=60
  export function setSpeed(motor: gigglebotWhichMotor, speed: gigglebotWhichSpeed) {
      speed = Math.min(Math.max(speed, -100), 100)
      currentMotorPower = speed
      motorPowerLeft = currentMotorPower
      motorPowerRight = currentMotorPower

      // apply trim
      if (trimRight != 0 && motor != gigglebotWhichMotor.Left) {
          if (speed > 0) {
              motorPowerRight = currentMotorPower - Math.idiv(trimRight * currentMotorPower, 100);
          } else {
              motorPowerRight = currentMotorPower + Math.idiv(trimRight * currentMotorPower, 100);
          }
      }
      if (trimLeft != 0 && motor != gigglebotWhichMotor.Right) {
          if (speed > 0) {
              motorPowerLeft = currentMotorPower - Math.idiv(trimLeft * currentMotorPower, 100);
          } else {
              motorPowerLeft = currentMotorPower + Math.idiv(trimLeft * currentMotorPower, 100);
          }
      }
  }

  ///////////////////////////////////////////////////////////////////////
  /////////// LINE FOLLOWER BLOCKS
  ///////////////////////////////////////////////////////////////////////

  /**
   * A javascript method to change the line follower threshold.
   * Not exposed as a block
   */
  export function setLineFollowerThreshold(newThreshold: number) {
      line_follower_threshold = newThreshold
  }

  /**
   * A thin black line would fall between the two sensors. The GiggleBot will stop when both sensors are reading black.
   * A thick black line would have the two sensors on top of it at all times. The GiggleBot will stop when both sensors are reading white.
   * Calling this block puts the GiggleBot into "Line Follower Mode". To exit "Line Follower Mode" you need to call the "stop" block.
   * @param type_of_line thin line or thick line
   * @param specific_line_threshold overwrite the default line threshold to adapt to your particular tape and lighting condition.
  */
  //% group="Line Follower"
  //% blockId="gigglebot_follow_line" block="follow a %type_of_line| black line"
  //% weight=50
  export function lineFollow(type_of_line: gigglebotLineType, specific_line_threshold: number = 200) {
      // test if the line follower is already in action in case this was put
      // in a loop. Only launch one in background
      if (!line_follow_in_action) {
          line_follow_in_action = true
          light_follow_in_action = false // mutually exclusive
          line_follower_threshold = specific_line_threshold
          control.inBackground( () => {
              followLine(type_of_line)
              if (line_follow_in_action){
                  stop()
              }
          })
      }
  }

  /**
   * True if robot is currently following a line.
   * False otherwise
   */
  //% group="Line Follower"
  //% blockId="gigglebot_follow_line_status" block="following line"
  //% weight= 47
  export function lineFollowStatus() : boolean {
      return (line_follow_in_action)
  }

  /**
   * Will return true if the whole line sensor is reading either black or white.
   * @param color: black or white
  */
  //% blockId="gigglebot_test_line" block="%which|line is detected"
  //% advanced=true
  //% group="Line Follower"
  export function lineTest(color: gigglebotLineColor): boolean {
      lineSensorsRaw()
      for (let _i = 0; _i < lineSensors.length; _i++) {
          if (color == gigglebotLineColor.Black && lineSensors[_i] > line_follower_threshold) {
              return false
          }
          if (color == gigglebotLineColor.White && lineSensors[_i] < line_follower_threshold) {
              return false
          }
      }
      return true
  }

  /**
  * Reads left or right line sensor
  * @param which left or right
  */
  //% blockId="gigglebot_read_line_sensors" block="%which|line sensor"
  //% advanced=true
  //% group="Line Follower"
  export function lineReadSensor(which: gigglebotWhichTurnDirection): number {
      lineSensorsRaw()
      return lineSensors[which]
  }


  ///////////////////////////////////////////////////////////////////////
  /////////// LIGHT SENSOR BLOCKS
  ///////////////////////////////////////////////////////////////////////
  /**
   * Will follow a spotlight shone on its eyes. The GiggleBot will stop following
   * a light when it detects it is in darkness. It will not automatically
   * restart following a light once it leaves darkness
   * Using javascript, you can switch to light avoiding mode by changing the first
   * parameter to "gigglebotLightFollowMode.Avoid"
   * @param mode either follow or avoid light
   * @param sensitivity how much of a difference between the two sides is needed for GiggleBot to react; eg: 20
   * @param light_threshold how much light is needed to consider the loop needs to end. This can happen when a light following robot is covered with a box; eg: 10
   */
  //% blockId="gigglebot_follow_light" block="follow light"
  //% group="Light Sensors"
  //% weight=80
export function lightFollow(mode: gigglebotLightFollowMode = gigglebotLightFollowMode.Follow,
                            sensitivity: number = 20,
                            light_threshold: number = 10) {
  // test if the light follower is already in action in case this was put
  // in a loop. Only launch one in background
  if ( ! light_follow_in_action) {
    light_follow_in_action = true
    line_follow_in_action = false // mutually exclusive
    let giveup_count = 0;
    control.inBackground( () => {
      while ( light_follow_in_action && giveup_count < 5) {
        lightSensors = lightSensorsRaw()
        if (lightSensors[0] > lightSensors[1] + sensitivity) {
            // it's brighter to the right
            if (mode == gigglebotLightFollowMode.Follow){
                turn(gigglebotWhichTurnDirection.Right)
            } else {
                turn(gigglebotWhichTurnDirection.Left)
            }
        } else if (lightSensors[1] > lightSensors[0] + sensitivity) {
            // it's brighter to the left
            if (mode == gigglebotLightFollowMode.Follow){
                turn(gigglebotWhichTurnDirection.Left)
            } else {
                turn(gigglebotWhichTurnDirection.Right)
            }
        } else {
            driveStraight(gigglebotWhichDriveDirection.Forward)
        }

        if (mode == gigglebotLightFollowMode.Follow &&
            lightSensors[0] < light_threshold &&
            lightSensors[1] < light_threshold) {
            giveup_count = giveup_count + 1
        }
        else if (mode == gigglebotLightFollowMode.Avoid &&
            lightSensors[0] > 1000 - light_threshold &&
            lightSensors[1] > 1000 - light_threshold) {
            giveup_count = giveup_count + 1
        }
        else {
            // must have consecutive readings before giving up
            giveup_count = 0
        }

        // play well with others
        basic.pause(20);
      }
      // If we're still in light following mode, then stop
      // It's possible that stop() was called elsewhere
      if (light_follow_in_action){
          stop()
      }
    })
  }
}

    /**
     * Will orient the GiggleBot towards a light, just once.
     * To do a light following robot, this block needs to be inserted in a loop.
     * You control when the loop ends.
     * @param diff the difference between the two sensors that will trigger a reaction; eg: 50
     * @param delay how long in milliseconds to turn for before stopping; eg: 200
     *
     */
    //% blockId="gigglebot_orient_light" block="turn %dir light"
    //% group="Light Sensors"
    //% weight=80
    export function lightOrient(dir: gigglebotTowardsAway = gigglebotTowardsAway.Towards, diff: number = 50, delay: number = 200) {
      let current_lights = lightSensorsRaw()
      if (current_lights[0] > current_lights[1] + diff) {
          // it's brighter to the right
          if (dir == gigglebotTowardsAway.Towards){
            turn(gigglebotWhichTurnDirection.Right)
          }
          else{
            turn(gigglebotWhichTurnDirection.Left)
          }
      }
      else if (current_lights[1] > current_lights[0] + diff) {
          // it's brighter to the left
          if (dir == gigglebotTowardsAway.Towards){
            turn(gigglebotWhichTurnDirection.Left)
          }
          else {
            turn(gigglebotWhichTurnDirection.Right)
          }
      }
      else {
        stop()
      }
      basic.pause(delay)
      stop()
}

  /**
   * True if robot is currently following light.
   * False otherwise
   */
  //% group="Light Sensors"
  //% blockId="gigglebot_follow_light_status" block="following light"
  //% weight= 70
  export function lightFollowStatus() : boolean {
      return (light_follow_in_action)
  }

  /**
  * Reads left or right light sensor.
  * The light sensors are placed in front of each eye neopixel, they're tiny!
  * The range is 0 through 1023, although in reality rarely above ~950.
  * @param which left or right
  */
  //% blockId="gigglebot_read_light_sensors" block="%which|light sensor"
  //% advanced=true
  //% group="Light Sensors"
  export function lightReadSensor(which: gigglebotWhichTurnDirection): number {
      lightSensorsRaw()
      return lightSensors[which]
}

  /**
   * Will return true if both light sensors are detecting bright light, or darkness.
   * @param level: bright or darness
   * @param threshold: how sensitive the detection will be. The smaller the number, the less sensitive it will be.
  */
  //% blockId="gigglebot_test_light" block="%level| is detected"
  //% advanced=true
  //% group="Light Sensors"
  export function lightTest(level: gigglebotLightLevel, threshold: number = 10 ): boolean {
      lightSensorsRaw()
      for (let _i = 0; _i < lightSensors.length; _i++) {
        // if we're lookin for darkness and one of the sensor is above the threshold,
        // then we don't have darkness
          if (level == gigglebotLightLevel.Darkness && lightSensors[_i] > threshold) {
              return false
          }
          // if we're looking for brightness and one of the sensors is below the threshold,
          // then we don't have brightness
          // while the sensor reads theoretically up to 1023, in practice a bright environment is over 900
          if (level == gigglebotLightLevel.Brightness && lightSensors[_i] < (900-threshold)) {
              return false
          }
      }
      return true
  }

  ////////////////////////////////////////////////////////////////////////
  /////////// DISTANCE SENSOR
  ////////////////////////////////////////////////////////////////////////

  /**
   * Get a reading of how far an obstacle is from the distanse sensor.
   */
  //% blockId="distanceSensorReadRangeContinuous" block="distance to obstacle (mm)"
  //% advanced=true
  //% group="Distance Sensor (Add-On)"
  export function distanceSensorReadRangeContinuous(): number {
      if (distanceSensorInitDone == false) {
          distanceSensorConfigure()
      }
      return distanceSensor.readRangeContinuousMillimeters()
  }

  /**
   * Test for the presence of an obstacle.
   * @param inequality less than or more than, closer than or farther than
   * @param dist how many millimeters; eg: 100
   */
  //% blockId="distanceSensorTestForObstacle" block="obstacle is %inequality| %dist| mm"
  //% group="Distance Sensor (Add-On)"
  //% weight=30
  export function distanceSensorTestForObstacle(inequality: gigglebotInequality, dist: number): boolean {
      if (distanceSensorInitDone == false) {
          distanceSensorConfigure()
      }
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

  /**
   * Distance Sensor: takes a single reading.
   */
  export function distanceSensorReadRangeSingle(): number {
      if (distanceSensorInitDone == false) {
          distanceSensorConfigure()
      }
      return distanceSensor.readRangeSingleMillimeters()
  }

  /////////// SERVO BLOCKS

  /**
   * Positions a servo motor to a specified position
   * @param which left or right servo
   * @param degree which position, from 0 to 180
   */
  //% blockId="gigglebot_servo" block="set %which|servo to |%degree"
  //% group="Servo"
  //% degree.min=0 degree.max=180
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

  ///////////////////////////////////////////////////////////////////////
  /////////// THP SENSOR
  ///////////////////////////////////////////////////////////////////////

  /**
   * Get a temperature reading from the Temp/Humidity/Pressure sensor
   */
  //% group="Temperature Humidity Pressure (Add-On)"
  //% blockId="gigglebot_temperature" block="temperature in %scale"
  export function temperature(scale: gigglebotTempScale): number {
    if (thpSensorInitDone == false)
    {
      TempHumPressSensor.initialize()
      thpSensorInitDone = true
    }
    let temp =  TempHumPressSensor.read_temperature()
    if (scale == gigglebotTempScale.Celsius) {
      return (Math.round(temp))
    }
    else {
      return (Math.round((temp * 1.8)+32))
    }
  }

  /**
   * Get an atmospheric humidity reading from the Temp/Humidity/Pressure sensor
   */
  //% group="Temperature Humidity Pressure (Add-On)"
  //% blockId="gigglebot_humidity" block="humidity"
  export function humidity(): number {
    if (thpSensorInitDone == false)
    {
      TempHumPressSensor.initialize()
      thpSensorInitDone = true
    }
    return Math.round(TempHumPressSensor.read_humidity())
  }

  /**
   * Get an atmospheric pressure reading from the Temp/Humidity/Pressure sensor
   */
  //% group="Temperature Humidity Pressure (Add-On)"
  //% blockId="gigglebot_pressure" block="pressure in %pressureScale"
  export function pressure(pressureScale: gigglebotPressureScale): number {
    // call to read_temperature() to udpate temperature compensation
    TempHumPressSensor.read_temperature()

    if (thpSensorInitDone == false)
    {
      TempHumPressSensor.initialize()
      thpSensorInitDone = true
    }
    let p = TempHumPressSensor.read_pressure()
    if (pressureScale == gigglebotPressureScale.Pascal) {
      return Math.round(p)
    }
    else {
      return Math.round(p * 0.0002953)
    }
  }

  /**
   * Read Dew Point
   */
  //% group="Temperature Humidity Pressure (Add-On)"
  //% blockId="gigglebot_dewpoint" block="dew point in %scale"
  //% advanced=true
  export function dewPoint(scale: gigglebotTempScale): number {
    if (thpSensorInitDone == false)
    {
      TempHumPressSensor.initialize()
      thpSensorInitDone = true
    }
    if (scale == gigglebotTempScale.Celsius) {
      return Math.round(TempHumPressSensor.read_dewpoint())
    }
    else {
      return Math.round(TempHumPressSensor.read_dewpoint() * 1.8 + 32)
    }
  }

  ///////////////////////////////////////////////////////////////////////
  /////////// MORE BLOCKS
  ///////////////////////////////////////////////////////////////////////




  /**
   * This allows the user to correct the motors on the GiggleBot if it's not driving straight
   * @param dir: if the GiggleBot drives to the left, then correct to the right. Vice versa.
   * @param trim_value: a correction value between 0 and 100, but most likely below 10
   */
  //% blockId="gigglebot_trim" block="correct towards %dir|by %trim_value"
  //% weight=100
  //% advanced=true
  export function motorTrimSet(dir: gigglebotWhichTurnDirection, trim_value: number) {
      if (trim_value < 0) {
          trim_value = 0
      }
      if (dir == gigglebotWhichTurnDirection.Left) {
          trimLeft = trim_value
          trimRight = 0
      }
      if (dir == gigglebotWhichTurnDirection.Right) {
          trimRight = trim_value
          trimLeft = 0
      }
      if (motorPowerLeft > 0){
          motorPowerLeft = currentMotorPower - Math.idiv(trimLeft * currentMotorPower, 100)
      } else {
          motorPowerLeft = currentMotorPower + Math.idiv(trimLeft * currentMotorPower, 100)
      }
      if (motorPowerRight > 0) {
          motorPowerRight = currentMotorPower - Math.idiv(trimRight * currentMotorPower, 100)
      } else {
          motorPowerRight = currentMotorPower + Math.idiv(trimRight * currentMotorPower, 100)
      }
  }

  /**
   * Assigns power to a motor, or the same power to both motors
   * Values from 101 through 127, and -128 through -101 are used to float the  motor.
   * @param motor:  left or right motor, or both
   * @param power: a value between -100 and 100
   */
  //% blockId="gigglebot_set_motor" block="set power on %motor| to | %power"
  //% advanced=true
  //% weight=90
  export function motorPowerAssign(motor: gigglebotWhichMotor, power: number) {
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

  /**
   * Assigns potentially different powers to both motors in one call.
   * Values from 101 through 127, and -128 through -101 are used to float the  motor.
   * @param left_power: the power to assign to the left motor (between -100 and 100)
   * @param right_power: the power to assign to the right motor (between -100 and 100)
   */
  //% blockId="gigglebot_set_motors" block="set left power to %left_power|and right to | %right_power"
  //% advanced=true
  //% weight=90
  export function motorPowerAssignBoth(left_power: number, right_power: number) {
      let buf = pins.createBuffer(3)
      buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.SET_MOTOR_POWERS)
      buf.setNumber(NumberFormat.UInt8BE, 1, right_power)
      buf.setNumber(NumberFormat.UInt8BE, 2, left_power)
      pins.i2cWriteBuffer(ADDR, buf, false);
  }


  //% blockId="gigglebot_get_firmware" block="firmware version number"
  //% advanced=true
  //% weight=10
  //% group="Firmware"
  export function firmwareVersion(): number {
      /**
       * returns the firmware version that is installed.
       */
      let buf = pins.createBuffer(1)
      buf.setNumber(NumberFormat.UInt8BE, 0, gigglebotI2CCommands.GET_FIRMWARE_VERSION)
      pins.i2cWriteBuffer(ADDR, buf)
      let val = pins.i2cReadBuffer(ADDR, 2)
      return val.getNumber(NumberFormat.UInt16BE, 0);
  }

  /**
   * Displays the current battery voltage. Anything lower than 3.4 is too low to run the motors
   */
  //% blockId="gigglebot_show_voltage" block="show battery voltage (mv)"
  //% advanced=true
  //% group="Voltage"
  export function voltageShow() {
      let voltage = voltageBattery()
      basic.showNumber(voltage)
  }

  //% blockId="gigglebot_get_voltage" block="battery voltage (mv)"
  //% advanced=true
  //% group="Voltage"
  export function voltageBattery(): number {
      /**
       * Returns the voltage level of the batteries.
       */
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
  //% group="On Board Sensors"
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
  //% group="On Board Sensors"
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
