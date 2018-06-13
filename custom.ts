
/**
 * Use this file to define custom functions and blocks.
 * Read more at https://makecode.microbit.org/blocks/custom
 */

enum MyEnum {
    //% block="one"
    One,
    //% block="two"
    Two
}

enum WhichUniqueMotor {
    //% block="right motor"
    Right,
    //% block="left motor"
    Left
}
enum WhichMotor {
    //% block="right motor"
    Right,
    //% block="left motor"
    Left,
    //% block="both motors"
    Both
}

enum WhichDriveDirection {
    //% block="forward"
    Forward,
    //% block="backward"
    Backward
}

enum WhichTurnDirection {
    //% block="right"
    Right,
    //% block="left"
    Left
}

enum WhichUnitSystem {
    //% block="mm"
    mm,
    //% block="inches"
    inches
}

enum WhichSpeed {
    //% block="slowest"
    Slowest = 100,
    //% block="slower"
    Slower = 200,
    //% block="normal"
    Normal = 300,
    //% block="faster"
    Faster = 500,
    //% block="fastest"
    Fastest = 700
}

enum I2C_Commands {
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

enum I2C_Sensors {
    I2C_DISTANCE_SENSOR = 0x2A
}

/**
 * Custom blocks
 */



//% weight=99 color=#0fbc11 icon="\uf0d1"
namespace gigglebot {
    /**
     */

    let PIMULT = 31416
    let PIDIV = 10000
    let WHEEL_BASE_WIDTH = 108
    let WHEEL_DIAMETER10 = 665
    let WHEEL_BASE_CIRCUMFERENCE = 339
    let WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER10 * PIMULT / (10 * PIDIV)

    let MOTOR_GEAR_RATIO = 120
    let ENCODER_TICKS_PER_ROTATION = 6
    let MOTOR_TICKS_PER_DEGREE = (MOTOR_GEAR_RATIO * ENCODER_TICKS_PER_ROTATION) / 360

    let LINE_FOLLOWER_WHITE_THRESHOLD = 150
    let LINE_FOLLOWER_BLACK_THRESHOLD = 175
    let MOTOR_LEFT = 0x01
    let MOTOR_RIGHT = 0x02
    let ADDR = 0x04

    let init_done = false;

    let left_motor_dps = WhichSpeed.Normal
    let right_motor_dps = WhichSpeed.Normal
    let left_dir = WhichDriveDirection.Forward
    let right_dir = WhichDriveDirection.Forward
    let line_sensor = [0, 0, 0, 0, 0]

    let default_motor_power = 50;
    let trim = 0
    let motor_power_left = (default_motor_power + trim)
    let motor_power_right = (default_motor_power - trim)


    function init() {
        if (init_done == false) {
        }
        init_done = true;
        // serial.writeLine("INIT")
    }


    function is_black(sensor_reading: number): boolean {
        if (sensor_reading > LINE_FOLLOWER_BLACK_THRESHOLD) {
            return true
        }
        return false
    }

    function is_white(sensor_reading: number): boolean {
        if (sensor_reading < LINE_FOLLOWER_WHITE_THRESHOLD) {
            return true
        }
        return false
    }

    ////////// BLOCKS

    //% blockId="gobitgo_drive_straight" block="drive %dir"
    export function drive_straight(dir: WhichDriveDirection) {
        let dir_factor = 1
        if (dir == WhichDriveDirection.Backward) {
            dir_factor = -1
        }
        if (dir == WhichDriveDirection.Forward) {
            dir_factor = 1
        }
        set_motor_power(WhichMotor.Left, motor_power_left * dir_factor)
        set_motor_power(WhichMotor.Right, motor_power_right * dir_factor)
    }

    //% blockId="gobitgo_turn" block="turn %turn_dir"
    export function turn(turn_dir: WhichTurnDirection) {
        let left_dps = left_motor_dps
        let right_dps = right_motor_dps
        if (turn_dir == WhichTurnDirection.Left) {
            set_motor_power(WhichMotor.Right, motor_power_right)
            set_motor_power(WhichMotor.Left, motor_power_left)
        }
        else {
            set_motor_power(WhichMotor.Right, -motor_power_right)
            set_motor_power(WhichMotor.Left, motor_power_left)
        }
    }

    /**
    * stops the robot
    */
    //% blockId="gobitgo_stop" block="stop"
    export function stop() {
        init()
        set_motor_power(WhichMotor.Both, 0)
    }

    /**
     * Will follow a black line until it finds itself over a black square or a white square
    */
    //% blockId="gobitgo_follow_line" block="follow the black line"
    export function follow_line() {
        let line_status = 0b00000
        let in_movement = true
        while (in_movement) {
            get_raw_line_sensors()
            line_status = 0b00000
            for (let _i = 0; _i < line_sensor.length; _i++) {
                if (line_sensor[_i] > LINE_FOLLOWER_BLACK_THRESHOLD) {
                    line_status += 0b1 << _i
                }
                else if (line_sensor[_i] < LINE_FOLLOWER_WHITE_THRESHOLD) {
                    line_status += 0b0 << _i
                }
            }
            // if all black or all white
            if (line_status == 0b11111 || line_status == 0b0000) {
                stop()
                in_movement = false
            }
            // if centered
            if (line_status == 0b01110 || line_status == 0b00100) {
                drive_straight(WhichDriveDirection.Forward)
            }
            // if erring to the right
            else if (line_status == 0b11110 || line_status == 0b11100 || line_status == 0b11000 || line_status == 0b10000) {
                turn(WhichTurnDirection.Left)
            }
            // if erring to the left
            else if (line_status == 0b01111 || line_status == 0b00111 || line_status == 0b00011 || line_status == 0b00001) {
                turn(WhichTurnDirection.Right)
            }
        }
    }


    /**
     * Will return true if the whole line sensor is reading black, like when it's over a black square
    */
    //% blockId="gobitgo_test_black_line" block="black line is detected"
    export function test_black_line(): boolean {
        get_raw_line_sensors()
        for (let _i = 0; _i < line_sensor.length; _i++) {
            if (line_sensor[_i] < LINE_FOLLOWER_WHITE_THRESHOLD) {
                return false
            }
        }
        return true
    }

    /**
     * Will return true if the whole line sensor is reading white, like when it's over a blank page
    */
    //% blockId="gobitgo_test_white_line" block="white line is detected"
    export function test_white_line(): boolean {
        get_raw_line_sensors()
        for (let _i = 0; _i < line_sensor.length; _i++) {
            if (line_sensor[_i] > LINE_FOLLOWER_BLACK_THRESHOLD) {
                return false
            }
        }
        return true
    }

    /////////// MORE BLOCKS

    //% blockId="gobitgo_trim" block="correct power to motors by %trim_value to the %dir"
    //% advanced=true
    export function set_motor_trim(trim_value: number, dir: WhichTurnDirection) {
        init()
        trim = trim_value
        if (dir == WhichTurnDirection.Left) {
            // motor_power_left = default_motor_power - (trim_value )
            motor_power_right = default_motor_power + (trim_value)
        } if (dir == WhichTurnDirection.Right) {
            motor_power_left = default_motor_power + (trim_value)
            // motor_power_right = default_motor_power - (trim_value)
        }

    }

    //% blockId="gobitgo_set_motor" block="set power on %motor| to | %power"
    //% advanced=true
    export function set_motor_power(motor: WhichMotor, power: number) {
        init()
        let buf = pins.createBuffer(3)
        buf.setNumber(NumberFormat.UInt8BE, 0, I2C_Commands.SET_MOTOR_POWER)
        buf.setNumber(NumberFormat.UInt8BE, 2, power)
        // activate right motor
        if (motor == WhichMotor.Right) {
            buf.setNumber(NumberFormat.UInt8BE, 1, 0x01)
        }
        // activate left motor
        else if (motor == WhichMotor.Left) {
            buf.setNumber(NumberFormat.UInt8BE, 1, 0x02)
        }
        // activate both motors
        else if (motor == WhichMotor.Both) {
            buf.setNumber(NumberFormat.UInt8BE, 1, 0x03)
        }
        pins.i2cWriteBuffer(ADDR, buf, false);
    }

    //% blockId="gobitgo_set_motors" block="set left power to | %left_power and right to | %right_power"
    //% advanced=true
    export function set_motor_powers(left_power: number,  right_power: number) {
        init()
        let buf = pins.createBuffer(3)
        buf.setNumber(NumberFormat.UInt8BE, 0, I2C_Commands.SET_MOTOR_POWERS)
        buf.setNumber(NumberFormat.UInt8BE, 1, right_power)
        buf.setNumber(NumberFormat.UInt8BE, 2, left_power)
        pins.i2cWriteBuffer(ADDR, buf, false);
    }

    //% blockId="gobitgo_get_firmware" block="firmware version number"
    //% advanced=true
    export function get_firmware(): number {
        /**
         * TODO: describe your function here
         * @param value describe value here, eg: 5
         */
        init()
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, I2C_Commands.GET_FIRMWARE_VERSION)
        pins.i2cWriteBuffer(ADDR, buf)
        let val = pins.i2cReadBuffer(ADDR, 2)
        return val.getNumber(NumberFormat.UInt16BE, 0);
    }


    //% blockId="gobitgo_get_voltage" block="battery voltage (mv)"
    //% advanced=true
    export function get_voltage(): number {
        /**
         * TODO: describe your function here
         * @param value describe value here, eg: 5
         */
        init()
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, I2C_Commands.GET_VOLTAGE_BATTERY)
        pins.i2cWriteBuffer(ADDR, buf)
        let val = pins.i2cReadBuffer(ADDR, 2)
        return val.getNumber(NumberFormat.UInt16BE, 0);
    }


    //% blockId="gobitgo_read_raw_line_sensors" block="raw line sensors (x5)"
    //% advanced=true
    export function get_raw_line_sensors(): number[] {
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, I2C_Commands.GET_LINE_SENSORS)
        pins.i2cWriteBuffer(ADDR, buf)
        let raw_buffer = pins.i2cReadBuffer(ADDR, 7)
        for (let _i = 0; _i < line_sensor.length; _i++) {
            line_sensor[_i] = raw_buffer.getNumber(NumberFormat.UInt8BE, _i)
        }
        return line_sensor
    }
}
