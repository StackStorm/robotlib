'''
Based on Jesse Merritt's script:
https://github.com/jes1510/python_dynamixels

and Josue Alejandro Savage's Arduino library:
http://savageelectronics.blogspot.it/2011/01/arduino-y-dynamixel-ax-12.html
'''

from time import sleep
from serial import Serial
import RPi.GPIO as GPIO


class Ax12:
    # important AX-12 constants
    # /////////////////////////////////////////////////////////// EEPROM AREA
    AX_MODEL_NUMBER_L = 0
    AX_MODEL_NUMBER_H = 1
    AX_VERSION = 2
    AX_ID = 3
    AX_BAUD_RATE = 4
    AX_RETURN_DELAY_TIME = 5
    AX_CW_ANGLE_LIMIT_L = 6
    AX_CW_ANGLE_LIMIT_H = 7
    AX_CCW_ANGLE_LIMIT_L = 8
    AX_CCW_ANGLE_LIMIT_H = 9
    AX_SYSTEM_DATA2 = 10
    AX_LIMIT_TEMPERATURE = 11
    AX_DOWN_LIMIT_VOLTAGE = 12
    AX_UP_LIMIT_VOLTAGE = 13
    AX_MAX_TORQUE_L = 14
    AX_MAX_TORQUE_H = 15
    AX_RETURN_LEVEL = 16
    AX_ALARM_LED = 17
    AX_ALARM_SHUTDOWN = 18
    AX_OPERATING_MODE = 19
    AX_DOWN_CALIBRATION_L = 20
    AX_DOWN_CALIBRATION_H = 21
    AX_UP_CALIBRATION_L = 22
    AX_UP_CALIBRATION_H = 23

    # ////////////////////////////////////////////////////////////// RAM AREA
    AX_TORQUE_STATUS = 24
    AX_LED_STATUS = 25
    AX_CW_COMPLIANCE_MARGIN = 26
    AX_CCW_COMPLIANCE_MARGIN = 27
    AX_CW_COMPLIANCE_SLOPE = 28
    AX_CCW_COMPLIANCE_SLOPE = 29
    AX_GOAL_POSITION_L = 30
    AX_GOAL_POSITION_H = 31
    AX_GOAL_SPEED_L = 32
    AX_GOAL_SPEED_H = 33
    AX_TORQUE_LIMIT_L = 34
    AX_TORQUE_LIMIT_H = 35
    AX_PRESENT_POSITION_L = 36
    AX_PRESENT_POSITION_H = 37
    AX_PRESENT_SPEED_L = 38
    AX_PRESENT_SPEED_H = 39
    AX_PRESENT_LOAD_L = 40
    AX_PRESENT_LOAD_H = 41
    AX_PRESENT_VOLTAGE = 42
    AX_PRESENT_TEMPERATURE = 43
    AX_REGISTERED_INSTRUCTION = 44
    AX_PAUSE_TIME = 45
    AX_MOVING = 46
    AX_LOCK = 47
    AX_PUNCH_L = 48
    AX_PUNCH_H = 49

    # /////////////////////////////////////////////////////////////// Status Return Levels
    AX_RETURN_NONE = 0
    AX_RETURN_READ = 1
    AX_RETURN_ALL = 2

    # /////////////////////////////////////////////////////////////// Instruction Set
    AX_PING = 1
    AX_READ_DATA = 2
    AX_WRITE_DATA = 3
    AX_REG_WRITE = 4
    AX_ACTION = 5
    AX_RESET = 6
    AX_SYNC_WRITE = 131

    # /////////////////////////////////////////////////////////////// Lengths
    AX_RESET_LENGTH = 2
    AX_ACTION_LENGTH = 2
    AX_ID_LENGTH = 4
    AX_LR_LENGTH = 4
    AX_SRL_LENGTH = 4
    AX_RDT_LENGTH = 4
    AX_LEDALARM_LENGTH = 4
    AX_SHUTDOWNALARM_LENGTH = 4
    AX_TL_LENGTH = 4
    AX_VL_LENGTH = 6
    AX_AL_LENGTH = 7
    AX_CM_LENGTH = 6
    AX_CS_LENGTH = 5
    AX_COMPLIANCE_LENGTH = 7
    AX_CCW_CW_LENGTH = 8
    AX_BD_LENGTH = 4
    AX_TEM_LENGTH = 4
    AX_MOVING_LENGTH = 4
    AX_RWS_LENGTH = 4
    AX_VOLT_LENGTH = 4
    AX_LOAD_LENGTH = 4
    AX_LED_LENGTH = 4
    AX_TORQUE_LENGTH = 4
    AX_POS_LENGTH = 4
    AX_GOAL_LENGTH = 5
    AX_MT_LENGTH = 5
    AX_PUNCH_LENGTH = 5
    AX_SPEED_LENGTH = 5
    AX_GOAL_SP_LENGTH = 7

    # /////////////////////////////////////////////////////////////// Specials
    AX_BYTE_READ = 1
    AX_INT_READ = 2
    AX_ACTION_CHECKSUM = 250
    AX_BROADCAST_ID = 254
    AX_START = 255
    AX_CCW_AL_L = 255
    AX_CCW_AL_H = 3
    AX_LOCK_VALUE = 1
    LEFT = 0
    RIGTH = 1
    RX_TIME_OUT = 10
    TX_DELAY_TIME = 0.00002

    # RPi constants
    RPI_DIRECTION_PIN = 18
    RPI_DIRECTION_TX = GPIO.HIGH
    RPI_DIRECTION_RX = GPIO.LOW
    RPI_DIRECTION_SWITCH_DELAY = 0.0001

    # static variables
    port = None
    gpioSet = False

    def __init__(self):
        if Ax12.port is None:
            Ax12.port = Serial("/dev/serial0", baudrate=1000000, timeout=0.001)
        if not Ax12.gpioSet:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(Ax12.RPI_DIRECTION_PIN, GPIO.OUT)
            Ax12.gpioSet = True
        self.direction(Ax12.RPI_DIRECTION_RX)

    connectedServos = []

    # Error lookup dictionary for bit masking
    dictErrors = {
        1: "Input Voltage",
        2: "Angle Limit",
        4: "Overheating",
        8: "Range",
        16: "Checksum",
        32: "Overload",
        64: "Instruction"
    }

    class AXError(Exception):
        """Custom error class to report AX servo errors
        """
        pass

    class TimeoutError(Exception):
        """Servo timeout
        """
        pass

    def direction(self, direction):
        """Set IO direction
        """
        GPIO.output(Ax12.RPI_DIRECTION_PIN, direction)
        sleep(Ax12.RPI_DIRECTION_SWITCH_DELAY)

    def readData(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_RX)
        reply = Ax12.port.read(5)  # [0xff, 0xff, origin, length, error]
        try:
            assert reply[0] == 255
        except Exception as details:
            print(details)
            error = "Timeout on servo " + str(motor_id)
            raise Ax12.TimeoutError(error)

        try:
            length = reply[3] - 2
            error = reply[4]

            if error != 0:
                print(
                    "Error from servo: " +
                    Ax12.dictErrors[error] +
                    ' (code  ' + hex(error) +
                    ')'
                )
                return -error
            # just reading error bit
            elif length == 0:
                return error
            else:
                if length > 1:
                    reply = Ax12.port.read(2)
                    returnValue = (reply[1] << 8) + (reply[0] << 0)
                else:
                    reply = Ax12.port.read(1)
                    returnValue = reply[0]
                return returnValue
        except Exception as detail:
            raise Ax12.AXError(detail)

    def ping(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (~(motor_id + Ax12.AX_READ_DATA + Ax12.AX_PING)) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_READ_DATA])
        out_data += bytes([Ax12.AX_PING])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def factoryReset(self, motor_id, confirm=False):
        if confirm:
            self.direction(Ax12.RPI_DIRECTION_TX)
            Ax12.port.flushInput()
            checksum = (~(motor_id + Ax12.AX_RESET_LENGTH + Ax12.AX_RESET)) & 0xff
            out_data = bytes([Ax12.AX_START])
            out_data += bytes([Ax12.AX_START])
            out_data += bytes([motor_id])
            out_data += bytes([Ax12.AX_RESET_LENGTH])
            out_data += bytes([Ax12.AX_RESET])
            out_data += bytes([checksum])
            Ax12.port.write(out_data)
            sleep(Ax12.TX_DELAY_TIME)
            return self.readData(motor_id)
        else:
            print(
                "nothing done, please send confirm = True as this fuction"
                "reset to the factory default value, i.e reset the motor ID"
            )
            return

    def setID(self, motor_id, newId):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(motor_id + Ax12.AX_ID_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_ID + newId)
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_ID_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_ID])
        out_data += bytes([newId])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setBaudRate(self, motor_id, baudRate):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        baud_rate = ((2000000 / long(baudRate)) - 1)
        checksum = (
            ~(
                motor_id + Ax12.AX_BD_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_BAUD_RATE +
                baud_rate
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_BD_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_BAUD_RATE])
        out_data += bytes([baud_rate])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setStatusReturnLevel(self, motor_id, level):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_SRL_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_RETURN_LEVEL +
                level
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_SRL_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_RETURN_LEVEL])
        out_data += bytes([level])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setReturnDelayTime(self, motor_id, delay):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_RDT_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_RETURN_DELAY_TIME +
                (int(delay) / 2) & 0xff
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_RDT_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_RETURN_DELAY_TIME])
        out_data += bytes((int(delay) / 2) & 0xff)
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def lockRegister(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_LR_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_LOCK +
                Ax12.AX_LOCK_VALUE
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_LR_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_LOCK])
        out_data += bytes([Ax12.AX_LOCK_VALUE])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def move(self, motor_id, position):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        pos = [position & 0xff, position >> 8]
        checksum = (
            ~(
                motor_id +
                Ax12.AX_GOAL_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_GOAL_POSITION_L +
                pos[0] +
                pos[1]
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_GOAL_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_GOAL_POSITION_L])
        out_data += bytes([pos[0]])
        out_data += bytes([pos[1]])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def moveSpeed(self, motor_id, position, speed):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        pos = [position & 0xff, position >> 8]
        spd = [speed & 0xff, speed >> 8]
        checksum = (
            ~(
                motor_id +
                Ax12.AX_GOAL_SP_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_GOAL_POSITION_L +
                pos[0] +
                pos[1] +
                spd[0] +
                spd[1]
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_GOAL_SP_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_GOAL_POSITION_L])
        out_data += bytes([pos[0]])
        out_data += bytes([pos[1]])
        out_data += bytes([spd[0]])
        out_data += bytes([spd[1]])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def moveRW(self, motor_id, position):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        pos = [position & 0xff, position >> 8]
        checksum = (
            ~(
                motor_id +
                Ax12.AX_GOAL_LENGTH +
                Ax12.AX_REG_WRITE +
                Ax12.AX_GOAL_POSITION_L +
                pos[0] +
                pos[1]
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_GOAL_LENGTH])
        out_data += bytes([Ax12.AX_REG_WRITE])
        out_data += bytes([Ax12.AX_GOAL_POSITION_L])
        out_data += bytes([pos[0]])
        out_data += bytes([pos[1]])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def moveSpeedRW(self, motor_id, position, speed):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        pos = [position & 0xff, position >> 8]
        spd = [speed & 0xff, speed >> 8]
        checksum = (
            ~(
                motor_id +
                Ax12.AX_GOAL_SP_LENGTH +
                Ax12.AX_REG_WRITE +
                Ax12.AX_GOAL_POSITION_L +
                pos[0] +
                pos[1] +
                spd[0] +
                spd[1]
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_GOAL_SP_LENGTH])
        out_data += bytes([Ax12.AX_REG_WRITE])
        out_data += bytes([Ax12.AX_GOAL_POSITION_L])
        out_data += bytes([pos[0]])
        out_data += bytes([pos[1]])
        out_data += bytes([spd[0]])
        out_data += bytes([spd[1]])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def action(self):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_BROADCAST_ID])
        out_data += bytes([Ax12.AX_ACTION_LENGTH])
        out_data += bytes([Ax12.AX_ACTION])
        out_data += bytes([Ax12.AX_ACTION_CHECKSUM])
        Ax12.port.write(out_data)
        # sleep(Ax12.TX_DELAY_TIME)

    def setTorqueStatus(self, motor_id, status):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        status_value = 1 if ((status is True) or (status == 1)) else 0
        checksum = (
            ~(
                motor_id +
                Ax12.AX_TORQUE_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_TORQUE_STATUS +
                status_value
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_TORQUE_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_TORQUE_STATUS])
        out_data += bytes([status_value])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setLedStatus(self, motor_id, status):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        status_value = 1 if ((status is True) or (status == 1)) else 0
        checksum = (
            ~(
                motor_id +
                Ax12.AX_LED_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_LED_STATUS +
                status_value
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_LED_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_LED_STATUS])
        out_data += bytes([status_value])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setTemperatureLimit(self, motor_id, temp):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_TL_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_LIMIT_TEMPERATURE +
                temp
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_TL_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_LIMIT_TEMPERATURE])
        out_data += bytes([temp])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setVoltageLimit(self, motor_id, lowVolt, highVolt):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_VL_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_DOWN_LIMIT_VOLTAGE +
                lowVolt +
                highVolt
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_VL_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_DOWN_LIMIT_VOLTAGE])
        out_data += bytes([lowVolt])
        out_data += bytes([highVolt])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setAngleLimit(self, motor_id, cwLimit, ccwLimit):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        cw_value = [cwLimit & 0xff, cwLimit >> 8]
        ccw_value = [ccwLimit & 0xff, ccwLimit >> 8]
        checksum = (
            ~(
                motor_id +
                Ax12.AX_AL_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_CW_ANGLE_LIMIT_L +
                cw_value[0] +
                cw_value[1] +
                ccw_value[0] +
                ccw_value[1]
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_AL_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_CW_ANGLE_LIMIT_L])
        out_data += bytes([cw_value[0]])
        out_data += bytes([cw_value[1]])
        out_data += bytes([ccw_value[0]])
        out_data += bytes([ccw_value[1]])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setTorqueLimit(self, motor_id, torque):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        torque_value = [torque & 0xff, torque >> 8]
        checksum = (
            ~(
                motor_id +
                Ax12.AX_MT_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_MAX_TORQUE_L +
                torque_value[0] +
                torque_value[1]
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_MT_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_MAX_TORQUE_L])
        out_data += bytes([torque_value[0]])
        out_data += bytes([torque_value[1]])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setPunchLimit(self, motor_id, punch):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        punch_value = [punch & 0xff, punch >> 8]
        checksum = (
            ~(
                motor_id +
                Ax12.AX_PUNCH_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_PUNCH_L +
                punch_value[0] +
                punch_value[1]
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_PUNCH_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_PUNCH_L])
        out_data += bytes([punch_value[0]])
        out_data += bytes([punch_value[1]])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setCompliance(self, motor_id, cwMargin, ccwMargin, cwSlope, ccwSlope):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_COMPLIANCE_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_CW_COMPLIANCE_MARGIN +
                cwMargin +
                ccwMargin +
                cwSlope +
                ccwSlope
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_COMPLIANCE_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_CW_COMPLIANCE_MARGIN])
        out_data += bytes([cwMargin])
        out_data += bytes([ccwMargin])
        out_data += bytes([cwSlope])
        out_data += bytes([ccwSlope])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setLedAlarm(self, motor_id, alarm):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_LEDALARM_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_ALARM_LED +
                alarm
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_LEDALARM_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_ALARM_LED])
        out_data += bytes([alarm])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def setShutdownAlarm(self, motor_id, alarm):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_SHUTDOWNALARM_LENGTH +
                Ax12.AX_WRITE_DATA +
                Ax12.AX_ALARM_SHUTDOWN +
                alarm
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_SHUTDOWNALARM_LENGTH])
        out_data += bytes([Ax12.AX_WRITE_DATA])
        out_data += bytes([Ax12.AX_ALARM_SHUTDOWN])
        out_data += bytes([alarm])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def readTemperature(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_TEM_LENGTH +
                Ax12.AX_READ_DATA +
                Ax12.AX_PRESENT_TEMPERATURE +
                Ax12.AX_BYTE_READ
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_TEM_LENGTH])
        out_data += bytes([Ax12.AX_READ_DATA])
        out_data += bytes([Ax12.AX_PRESENT_TEMPERATURE])
        out_data += bytes([Ax12.AX_BYTE_READ])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def readPosition(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_POS_LENGTH +
                Ax12.AX_READ_DATA +
                Ax12.AX_PRESENT_POSITION_L +
                Ax12.AX_INT_READ
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_POS_LENGTH])
        out_data += bytes([Ax12.AX_READ_DATA])
        out_data += bytes([Ax12.AX_PRESENT_POSITION_L])
        out_data += bytes([Ax12.AX_INT_READ])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def readVoltage(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_VOLT_LENGTH +
                Ax12.AX_READ_DATA +
                Ax12.AX_PRESENT_VOLTAGE +
                Ax12.AX_BYTE_READ
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_VOLT_LENGTH])
        out_data += bytes([Ax12.AX_READ_DATA])
        out_data += bytes([Ax12.AX_PRESENT_VOLTAGE])
        out_data += bytes([Ax12.AX_BYTE_READ])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def readSpeed(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_SPEED_LENGTH +
                Ax12.AX_READ_DATA +
                Ax12.AX_PRESENT_SPEED_L +
                Ax12.AX_INT_READ
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_SPEED_LENGTH])
        out_data += bytes([Ax12.AX_READ_DATA])
        out_data += bytes([Ax12.AX_PRESENT_SPEED_L])
        out_data += bytes([Ax12.AX_INT_READ])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def readLoad(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_LOAD_LENGTH +
                Ax12.AX_READ_DATA +
                Ax12.AX_PRESENT_LOAD_L +
                Ax12.AX_INT_READ
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_LOAD_LENGTH])
        out_data += bytes([Ax12.AX_READ_DATA])
        out_data += bytes([Ax12.AX_PRESENT_LOAD_L])
        out_data += bytes([Ax12.AX_INT_READ])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def readMovingStatus(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_MOVING_LENGTH +
                Ax12.AX_READ_DATA +
                Ax12.AX_MOVING +
                Ax12.AX_BYTE_READ
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_MOVING_LENGTH])
        out_data += bytes([Ax12.AX_READ_DATA])
        out_data += bytes([Ax12.AX_MOVING])
        out_data += bytes([Ax12.AX_BYTE_READ])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def readRWStatus(self, motor_id):
        self.direction(Ax12.RPI_DIRECTION_TX)
        Ax12.port.flushInput()
        checksum = (
            ~(
                motor_id +
                Ax12.AX_RWS_LENGTH +
                Ax12.AX_READ_DATA +
                Ax12.AX_REGISTERED_INSTRUCTION +
                Ax12.AX_BYTE_READ
            )
        ) & 0xff
        out_data = bytes([Ax12.AX_START])
        out_data += bytes([Ax12.AX_START])
        out_data += bytes([motor_id])
        out_data += bytes([Ax12.AX_RWS_LENGTH])
        out_data += bytes([Ax12.AX_READ_DATA])
        out_data += bytes([Ax12.AX_REGISTERED_INSTRUCTION])
        out_data += bytes([Ax12.AX_BYTE_READ])
        out_data += bytes([checksum])
        Ax12.port.write(out_data)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(motor_id)

    def learnServos(self, minValue=1, maxValue=6, verbose=False):
        servo_list = []
        for i in range(minValue, maxValue + 1):
            try:
                self.ping(i)
                servo_list.append(i)
                if verbose:
                    print("Found servo #" + str(i))
                sleep(0.1)

            except Exception as detail:
                if verbose:
                    print("Error pinging servo #" + str(i) + ': ' + str(detail))
        return servo_list
