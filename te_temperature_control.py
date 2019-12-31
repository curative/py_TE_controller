#!/usr/bin/env python3

"""
te_temperature_control.py
Copyright 2019, Shield Diagnostics and the py_TE_controller contributors
License: Apache 2.0

This code allows the user to send commands and read data from the TC-720
temperature controller.
Link to manufacturers website: https://tetech.com/product/tc-720-oem/
"""

import logging
from typing import Optional, List, Iterable
from serial import Serial, SerialException
import serial.tools.list_ports


START_BYTE = '*'
END_BYTE = '\r'
ACK_BYTE = '^'
BAUDRATE = 230400
READ_TIMEOUT = 1
WRITE_TIMEOUT = 1
MAX_TEMP = 100.0
MIN_TEMP = 10.0
PROPORTIONAL_GAIN = 3.85
INTEGRAL_GAIN = 1.08
DERIVATIVE_GAIN = 0.0


class TEResponseInvalid(Exception):
    pass


def twos_complement(value: int) -> int:
    """
    This method returns the 16-bit two's complement of a positive number
    (used for sending negative values to the controller). It raises an error
    if the argument is too large or is negative.
    """
    assert value.bit_length() <= 16, "Value too large!"
    assert value > 0, "Value is negative!"
    return (2 ** 16) - value


def compute_checksum(message: Iterable) -> str:
    """
    This method returns the last two characters of the hex representation of
    the sum of all the decimal representations of the ASCII characters in
    message (aka the checksum). Raises an error if any part of the message is
    not a string.
    """
    checksum_value = 0
    for letter in message:
        assert type(letter) == str, \
            "Message is invalid (one or more parts are not strings)"
        checksum_value += ord(letter)
    return hex(checksum_value)[-2:]


def compose_command(command: str, arg: int = None) -> List[str]:
    """
    This method is used for creating a proper 10 byte command. It computes
    the checksum, adds a start and end byte, and returns a list of 1 character
    strings that make up a properly formatted command message that can be sent
    via serial to the TC-720 controller.
    """
    message = []
    message.append(START_BYTE)
    message.extend(command)
    hex_string = ''
    if arg is not None:
        if arg < 0:
            arg = twos_complement(abs(arg))
        hex_string = hex(arg)[2:]
    # Ensure hex string is exactly four characters
    hex_string = '0'*(4 - len(hex_string)) + hex_string
    message.extend(hex_string)
    checksum = compute_checksum(message[1:])
    message.extend(checksum)
    message.append(END_BYTE)
    return message


def send_command(port: Serial, command: str, arg: int = None) -> int:
    """
    This method is used for sending a serial command to the TC-720
    controller and reading the response. It returns an integer corresponding
    to argument value of the received message from the controller.
    Optionally it raises an error if the response message isn't formatted
    properly or doesn't match what the expected response is.
    """
    # Clear input buffer first
    port.flush()
    # Send message
    message = compose_command(command, arg)
    port.write(message.encode('ascii'))
    # Read response
    ack = port.read(8)
    if len(ack) != 8:
        raise TEResponseInvalid("Invalid Ack from TE controller "
                                "(not 8 bytes long: %i != 8" % (len(ack)))
    if chr(ack[0]) != START_BYTE:
        raise TEResponseInvalid("Invalid Ack from TE controller "
                                "(first byte not start byte): %i != %i" % (
                                    ack[0], int(START_BYTE)))
    if chr(ack[7]) != ACK_BYTE:
        raise TEResponseInvalid("Invalid Ack from controller "
                                "(last byte not end byte): %i != %i" % (
                                    ack[7], int(ACK_BYTE)))
    return_val_string = ''.join(chr(b) for b in ack[1:5])
    checksum = compute_checksum(return_val_string)
    if checksum[0] != chr(ack[5]) or checksum[1] != chr(ack[6]):
        raise TEResponseInvalid("Invalid response from controller "
                                "(checksum incorrect): %i,%i != %i,%i" % (
                                    ack[5], ack[6],
                                    int(checksum[0]), int(checksum[1])))
    try:
        return_val = int(return_val_string, 16)
    except Exception:
        raise TEResponseInvalid("Invalid octal value in TE respose: %s" % (
                                return_val_string))
    if arg is not None:
        if arg != return_val:
            raise TEResponseInvalid("Invalid response from controller "
                                    "(args don't match): %s != %s" % (
                                        arg, return_val))
    return return_val


def detect_temperature_controller() -> Optional['TemperatureController']:
    """
    This method is used for connecting to a TC-720 temperature controller
    via serial. If it successfully finds a controller it returns a
    TemperatureController class object, otherwise it returns None.
    """
    # Can alternatively fall back to globbing (requires `import glob`) with:
    # ports = glob.glob("/dev/ttyUSB[0-9]*")
    ports = [port.device for port in serial.tools.list_ports.comports()]

    for port in ports:
        logging.info("Attempting to connect to port %s" % (port))
        try:
            ser = Serial(port, BAUDRATE,
                         timeout=READ_TIMEOUT,
                         write_timeout=WRITE_TIMEOUT,
                         exclusive=True)

            if not ser.readable() or not ser.writable():
                ser.close()
            else:
                try:
                    response = send_command(ser, command='00')
                    if response == 9625:
                        logging.info("Succesfully connected to TC-720 "
                                     "Temperature Controller on port %s", port)
                        return TemperatureController(ser)
                except TEResponseInvalid:
                    pass
                ser.close()
        except (FileNotFoundError, IOError, OSError, EOFError, BlockingIOError,
                SerialException) as err:
            # Silently catch connection errors thrown by pyserial
            #  e.g. when the port is already held exclusively
            # Some of these may be platform dependent and change based on
            # pyserial version. These are exceptions we have seen in the past.
            # e.g. OSError is raised when doing fnctl/ioctl system calls on
            #   linux
            logging.debug("Failed to connect to '%s': %r", port, err)

    logging.error("Temperature Controller not found :'(")
    return None


class TemperatureController:
    """
    This class is used to represent one or more instances of a TC-720
    temperature controller. It contains methods for setting temperatures and
    reading values from the controller.
    """

    def __init__(self, port: Serial,
                 min_temp: float = MIN_TEMP,
                 max_temp: float = MAX_TEMP,
                 p_gain: float = PROPORTIONAL_GAIN,
                 i_gain: float = INTEGRAL_GAIN,
                 d_gain: float = DERIVATIVE_GAIN) -> None:
        self.port = port
        self.min_temp = min_temp
        self.max_temp = max_temp
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain

    def set_temperature(self, desired_temp: float) -> None:
        """
        This method changes the set temperature of the temperature controller.
        """
        if desired_temp < self.min_temp or desired_temp > self.max_temp:
            print("Temperature out of allowed range!")
            return
        command_temperature = int(desired_temp * 100)
        send_command(self.port, '1c', command_temperature)

    def read_sensor_temp(self, sensor: int = 1) -> float:
        """
        This method is used to read the temperature from sensor 1 or 2
        on the controller. Returns a float that is the temperature from the
        sensor, or None if an invalid sensor is chosen.
        """
        if sensor == 1:
            sensor_temp = send_command(self.port, '01')
            return sensor_temp / 100.0
        elif sensor == 2:
            sensor_temp = send_command(self.port, '04')
            return sensor_temp / 100.0
        else:
            raise ValueError("Sensor number must be 1 or 2: %s" % (sensor))

    def read_set_temp(self) -> float:
        """
        This method is used to read the current set temperature from the
        controller. It returns the set temperature as a float.
        """
        return send_command(self.port, '50') / 100.0

    def read_peltier_power(self) -> float:
        """
        This method is used to read the current power output (in percent)
        of the peltier. A positive percent means that the peltier is heating,
        a negative percent means that the peltier is cooling.
        """
        return send_command(self.port, '02') / 511.0

    def set_pid(self, p: float = None, i: float = None, d: float = None) \
            -> None:
        """
        This method is used to set the P, I, and D values for the PID loop.
        NOTE: SETTING THESE VALUES INCORRECTLY CAN DESTROY THE PELTIER ATTACHED
        TO THE CONTROLLER, USE EXTREME CAUTION WHEN CHANGING THESE VALUES
        """
        p = p or self.p_gain
        i = i or self.i_gain
        d = d or self.d_gain
        assert 0.0 <= p <= 20.0, "Extreme PID P value %f" % (p)
        assert 0.0 <= i <= 10.0, "Extreme PID I value %f" % (i)
        assert 0.0 <= d <= 10.0, "Extreme PID D value %f" % (d)
        send_command(self.port, '1d', int(p * 100.0))
        send_command(self.port, '1e', int(i * 100.0))
        send_command(self.port, '1f', int(d * 100.0))
        self.p_gain = p
        self.i_gain = i
        self.d_gain = d

    def read_pid(self) -> List[float]:
        """
        This method returns the current P, I, and D values that are set on
        the controller as a list in the form [P, I, D].
        """
        pid_list = []  # type: List[float]
        pid_list.append(send_command(self.port, '51') / 100.0)
        pid_list.append(send_command(self.port, '52') / 100.0)
        pid_list.append(send_command(self.port, '53') / 100.0)
        return pid_list

    def set_controller_max_set_temp(self, max_temp_c: float = MAX_TEMP) -> None:
        """
        This method sets the max temperature in celsius that the controller
        will allow the set point to be.
        """
        assert 0.0 <= max_temp_c <= 100.0
        send_command(self.port, '23', int(max_temp_c))
        self.max_temp = max_temp_c

    def set_controller_min_set_temp(self, min_temp_c: float = MIN_TEMP) -> None:
        """
        This methond sets the minimum temperature that the controller will
        allow the set point to be.
        """
        assert 0.0 <= min_temp_c <= 100.0
        send_command(self.port, '22', int(min_temp_c))
        self.min_temp = min_temp_c

    def set_control_mode(self, cntrl_mode: int = 0) -> None:
        """
        This method sets the control mode for the controller. 0 is normal
        set mode, 1 is ramp/soak mode, and 2 is proportional/deadband mode.
        Unless you know what you're doing, you want to be in mode 0.
        """
        assert cntrl_mode in (0, 1, 2), "Invalid mode not in {0, 1, 2}"
        send_command(self.port, '3d', int(cntrl_mode))

    def set_control_type(self, cntrl_type: int = 0) -> None:
        """
        This method sets the control type for the controller. 0 is PID/PWM
        mode, 1 is manual mode, and 2 is PID analog output mode. Unless you
        know what you're doing, you want to be in mode 0.
        """
        assert cntrl_type in (0, 1, 2), "Invalid type not in {0, 1, 2}"
        send_command(self.port, '3f', int(cntrl_type))

    def read_pcb_temp(self) -> float:
        """
        This method reads the temperature of the controller PCB.
        """
        return send_command(self.port, '0c') / 100.0

    def reset(self, minimum_temp: float = MIN_TEMP,
              maximum_temp: float = MAX_TEMP,
              p_g: float = PROPORTIONAL_GAIN,
              i_g: float = INTEGRAL_GAIN,
              d_g: float = DERIVATIVE_GAIN,
              eeprom_write_setting: bool = False,
              control_mode: int = 0,
              control_type: int = 0,
              heat_multiplier: float = 1.00,
              cool_multiplier: float = 1.00) -> None:
        """
        This method resets the controller to its default settings.
        """
        self.eeprom_write(eeprom_write_setting)
        self.set_control_mode(control_mode)
        self.set_control_type(control_type)
        self.set_heat_multiplier(heat_multiplier)
        self.set_cool_multiplier(cool_multiplier)
        self.set_controller_min_set_temp(minimum_temp)
        self.set_controller_max_set_temp(maximum_temp)
        self.set_pid(p_g, i_g, d_g)

    def set_peltier_power(self, power: float) -> None:
        """
        This method allows the user to set the power of the peltier manually.
        The range for setting is +1.00 (heating at full power) to -1.00
        (cooling at full power).
        """
        assert -1.0 < power < 1.0, "Invalid power not between -1.00 and 1.00"
        send_command(self.port, '40', int(511*power))

    def eeprom_write(self, setting: bool) -> None:
        """
        This method allows the user to enable eeprom write (True) or disable
        eeprom write (False). On powerup, the controller initializes all
        variables from eeprom. When write is enabled, setting a parameter on
        the controller will also write that parameter to eeprom. When write is
        disabled, setting a parameter on the controller will not write to
        eeprom and that parameter will revert to the eeprom value if the
        controller is power cycled.
        NOTE: The maximum number of writes to eeprom is 1,000,000
        """
        send_command(self.port, '31', int(setting))

    def read_eeprom_write_setting(self) -> bool:
        """
        This method allows the user read if eeprom write is on (True) or off
        (False)
        """
        return bool(send_command(self.port, '65'))

    def set_heat_multiplier(self, heat_mult: float) -> None:
        """
        This method allows the user to set the heat multiplier variable.
        The heat multiplier multiplies the output power percentage when the
        peltier is heating, e.g. with a heat multiplier of 0.5 the controller
        will output 50% of the power that it normally does while heating.
        """
        assert 0 <= heat_mult <= 1.00
        send_command(self.port, '34', int(heat_mult * 100.0))

    def set_cool_multiplier(self, cool_mult: float) -> None:
        """
        This method allows the user to set the cool multiplier variable.
        The cool multiplier multiplies the output power percentage when the
        peltier is cooling, e.g. with a cool multiplier of 0.5 the controller
        will output 50% of the power that it normally does while cooling.
        """
        assert 0 <= cool_mult <= 1.00
        send_command(self.port, '33', int(cool_mult * 100.0))

    def set_output(self, val: bool) -> None:
        """
        This method is called to turn on (True) or off (False) output to
        peltier from the controller.
        """
        send_command(self.port, '30', int(val))

    def read_output_setting(self) -> bool:
        """
        This method is used to read whether output to the peltier is turned
        on (True) or off (False).
        """
        return bool(send_command(self.port, '64'))

    def close(self) -> None:
        """
        Closes serial port associated with temperature controller
        """

        # Attempt to cleanly flush the serial buffers
        self.port.flush()

        # We tried to flush cleanly, but lets reset them just in case
        # see: https://github.com/pyserial/pyserial/issues/226
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()

        # Close the serial port
        self.port.close()
