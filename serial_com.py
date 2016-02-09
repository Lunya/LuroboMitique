#!/usr/bin/python
# -*- coding: utf-8 -*-

import serial
import time import time as t


# instructions set      | number of arguments
PING = 0x01             # 0
READ_DATA = 0x02        # 2
WRITE_DATA = 0x03       # 2~
REG_WRITE = 0x04        # 2~
ACTION = 0x05           # 0
RESET = 0x06            # 0
SYNK_WRITE = 0x83       # 4~

# controls
CONTROLS = {
    Model_Number_L: 0x00,
    Model_Number_H: 0x01,
    Versionçof_Firmware: 0x02,
    ID: 0x03;
    Baud_rate: 0x04,
    Return_Delay_Time: 0x05,
    CW_Angle_Limit_L: 0x06,
    CW_Angle_Limit_H: 0x07,
    CCW_Angle_Limit_L: 0x08,
    CCW_Angle_Limit_H: 0x09,
    the_Highest_Limit_Temperature: 0x0b,
    the_Lowest_Limit_Voltage: 0x0c,
    the_Highest_Limit_Voltage: 0x0d,
    Max_Torque_L: 0x0e,
    Max_Torque_H: 0x0f,
    Status_Return_Level:0x10,
    Alarm_LED: 0x11,
    Alarm_Shutdown: 0x12,
    Down_Calibration_L: 0x14,
    Down_Calibration_H: 0x15,
    Up_Calibration_L: 0x16,
    Up_Calibration_H: 0x17,
    Torque_Enable: 0x18,
    LED: 0x19,
    CW_Compliance_Margin: 0x1a,
    CCW_Compliance_Margin: 0x1b,
    CW_Compliance_Slope: 0x1c,
    CCW_Compliance_Slope: 0x1d,
    Goal_Position_L: 0x1e,
    Goal_Position_H: 0x1f,
    Moving_Speed_L: 0x20,
    Moving_Speed_H: 0x21,
    Torque_Limit_L: 0x22,
    Torque_Limit_H: 0x23,
    Present_Position_L: 0x24,
    Present_Position_H: 0x25,
    Present_Speed_L: 0x26,
    Present_Speed_H: 0x27,
    Present_Load_L: 0x28,
    Present_Load_H: 0x29,
    Present_Voltage: 0x2a,
    Present_Temperature: 0x2b,
    Registered_Instruction: 0x2c,
    Moving: 0x2e,
    Lock: 0x2f,
    Punch_L: 0x30,
    Punch_H: 0x31
}


def open_serial(port, baud):
    ser = serial.Serial(port=port, baudrate=baud)
    if ser.isOpen():
        return ser
    else:
        print 'SERIAL ERROR'

def to_hex(val):
    return chr(val)


DATA_START = to_hex(0xff) + to_hex(0xff)


def close(ser):
    ser.close()


def write_data(ser, data):
    ser.write(data)


def read_data(ser, size=1):
    return ser.read(size)



def decode_data(data):
    res = ''
    for d in data:
        res += hex(ord(d)) + ' '
    
    return res


def set_id():

def move_motor():

def create_data(id, instruction, parameters = []):
    # we create the packet for a LED ON command
    # two start bytes
    data_start = to_hex(0xff)
    
    # length of the packet
    data_length = to_hex(2 + parameters.length)
    
    # checksum (read the doc)
    # attention au fait que les data sont des caractères
    data_checksum = id + data_length + instruction
    data = data_start + data_start + to_hex(id) + to_hex(instruction)
    for i from 0 to parameters.length:
        data += to_hex(parameters[i])
        data_checksum += parameters[i]

    if (data_checksum > 255):
        data_checksum = 0xff & data_checksum # bitwise AND opération to mask the 4 lowers bytes to checksum
    
    data += to_hex(~data_checksum) # negate checksum and convert it to hex
    return data

def ping_motors():
    serial_port = open_serial('/dev/ttyUSB0', 1000000, timeout=0.1)
    data = create_data(1,3)
    chrono = t.time()
    
    write_data(serial_port, data)
    
    # read the status packet (size 6)
    d = read_data(serial_port, 6)
    print(t.time() - chrono)
    
    
if __name__ == '__main__':
    
    # we open the port
    serial_port = open_serial('/dev/ttyUSB0', 1000000, timeout=0.1)
    
    # we create the packet for a LED ON command
    # two start bytes
    data_start = to_hex(0xff)
    
    # id of the motor (here 1), you need to change
    data_id = to_hex(0x01)
    
    # length of the packet
    data_length = to_hex(0x04)
    
    # instruction write= 0x03
    data_instruction = to_hex(0x03) # je vais écrire
    
    # instruction parameters
    data_param1 = to_hex(0x19)  # LED address=0x19 ;;; a l'adresse mémoire de la led
    data_param2 = to_hex(0x01)  # write 0x01 ;;; que j'exige qu'elle s'allume
    
    # checksum (read the doc)
    data_checksum = to_hex(0xdd)
    
    # we concatenate everything
    data = data_start + data_start + data_id + data_length + \
        data_instruction + data_param1 + data_param2 + data_checksum
    
    print decode_data(data)
    write_data(serial_port, data)
    
    # read the status packet (size 6)
    d = read_data(serial_port, 6)
    print decode_data(d)