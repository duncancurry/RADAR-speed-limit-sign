#!/usr/bin/env python3
####################################################
#
# Description: Speed Limit sign using
#              OmniPreSense OPS241-A RADAR Sensor
# By: Duncan Curry
#
# Last Modified: 2017/11/1
#
# Rev: 0.1 :    0. 2017/06/28
#               1. Initial release
# Rev: 0.2 :    0. 11/1/2017
#			1. Add speed value averaging and maxValue
#               1. Ensure that USB buffer does not overflow
# 
#####################################################
# Modifiable parameters

# Raspberry Pi gpio output pin assignments
segment_latch = 7
segment_clock = 16
segment_data = 29
reset_speed_time = 5
display_max_speed_time = 1

# Ops241A module settings:  mph, dir off, 20Ksps, min -9dB pwr, squelch 5000
Ops241A_Speed_Output_Units = 'US'
Ops241A_Direction_Control = 'Od'
Ops241A_Sampling_Frequency = 'S2'
Ops241A_Transmit_Power = 'PX'
Ops241A_Threshold_Control = 'QI'
Ops241A_Module_Information = '??'
Ops241A_Data_Accuracy = 'F1'

#####################################################
# Do not make any changes below here
#####################################################
# Import time, decimal, serial, GPIO, reg expr, sys, and pygame modules
import os
import sys
from time import *
from decimal import *
import serial
import RPi.GPIO as GPIO
import re

# Configure GPIO modes
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Configure GPIO output and input channels
GPIO.setup(segment_latch, GPIO.OUT)
GPIO.setup(segment_clock, GPIO.OUT)
GPIO.setup(segment_data, GPIO.OUT)

# Initialize the USB port to read from the OPS-241A module
ser=serial.Serial(
    port = '/dev/ttyACM0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1,
    writeTimeout = 2
)
ser.flushInput()
ser.flushOutput()

# sendSerialCommand: function for sending commands to the OPS-241A module
def sendSerCmd(descrStr, commandStr) :
    data_for_send_str = commandStr
    data_for_send_bytes = str.encode(data_for_send_str)
    print(descrStr, commandStr)
    ser.write(data_for_send_bytes)
    # Initialize message verify checking
    ser_message_start = '{'
    ser_write_verify = False
    # Print out module response to command string
    while not ser_write_verify :
        data_rx_bytes = ser.readline()
        data_rx_length = len(data_rx_bytes)
        if (data_rx_length != 0) :
            data_rx_str = str(data_rx_bytes)
            if data_rx_str.find(ser_message_start) :
                print(data_rx_str)
                ser_write_verify = True
            
# Initialize and query Ops241A Module
print("\nInitializing Ops241A Module")
sendSerCmd("\nSet Speed Output Units: ", Ops241A_Speed_Output_Units)
sendSerCmd("\nSet Direction Control: ", Ops241A_Direction_Control)
sendSerCmd("\nSet Sampling Frequency: ", Ops241A_Sampling_Frequency)
sendSerCmd("\nSet Transmit Power: ", Ops241A_Transmit_Power)
sendSerCmd("\nSet Threshold Control: ", Ops241A_Threshold_Control)
sendSerCmd("\nSet Data Accuracy: ", Ops241A_Data_Accuracy)
sendSerCmd("\nModule Information: ", Ops241A_Module_Information)


ser=serial.Serial(
    port = '/dev/ttyACM0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 0.01,
    writeTimeout = 2
    )

def postNumber(number, decimal) :
    # 7 segment display description
    #  -  A
    # / / F/B
    #  -  G
    # / / E/C
    #  _. D/DP
    a  = 1<<0
    b  = 1<<6
    c  = 1<<5
    d  = 1<<4
    e  = 1<<3
    f  = 1<<1
    g  = 1<<2
    dp = 1<<7
    # case statement for turning on segments based on number
    if number == 1 :
        segments = b | c
    elif number == 2 :
        segments = a | b | d | e | g
    elif number == 3 :
        segments = a | b | c | d | g
    elif number == 4 :
        segments = f | g | b | c
    elif number == 5 :
        segments = a | f | g | c | d
    elif number == 6 :
        segments = a | f | g | e | c | d
    elif number == 7 :
        segments = a | b | c
    elif number == 8 :
        segments = a | b | c | d | e | f | g
    elif number == 9 :
        segments = a | b | c | d | f | g
    elif number == 0 :
        segments = a | b | c | d | e | f
    else :
        segments = 0
    if (decimal) :
        segments |= dp
    # output the bits to the LED segment driver
    for x in range (0, 8) :
        GPIO.output(segment_clock, GPIO.LOW)
        GPIO.output(segment_data, segments & 1 << (7 - x) )
        GPIO.output(segment_clock, GPIO.HIGH)
    
def showNumber(value) :
    # remove negative signs and decimals
    number = int(abs(value))
    if number > 99 :
        number = 99
    for x in range (0, 2) :
        remainder = int(number % 10)
        postNumber(remainder, False)
        number /= 10
    # latch the current segment data, data is latched on rising edge of RCK 
    GPIO.output(segment_latch, GPIO.LOW)
    GPIO.output(segment_latch, GPIO.HIGH)

# Speed limit display loop
done = False
# Flush serial buffers
ser.flushInput()
ser.flushOutput()
# Reset timers
start_time = time()
current_time = start_time
delta_time = 0.0
# Initial state 
GPIO.output(segment_clock, GPIO.LOW)
GPIO.output(segment_data, GPIO.LOW)
GPIO.output(segment_latch, GPIO.LOW)
speed_max = 0.0
while not done:
    # Check for speed info from OPS241-A
    speed_available = False
    Ops241_rx_bytes = ser.readline()
    Ops241_rx_bytes_length = len(Ops241_rx_bytes)
    if (Ops241_rx_bytes_length != 0) :
        Ops241_rx_str = str(Ops241_rx_bytes)
        if Ops241_rx_str.find('{') == -1 :
            # Speed data found
            Ops241_rx_float = float(Ops241_rx_bytes)
            speed_available = True
    if speed_available == True :
        print('speed is:', Ops241_rx_float)
        if Ops241_rx_float > speed_max :
            speed_max = Ops241_rx_float
            showNumber(Ops241_rx_float)
            start_time = time()
            current_time = start_time
        else :
            current_time = time()
            delta_time = current_time - start_time
            if delta_time > display_max_speed_time :
                showNumber(Ops241_rx_float)
                speed_max = Ops241_rx_float
                start_time = time()
                current_time = start_time
    else :
        current_time = time()
        delta_time = current_time - start_time
        # Reset speed limit to zero if no motion detected after reset_speed_time
        if delta_time > reset_speed_time :
            print('no speed detected, resetting to 0')
            showNumber(0)
            start_time = time()
            current_time = start_time
# Post game cleanup                          
GPIO.cleanup()
