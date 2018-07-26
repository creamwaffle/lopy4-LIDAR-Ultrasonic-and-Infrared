###Main script that run Multithreading sensor reading and sending with ultra low power code
###author: ICCCCP

import time
import machine
from machine import Pin
from machine import Timer
from network import LoRa
import socket
import binascii
import struct
import _thread
import pycom

#setting pin10 to input mode
pinAlert = Pin('P10', mode=Pin.IN, pull= Pin.PULL_DOWN)

#turn of the led
pycom.heartbeat(False)

#turn on green led
pycom.rgbled(0x00FF00)  # Green

#setting pin8 to output mode
pinDrive = Pin('P8', mode=Pin.OUT)

#pin8 doest hold the value when sleep
pinAlert.hold(False)

#set pin8 to high
pinDrive.value(1)

#creating thread lock
a_lock = _thread.allocate_lock()

#initial distance as 0
dist = 0
old_dist = 0


#function to read ultrasonic sensor value
def reading():
    global dist
    global old_dist
    adc = machine.ADC()             # create an ADC object
    apin = adc.channel(pin='P16',attn=adc.ATTN_6DB)   # create an analog pin on P16
    while True:
        val = apin()                    # read an analog value
        time.sleep(0.1)
        a_lock.acquire()
        dist = (val + 253.09) / 7.56    #ultrasonic input to distance conversion

        #noise rejection
        if (dist > 128):
            dist = old_dist
        else:
            old_dist = dist
        a_lock.release()

#lorawan setup and send data
def sending():
    time.sleep(5)

    global dist
    #Initialize LoRa in LORAWAN mode.
    #Initialize LoRa node
    lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.AS923)

    # create an ABP authentication params
    dev_addr = struct.unpack(">l", binascii.unhexlify('9BF6D450'))[0] #70B3D5499BF6D450
    nwk_swkey = binascii.unhexlify('2B7E151628AED2A6ABF7158809CF4F3C')
    app_swkey = binascii.unhexlify('2B7E151628AED2A6ABF7158809CF4F3C')

    # join a network using ABP (Activation By Personalization)
    lora.join(activation=LoRa.ABP, auth=(dev_addr, nwk_swkey, app_swkey))

    # create a LoRa socket
    s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

    # set the LoRaWAN data rate
    s.setsockopt(socket.SOL_LORA, socket.SO_DR, 2)
    s.setsockopt(socket.SOL_LORA, socket.SO_CONFIRMED, False)
    s.settimeout(3)

    # make the socket blocking
    # (waits for the data to be sent and for the 2 receive windows to expire)
    #set it to run in a loop
    while True:
        #only send the last package
        if(pinAlert() == 1):

            #set the socket blocking equal to true
            s.setblocking(True)

            #lock the thread
            a_lock.acquire()
            dist = int(dist)
            # send some data
            s.send(bytes([dist]))
            a_lock.release() #release the thread

            # make the socket non-blocking
            # (because if there's no data received it will block forever...)
            s.setblocking(False)
            # get any data received (if any...)
            data = s.recv(64)

            # send some data(weirdly have to send data twice otherwise the gateway does not receive uplink package sometime)
            a_lock.acquire() #lock the thread
            dist = int(dist)
            s.setblocking(True)
            s.send(bytes([dist])) #send data again
            a_lock.release() #release the thread
            s.setblocking(False)

            #turn off the sensor
            pinDrive.value(0)
            #send the controller to deep sleep
            machine.deepsleep()

#starting new thread
_thread.start_new_thread(reading, ())
_thread.start_new_thread(sending, ())
