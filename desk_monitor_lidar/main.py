###Main script that run Multithreading sensor reading and sending with ultra low power code
###author: ICCCCP

import time
import machine
from machine import I2C
from machine import Pin
from machine import Timer
from network import LoRa
import socket
import binascii
import struct
import _thread
import pycom

#setting pin19 to input mode
pinAlert = Pin('P19', mode=Pin.IN, pull= Pin.PULL_DOWN)

#turn of the led
pycom.heartbeat(False)

#turn on green led
pycom.rgbled(0x00FF00)  # Green

#setting pin8 to output mode
pinDrive = Pin('P8', mode=Pin.OUT)

#pin8 doest hold the value when sleep
pinAlert.hold(False)
pinDrive.hold(False)
#set pin8 to high
pinDrive.value(1)

#creating thread lock
a_lock = _thread.allocate_lock()

#initial I2C
lidarReady = bytearray([0xff])          #  holds the returned data for ready check
readyBuf = bytearray([0x01])            #  step 2 address for readiness check
distance = 0                           #  variable for distance reading
dist = 0

#function to read ultrasonic sensor value
def reading():
    global dist
    global distance
    global old_dist
    global lidarReady
    global readyBuf
    i2c = I2C(0, I2C.MASTER, baudrate=100000, pins=("P9","P10"))

    while True:
        #write 0x04 to register 0x00
        i2c.writeto_mem(0x62, 0x00, 0x04) # write 0x04 to slave 0x62, slave memory 0x00

        #  Step 2 Read register 0x01 and wait for bit 0 to go low
        while (lidarReady[0] & readyBuf[0]):
             i2c.writeto(0x62, 0x01)
             lidarReady=i2c.readfrom(0x62,1)
             time.sleep(0.05)               #  This seems to help reduce errors on the I2C bus

        lidarReady=bytearray([0xff])    #  reset the ready check data for next reading

        #  Step 3 Read the distance measurement from 0x8f (0x0f and 0x10)
        i2c.writeto(0x62,0x8f)
        #lock the thread
        a_lock.acquire()
        dist=i2c.readfrom(0x62,2)
        distance=dist[0]
        distance<<=8                    #  move 2 bytes into a 16 bit int
        distance|=dist[1]
        distance = distance - 15


        #noise rejection
        if (distance > 128 or distance < 0):
            distance = old_dist
        else:
            old_dist = distance
        a_lock.release()
        time.sleep(0.1)                 #  allow time between readings, can go faster but more errors

#lorawan setup and send data
def sending():
    time.sleep(5)

    global distance
    #Initialize LoRa in LORAWAN mode.
    #Initialize LoRa node
    lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.AS923)

    # create an ABP authentication params
    dev_addr = struct.unpack(">l", binascii.unhexlify('9CAB57AB'))[0] #70B3D5499BF6D450
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
            #distance = int(distance)
            # send some data
            s.send(bytes([distance]))
            a_lock.release() #release the thread

            # make the socket non-blocking
            # (because if there's no data received it will block forever...)
            s.setblocking(False)
            # get any data received (if any...)
            data = s.recv(64)

            # send some data(weirdly have to send data twice otherwise the gateway does not receive uplink package sometime)
            a_lock.acquire() #lock the thread
            #distance = int(distance)
            s.setblocking(True)
            s.send(bytes([distance])) #send data again
            a_lock.release() #release the thread
            s.setblocking(False)

            #turn off the sensor
            pinDrive.value(0)
            #send the controller to deep sleep
            machine.deepsleep()

#starting new thread
_thread.start_new_thread(reading, ())
_thread.start_new_thread(sending, ())
