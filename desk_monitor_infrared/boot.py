###pycom run this file when booting
###setting up interrupt pin and disable wifi on boot
from machine import Pin
import machine
import pycom

#Setting up Pin mode and Initialize interrupt
pinAlert = Pin('P10', mode=Pin.IN, pull= Pin.PULL_DOWN)
machine.pin_deepsleep_wakeup(['P10'], machine.WAKEUP_ALL_LOW, True)

#Disable Wifi to reduce power consumption
pycom.wifi_on_boot([False])
