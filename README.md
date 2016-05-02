# Bike Performance Monitoring using Android Phone and nRF chips

A complete bike monitoring system including the following modules:
- Speed
- Cadence
- Heart Rate
- Power
- Distance to following vehicle - Ultra sonic ranger (USR)
- Temparature
- Gradient

Each module will be run using an nRF51 chip, interfacing with a hub (nRF52), which will communicate with an andriod smartphone using BLE.

This repo contains all code for the sensors and hub.

This project is being undertaken as part of my Electronics and Electrical Engineering MEng degree at the University of Glasgow, along with 3 others.

# System overview

All modules connect to each other via BLE, with the network looking like this:

Phone   <-BLE->    Hub


Hub     <-I2C->    Temparature, Gradient

Hub     <-ADC->    Heart Rate

Hub     <-BLE->    Speed

Hub     <-BLE->    USR

Hub     <-BLE->    Cadence  <-BLE->   Force


Each module has further details in a readme file in each sub-folder

# BLE packet types used

To simplify the BLE communication, and reduce on-air packages, the following BLE characteristics
are used to communicate between the Hub and Phone.
|Data	        |Data range (all int)	    |min size	|Packaged in|
|Speed	        |0-500 (speed in km/h * 10)	|uint16	|cscs measurement, last wheel event time |
|Total dist	    |0-65000 (m)	            |uint16	|cscs measurement, cumulative wheel revs |
|Speed Bat	    |0-100	                    |uint8	|cscs sensor location |
|Cadence	    |0-200	                    |uint8	|cscs measurement, cumulative crank revs |
|Cadence bat	|0-100	                    |uint8	|cscs measurement, last crank event time |
|Power	        |0-1000	                    |uint16	|rcrs measurement, inst speed |
|Power bat	    |0-100	                    |uint8	|rcrs measurement, inst cadence |
|USR	        |0-250 (meters*10)	        |uint8	|rcrs measurement, inst stride length |
|USR bat	    |0-100	                    |uint8	|rcrs measurement, total distance |
|Gradient	    |0-100 (degrees +50)	    |uint8	|hrs measurment, RR-interval |
|Temp	        |0-120 (degrees +50)	    |uint8	|hrs measurment, energy expended |
|Heart Rate	    |0-250	                    |uint8	|hrs measurment, heart rate measurement value |
|Hub Bat	    |0-100	                    |uint8	|Battery service |
|Device info	|-	                        |-	    |Device Information Service |
