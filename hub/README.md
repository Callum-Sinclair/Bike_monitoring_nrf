# Hub

This module connects with the phone (as a peripheral), and sends data from the various
sensors to the phone.

The hub interfaces with sensors detecting:
- Speed
- Cadence
- Heart Rate
- Power
- Distance to following vehicle - Ultra sonic ranger (USR)
- Temparature
- Gradient

The Hub uses an nRF52 chip

The Hub connects to the USR, Speed and Cadence sensors via BLE (the cadence sensor
connects via BLE to the force sensor, and calculates the power). The hub also connnects
via I2C to the temparature and gradient sensors.  The heart rate detector is connected
directly to the ADC on the hub, and code running on the hub detects the pulse rate.

The hub sends all the information to the phone using the following packet types 

| Data	        |Data range (all int)	        | min size	| Packaged in|
| ------------- |------------------------------ | -------   | ---------------|
| Speed	        | 0-500 (wheel rpm)             | uint16	| cscs measurement, last wheel event time |
| Total dist    | 0-65000 (m)	                | uint16	| cscs measurement, cumulative wheel revs |
| Speed Bat	    | 0-100	                        | uint8	    | (cscs measurement, last crank event time) upper byte |
| Cadence	    | 0-200	                        | uint8	    | cscs measurement, cumulative crank revs |
| Cadence bat	| 0-100	                        | uint8	    | (cscs measurement, last crank event time) lower byte |
| Power	        | 0-1000	                    | uint16	| rcrs measurement, inst speed |
| Power bat	    | 0-100	                        | uint8	    | rcrs measurement, inst cadence |
| USR	        | 0-250 (meters*10)	            | uint8	    | rcrs measurement, inst stride length |
| USR bat	    | 0-100	                        | uint8	    | rcrs measurement, total distance |
| Gradient	    | 0-100 (degrees +50)	        | uint8	    | (hrs measurment, RR-interval) upper byte |
| Temp	        | 0-120 (degrees +50)	        | uint8	    | (hrs measurment, RR-interval) lower byte |
| Heart Rate	| 0-250	                        | uint8	    | hrs measurment, heart rate measurement value |
| Hub Bat	    | 0-100	                        | uint8	    | Battery service |
| Device info	| -	                            | -	        | Device Information Service |

The hub recieves data from the modules in the following packet types

| Data	        | Data range (all int)	    | min size	| Packaged in |
| ------------- | ------------------------- | --------- | ----------- |
| Speed	        | 0-500 (wheel rpm)         | uint16	| rcrs measurement, inst speed |
| Total dist	| 0-65000 (m)	            | uint16	| rcrs measurement, total distance |
| Speed Bat	    | 0-100	                    | uint8	    | rcrs measurement, inst cadence |
| Cadence	    | 0-200	                    | uint8	    | rcrs measurement, inst stride length |
| Cadence bat	| 0-100	                    | uint8	    | rcrs measurement, total distance |
| Power	        | 0-1000	                | uint16	| rcrs measurement, inst speed |
| Power bat	    | 0-100	                    | uint8	    | rcrs measurement, inst cadence |
| USR	        | 0-250 (meters*10)	        | uint8	    | hrs measurment, energy expended |
| USR bat	    | 0-100	                    | uint8	    | hrs measurment, heart rate measurement value |
