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