# Cadence - DEPRICATED - SEE CANDENCE NRF51

This module measures the cyclist's cadence or speed.  The hardware uses a reed switch connected to
a GPIO pin on the nRF51 chip, and a magnet attached to the pedal's crank.

The code uses PPI connections between the GPIO pin and a counter to implement the counting
of revolutions fully in hardware, thereby reducing power consumption.

This module connects to the Hub, and sends battery and cadence information to the hub.

The packet types used are:
Cadence,        CSC measurement - cumulative crank revs
Battery,        CSC measurement - last crank event time
