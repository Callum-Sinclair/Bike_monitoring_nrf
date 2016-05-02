# Force

This module measures the force applied to one pedal

This uses a presure sensing resistor, connected via an op-amp, to the chip's ADC

The ADC also measures the battery level

This module connects to the cadence module, which sends battery and power
information to the hub.

The cadence sensor using the force and cadence values approximates
the input power, this is then communicated to the hub

The packet types used are:
Power,    RSC mearurement - inst speed
Battery,  RSC measruememt - inst cadence
