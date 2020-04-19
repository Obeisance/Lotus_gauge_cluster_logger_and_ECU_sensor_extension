I've added an Arduino Uno, Seeed Studio's CAN shield and Seeed Studio SD shield, 
and a custom prototype board with screw terminals.

This allows me to connect to the CAN lines at the OBD port and send messages at 
1 MBps to the ECU, to the gauge cluster and to read those send by OBD tools and
by the ECU.

I use this setup to read in a thermistor (via a voltage divider on the protoboard) 
that is on the oil pan, read in a Bosch TMAP sensor, and read in and control power 
to an Innovate oxygen sensor. I also send some of that analog input data to the 
ECU via the 0x55 CAN message.

I also log the gauge cluster data and the thermistor data in order to create
histograms of in-use oil temperature for my use case.