Remedios The Beauty Avionics Package
=================

Data capturing avionics for the near-space balloon mission series, "Remedios The Beauty"





This is avionics code for a payload for a near-space balloon mission. Using an arduino, and an array of sensors,
it aims to capture and log position, height, temperature, outside colour, orientation, and other measurements
throughout each flight. Location and height data will be transmitted to an control station on Earth via radio transceiver.




10 October: Breadboarded GPS and Barometric systems, and have data displaying at 2hz via the Serial Monitor:

Currently gives this output via the Serial Monitor:

Time: 15:11:10.0
Date: 10/10/2013
Fix: 1 quality: 1
Temperature = 27.12 *C
Pressure = 100164 Pa
Altitude = 97.03 meters
Real altitude = 111.05 meters
Location: 4346.1176N, 1115.4583E
Speed (knots): 0.30
Angle: 174.69
GPS Altitude: 74.70
Satellites: 6


