# IRSpectroscope
Embedded code for MEMS-FPI spectroscope

This code was developped as part of my final project for my Engineering techologies studies. This version of the code is at the physical debugging step. 
The goal of this 8 weeks project was to use the MEMS-FPI to recognise and sort different types of plastics using an embedded AI. In this project, I designed the 
electronic circuit, the AI training and application, a LabView interface to gather data for the AI training and the embedded C++ firmware of the spectroscope.


Functionnalities of the firmware:
- Control of the variable booster (24 to 48V) for the tunable Filter of the MEMS-FPI
- Automatic step calculator for the number of datapoint needed
- Control of the light source
- Data gathering  and normalization of the photodiode for the spectrum analysis
- Use of embedded AI (tensorflow lite) for recognition of plastics
- print of various data over serial communication port (reference spectrum, item spectrum, normalized spectrum)
