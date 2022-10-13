# motion_processing
sample code focused on my motion processing from IMUs in embedded systems

Two of the files are just the raw code I used in prototype 3 of my sonification unit. 
This code was intended to be used with a two-sensor system, one sensor affixed to the forearm and one to the upper arm. 
The system would track reaching motions and sonify them based either on their distance from an initial reach, or position within a "reach space". 

The third file pulls out the most relevant motion processing code, with a few more notes explaining how it works. 

All the files include all the math one needs to convert raw gyro readings into quaternions, to track body position through 3D cartesian space. 

The code is intended to track short motions, e.g. reaches under a second or two. It works pretty well for that, usually being less than 1cm off the readings of an opitcal motion tracking system. Obviously, for longer tracking, more work would be needed to handle the gyro drift. 
