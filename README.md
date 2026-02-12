# Abstract:
The use of unmanned aerial vehicles (UAVs) is rapidly expanding across search-and-rescue (SAR), defence, and surveillance applications. In these domains, autonomous operations in confined spaces are increasingly required for tasks such as victim detection, structural inspection, and situational awareness for rescuers. However, operation in indoor environments removes access to GPS, creating significant challenges for accurate localization and mapping, especially on small aerial platforms with limited onboard memory and sensing. This project investigates whether a modular nano-quadcopter can perform indoor localization and mapping without GPS under strict size, computational, and energy constraints.
A fully custom autonomous system was developed, integrating optical flow-based velocity estimates, short-range time-of-flight depth ranging, and an onboard Single Board Computer (SBC)- based data-logging pipeline. The system architecture was designed with modularity in mind, allowing sensors or modules to be expanded and reconfigured without the need for disassembly.
Flight and sensor data were recorded and processed to create a post-exploration two-dimensional map of the environment. System performance was repeatedly evaluated through test flights, iterative refinement, and documentation. 
This system demonstrates that short-duration indoor mapping on a nano-scale aerial platform is feasible, while highlighting the challenges of drift and sensor limitations as long-term obstacles for indoor navigation.



# Hardware:    

# Version 1:
Waveshare ESP32-S3 Zero (sensor hub)   
Luckfox Pico Mini B (occupancy grid + mapping + vision)   
MicoAir H743 45A V2 AIO (sensor fusion + optical flow)   
Motors: 1202.5 11500kv   
Frame: 85mm Mobula 8 Whoop Frame   

# Version 2:
Waveshare ESP32-S3 Zero (sensor hub)   
LicheeRV Nano (occupancy grid + mapping + vision)   
MicoAir H743 45A V2 AIO (sensor fusion + optical flow)   
Motors: 1103 11000kv   
Frame: 2" Carbon Fiber   




January 30, 2026 Changes   
ATC_RAT_RLL_P = 0.07   
ATC_RAT_PIT_P = 0.07   

ATC_RAT_RLL_I = 0.06   
ATC_RAT_PIT_I = 0.06   

ATC_RAT_RLL_D = 0.0045   
ATC_RAT_PIT_D = 0.0045   


YAW   
ATC_RAT_YAW_P = 0.05   
ATC_RAT_YAW_I = 0.015   
ATC_RAT_YAW_D = 0.0    


# Notes
Quadcopter was the most stable in hover mode with the Jan 28th code/commit.




