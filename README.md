# Collaborative-Construction-Robot-Samsung-PRISM
Developed a robotic construction framework where two 4-DOF robotic arms autonomously assembled a 3×3 block grid, demonstrating automation in construction tasks. • Implemented synchronization strategies using Arduino Uno and Raspberry Pi, achieving over 80% block placement accuracy during autonomous 3×3 grid trials.

![Project Demo](Media/demo_assembly.mp4) 

## Project Overview
Developed as part of the Samsung PRISM program (July 2024 – December 2024), this project features a dual-arm robotic system designed to automate construction tasks. The core challenge involved synchronizing two 4-DOF robotic arms to assemble a 3x3 grid with high precision and collision avoidance.

## Technical Stack
* **Controllers:** Arduino Uno , Raspberry Pi 
* **Vision:** Python, OpenCV (Object Detection and Spatial Mapping)
* **Actuators:** 2 x 4-DOF Robotic Arms
* **Communication:** Serial Interface

## Repository Structure
* **/Vision**: Python implementation for real-time block detection and coordinate calculation.
* **/Firmware**: Arduino sketches utilizing a Sequential Logic Engine for high-reliability movement.
* **/Docs**: System architecture diagrams and hardware pin mapping.
* **/Media**: Visual demonstrations and setup images.

## Key Achievements
* **High Precision:** Achieved over 80% placement accuracy across multiple 3x3 grid trials.
* **Hardware Synchronization:** Successfully integrated dual-controller logic to handle real-time motor control and vision feedback.

## Challenges and Solutions
* **Constraint:** The arm movement uses a deterministic (sequential) logic to ensure repeatability in a construction environment.
* **Solution:** Calibrated timing sequences to compensate for motor jitter and environmental variables, ensuring the 3x3 grid remained aligned.

