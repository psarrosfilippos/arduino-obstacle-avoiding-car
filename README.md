# Arduino-Based Autonomous Obstacle Avoidance Robot

This repository contains the implementation of an **autonomous ground vehicle** developed using **Arduino UNO**, a **HC-SR04 ultrasonic sensor**, and an **L298N motor driver**.  
The project demonstrates fundamental concepts of **embedded systems**, **robotics**, and **autonomous navigation** through real-time obstacle avoidance.

---

## Project Overview

The goal of this project is to design and implement a simple autonomous robot capable of navigating an environment and avoiding obstacles **without human intervention**.  
Obstacle detection is performed using an ultrasonic distance sensor, while motion control is achieved through a dual H-bridge motor driver.

To improve reliability, the control algorithm includes an **anti-loop mechanism**, reducing the chance of the robot getting stuck in repetitive turning patterns.

---

## Hardware Components

- **Arduino UNO** – main processing unit  
- **L298N Motor Driver** – control of DC motors  
- **HC-SR04 Ultrasonic Sensor** – distance measurement  
- **4 DC Motors with Wheels (4WD configuration)**  
- **Chassis**  
- **Battery Pack**  
- **Jumper wires & breadboard**

---

## Software & Tools

- **Arduino IDE** (C/C++)
- **Tinkercad** – circuit simulation
- **EasyEDA** – electronic schematic design

---

## System Architecture

The robot follows a **differential drive** approach using two motor channels (left and right side).  
The ultrasonic sensor continuously measures the distance in front of the vehicle, and the Arduino processes this data to decide the robot’s motion.

The system operates in a closed control loop:
- sensing,
- decision making,
- actuation.

---

## Control Logic Summary

1. Measure distance using the HC-SR04 sensor.
2. If the distance is greater than a predefined threshold, the robot moves forward.
3. If an obstacle is detected:
   - the robot stops,
   - moves backward briefly,
   - performs a left or right turn.
4. If obstacles are detected repeatedly in a short time, the algorithm modifies the turning behavior to avoid looping.

---

## Key Features

- Real-time obstacle detection  
- Fully autonomous operation  
- Anti-loop behavior to prevent repetitive motion  
- Simple, expandable architecture  

---

## Limitations

- The system uses a single forward-facing ultrasonic sensor, resulting in limited peripheral awareness.
- Obstacles outside the sensor’s detection cone (e.g., sides or protruding parts) may not be detected.
- No environment mapping or global path planning is implemented.

---

## Possible Improvements

- Addition of a **servo motor** to rotate the ultrasonic sensor for wider scanning
- Integration of multiple sensors (ultrasonic or IR) for peripheral detection
- Basic environment mapping or memory-based navigation
- Upgrade to more advanced microcontrollers or sensor fusion techniques

---

## Relation to Autonomous Drones

Although this project focuses on a ground vehicle, the core principles—**sensor-based perception**, **real-time decision making**, and **closed-loop control**—are directly applicable to autonomous aerial systems such as drones.

This project serves as an introductory platform for understanding obstacle avoidance mechanisms used in more advanced unmanned systems.

---

## Repository Structure
src/ -> Arduino source code

simulation/ -> Tinkercad screenshots

schematics/ -> Circuit diagrams

report/ -> Project report (PDF)

---


---

## Author

**Psarros Filippos**  
MSc Student – Unmanned Autonomous and Remote Controlled Systems
Athens, 2026

---

## License

This project is provided for educational purposes.

