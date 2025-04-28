# S25-18 - Semi Autonomous Navigation Vehicle Software

**Project Overview**

The Semi-Autonomous Navigation Vehicle is a mobile robotics platform designed to deliver real-time navigation with human intervention capabilities, focusing on low-cost, modular, and user-configurable design. Developed to address the lack of affordable mobile transport solutions for commercial environments, the system features a 3D-printed holonomic drive chassis, custom-designed PCBs, and a wireless user interface via WiFi. The vehicle supports both autonomous line-following and dynamic tuning of parameters during operation to ensure precise, efficient, and adaptable performance across various applications.

**Software Responsibilities**
- Sensor Data Processing: Reads and interprets raw data from QTR-8A reflectance sensors to detect and follow lines for autonomous navigation.
- Motor Control: Translates PID control calculations into PWM signals for the motor drivers to control movement.
- Wireless Communication: Manages two-way data transmission between the microcontroller and the user's control device over a WiFi connection, enabling real-time remote commands and parameter adjustments.
- OLED and Interface: Continuously updates both platforms to display matching information.
- Dynamic Tuning: Allows for real-time adjustment of key control parameters (PID coefficients, motor speed settings, etc.) through the wireless user interface without requiring code redeployment.

**Technology Stack**
- Microcontrollers: Adafruit ESP32 FeatherV2s
- OLED Display: Adafruit 128x64 OLED FeatherWing
- Sensors: QTR-8A Reflectance Sensors
- Programming Language: Arduino IDE (C++)
- Communication: WiFi

**Please visit our Documentation Files for more information!**
