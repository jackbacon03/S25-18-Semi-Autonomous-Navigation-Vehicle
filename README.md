# S25-18 - Semi Autonomous Navigation Vehicle Software

The software for the Semi-Autonomous Navigation Vehicle is responsible for integrating and managing all aspects of the robot’s operation. The primary purpose of the code is to process the inputs from the robot’s various subsystems such as the sensors and user commands via Bluetooth and output action commands to the motors. The software enables the vehicle to navigate its environment autonomously while allowing real-time adjustments from the user, ensuring smooth, reliable, and efficient performance. 

Software Components: 
- Sensor Data Processing: Interprets data from the QTR-8RC reflectance sensors to detect and follow a line. 
- Motor Control: Converts processed commands from either the user or sensor readings into physical movement by sending signals to the motor drivers.  
- Bluetooth Communication: Handles the two-way data exchange between the user’s remote control and the microprocessor. 
- General Logic Control: Coordinates the behavior of all subsystems, manages operational states, and ensures that real-time decisions are executed efficiently.  
- Error Handling: Detects and handles and failures or unexpected inputs or actions to ensure smooth operation of robot.  

The software is the core of the vehicle’s functionality. It ensures that the subsystems all work together in harmony to achieve the desired autonomous and semi-autonomous functionality. It is designed to be modular, scalable, and responsive to ensure adaptability for different environments and applications. 
