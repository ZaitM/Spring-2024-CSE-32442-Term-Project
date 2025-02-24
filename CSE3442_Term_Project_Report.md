# CSE 3442 Spring 2024 Term Project: Obstacle-Avoiding Miniature Robot

## 1. Introduction
This term project involved designing and building an autonomous surveillance robot using the EK-TM4C123GXL embedded platform providing various modules for functionality, including UART, timers, PWM, and GPIO, with physical components provided by the course instructor.

## 2. Theory of Operation
The EK-TM4C123GXL platform supported key modules:
- **UART**: Enabled serial communication for data transmission and reception.
- **Timers and PWM**: Controlled motor speed and timing for precise robot movement.
- **GPIO**: Managed input/output operations, including IR LED detection via phototransistors.
- **IR LEDs and Phototransistors**: Implemented odometry measurements for navigation.
- **Ultrasonic Sensor**: Facilitated autonomous navigation by detecting obstacles.
- **IR Receiver**: Decoded NEC protocol signals from a remote control for manual operation.
- **DRV8833 Module**: Drove brushed DC motors for robot locomotion.

## 3. Implementation and Observations
- **UART Interface**: A continuous UART interface in the main loop sent and received serial data. I used the `kbhitUart0()` to check the receive buffer status, integrated with global variables like `bool autonav` set by the IR receiver’s interrupt service routine (ISR).
- **Speed Measurement**: To determine motor speeds (mm/min) for PWM values (675–1023), I configured a timer to measure the time for one wheel rotation. Speed was calculated as distance (wheel circumference) divided by time. For turns, I calculated arc length (radius × angle) and divided it by maximum speed to estimate turn time.
- **IR Receiver**: Implemented a state machine for NEC protocol decoding, using edge-time capture mode on timers. An oscilloscope analyzed the waveform: a 9 ms low preamble followed by a 4.5 ms high, with the first four bytes representing the address and its inverse, and the last four bytes for data and its inverse.
- **Ultrasonic Sensor**: Measured distances using two methods: (1) polling the GPIO data register for ECHO signal transitions (high to low), or (2) edge-time capture with a wide timer, similar to the IR receiver.
- **IR LED Detection**: Configured GPIO to detect rising and falling edges from phototransistors, ensuring accurate odometry.
- **Interrupts and Challenges**: Managed multiple interrupts (e.g., UART, timers, GPIO) to avoid ISR saturation. Blocking functions helped monitor register status.
- **Challengers** A key challenge was ensuring robust soldering connections, requiring careful technique.

## 4. Conclusion
The project successfully introduced me to embedded development, providing practical experience with timing considerations, datasheet analysis, and module functionality. It sharpened my skills in microcontroller programming, hardware integration, and problem-solving, and I look forward to applying these insights to future projects.
