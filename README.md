# STM32-washing-machine-GRAFCET

In this project, sleep mode is used to reduce power consumption and operates only when an interrupt occurs. The state of the system is controlled by a GRAFCET, which depends on the current state and the inputs (sensors) or timer interrupts. The system includes a door lock, water level control, heater, washing cycle, and drain.

The washing cycle consists of three sequential modes: first, right rotation of the motor, followed by left rotation, and ending with fast rotation. Each of these modes has a specific duration, which can be controlled via a UART interface.



