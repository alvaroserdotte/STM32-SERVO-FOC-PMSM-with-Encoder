# Stm32-Servo-FOC-PMSM-with-Encoder
I made this inverter to experiment the servo motor FOC algorithm. As you can see in the files I made it with Keil MDK and STM32CUBEMX with STM32F103. This software version is not the final version, I want to add another things like NRF24L01 Communication.
# Features
- **STM32F103** ST 32 bit microcontroller
- **ACS712** Current measurement / Motor Phase A and B current
- VBus Measurement
- STK621-61 30A Three Phase hybrid power inverter with predriver, overcurrent and overtemperature protection
- Iq and Id PID current control


# TODO List
- Space vector modulation
- NRF24L01 Communication

# Be safe
**You have to use an ISOLATED USB or an ISOLATED STLINK to protect you PC of any transient or short circuit.**
I´m sharing it for educational purposes.
Remember to use it with care, we are dealing with high voltage 220AC and it is very dangerous :)
