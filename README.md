# STM32 SERVO FOC PMSM with Encoder
I made this inverter to experiment the servo motor FOC algorithm. As you can see in the files I made it with Keil MDK and STM32CUBEMX with STM32F103. This software version is not the final version, I want to add another things like NRF24L01 Communication.
I´m testing it with an old Fanuc 5S/3000 junkyard servo motor (3000RPM 107V 10A 5,9Nm).

# Features
- **STM32F103** ST 32 bit microcontroller
- **STK621-061** 30A Three Phase hybrid power inverter with predriver, overcurrent and overtemperature protection
- **ACS712** Current measurement / Motor Phase U and V current
- **Encoder** A-/A B-/B Z-/Z input
- **VBus** Measurement

# Working until now
- Center Aligned PWM TIMER 1
- Iq and Id PID current control
- My Encoder have 2000 cpr, that means 8000 rising and falling edges in TIMER 2 Encoder Input
- The motor is running fw and bw depending on Iq Set point, positive and negative POT variable values changed in debug mode (Simulating a potenciometer)


# TODO List
- Space vector modulation
- NRF24L01 Communication

# Safety instructions
**You have to use an ISOLATED USB or an ISOLATED STLINK to protect you PC of any transient or short circuit.**
I bought in Aliexpress an USB isolation board that works great.
I´m sharing this files for educational purposes.
Remember to use it with care, building a system like it you will deal with high voltage 220AC and it is very dangerous :)

