# STM32 SERVO FOC PMSM with Encoder
I made this inverter to experiment the servo motor FOC algorithm. As you can see in the files I made it with Keil MDK and STM32CUBEMX with STM32F103. This software version is not the final version, I want to add another things like NRF24L01 Communication.
I´m testing it with an old Fanuc 5S/3000 junkyard servo motor (3000RPM 107V 10A 5,9Nm) with 2000ppr incremental encoder.

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
- Encoder Z input to "align" incremental encoder and the software
- Absolute position measurement in degrees
  - TIM2 compare generates interrupt each time the Encoder input counts 2000 pulses up, or down. 


# PHOTOS
## Fanuc motors
![IMG_1908](https://user-images.githubusercontent.com/84080967/120412340-c66de400-c32c-11eb-8e79-f12ca47a14ff.jpg)
![IMG_1909](https://user-images.githubusercontent.com/84080967/120411772-ba355700-c32b-11eb-85e6-382903a11c1a.jpg)
## Rectifier and Inverter
![IMG_1910](https://user-images.githubusercontent.com/84080967/120411776-bbff1a80-c32b-11eb-82f1-4319a9326f66.jpg)
![IMG_1911](https://user-images.githubusercontent.com/84080967/120411778-bd304780-c32b-11eb-9d0f-cb345f32a701.jpg)
## Inverter PCB
![IMG_1912](https://user-images.githubusercontent.com/84080967/120411782-be617480-c32b-11eb-95d7-f94fb46126a3.jpg)
## Inverter PCB + Home Made Stencil
![IMG_1913](https://user-images.githubusercontent.com/84080967/120412382-d8e81d80-c32c-11eb-9fa9-2cdadecbbc90.jpg)



# TODO List
- [ ] Space vector modulation?? maybe not necessary
- [ ] NRF24L01 Communication
- [ ] Speed control loop
- [ ] Position control loop

# Safety instructions
**You have to use an ISOLATED USB or an ISOLATED STLINK to protect you PC of any transient or short circuit.**
I bought in Aliexpress an USB isolation board that works great.
I´m sharing this files for educational purposes.
Remember to use it with care, building a system like it you will deal with high voltage 220AC and it is very dangerous :)

