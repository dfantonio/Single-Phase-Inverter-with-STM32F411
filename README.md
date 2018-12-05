# Single-Phase-Inverter-with-STM32F411

This is a project intended for the matter of industrial eletronics. The goal is to develop a single phase inverter using SPWM modulation.

It's using the pins PA7 and PA8 to generate the PWM. PA7 is the normal PWM and PA8 is opposite to PA7. This two pins go in the H bridge.

The code also has a feature to change the signal frequency trough serial. You just need to send the new frequency using a baudrate of 115200 and the code will recalculate everything.

**If you used this code, please leave a star**
