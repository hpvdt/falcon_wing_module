# Wing Module Firmware

This firmware is meant to be multipurpose, allowing for all nodes to be flashed with the same firmware and then configured in the field over CAN as needed.

## Known Issues

Enabling the I2C bus causes TIM3 CH2 PWM output to be overriden by the SMBA signal, even if it is not in active use by that I2C peripheral. This seems to be an errata from ST's alternate pin function system, and there is no way the two can coexist on my choice of hardware since as long as that pin is put into alternate function mode, SMBA trumps TIM PWM. The likely work around for the time being is to make separate firmware where I2C is needed and then to simply co-locate two boards if I2C and servo control are needed. Should a future revision be made, we should look to make PB5 a general GPIO (ADD) and then get PWM from a different ADD pin.