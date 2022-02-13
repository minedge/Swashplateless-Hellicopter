## Swashplateless Hellicopter

#### requirements

+ AS5147 - Magnet Encoder
+ Development Board - NUCLEO-STM32G431KB
+ Main Motor - TAROT 4006 kv 620
+ Main Propeller - 1552 CW (DJI S1000 prop)
+ Tail Motor - HGLRC AEOLUS 1202.5 8000KV
+ Radio Controller - Spektrum (receiver : SPM9645)
+ Hings - https://www.thingiverse.com/thing:5239549



#### original idea

[MODLAB UPENN](http://www.modlabupenn.org/) : https://www.modlabupenn.org/2014/10/23/underactuated-rotor/



### Description

##### Novus_RTOS

> This project was made at STM32CubeIDE.
>
> You can download at [[HERE]](https://www.st.com/en/development-tools/stm32cubeide.html) then you can also open project 
>
> If you wondering about pin number. you can see it at Heli.ioc



##### NUCLEO-G431KB Pinmap.xlsx

> This file contains information about the connection between NUCLEO and other sensors.


##### Before flight

> The program has been developed for CW propeller. so you have to use CW prop at main motor
> The moment direction is determined according to the position of the motor, and the position of the motor is measured based on a blade in which the pitch angle increases when the blade is in the lag position.
> Therefore, power must be applied after the blade is positioned forward.
