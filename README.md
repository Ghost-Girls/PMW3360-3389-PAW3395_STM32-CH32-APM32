# STM32F042F6P6-PMW3360DM_HID-Mouse

> software: keil 5
> hardware: 
1. MCU: stm32f042
2. Sensor: pmww3360-dM
3. five buttons with hardware debounce
4. Encoder

M0 core MCU is not enough for mouse sensor, when i turn on the UART debug，polling rate down to 170hz~200hz.
If you wanna find a TSSOP20 package, I recommend CH32V305FBT6, 144Mhz and HS USB 2.0 olny pay for $2；

This is my ACFUN Home page: https://www.acfun.cn/u/7503062

it can be work, but one thing that i don't know why!
the mouse speed is too low, i really hope somebody can tell me why!

the thing is : 
same speed = Intellimouse_3.0 400DPI 125Hz = Atmega32u4 PMW3360 800DPI 1000Hz =STM32F042 PMW3360 3200DPI 1000Hz.
	
you can download full project from: https://pan.baidu.com/s/1XnpNVsO6pANIwLP8uDlTRA?pwd=3378 

part of project code and circut sch/pcb file post on my gayhub: https://github.com/Ghost-Girls/STM32F042-PMW3360_HID-Mouse
