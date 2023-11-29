## you need to open with RAW button!

# PMW3360-3389-PAW3395_STM32-CH32-APM32
This is my ACFUN Home page: https://www.acfun.cn/u/7503062


> software: keil 5
> hardware: 
1. MCU: STM32(M0/M3)/CH32(Risc-v)/APM32(M4)
2. Sensor: PMW3360-DM\PMW3389-DM\PAW3395
3. five buttons with hardware debounce
4. Encoder

Mouse Timing Diagram:
|SOF|<------------------------------(last frame)-------------------------------------------->|SOF|<------------------------------(next frame)--------------------------------------------->|SOF|
---->|timer satart|<------------(about 77us)------->|timer stop|-----------------------------|--->|timer satart|<------------(about 77us)------->|timer stop|------------------------------>
----------------->|USB send data|------------------------------|-----------------------------|---------------->|USB send data|------------------------------|------------------------------>
-------------------------------------------------------------->|get SPI data|----------------|------------------------------------------------------------->|get SPI data|----------------->
--------------------------------------------------------------------------->|key/encoder scan|--------------------------------------------------------------------------->|key/encoder scan|

1. SOF number change, mean this a new USB frame, and then start timer stick about 77us, couse the PMW3360 SPI clock is 2Mbps, PAW3395 might be count stick about 10us, HAL lib might have huge send interval!
2. Get datas at last frame end, USB send datas at current frame begin.


M0 core MCU is not enough for mouse sensor!
If you wanna find a TSSOP20 package, I recommend CH32V305FBT6, 144Mhz and HS USB 2.0 olny pay for $2；

the progam just a demo, you need to modify it!
you can download full project from: [https://pan.baidu.com/s/1XnpNVsO6pANIwLP8uDlTRA?pwd=3378 ](https://pan.baidu.com/s/118rVzi9ttPwQ6cbSj48kbg?pwd=3378 )

part of project code post on my gayhub: https://github.com/Ghost-Girls/
