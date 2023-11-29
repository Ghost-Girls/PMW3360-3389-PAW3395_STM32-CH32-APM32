# PMW3360-3389-PAW3395_STM32-CH32-APM32
This is my ACFUN Home page: https://www.acfun.cn/u/7503062


> software: keil 5
> hardware: 
1. MCU: STM32(M0/M3)/CH32(Risc-v)/APM32(M4)
2. Sensor: PMW3360-DM\PMW3389-DM\PAW3395
3. five buttons with hardware debounce
4. Encoder

M0 core MCU is not enough for mouse sensor!
If you wanna find a TSSOP20 package, I recommend CH32V305FBT6, 144Mhz and HS USB 2.0 olny pay for $2；
	
you can download full project from: [https://pan.baidu.com/s/1XnpNVsO6pANIwLP8uDlTRA?pwd=3378 ](https://pan.baidu.com/s/118rVzi9ttPwQ6cbSj48kbg?pwd=3378 )

part of project code post on my gayhub: https://github.com/Ghost-Girls/

******************************************************************************
	sensor communication hava those step:
	1. Mouse surface -> Mouse sensor: mouse frames smothing depend on Sensor-SROM, Pixart prabably know how to change it.
	(the sensor needs firmware to be uploaded to it at startup, we call this SROM)
	the Pixart Sensor FPS might depend on sensor? i dont know!
	the Pixart Sensor scan  by full sensor or part of sensor? i dont know!
		 
	but in this post [https://www.overclock.net/threads/patent-detailing-how-mlt04-works.1579636/], qsxcv said:
	"I believe this is why 3366 and the 9500 family (9500,9800,3988,3989,3310) have dynamic framerate modes. 
	at lower tracking speeds, the sensor runs in a low framerate mode where the full frame is used for calculating correlations. 
	at higher tracking speeds, using such a low framerate would result in worse tracking performance (since consecutive frames don't have as much overlap), 
	so the framerate is increased, but the dsp no longer has the time to calculate correlations from the whole frame, 
	so it only calculates using smaller portions of the frame."
		 
	belive it or not, dynamic framerate is a big problem on old sensor, but dynamic framerate is not too bad as your thinking.
	as far as I know, big dynamic range is really bad thing, just like Logitech G304/G305 dynamic range is 1000~12000 fps.
	obviously, G304/G305 is not a High-End gaming mouse.
	I don't know what different between PAW3395 and PAW3399, PAW3399 might be your best choice?
		 
	2. Mouse sensor -> Mouse MCU/SOC: SPI rate plabaly can not sync to sensor FPS, because PAW3399 still have dynamic framerate. but SPI rate is a Constant rate.
		 SPI rate must be above sensor framerate, but high speed is not necessary.
		 
	3.1. Mouse MCU/SOC (wireless transfer) -> wireless receiver(chipset) ->: encoder and decoder might Increase latency, there is some overhead for wireless too.
		 
	3.2. Mouse MCU/SOC -> USB Port(chipset) -> CPU (not CPU interrupt, is Event traversal): using Mousetest or Mousetester or other software to analyze mouse latency is inaccuracy.
	the software read data before the CPU process, not just from the usb port.
	if you wanna get the accuracy data from USB, "USB protocol analyzer or an instrumented LDAT (i.e. not the screen-capture method)"(from: Razer R&D) is the best.
	(https://www.reddit.com/r/MouseReview/comments/1189q53/introducing_daves_cousin_the_razer_deathadder_v3/j9gdlo1/?context=3). 
******************************************************************************

1) Mouse senosr latency? what is the best?
	
"For top 1000Hz wireless mice, the 1-2ms range is about right, since there is some overhead for wireless too.
Motion sync is a sensor feature, and in no way related to click latency."
		 
"Did you mean 0.0625ms? Because that's the theoretical ideal latency for an 8000Hz mouse.
Our internal measurements clock it in ~0.08-0.09 range. Even 1000Hz mice are in the 0.6ms ballpark."
		 
		 
2) Click latency?
Click latency depend on hardware an software.
softward debounce: increase latency in software, the latency depend on the hardware(switch bouncing). 
hardward debounce: increase latency in hardware (RC debounce circuit) or no latency(RS Latch, only SPDT), the latency depend on the hardware(switch bouncing).
the switch bouncing about 0.5ms~1.5ms, the debouncing latency about 1ms~2ms.
		 
3) how to know frames is sync? 
oscilloscope or USB protocol analyzer!

******************************************************************************
Mouse just a Office consumables.

鼠标只是一个办公耗材！

latency it's just data, not your experience of mouse.

延迟只是个数字，并不能代表你的实际体验！

where shoes are feet, only they know.

鞋子合不合脚只有自己知道！

