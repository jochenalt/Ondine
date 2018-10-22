
# Electronics

Electronics takes care of the following topics

* <b>Control of the three brushless motors</b>. Brushless motoros require more control and electronics than brushed motors. Since industrial controlers are quite expensive, I did my own based on the [L6234 three phase motor driver](https://www.st.com/content/ccc/resource/technical/document/application_note/78/44/47/d5/a8/63/4a/8e/CD00004062.pdf/files/CD00004062.pdf/jcr:content/translations/en.CD00004062.pdf), which is kind of outdated, but still has nice specs.
* <b>Running Kinematics and state control</b>. Kinematics and state control need to run with a high frequency and reasonable precision. I went with a [Teensy 3.5](https://www.pjrc.com/store/teensy35.html) ARM controller with 120MHz providing floating point arithmetics in hardware.
* <b>Communicate with the outside world</b>. The bot communicates via a small webserver connected to a local Wifi using the popular [ESP8266](https://www.seeedstudio.com/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) device.

# Control of a Brushless Motor

A brushless motor is driven by a magnetic field that turns 90° in advance of the motor's rotor. For that purpose I have optical encoders detecting the rotors position and three half bridges driving the lines with a sine wave that produces the magnetic field. 

The diagram shows how this works in general.

<img width="250" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/bldc motor animation.gif"/>

In real life, this would result in a very jerky rotation, since the magnetic field will turn abruptly in steps of 120°.
So, real controllers use a 6 step sequence. to turn the magnetic field in a smoother way. The left diagram shows this, but the output voltage is still jerky resulting in unsmooth rotation of the motor. So, we need to smooth that by having a PWM signal on the lines like shown on the right hand diagram. In this scenario, enables lines are always high, and the PWM signal controls the voltage on the output lines.

<img width="400" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/BLDC 6 step sequence.png"/>
<img width="400" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/SPWM.png"/>

In general there are two choices to generate the signal: You can generate traditional sine waves (SPWM) (top diagram) and Space-Vector PWMs (SVPWM) (bottom diagram). I always want to have the most fancy thing, so I went with SVPWM.

<img width="400" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/SPWM vs SVPWM.png"/>

# Communication

The device should communicate with the outside world. A fully fledged SOC board (Raspberry or ODroid) seemed to be an overkill, so I used a small web module that became famous a couple of years ago. The [ESP8266](https://www.seeedstudio.com/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) can be programmed in C++ in an Eclipse IDE and already has a library implementing a webserver. Those weired scripting languages that always come with a device like that (in this case "LUA"), is nothing I wanted to spend time with.

<img width="200" src="https://statics3.seeedstudio.com/seeed/img/2017-03/QluwTVU7FQIvaC8dZy6x2JaM.jpg"/>

# Microcontroller

The microcontroller should run the state control to make the bot balance and move, it has to control the motor drivers which uses up some capacity, and it has to receive commands coming from the webserver module. An Avr uC would be too much of a tuning pain, a fully fledged Linux board would be overengineered, so I went with my favrourite controller, a [Teensy 3.5](https://www.pjrc.com/store/teensy35.html) that has a 32bit-Arm M4 controller with enough memory, 120MHz, 5V compatibility, and a 32-bit FPU which comes in handy for kinematics and state control.

<img width="300" src="https://www.pjrc.com/store/teensy35.jpg"/>

# Schematics 

[<img width="1000" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/schematics.png"/>](https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/schematics.png)


The motors pull 2A max, so I choose the [L6234](https://www.st.com/content/ccc/resource/technical/document/application_note/78/44/47/d5/a8/63/4a/8e/CD00004062.pdf/files/CD00004062.pdf/jcr:content/translations/en.CD00004062.pdf) in order to not have 18 MOSFETs to be soldered. Luckily, there's a nice breakout from [Drotek](https://drotek.com/shop/en/home/212-brushless-gimbal-controller-l6234.html) that is very convinient to use.

<img height="200" src="https://drotek.com/shop/505-large_default/brushless-gimbal-controller-l6234.jpg"/>



<img width="1000" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/board.png"/>

