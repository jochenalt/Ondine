
# Electronics

Electronics takes care of the following topics

* <b>Control of the three brushless motors</b>. Brushless motoros require more control and electronics than brushed motors. Since industrial controlers are quite expensive, I did my own based on the [L6234 three phase motor driver](https://www.st.com/content/ccc/resource/technical/document/application_note/78/44/47/d5/a8/63/4a/8e/CD00004062.pdf/files/CD00004062.pdf/jcr:content/translations/en.CD00004062.pdf), which is kind of outdated, but still has nice specs.
* <b>Running Kinematics and state control</b>. Kinematics and state control need to run with a high frequency and reasonable precision. I went with a [Teensy 3.5](https://www.pjrc.com/store/teensy35.html) ARM controller with 120MHz providing floating point arithmetics in hardware.
* <b>Communicate with the outside world</b>. The bot communicates via a small webserver connected to a local Wifi using the popular [ESP8266](https://www.seeedstudio.com/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) device.

# Control of a Brushless Motor

A brushless motor is driven by a magnetic field that turns 90° in advance of the motor's rotor. The diagram shows how this works in general.

<img width="250" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/bldc motor animation.gif"/>

But, the magnetic field turns in jumps of 120° which leads to high vibrations and an unevent rotation especially at  low speed. To improve that, the magnetic field has to turn smoothly, which we can achieve by applying three sin waves modelled by PWM to each stator resulting in a slowly turning magnetic field.

<img width="400" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/BLDC Chopper.png"/>

To improve the ripple of the movement, space vector PWM (SVPM) can be used instead of PWM modulating sine waves. This is kind of electronic engineering black magic, an introduction can be found [here](https://www.switchcraft.org/learning/2017/3/15/space-vector-pwm-intro), frankly I did not spend the effort to dig into it, since it is so easy to implement and gains you a smoother movement.

<img width="400" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/SPWM vs SVPWM.png"/>

Computation is done by 
<img height="50" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/image001.png"/>
<img height="50" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/image003.png"/>

The motor driver L6234 has three PWM inputs, which need to be fed with svpm(t), svpwm (t + 120°), and svpwm(t + 240°). 

The initial position of the rotor needs to be considered, in order to drive the magnetic field 90° ahead of the rotor's positon. For this purpose, I use an optical encoder with a special startup procudure to identify the initial angle of the rotor (later on, it came to my mind that an absolute magnetic encoder would be more practical. It would have released me from executing this procedure. But it was too late).

<img height="50" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/BLDC motor initialization.png"/>

After the initialization procedure we know the current angle of the rotor which we can use later on to have the magnetic field always 90° ahead of the rotor. When we switch on the motor, we start with a very little amount of torque. We slowly increase the torque until the encoder identifies a movement. This happens after approx 0.3° (I have 1024 CPR encoders). The direction of the movement indicates the direction of the rotor's position relatively to the magnetic field. Now the magnetic field is turned torwards the rotor until the rotor has reached its original position. We slightly increase the torque and repeat that loop until the torque is at its max. At the end of this, the magnetic field should be fully aligned with the rotor. It takes approx. 1s and can be recognized with a small short osccillation at the wheel.

All this is implemented in [BrushlessMotorDriver.cpp](https://github.com/jochenalt/Ondine/blob/master/code/BotController/BrushlessMotorDriver.cpp).

# Communication

The device should communicate with the outside world. A fully fledged SOC board (Raspberry or ODroid) seemed to be an overkill, so I used a small web module that became famous a couple of years ago. The [ESP8266](https://www.seeedstudio.com/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) can be programmed in C++ in an Eclipse IDE and already has a library implementing a webserver. Those weired scripting languages that always come with a device like that (in this case "LUA"), is nothing I wanted to spend time with.

<img width="200" src="https://statics3.seeedstudio.com/seeed/img/2017-03/QluwTVU7FQIvaC8dZy6x2JaM.jpg"/>

# Microcontroller

The microcontroller should run the state control to make the bot balance and move, it has to control the motor drivers which uses up some capacity, and it has to receive commands coming from the webserver module. An Avr uC would be too much of a tuning pain, a fully fledged Linux board would be overengineered, so I went with my favrourite controller, a [Teensy 3.5](https://www.pjrc.com/store/teensy35.html) that has a 32bit-Arm M4 controller with enough memory, 120MHz, 5V compatibility, and a 32-bit FPU which comes in handy for kinematics and state control.

<img width="300" src="https://www.pjrc.com/store/teensy35.jpg"/>
(click to enlarge)

# Schematics 

[<img width="1000" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/schematics.png"/>](https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/schematics.png)


The motors pull 2A max, so I choose the [L6234](https://www.st.com/content/ccc/resource/technical/document/application_note/78/44/47/d5/a8/63/4a/8e/CD00004062.pdf/files/CD00004062.pdf/jcr:content/translations/en.CD00004062.pdf) in order to not have 18 MOSFETs to be soldered. Luckily, there's a nice breakout from [Drotek](https://drotek.com/shop/en/home/212-brushless-gimbal-controller-l6234.html) that is very convinient to use.

[<img height="200" src="https://drotek.com/shop/505-large_default/brushless-gimbal-controller-l6234.jpg"/>](https://drotek.com/shop/505-large_default/brushless-gimbal-controller-l6234.jpg)
(click to enlare)


<img width="1000" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/board.png"/>

