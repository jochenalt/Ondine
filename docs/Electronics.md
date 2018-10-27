
# Electronics

Electronics takes care of the following topics

* <b>Control of the three brushless motors</b>. Brushless motoros require more control and electronics than brushed motors. Since industrial controlers are quite expensive, I did my own based on the [L6234 three phase motor driver](https://www.st.com/content/ccc/resource/technical/document/application_note/78/44/47/d5/a8/63/4a/8e/CD00004062.pdf/files/CD00004062.pdf/jcr:content/translations/en.CD00004062.pdf), which is kind of outdated, but still has nice specs.
* <b>Running Kinematics and state control</b>. Kinematics and state control need to run with a high frequency and reasonable precision. I went with a [Teensy 3.5](https://www.pjrc.com/store/teensy35.html) ARM controller with 120MHz providing floating point arithmetics in hardware.
* <b>Communicate with the outside world</b>. The bot communicates via a small webserver connected to a local Wifi using the popular [ESP8266](https://www.seeedstudio.com/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) device.

# Control of a Brushless Motor

A brushless motor is driven by a magnetic field that turns 90° in advance of the motor's rotor. The diagram shows how this works in general.

<img width="250" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/bldc motor animation.gif"/>

But, the magnetic field turns in jumps of 120° which leads to high vibrations and jerky rotation especially when working at low speed. To improve that, the magnetic field has to turn continuously, which we can achieve by applying three sin waves modulated by PWM to each stator resulting in a smoothly turning magnetic field.

<img width="400" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/BLDC Chopper.png"/>

To improve the ripple of the movement, space vector PWM (SVPM) can be used instead of plain PWM modulating sine waves. This is kind of electrical engineering black magic, an introduction can be found [here](https://www.switchcraft.org/learning/2017/3/15/space-vector-pwm-intro), frankly I did not spend the effort to dig into it, since it is so easy to implement and gains you a smoother movement.

<img width="400" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/SPWM vs SVPWM.png"/>

Computation is done by 

<img height="55" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/image001.png"/>

<img height="30" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/image003.png"/>

The motor driver L6234 has three PWM inputs, which need to be fed with svpm(t), svpwm (t + 120°), and svpwm(t + 240°). 

## Initial Calibration

The initial position of the rotor has to be considered in order to drive the magnetic field 90° ahead of the rotor's positon. For this purpose, I use an optical encoder with a startup procedure identifying the initial angle of the rotor (later on, it came to my mind that an absolute magnetic encoder could do this without the startup procedure. But it was too late).

<img height="200" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/BLDC motor initialization.png"/>
<img height="200" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/startup.gif"/>

After the initialization procedure we know the current angle of the rotor. It is used later on to turn the magnetic field 90° ahead of the rotor. When we switch on the motor, we start with a very little amount of torque. The torque is slowly increased until the encoder recognized a movement. This happens after approx 0.3° (I have 1024 CPR encoders). The direction of the movement indicates the direction of the rotor's position relatively to the magnetic field. Now the magnetic field is turned torwards the rotor until the rotor has reached its starting position. We slightly increase the torque and start again waiting for a small movement which is then compensatd by turning the magnetic field. We repeat that until the magnetic field is perfectly aligned with the rotor while applying maximum torque. 

By this procdure, the magnetic files turns towards the current position of the rotor with increasing verve. It takes approx. 1s and is recognizable by a small and short osccillation when starting up.

<img height="200" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/realstartup.gif"/>


The advance angle of 90 degrees implies an issue when doing position control: If the motor is supposed to stick at one position and there's varying torque, the advance angle jumps from +90° to -90° everytime the control algrithm changes its direction. To avoid that jump, the PID controller's output is smoothed by a sigmoid function:

<img height="40" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/image010.png"/>

<img height="200" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/BLDC controller sigmoid.png"/>


## PID Controller

Speaking of the PID controller of a brushless motor : I played around with a couple of controllertypes, I even implemented one of these fancy fuzzy controllers. Did not work out well, actually I got the impression that fuzzy controllers are kind of an academic hype only. I ended up with a plain gain-scheduled PID controller, i.e. a PID controller that has two sets of PID values: One for balancing, one for maximum speed. The input speed is used to identify the gains by interpolation between these two configuration sets.

All this is implemented in [BrushlessMotorDriver.cpp](https://github.com/jochenalt/Ondine/blob/master/code/BotController/BrushlessMotorDriver.cpp).

# Communication

The device should communicate with the outside world. A fully fledged SOC board (Raspberry or ODroid) seemed to be an overkill, so I used a small web module that became famous a couple of years ago. The [ESP8266](https://www.seeedstudio.com/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) can be programmed in C++ in an Eclipse IDE and already has a library implementing a webserver. Those weired scripting languages that always come with a device like that (in this case "LUA"), is nothing I wanted to spend time with.

<img width="200" src="https://statics3.seeedstudio.com/seeed/img/2017-03/QluwTVU7FQIvaC8dZy6x2JaM.jpg"/>

# Microcontroller

The microcontroller should run the state control to make the bot balance and move, it has to control the motor drivers which uses up some capacity, and it has to receive commands coming from the webserver module. An Avr uC would be too much of a tuning pain, a fully fledged Linux board would be overengineered, so I went with my favrourite controller, a [Teensy 3.5](https://www.pjrc.com/store/teensy35.html) that has a 32bit-Arm M4 controller with enough memory, 120MHz, 5V compatibility, and a 32-bit FPU which comes in handy for kinematics and state control.

<img width="300" src="https://www.pjrc.com/store/teensy35.jpg"/>

# Schematics 

The motors pull 2A max, so I choose the [L6234](https://www.st.com/content/ccc/resource/technical/document/application_note/78/44/47/d5/a8/63/4a/8e/CD00004062.pdf/files/CD00004062.pdf/jcr:content/translations/en.CD00004062.pdf) to avoid soldering 18 MOSFETs. Luckily, there's a nice breakout from [Drotek](https://drotek.com/shop/en/home/212-brushless-gimbal-controller-l6234.html) that is convinient to use.

[<img height="200" src="https://drotek.com/shop/505-large_default/brushless-gimbal-controller-l6234.jpg"/>](https://drotek.com/shop/505-large_default/brushless-gimbal-controller-l6234.jpg)

On the IMU side I went with the cheap and reliable MPU9250 on a drotek breakout
[<img height="200" src="https://drotek.com/shop/2650-large_default/mpu9250-gyro-accelerometer-magnetometer.jpg"/>](https://drotek.com/shop/505-large_default/brushless-gimbal-controller-l6234.jpg)

There's not a big deal in the schematics, pretty standard I would say. There's a power supply with a switching 7805 (D3) (a normal power regulator would have required a heatsink for which I dont have enough space on the PCB). A relay REL1 turns on the power to the motors indiated by LED1, since I wanted to avoid issues during startup when the motors are not yet controlled by the uC but the drivers get power already. The Teensy uC is in the middle, its PWM pins go to the L6234 drivers, which output lines go to the motor sockets. To protect the uC, the PWM lines are connected with a resistor and z-diode limiting the voltage that is induced by the motors and might come back to the uC (I did that not as a precaution but as a learning point after I bricked a uC. The L6234 has the issue that there is an inner connection from the motor output to the incoming pwm pin, if it is not controlled correctly).

The optical encoders are connected to interrupt-able GPIOs of the uC (luckily, all Teensy GPIOs are interruptable). The ESP8266 breakout's D2/D1 is connected to the uC's  I2C-0 port, the logging output coming from the uC via UART5 is fetched by the ESP8266 via its half (only RX) serial port. The IMU MPU9250 is connected to the uC's I2C-1 port. Since the teensy is full of ports, there's no need to cope with the hazzle of multiple slaves on the same I2C bus. 

The S1 dip switch can be set to identify the board. The pololu motor driver U$1 is a used to control the brushed and encoded motor that lifts the enclosure from the ground.  

[<img width="1000" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/schematics.png"/>](https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/schematics.png)
(click to enlarge)

In the end, all parts fit into a standard 100x80 board

[<img width="1000" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/board.png"/>](https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/board.png)
(click to enlare)


Schematics and board in Eagle format is [here](https://github.com/jochenalt/Ondine/blob/master/schematics).