
# Electronics

Electronics takes care of the following topics

* <b>Control of the three brushless motors</b>. Brushless motoros require more control and electronics than brushed motors. Since industrial controlers are quite expensive, I did my own based on the [L6234 three phase motor driver](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&uact=8&ved=2ahUKEwjyubq8yJneAhWQ6qQKHc-_DH4QFjAAegQICRAC&url=https%3A%2F%2Fwww.st.com%2Fresource%2Fen%2Fapplication_note%2Fcd00004062.pdf&usg=AOvVaw3LNxqsRuPNezNOGpHoja4f), which is kind of outdated, but still has nice specs.
* <b>Running Kinematics and state control</b>. Kinematics and state control need to run with a high frequency and reasonable precision. I went with a [Teensy 3.5](https://www.pjrc.com/store/teensy35.html) ARM controller with 120MHz providing floating point arithmetics in hardware.
* <b>Communicate with the outside world</b>. The bot communicates via a small webserver connected to a local Wifi using the popular [ESP8266](https://www.seeedstudio.com/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) device.

# Control of a Brushless Motor


A brushless motor is driven by a magnetic field that turns 90° in advance of the motor's rotor. For that purpose I have optical encoders detecting the rotors position and three half bridges driving the lines with a sine wave that produces the magnetic field. 

The diagram shows how this works in general.

<img width="200" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/bldc motor animation.gif"/>

For basic BLDC driving it is common to use a 6 step sequence

<img width="100" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/BLDC 6 step sequence.png"/>

In real life, this would result in a very jerky rotation, since the magnetic field will turn abruptly in steps of 60° .
To drive a motor smoothly at low speeds, we need to chop that by having a PWM signal on the lines:

<img width="100" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/SPWM.png"/>

In general there are two choices to generate the signal: You can generate traditional sine waves (SPWM) and Space-Vector PWMs (SVPWM).

<img width="100" src="https://raw.githubusercontent.com/jochenalt/Ondine/master/docs/images/electronics/SPWM vs SVPWM.png"/>

I always want to have the most fancy thing, so I went with SVPWM.

