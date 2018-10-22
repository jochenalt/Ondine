
# Electronics

Electronics takes care of the following topics

* <b>Control of the three brushless motors</b>. Brushless motoros require more control and electronics than brushed motors. Since industrial controlers are quite expensive, I did my own based on the little outdated [L6234 three phase motor driver](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&cad=rja&uact=8&ved=2ahUKEwjyubq8yJneAhWQ6qQKHc-_DH4QFjAAegQICRAC&url=https%3A%2F%2Fwww.st.com%2Fresource%2Fen%2Fapplication_note%2Fcd00004062.pdf&usg=AOvVaw3LNxqsRuPNezNOGpHoja4f)
* <b>Running Kinematics and state control</b>. Kinematics and state control need to run with a high frequency and reasonable precision. I went with a [Teensy 3.5](https://www.pjrc.com/store/teensy35.html) ARM controller with 120MHz providing floating point arithmetics in hardware.
* <b>Communicate with the outside world</b>. The bot communicates via a small webserver connected to a local Wifi using the popular [ESP8266](https://www.seeedstudio.com/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) device.

