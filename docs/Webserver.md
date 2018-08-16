Trajectory Execution is done in a webserver that accepts a trajectory from the [Trajectory Planner](./Trajectory). The sent trajectory is *compiled* already, i.e. all intermediate points with a sample rate of 10Hz have been computed, speed profile is applied and inverse [kinematics](./Kinematics) has been precomputed.

So, the webservers receives a long list of interpolated poses including all actuator angles and all the timing information.
The webserver has the simple job of running the trajectors by sending the interpolated points with the correct timing to the [Cortex](./Cortex). It is implemented with [Mongoose](https://www.cesanta.com). 

In addition to its main task, it provides a webpage for debugging purposes, where direct commands to the cortex can be entered. Since the Cortex does not unterstand poses but expects actuator angles, the webserver is able to run the kinematics as well, especially during startup and teardown, when the Walter will move to its default position.

This webpage is done with [Webix](http://webix.com), a small JS-Framework and implemented in [index.html](https://github.com/jochenalt/Walter/blob/master/code/WalterServer/web_root/index.html). The Webserver can be found [here](https://github.com/jochenalt/Walter/tree/master/code/WalterServer).

<img width="1000" align="center" src="../images/website.png" >


