# Kinematics

Kinematics means to compute the speed of each wheel out of the body's movement and vice versa.
I will not bother you with the details of two weekend’s work, but make this story short, the final formula is:

<img width="500" src="../images/kinematics/image001.png" >


which gives you the speed of every omniwheel out of the speed in x/y direction and the angular speed around the z-axis.
The system-specific variables used are

| Symbol        | Description                                                              |       Value |
|-------------- |--------------------------------------------------------------------------|------------ |
| θ             | angle between horizontal plane and the omniwheels directions [rad]       | 45° = π/4   |
| r<sub>w</sub> | radius of the omniwheel in [mm]                                          | 35mm        |
| r<sub>b</sub> | radius of the ball in [mm]                                               | 180mm       |

The kinematic parameters are 

| Symbol          | Description                                                                   |       
|-------------- |---------------------------------------------------------------------------------|
| ω<sub>wi</sub>| angular velocity of the i<sup>th</sup> wheel in [revolutions/second]            |
| v<sub>x</sub> | body speed in x-direction in [mm/s]							                  |
| v<sub>y</sub> | body speed in y-direction in [mm/s]							                  |

In the formula above, <i>R</i> is the rotation matrix of the current tilt of the bot, it is a regular rotation matrix 

<img width="650" src="../images/kinematics/image021.png" >

with <br>
φ<sub>y</sub> tilt angle of the bot in y direction in [rad]<br>	
φ<sub>x</sub> tilt angle of the bot in x direction in [rad]

For forward kinematics we need the formula reversed, which is

<img  width="500" src="../images/kinematics/image027.png" >

This gives you the speed in x and y direction out of the speed of all omniwheels

## Coordinate system

For kinematics, IMU and balance control, this coordinate system as seens from the top is used:

<img  width="150" src="../images/kinematics/coordsystem.png" >

## Implementation

During setup of the bot, we can precompute the so-called construction matrix <i>CM</i>.

<img  width="250" src="../images/kinematics/image031.png" >

During runtime within a continously loop, we need to apply the formula above with the precomputed construction matrix. Luckily, the tilt correction matrix <i>R</i> does not need to be multiplied completely with the construction matrix, since we have the sparse matrix

<img  width="100" src="../images/kinematics/image033.png" >

that reduces the amount of necessary multiplications drastically. Instead of the full matrix multiplication of <i>CM*R</i> with 81 floating point operations,  we can omit the cells where the sparse matrix is zero and would null out the result anyhow. By this, we need 10 out of 81 possible multiplications only.


All this is implemented in [Kinematics.cpp](https://github.com/jochenalt/ondine/tree/master/code/BotController/Kinematics.cpp)


