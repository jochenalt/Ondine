
Kinematics is about computation of the tool-centre-point (*TCP*) out of joint angles and vice versa. First is simple, latter is more tricky, but lets see later on.
But before starting any kinematics, it is necessary to define all coordinate systems.

<img width="400px" src="../images/image027.png"/>

The most important design decision is to let the three upper axis’ intersect in one point, the so-call wrist-center-point (*WCP*). This decision makes the computation of the inverse kinematic solvable without numeric approaches.

The picture shows the used coordinate systems in the default position of the bot, having all angles at 0°, starting from the base (angle<sub>0</sub>) and ending with the coordinate system of the hand (angle<sub>6</sub>). For convenience the forearm (angle<sub>1</sub>) adds +90° to the real angle in order to have the base position at 0° of the bot, although the illustrated actually is -90°. The coordinate systems have been are arranged according to the Denavit Hardenberg convention, which is:


The transformation from angle<sub>i</sub> to angle<sub>i+1</sub> is given via 

1. rotation around the x-axis by α

2. translation along the x-axis by α

3. translation along the z-axis by *d*, and

4. rotation around the z-axis by θ

So, the Denavit Hardenberg parameters are:

| Joint      | a[°]  | a[mm]            | d[mm]           |
|----------  | ------| ---------------- | --------------- |
| Hip        | *-90°*| *0*              | *d<sub>0</sub>* |
| Upper arm  | *0*   | *a<sub>1</sub>*  | *0*             |
| Forearm    | *-90°*| *0*              | *0*             |
| Elbow      | *90°* | *0*              | *d<sub>3</sub>* |
| Wrist      | *-90°*| *0*              | *0*             |
| Hand       | *0*   | *0*              | *d<sub>5</sub>* |

The general definition of a Denavit Hardenberg (DH) transformation is

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="../images/image029.png"/>

which is a homogeneous matrix with two rotations *(x,z)* and two translations *(x,z)*.

Combined with the DH parameters, the following DH matrixes define the transformation from one joint to its successor:

<img  align="left" src="../images/image030.png"/>

<img  align="left" src="../images/image031.png"/>

<img   src="../images/image032.png"/>

<img  align="left" src="../images/image033.png"/>

<img  align="left" src="../images/image034.png"/>

<img src="../images/image035.png"/>

## Forward Kinematics

With the DH transformation matrixes at hand, computation of the bot’s pose (i.e the position and orientation of the gripper) out of the joint angles is straight forward. The matrix representing the gripper’s pose <img align="center"  src="../images/image036.png"/> is 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image037.png"/> 

By multiplying the transformation matrix with the origin (as homogeneous vector), we get the absolute coordinates of the tool centre point in world coordinate system (i.e. relative to the bot’s base).

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="../images/image038.png"/>

The orientation in terms of roll/nick/yaw of the tool centre point can be derived out of <img align="center"  src="../images/image036.png"/>by taking the part representing the rotation matrix (<img align="center"  src="../images/image040.png"/>). ([Wikipedia Roll/Nick/Yaw](https://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel#Berechnung_aus_Rotationsmatrix) )

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image041.png"/>

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image042.png"/>  
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image043.png"/>  
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image044.png"/>  

For <img align="center" src="../images/image045.png"/> we have a singularity (*atan2* never becomes this), but wikipedia has a solution for that as well
	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="../images/image046.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image047.png"/>

if <img align="center" src="../images/image048.png"/> we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="../images/image046.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image049.png"/>

Note: Unfortunately, the gripper’s coordinate system is not appropriate for human interaction, since the default position as illustrated in the [Coordinate Systems](images/image027.png) is not nick/roll/yaw=(0,0,0). So, in the Trajectory Visualizer it is handy to rotate the gripper matrix such that the default position becomes (0,0,0). The according rotation matrix represents a rotation of -90° along *x*,*y*, and *z*, done by the rotation matrix

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image051.png"/>

In the following equations, this is not considered, since it is for convenience in the UI only, so you will find that additional rotation in the source code only. 

## Inverse Kinematics 
Inverse kinematics denotes the computation of all joint angles out of the tool-centre-point’s position and orientation. In general this is hard, and giving a non iterative solution for a 6DOF robot is only feasable, when computation of the grippers position and the grippers orientation can be considered separately, i.e. the angles of the lower three actuators is not depending on the orientation of the gripper. Still, I do not like numerical solutions, even though with todays processors (or FPGAs) this is no more a question of computational power. I just think that a numerical solution is not a real solution but a surrender to complexity. That's why I let the upper three joint angles intersect in the WCP, which is a basic assumption of the following. 

Input of inverse kinematics is the TCP’s position and orientation in terms of roll, nick, yaw, abbreviated by *γ*, *β*,and *α*.

<img align="left" src="../images/image053.png"/>

<img align="center" src="../images/image054.png"/>

First, we need to compute the wrist-centre-point out the tool-centre-point. This is possible by taking the TCP and moving it back along the TCP’s orientation by the hand length. For doing so, we need the transformation matrix from the base to the last joint <img align="center" src="../images/image036.png"/> 
which we can derive out of the TCP’s position and orientation.

To build the transformation matrix <img align="center" src="../images/image036.png"/> we need the rotation matrix defining the orientation of the TCP. This is given by multiplying the rotation matrixes for all axis (*γ*, *β*, *α*) which gives <img align="center" src="../images/image056.png"/> (see also *[computation of rotation matrix out of Euler Angles](http://kos.informatik.uni-osnabrueck.de/download/diplom/node26.html)*).

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image057.png"/>

Now we can denote the transformation matrix of the TCP by building a homogenous matrix out of *TCP<sub>orientation</sub>* and *TCP<sub>position</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image058.png"/>

From the TCP’s perspective, WCP is just translated by *d<sub>5</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image059.png"/>

Furthermore, <img align="center" src="../images/image060.png"/>, so we get the WCP by

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image061.png"/>

in world coordinates.

Having a top view on the robot shows how to compute the first angle *θ<sub>0</sub>*:

<div align="center"><img width="600px" src="../images/image062.png"/></div>

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image064.png"/>

Actually, this angle exists in two variants: if the bot looks backwards, we get the formula above. But another valid solution is looking backward when we get 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image065.png"/>

Thanks to the design having a wrist-centre-point where the axes of the three upper actuators intersect, the next two angles can be computed by the triangle denoted in orange:

<div align="center"><img align="center" width="400px" src="../images/image027.png"/></div>

Again, there are two solutions (aka configurations), one configuration corresponds with a natural pose of the elbow, solution II is a rather unhealthy position:

<div align="center"><img width="600px" src="../images/image069.png"/></div>

*a* and *b* is given by the length of the actuators *a<sub>1</sub>* und *d<sub>3</sub>*. So, with cosine law we get the angles *α* and *γ*.

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image072.png"/>

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image073.png"/>
	
Finally, we get

<img align="left" src="../images/image074.png"/>
<img align="center" src="../images/image075.png"/>	

and the second solution 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="../images/image076.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image077.png"/>	
	

The upper angles *θ<sub>4</sub>*, *θ<sub>5</sub>*, *θ<sub>5</sub>* can be obtained by considering the chain of transformation matrixes. With

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image078.png"/>	

we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image079.png"/>	

To ease the annoying multiplication <img align="center" src="../images/image080.png"/> we only need to consider the rotation part of the homogenous matrixes, translation is no more relevant, since the orientation alone defines the three upper angles. 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image081.png"/>	

<img align="center" src="../images/image036.png"/> - and therefore the rotation part <img align="center" src="../images/image040.png"/> - is already known resp. can be obtained out of the given angles *θ<sub>0</sub>*, *θ<sub>1</sub>*, *θ<sub>2</sub>* by
	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image086.png"/>	

By equalizing the previous two equations we get a bunch of equations defining the upper angles, still these equations are hard to solve, due to the complex combination of trignometric functions. But, there are some equations which are simpler and can be solved for an angle. First angle that seems to be easily computable is *θ<sub>4</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image088.png"/>	

gives two solutions

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image089.png"/>	

For *θ<sub>3</sub>* there is no easy matrix element, but we can combine

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image090.png"/>	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image091.png"/>		

to

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image092.png"/>		

which ends up in

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image093.png"/>		

again having two solutions depending on *θ<sub>4</sub>*. Same is done on *θ<sub>5</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image094.png"/>		


If *θ<sub>4</sub>=0*, we have an infinite number of solutions *θ<sub>3</sub>* and *θ<sub>5</sub>* (gimbal lock). In that case, we consider  <img align="center" src="../images/image095.png"/> :		

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image096.png"/>.		

Since we know the trigonometric addition theorem from school

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image097.png"/>		

we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="../images/image098.png"/>		

We are free to choose *θ<sub>3</sub>* and arbitrarily select the bot’s current angle *θ<sub>3</sub>*, such that this one will not move in that specific case.

&nbsp;&nbsp;&nbsp;&nbsp;<img src="../images/image101.png"/>		
 
In the end, we get eight solutions by combining the possible pose configurations of  *θ<sub>0</sub>*(forward/backward), *θ<sub>1</sub>* and *θ<sub>2</sub>*(triangle flip), and *θ<sub>4</sub>*(hand orientation turn).

The correct solution is chosen by taking the one that differs the least from the current bot’s joint angles. 
What is with all the other equations from <img align="center" src="../images/image040.png"/>? We could use them to invalidate some of the four solutions, but still there will remain a list of solutions we need to choose from. So, it does not really make sense to take this route.

The selection algorithm is quite simple:

1. consider only mechanically valid solutions, i.e. omit those that violate mechanical boundaries.

2. Compute an angle-wise "*distance*" to the current position and take the solution with the least distance, i.e. take the solution with the least movement.

The latter has the consequence that the pose will try to remain in a configuration and no sudden movements like a turn of 180° happens.

All this is implemented in [Kinematics.cpp](https://github.com/jochenalt/Walter/blob/master/code/WalterKinematics/src/Kinematics.cpp). But - as usual - trying out before implementing this is a good idea, so I did that in this [spreadsheet](https://github.com/jochenalt/Walter/blob/master/theory/Kinematik.xlsx).

Speaking of configurations: if you want to change the configuration of the bot, e.g. from elbow down to elbow up which includes turning the base by 180° the approach of having a linear movement from one pose to the other and interplating in between does not work anymore, since the start and end poses are identical, but with a different configuration.

<img width="500px" src="../images/different configurations.png"/>

First possibility to change configuration is to have a non-linear movement that interpolates angle-wise resulting in a weired movement that would be really dangerous in a real world:

<img width="500px" src="../videos/angle-wise configuration change.gif">

So, you rather introduce an intermediate pose where you can safely turn the critical actuator (mostly the shoulder). This is not less expansing, but a more controlled way:

<img width="500px" src="../videos/singularity configuration change.gif">


Thing is, that one has to go via a singularity (the upright position) which is normally avoided like hell. One reason is the way how we compute *θ<sub>4</sub>=0*, and a numerical reason that singularities are kind of a black hole, the closer you get, the more you are sucked into imprecisions of floating numbers since you approach poles of the underlying functions. So it is defininitely best to simply avoid changing configurations or getting too close to singularities.
