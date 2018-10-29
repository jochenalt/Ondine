# Drive Chain

Driving Omniwheels requires precise control, particularly when the bot is not moving. During balancing all motors move around the same position with many changes in directon. A regular drive with a geared brushed motor has a certain amount of backlash. Small corrections of the bot's position won't have an impact until the motor overcomes the backlash. Therefore, the controller will always overshoot allowing a smaller gain only. Therefore, it is essential to have a small backlash in the motor/drive combination.

# Torque, Power, and Gearbox Ratio 
I estimated the bot's weight approx. 3kg (=29N). A maximum tilt angle around 15° gives a force of 7N to be delivered by the drive. Ignoring the fact that a wheel's axis is not always perpendicular to the direction, and having wheels with a radius of 35mm results in a torque provided by one wheel of 7N*35mm = 0.025Nm. Motors typically provide 3000 RPM, the Bot's to-be speed should be 1m/s, so a wheel has a maximum speed of 2.4 rev/s. This gives a gearbox ratio of 1:20 max. I choose a gearbox ratio of 1:9. So, the motors torque should be 0.002Nm at 1500 RPM.
When I started looking for motors, I was lucky and made a snip on ebay, so all the computations above were useless. Anyhow, I got these wonderfull brushless [40W Maxon motors](https://www.maxonmotor.com/maxon/view/product/272768) for 5€ each

<img width="250px" src="../images/beltdrive/EC-max-30-BL-40W-2WE-mKabel-Thumbnail.jpg" >

# Failed Gearboxes
The quality of these motors were setting the scene, the gearbox had to be close to this. A gearbox from maxon was no option (starts at 150€), and I had the ambition to build something without backlash. So, I tried a [harmonic drive with timing belts](https://hackaday.io/project/19405-strain-wave-gear-with-timing-belts) based on Simon Merrets ideas, and I had a go with a cycloid drive due to its mechanical beauty. Both suffered from the same deficiency: The parts were 3D-printed with a precision of 0.2mm at best, which is not sufficient to get a proper efficiency. Boths gears were groaning loudly and had a significant inner friction. The motor had to supply a significant amount of power to overcome this friction and started moving with a small jerk.

Next try was a planetary box which turned out to be acceptable. Play was low, required space was ok as well:

<img  width="350px" src="../images/beltdrive/planetary_gear.gif" >

But, after reducing the distance between the gears to minimize the play, the gearbox started to produce an annoying plastic sound even after adding a heringbone gearing, so I dismissed this as well.

# Final Gearbox 
Last and successful try was the most boring gear type: timing belts. Still, the advantages were convincing: Low play by nature, silent, and cheap. Only issue is the space required. But this could be mitigated by a design which outer dimensions is dominated by the wheels only not wasting too much space with hubs or flanges etc:

<img  height="300px" src="../images/beltdrive/Beltdrive1.png" >
<img  height="300px" src="../images/beltdrive/Beltdrive2.png" >

In total the belt drive is embedded into the enclosure like this
<img  height="300px" src="../images/beltdrive/total.png" >

Especially the bearing of the output axis inside the pinion at the motor made this thing compact:

<img  height="300px" src="../images/beltdrive/Beltdrive3.png" >

In the first try the timing belts touched the verge of the pinions resulting in an awful squeaking, until I gave the pinion's teeth a round profile that centred the timing belt in the middle of the teeth.  In order to not have remains of suport material coming from the 3D printing process, I cut the pinion's CAD model by half, and glued it after printing. It can be  seen on the original parts (the fine line in the middle of the pinion)

<img  height="200px" src="../images/beltdrive/IMG_20181013_110952.jpg" >
<img  height="200px" src="../images/beltdrive/IMG_20181013_111134.jpg" >
<img  height="200px" src="../images/beltdrive/IMG_20181013_111159.jpg" >

The big wheel has bushings on both sides of the axis

<img  height="200px" src="../images/beltdrive/IMG_20181013_113617.jpg" >
<img  height="200px" src="../images/beltdrive/IMG_20181013_113650.jpg" >

The output wheel incorporates a mounting hub to connect the wheel and the axis 

<img  height="200px" src="../images/beltdrive/IMG_20180922_095326.jpg" >
<img  height="200px" src="../images/beltdrive/IMG_20180922_095413.jpg" >

Assembly starts with the enclosure and the output wheel. The three threaded rods that look out of the enclosure are hex head screws I glued into the enclosure to prevent them falling into the enclosure.

<img  height="300px" src="../images/beltdrive/IMG_20180922_092706.jpg" >
<img  height="300px" src="../images/beltdrive/IMG_20181013_113918.jpg" >

Then the big wheel and the output timing belt is added (the stains on top of the enclosure is my blood)

<img  height="300px" src="../images/beltdrive/IMG_20181013_114005.jpg" >

The pinion and the motor is assembled like this. 

<img  height="300px" src="../images/beltdrive/IMG_20180922_100241.jpg" >

And in a tricky operation the motor is moved into the enclosure, keeping the second timing belt at the right position

<img  height="300px" src="../images/beltdrive/IMG_20180922_100334.jpg" >

Finally it looks like this (without encoder)

<img  height="300px" src="../images/beltdrive/IMG_20180922_100613.jpg" >

Now, the motor needs to be pushed into the enclosure until the output axis has no axial play anymore. Then, the motor can be screwed in its enclosure.

<img  height="300px" src="../images/beltdrive/IMG_20181013_115437.jpg" >

This is done three times. Then, the drive chains are screwd to the bottom part of the bot 

<img  height="500px" src="../images/beltdrive/IMG_20180922_120050.jpg" >

and it is really moving!

<img  height="500px" src="../images/beltdrive/DrivingWithBall.gif" >



