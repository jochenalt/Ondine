# Drive Chain

Driving Omniwheels is not as easy as expected in the beginning. First, the drive chain needs to be relatively silent for emotional reasons.
Then, there should be no play or backlash, since during balacing motors continously drive forward and backward with small movements only. 
Out of the weight and acceleration of the bot, the motor's output power should be something around 30W with a gearbox around 1:5 to 1:10. I was lucky with the motors, I made a snip on ebay and got these brushless [40W Maxon motors](https://www.maxonmotor.com/maxon/view/product/272768) for 5€ each

<img width="200px" src="https://www.maxonmotor.com/medias/sys_master/root/8797319790622/EC-max-30-BL-40W-2WE-mKabel-Detail.jpg" >

It is clear, that the quality of these motors were setting the scene, the gearbox features had to be at least close to that. But a gearbox from maxon starts at 150€. So, I tried a [harmonic drive with timing belts](https://hackaday.io/project/19405-strain-wave-gear-with-timing-belts) based on Simon Merrets ideas, and I had a go with a cycloid drive due to its mechanical beauty. Both suffered from the same deficiency: The parts were 3D-printed with a precision of 0.2mm at best, and this is not sufficient to get a proper efficiency. Boths gears were groaning loudly and had a significant inner friction such that the motor must give noticable power to start moving.

Next try was a planetary box which turned out to work acceptable. Play was low, required space was ok as well:

<img  width="350px" src="../images/beltdrive/planetary_gear.gif" >

Unfortunately, when I reduced distance between the gears, the gearbox started to produce an annoying plastic sound, even after adding a heringbone gearing, so I dismissed this as well.

Last and successful try was the most boring gear type with timing belts. But the advantages were convincing: Low play by nature, silent, and cheap. Only issue is the space required. But this could be mitigated by a design which outer dimensions is dominated by the wheels only not wasting too much space with hubs or flanges etc:

<img  height="250px" src="../images/beltdrive/Beltdrive1.png" >
<img  height="250px" src="../images/beltdrive/Beltdrive2.png" >

Especially the bearing of the outer axis inside the pinion at the motor saved a lot of space:
<img  height="250px" src="../images/beltdrive/Beltdrive3.png" >


