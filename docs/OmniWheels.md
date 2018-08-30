# Omniwheels

The omniwheel I was looking for should follow the idea of the [japanese patent 2001-191704](https://astamuse.com/ja/published/JP/No/2001191704) from 2001. For your convinience, [this](https://translate.google.com/translate?hl=en&sl=ja&tl=en&u=https%3A%2F%2Fastamuse.com%2Fja%2Fpublished%2FJP%2FNo%2F2001191704) is the google translation of it.

<img width="250px" src="../images/omniwheel/japanese patent.jpg" >

This patent has a surprising design of the big roll. Its width is much smaller than the small rolls, and it runs with one bearing only. I was not able to find out why the authors chose a design so complex to manufacture. 

<img  width="350px" src="../images/omniwheel/japanese patent big roll.png" >

My [CAD model](https://github.com/jochenalt/Ondine/blob/master/CAD/OmniWheel.iam) needs to be 3d-printed, and the spoke of the patent is so thin, that this would never work. It  carries the bot's full weight, so it requires more material on the spots where the bearings are mounted. So, I designed the spoke with a constant distant to the roll giving the spoke a round surface where it faces the roll.

I ended up with that design, that has an identical width of all rolls and two bearings each. 

<img align="left" width="300px" src="../images/omniwheel/omniwheel CAD total.png"/>
<img width="300px" src="../images/omniwheel/omniwheel CAD cut.png"/>

I was a bit scared that a bot of 3kg could be too heavy for the spoke, so I tried a stress analysis with a target acceleration of 1.0 m/s<sup>2</sup> resulting in a wheel torque of 0.04Nm (roughly). The weak point of the spoke is suprisingly not the marked thin part. The simulated weight of the bot seems to be much more significant than the torque applied.
<img width="600px" src="../images/omniwheel/VonMisesSpannung.png"/>

As a courtesy to the japanese patent, I arranged the 3D-printed parts in beautiful manner:

<img width="600px" src="../images/omniwheel/arrangement total.jpg"/>

The first assembly is necessary to check the tolerances. The slightest amount of warping during printing would would make the rolls wobble against the spoke, so perfect rolls need to be selected in this phase.

<img  width="300px" src="../images/omniwheel/first assembly.jpg"/>

Then I realized, that the friction of ABS rolls on a ball made of rubber or something was quite poor. There's little you can do about the 3d printed material, so I needed some kind of cover. I was thinking of rubber spray, but found out, that sand paper has a perfect friction on rubber. So, I looked for a material similar to sand paper and found corundum, which is sand with sharp edges. Originally it is used for blasting, so it is quite cheap to get. Thing is, that you can it order in quantities of at least 5kg only. The remaining 4,95kg can be put in the garden to attract and fool ants.

To adhere corundum to the rolls I used 2-component epoxy glue. It is a messy experience to coat small rolls with glue and rolling them in a can of corundum sand.

<img  width="300px" src="../images/omniwheel/rolling in glue.png"/>

To improve the quality of the adhesive layer, you can add instant adhesive with low viscosity (intended for porose surfaces) on the rolls. This is the way industrial sandpaper is manufactured to support a grain at its flanks. The final wheel looks really nice:

<img  width="400px" src="../images/omniwheel/final.jpg"/>
