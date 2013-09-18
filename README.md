# Introduction #

An omniwheel robot created for a class project. The robot uses three omniwheels mounted in a equilateral triangle formation to move in any direction without changing its orientation. A line following component was also added to be able to demo its movement easily. The CAD files for the body design, the circuit schematics, and the code are all included. Though only the simpler PID method was tested and used for the final project, I've also included another PID function written afterwards that would need to be tested to tweak the factors before use. 
All the code, cad files, and eagle files are included.

## Body Design ##

The frame was modeled in NX CAD. The part drawings were then saved as DXF files and printed out of .125”-thick, fluorescent green acrylic. The robot assembly was designed to be simple yet sturdy and was accomplished through a combination of press-fits, fastener screws, and hot glue.
The enclosed portion of the frame housed the three 6 volt battery packs, the cylindrical housing for the three motors, as well as the NU32’s PCB. The undercarriage held the motor gearboxes and attached omniwheels as well as the custom PCB with the H-bridges and phototransistor LED ring, described in detail below. The files are attached.

We used the faulhaber motor with encoder described here: http://www.robotroom.com/FaulhaberGearmotor.html 
Our main reason for using these was that they were available in the lab. But we had a few issues with it, such as being able to tighten the set screw. We did not have the right hex key available and so we just used a small screwdriver to tighten it whenever there was an issue. But by the end of the project, this had stripped the screw and made it impossible to tighten the wheels. Luckily we had already demoed by then, but it is the reason for not testing out the new PID.

## Circuit ##

We wanted line sensing, but it wasn't the main objective of the project so this part was kept simple and cheap. Just a ring of LEDs and phototransistors for the line detection. The ring of 16 LED/phototransistor pairs makes the detection cheap and orientation independent. 
We kept most of the circuit elements on the PCB on the underside of the robot where the LEDs would have to be. The phototransistors are connected to a MUX and cycled through by the PIC to read different points on the circle. The circuit was designed in EAGLE and the files are also included.


## Code ##

The encoder counts are counted by polling the ports at a tiny interval (faster than that of the motor’s fastest rpm * encoder lines) and decoding the quadrature.

The sensing and movement:

(LED 7 was 90 degrees, straight ahead)

1.  If there is no previous forward point saved, set previous forward point to 7

2.  Cycle through LEDs and check “high” phototransistors (those over a dark color)

3.  Collect contiguous blocks of high LEDs (so if LEDs 1-4 were high, that would be one 
block)

4.  Compute the point distance between the previous forward point and the border points of 
each block.

5.  Update previous forward point to the point that is the shortest distance from the previous forward point. (thus 
distinguishing between the front and back of the path)

7.  Convert point to degree heading and pass to movement

The movement algorithm was a simple linear system that was solved for the velocities of each 
wheel. The three parameters were the x,y directions and the angular velocity which we set to 
0 in order to achieve movement without change in orientation. 
We solve for each wheel's velocity by using the amount it would contribute to each direction while keeping rotation zero

## PID ##

The simple PID method we used was to check the speed of the wheel that was supposed to spin the fastest (or pick one 
of the two if two were supposed to spin at equal speeds, like for straight ahead movement) and use it and proportions of it
as the target value to judge error of off for feedback.

Obviously it isn't perfect but it worked for the slower speeds 
with only very slight orientation changes after a while of demoing. But we had a more advanced method we hadn't fully tweaked 
by the time it was due and can't test now due to the broken screw. 

Once again use relative velocities, but make the target an angular velocity of 0.  The error would be amount off 0, and feedback affects all three wheels by k*relative velocity, so wheels are corrected differently 
based on how fast they are going, and wheels rotating the robot in the opposite direction are corrected by the opposite value. 

This looked like it could work better but it was a bit more complicated. And to get it really smooth and going at a decent speed, 
there is the issue of correcting current relative velocities to prevent the orientation change while also keeping target velocities in mind
so that velocities aren't corrected and kept at too low of a speed. And waiting too long after a heading change to correct would mean 
that the orientation would change too much between heading changes. Implementing that and ironing out such issues is the next step.

