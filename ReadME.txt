Note:  Everything in this repository  is a Work In Progress (WIP), there are no 
warranties or guarantees of any kind.   Use at your own risk!  However I hope some 
of you have fun with it!

This folder contains the source code for Lynxmotion Phoenix code that has been adapted
to the Arbotix PhantomX robot sold by Trossen Robotics (http://www.trossenrobotics.com/c/phantomx-hexapod-kits.aspx)

The original Phoenix code was written by Jeroen Janssen [aka Xan] to run on the Lynxmotion Phoenix 
(http://www.lynxmotion.com/c-117-phoenix.aspx). It was originally written in Basic for the Basic Atom Pro 28
processor by Basic Micro.    The Lynxmotion Phoenix was based on the original Phoenix that 
was developed by Kåre Halvorsen (aka Zenta) and a lot of the software was based off of his earlier Excel 
spreadsheet (PEP).  More details up on his Project page (http://www.lynxmotion.com/images/html/proj098.htm).

I later ported the code to C/C++ and the Arduino environment and with the help of Kåre and Jeroen hopefully 
reduced the number of bugs I introduced as part of this port.   
With Kåre's and others help, I then ported this code to the PhantomX.  

There are lots of details up on the thread: http://forums.trossenrobotics.com/showthread.php?5429-Thinking-about-trying-out-a-different-Hex-robot...
I probably should create a new thread that summarizes all of the stuff in the thread above, so there is lots of stuff 

Hardware: PhantomX (either using AX-12A or AX-18 servos).  

We modified our legs some to allow for more movement. Zenta had some nice pictures showing this 
(http://forums.trossenrobotics.com/gallery/showimage.php?i=4617&catid=member&imageuser=1535)

In addition we made the Phoenix so that it could walk either upside down or right side up.  
To do this we rotated the tibias 90 degrees so that at 0 the legs stick straight out.  
This is controlled in the code by the option (OPT_WALK_UPSIDE_DOWN).  As part of this option we 
needed a way to detect which way is up.  We used one of these: http://www.lynxmotion.com/p-606-lynxmotion-buffered-2g-accelerometer.aspx
More details up on this page of the thread: http://forums.trossenrobotics.com/showthread.php?5429-Thinking-about-trying-out-a-different-Hex-robot.../page18

Note: There is a Sneak Peak Video by Zenta on Page 19 of the thread!

We have code in place to try to detect when the battery voltage gets too low as to try to minimize 
the risk of damaging the battery.  There are a couple versions voltage detection in the code.  
The first version I tried was to ask one of the Servos for the voltage.  At times I found this 
unreliable and also slow as it required sending serial packets.   
Later I added an external voltage Resistor divider, that I am currently using a 20K and 4.66K resistor for.  
There are defines in the Hex_Cfg.h file that allow you to define which IO pin you wish to use (or not).
Also it allows you to define which resistors you are using. I added this second method after I destroyed 
my first battery.  For a second level of testing I also added one of these: http://www.trossenrobotics.com/p/3S-LiPo-Battery-Monitor.aspx

Currently we have the PhantomX running using a few different Input controllers.  

The main one I have been playing with is a XBee DIY remote control that a few of us have.  
More details up on the thread: http://www.lynxmotion.net/viewtopic.php?f=21&t=5447

We have also done some testing using a Lynxmotion PS2 controller (http://www.lynxmotion.com/p-552-ps2-robot-controller.aspx)

We are currently working on the Input code for the Arbotix Robot Controller 2, 
which most of the PhantomX users are currently using the with original PhantomX software. 

The Source File Hex_Cfg.h, is the main place that you need to go to change which controller to use and other options 
that the program uses. 

Again this is a WIP!

Kurt

