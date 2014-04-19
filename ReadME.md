Warning
=======

This is not the primary project for my work to support Hex and Quad Robots.  The Primary project is the https://github.com/KurtE/Arduino_Phoenix_Parts.  In that project there are several different libraries, to build the Phoenix code for any of my Hex robots.  

The top library is the Phoenix and under this are several example projects that can create several different configurations, including one that is hopefully always set up for a stock PhantomX robot.

However this is well as a few other projects are easier to use for a specific type of robot, in this case the PhantomX Hexapods or Quads.  Instead of having to install all of the parts as libraries, they are all self contained in the sketch folder. 

When I make changes to the code base I try to migrate the changes to all of the differnt projects.  There will be times when the projects will be out of sync.  If you run into issues with this project, you might want to look at the primary project I mentioned above.   

I do the migration from the Arduino_Phoenix_Parts project by collecting all of the pieces which in the Arduino_Phoenix_Parts project that in separate directories and copy them into the main sketch folder.   When I do that I will rename these files as to not conflict with those files in the
parts project.  Example: Phoenix.h goes to _Phoenix.h.

Warning sometimes when you sync with this project it may be configured as a hexapod and other times it is configured as a quad. 

The code is configured to run on a Version 2 of the PhantomX Hexapod using an Arbotix Commander for input.  

This is a Work In Progress!  There are no warrantees or Guarantees of any type that this code is useable for anything. But I hope it is.

General
=======

Most of the code in these libraries are in header files, which allows them to be compiled specifically for each project.   


Installation
============

Note: Hopefully this code is reasonably self-contained and can be located anywhere.  I personally keep this project in my Arduino Sketchbook, such that I can load it, using the Arduino File/Sketchbook menu.   

**This code requires that you have also installed the Arbotix extensions to the Arduino environment as well as their libraries.**


File Descriptions
====================

Main Phoenix Code
-----------------

- Phantom_Phoenix.ino - Main sketch file
- _Phoenix.h - Contains the main header file with data descriptions
- _Phoenix_Code.h -  Primary code base 
- _Phoenix_Driver_AX12 - This is the servo driver used for Robotis AX-12 or AX-18 servos 
- _Phoenix_Input_Commander - Support for the Arbotix Commander.  This is used with most of the Trossen Robots

Some Notes about Capabilities and Options
========================================

There are features and options that are part of the code base that should probably be documented.  While the below is
not complete, I hope it might help out some.  Sorry that the information here is a bit rough.

\#define Options
--------------

Many of the different Input drivers as well as the servo drivers have options that can be defined.  Many of the options are in place to either include or exclude code from the executable.  Some are specific to each of the drivers, but some 
are more generic.  Here are a few of the ones that I picked out from the others.

First there is simple defines for selecting something major like four degrees for freedom and Quad mode (versus the default
of hexapod)

- \#define c4DOF - Use this option to turn on 4Dof - This will require several things to be defined for the 4th degree (Tars) in your configuration file.  Note: this option does also give you the capability to have a mix of 3 and 4 DOF legs.



- \#define QUADMODE - Turns on quad mode support.

- \#define OPT_SKETCHSETUP // If defined will call   SketchSetup();
Allow the actual program to do something at startup.  Example with Orion driver I may want to detect if a button is pressed and hang there in a loop as to allow me to update the firmware of the Orion.

- \#define OPT_BACKGROUND_PROCESS
Some Servo drivers needs the main code to call it whenever it can as to do some background work.  Example AX12 to do the default interpolation. 

- \#define OPT_GPPLAYER
The Original Phoenix code base has the ability to run General Purpose Sequences that were stored on the SSC-32 servo driver.  For some of the other servo drivers I have done code to emulate this.  On AX-12 I allow you to import Pypose sequences.  

- \#ifdef OPT_SINGLELEG - Turns on the Single Leg code support.  This was on by default and I will probably add this to many of the examples, but for many this is not needed and gives you back some code space. 

- \#define ADJUSTABLE_LEG_ANGLES
For some robots, I am still experimenting with the ability to adjust the leg positions (angles between legs) as well as the distance of
the leg from the center.  Currently only supported using the Commander.  

- \#define DBGSerial
For those robots whose controller boards who have a Serial port that you can use to talk to something like the Arduino Serial Monitor, you can define DBGSerial, to the actual Serial port.  On other boards, example Arbotix, where Serial is used by the XBee and Serial1 is used for
talking to the AX servos, you typically cannot define this.   

- \#define OVERWRITE_GAITS
- \#define ADD_GAITS
With some of the more recent builds, the gaits have been defined as a structure.  With this you now have the ability to have your specific robot 
completely replace the set of gaits used or add gaits to the default list.  An example of this is defined in the Phantom Phoenix Quad example under Phoenix. 

- \#define cTurnOffVol 1000
With the use of Lipo or LifePo4 types of batteries, it is good idea to try to shut off the robot before the battery is drained too far.  (100ths of volt)  When the power goes below this point, if possible it will shut off all of the servos. And if sound is enabled, it will make some noise.  You should still shut the system off as the processor will still be using up the battery.

- \#define cTurnOnVol 1100
If cTurnOffVol is defined you can also define a voltage that allows the robot to turn back on.  This was needed as sometimes you start a robot on USB, 
before power switch is turned on.  So if you then turn on power it would be ok to again allow the robot to turn on.

- \#define cVoltagePin 
With Some servo drivers like the AX12 driver, the servo driver code can get the voltage from the servos.  But if your processor board has the ability to get the  voltage, you can instead define which pin to do the analogRead from.

- \#define CVADR1  40
\#define CVADR2  10
Since by default the voltage to the IO pins cannot exceed the system voltage, often these pins are connected up through a resistor divider circuit (2 resistors).  
Example Lynxmotion BotBoarduino has a 10K and 30K resistor.  So you need to define this somewhere.  Actually you just need the ratios

- \#define CVREF   500
Up till now all of the boards were 5V, but now with Teensy and the like some or now 3.3v so needed somewhere to define.  Defaults to 5V

- \#define OPT_TERMINAL_MONITOR  
If DBGSerial is defined, then you can also enable other features.  In particular a simple Terminal monitor, that allows you to type in simple commands.  If defined, there are a few very basic commands that are part of the main code (Toggle Debug on and Off), and each of the Servo
drivers can add additional commands.   

Terminal Monitor 
-----------------

As by default the project is used on an Arbotix controller where the two serial ports are used for the XBee (Commander) and AX-12 servos.  As such most of the time the Terminal Monitor is not valid on this platform.   If you wish to see more information about the monitor see the Read me file associated with the main project. 

Major Contributors
==================

Jeroen Janssen [aka Xan] -  The original Phoenix code was written by to run on the Lynxmotion Phoenix 
(http://www.lynxmotion.com/c-117-phoenix.aspx). It was originally written in Basic for the Basic Atom Pro 28
processor by Basic Micro.  

Kare Halvorsen (aka Zenta) -  The Lynxmotion Phoenix was based on the original Phoenix that was developed by
him.  In addition a lot of the software was based off of his earlier Excel spreadsheet (PEP).  More details up on his 
Project page (http://www.lynxmotion.com/images/html/proj098.htm).

Me - I later ported the code to C/C++ and the Arduino environment and with the help of Kåre and Jeroen hopefully 
reduced the number of bugs I introduced as part of this port.   

Michael E. Ferguson (lnxfergy up on Trossen) - Arbotix Commander, Ax12.

Bill Porter - PS2 library for Arduino.


Again Warning
=============

This is a WIP - No promises or guarantees!

