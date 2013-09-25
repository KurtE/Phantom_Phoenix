Warning
=======

This is not the project that I do most of my work in to support my Hex Robots.  I do most of my work, in 
the Arduino_Phoenix_Parts project.  In that project there are several different libraries, to build the Phoenix code for any of my Hex robots.  The top
library is the Phoenix and under this are several example projects that can create several different configurations, including one that is 
hopefully always set up for a stock PhantomX robot.


This is a Work In Progress!  There are no warrantees or Guarantees of any type that this code is useable for anything.  

But I hope it is.


However from time to time I will drop in current code from the Arduino_Phoenix_Parts project that tries to bring this code base
somewhat up to date.  When I do that, I will try to collect all of the pieces which in that project are in separate directories
and bring them all into one directory.  When I do that I will rename these files as to not conflict with those files in the
parts project.  Example: Phoenix.h goes to _Phoenix.h.

Currently this project is setup with the Quad branch of the mentioned project.  The code is configured to run on a Version 2 of the PhantomX Hexapod 
using an Arbotix Commander for input.  If there is interest in these snap shots to also support the Quad, I can add a few compiler
options for that as well.


General
=======

Most of the code in these libraries are in header files, which allows them to be compiled specifically for each project.   


Installation
============

Note: Hopefully this code is reasonably self-contained and can be located anywhere.  I personally keep this project
in my Arduino Sketchbook, such that I can load it, using the Arduino File/Sketchbook menu.   

This code requires that you have also installed the Arbotix extensions to the Arduino environment as well as their libraries.


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

