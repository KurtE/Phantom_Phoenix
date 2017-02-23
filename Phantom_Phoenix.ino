
// Warning setup to build for standard hexapod, octopod, or for quad.
//  #define QUADMODE
//  #define HEXMODE
//  #define OCTOMODE
//  #define DISPLAY_GAIT_NAMES
//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!     
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Arbotix Robocontroller board
//
//=============================================================================
// Warning:: This configuration does not check voltages, so you should be careful to
// not allow the lipo to discharge too far. 
//
// This configuration should hopefully run on a stock PhantomX, without any
// of my changes.
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

#define DEFINE_HEX_GLOBALS
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#ifdef QUADMODE
#include "Quad_Cfg.h"
#elif defined(OCTOMODE)
#include "Octo_Cfg.h"
#else
#define HEXMODE   // default to hex mode
#include "Hex_Cfg.h"
#endif

#include <ax12.h>
#include "_Phoenix.h"

#ifdef QUADMODE
#define ADD_GAITS
#define PYPOSE_GAIT_SPEED 1 //98
//  Speed, Steps, Lifted, Front Down, Lifted Factor, Half Height, On Ground, 
//     Quad extra: COGAngleStart, COGAngleStep, CogRadius, COGCCW
//                      { RR, RF, LR, LF}
PHOENIXGAIT APG_EXTRA[] = { 
  {PYPOSE_GAIT_SPEED, 8, 2, 1, 2, 6, 1, 0, 0,0, true, {7, 1, 3, 5}},   // ripple
  {PYPOSE_GAIT_SPEED, 12, 2, 1, 2, 10, 1, 0, 0,0, true, {7, 1, 4, 10}},   // ripple
  {PYPOSE_GAIT_SPEED, 4, 2, 1, 2, 2, 1, 0, 0, 0, true,{3, 1, 1, 3}},  // Amble
  {PYPOSE_GAIT_SPEED, 6, 3, 2, 2, 3, 2, 0, 0,0, true, {1, 4, 4, 1}} }; // Smooth Amble 
#endif

#include "_Phoenix_Input_Commander.h"
#include "_Phoenix_Driver_AX12.h"
#include "_Phoenix_Code.h"

