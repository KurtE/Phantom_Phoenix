//==============================================================================
// GLOBALS - The main global definitions for the CPhenix program - still needs
//		to be cleaned up.
// This program assumes that the main files were compiled as C files
//==============================================================================
#ifndef _HEX_GLOBALS_H_
#define _HEX_GLOBALS_H_
#include <stdarg.h>
#include "Hex_Cfg.h"
#include "ServoDriver.h"
#include "InputController.h"

#ifdef USEXBEE
#include "diyxbee.h"
#endif

//=============================================================================
//[CONSTANTS]
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP 	1

#define	c1DEC		10
#define	c2DEC		100
#define	c4DEC		10000
#define	c6DEC		1000000

#define	cRR			0
#define	cRM			1
#define	cRF			2
#define	cLR			3
#define	cLM			4
#define	cLF			5

#define	WTIMERTICSPERMSMUL  	64	// BAP28 is 16mhz need a multiplyer and divider to make the conversion with /8192
#define WTIMERTICSPERMSDIV  	125 // 
#define USEINT_TIMERAV



#define NUM_GAITS    6
#define SmDiv    4  //"Smooth division" factor for the smooth control function, a value of 3 to 5 is most suitable
extern void GaitSelect(void);
extern short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider);


//-----------------------------------------------------------------------------
// Define global class objects
//-----------------------------------------------------------------------------
extern ServoDriver      g_ServoDriver;           // our global servo driver class
extern InputController  g_InputController;       // Our Input controller 
extern INCONTROLSTATE   g_InControlState;		 // State information that controller changes

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
extern boolean          g_fDebugOutput;
extern boolean          g_fEnableServos;      // Hack to allow me to turn servo processing off...


extern void MSound(uint8_t _pin, byte cNotes, ...);
extern boolean CheckVoltage(void);

void AdjustLegPositionsToBodyHeight(void);

// debug handler...
extern boolean g_fDBGHandleError;

#ifdef c4DOF
extern const byte cTarsLength[] PROGMEM;
#endif

#ifdef OPT_BACKGROUND_PROCESS
#define DoBackgroundProcess()   g_ServoDriver.BackgroundProcess()
#else
#define DoBackgroundProcess()   
#endif



#ifdef __AVR__
#if not defined(UBRR1H)
//extern NewSoftSerial SSCSerial;
extern SoftwareSerial SSCSerial;
#endif
#endif
#endif


