//==============================================================================
// InputController.h - This is is the class definition for the abstraction of
//     which input controller is used to control the Hex robot.  There will be
//     several implementations of this class, which include:
//         PS2 - 
//         XBEE
//         Serial
//         Autonomous
//==============================================================================
#ifndef _INPUT_CONTROLLER_h_
#define _INPUT_CONTROLLER_h_

#include "Hex_Cfg.h"  // make sure we know what options are enabled...
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

class InputController {
  public:
    virtual void     Init(void);
    virtual void     ControlInput(void);
    virtual void     AllowControllerInterrupts(boolean fAllow);

  private:
} ;   

// Define a function that allows us to define which controllers are to be used.
extern void  RegisterInputController(InputController *pic);



typedef struct _Coord3D {
    long      x;
    long      y;
    long      z;
} COORD3D;


//==============================================================================
// class ControlState: This is the main structure of data that the Control 
//      manipulates and is used by the main Phoenix Code to make it do what is
//      requested.
//==============================================================================
typedef struct _InControlState {
     boolean	    fHexOn;		//Switch to turn on Phoenix
     boolean	    fPrev_HexOn;	//Previous loop state 
//Body position
     COORD3D        BodyPos;
     COORD3D        BodyRotOffset;      // Body rotation offset;

//Body Inverse Kinematics
     COORD3D        BodyRot1;           // X -Pitch, Y-Rotation, Z-Roll

//[gait]
     byte	    GaitType;		//Gait type

     short	    LegLiftHeight;	//Current Travel height
     COORD3D        TravelLength;       // X-Z or Length, Y is rotation.

//[Single Leg Control]
     byte	    SelectedLeg;
     COORD3D        SLLeg;              // 
     boolean	    fSLHold;		//Single leg control mode


//[Balance]
     boolean        BalanceMode;

//[TIMING]
     byte	    InputTimeDelay;	//Delay that depends on the input to get the "sneaking" effect
     word	    SpeedControl;	//Adjustible Delay
     byte           ForceGaitStepCnt;          // new to allow us to force a step even when not moving
} INCONTROLSTATE;


#endif //_INPUT_CONTROLLER_h_
