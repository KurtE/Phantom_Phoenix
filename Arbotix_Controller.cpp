//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: Arbotix Commander version - Try to emulate most of PS2, but PS2 has 16 buttons and Commander 
// has 10. so some things may not be there, others may be doubled up.
// 
//NEW IN V1.0
//- First Release
//
//Walk method 1:
//- Left StickWalk/Strafe
//- Right StickRotate
//
//Walk method 2:
//- Left StickDisable
//- Right StickWalk/Rotate
//
//
// Quick and Dirty description of controls... WIP
// In most cases I try to mention what button on the PS2 things coorespond to..
// On/OFF - Turning the commander 2 on and off (PS2 start button)
// R1 - options (Change walk gait, Change Leg in Single Leg, Change GP sequence) (Select on PS2)
// R2 - Toggle walk method...  Run Sequence in GP mode
// R3 - Walk method (Not done yet) - (PS2 R3)
// L4 - Ballance mode on and off
// L5 - Stand/Sit (Triangle on PS2)
// L6+Right Joy UP/DOWN - Body up/down - (PS2 Dpad Up/Down)
// L6+Right Joy Left/Right - Speed higher/lower - (PS2 DPad left/right)
// Right Top(S7) - Cycle through options of Normal walk/Double Height/Double Travel) - (PS2 R1, R2)
// Left Top(S8) - Cycle through modes (Walk, Translate, Rotate, Single Leg) (PS2: Circle, X, L1, L2)

// Note: Left some descriptions of PS2 stuff, especially parts still left to Map/Implement.
//
//[Walk Controls]
//- selectSwitch gaits
//- Left Stick(Walk mode 1) Walk/Strafe
// (Walk mode 2) Disable
//- Right Stick(Walk mode 1) Rotate, 
//(Walk mode 2) Walk/Rotate
//- R1Toggle Double gait travel speed
//- R2Toggle Double gait travel length
//
//[Shift Controls]
//- Left StickShift body X/Z
//- Right StickShift body Y and rotate body Y
//
//[Rotate Controls]
//- Left StickRotate body X/Z
//- Right StickRotate body Y
//
//[Single leg Controls] - Need to check...
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls] - How to do sequences???
//- selectSwitch Sequences
//- R2Start Sequence
//
//====================================================================
// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include "Hex_Globals.h"
#ifdef USECOMMANDER
#include <Commander.h>
//[CONSTANTS]
#ifdef OPT_GPPLAYER
enum {
  WALKMODE=0, TRANSLATEMODE, ROTATEMODE, SINGLELEGMODE, GPPLAYERMODE, MODECNT};
#else
enum {
  WALKMODE=0, TRANSLATEMODE, ROTATEMODE, SINGLELEGMODE, MODECNT};
#endif
enum {
  NORM_NORM=0, NORM_LONG, HIGH_NORM, HIGH_LONG};


#define cTravelDeadZone 4      //The deadzone for the analog input from the remote

#define ARBOTIX_TO  1000        // if we don't get a valid message in this number of mills turn off



//=============================================================================
// Global - Local to this file only...
//=============================================================================
Commander command = Commander();
unsigned long g_ulLastMsgTime;
short  g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet

#ifdef USEMULTI
//==============================================================================
//
// Lets define our Sub-class of the InputControllerClass
//
//==============================================================================
class CommanderInputController : 
public InputController
{
public:
  CommanderInputController();        // A reall simple constructor...

  virtual void     Init(void);
  virtual void     ControlInput(void);
  virtual void     AllowControllerInterrupts(boolean fAllow);

};


CommanderInputController g_CommanderController;


//==============================================================================
// Constructor. See if there is a simple way to have one or more Input
//     controllers. Maybe register at construction time
//==============================================================================
CommanderInputController::CommanderInputController()
{
  RegisterInputController(this);
}

#else
#define CommanderInputController InputController
// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller 
#endif



static short   g_BodyYOffset; 
static short   g_BodyYShift;
static byte    ControlMode;
static byte    HeightSpeedMode;
//static bool  DoubleHeightOn;
static bool    DoubleTravelOn;
static bool    WalkMethod;
byte           GPSeq;             //Number of the sequence

static byte    buttonsPrev;
static byte    extPrev;

// some external or forward function references.
extern void CommanderTurnRobotOff(void);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void CommanderInputController::Init(void)
{
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  command.begin(38400);
  GPSeq = 0;  // init to something...

  ControlMode = WALKMODE;
  HeightSpeedMode = NORM_NORM;
  //    DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void CommanderInputController::AllowControllerInterrupts(boolean fAllow)
{
  // We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
void CommanderInputController::ControlInput(void)
{
  // See if we have a new command available...
  if(command.ReadMsgs() > 0){
    // If we receive a valid message than turn robot on...
    g_InControlState.fHexOn = true;

    // [SWITCH MODES]

    // Cycle through modes...
    if ((command.buttons & BUT_LT) && !(buttonsPrev & BUT_LT)) {
      if (++ControlMode >= MODECNT) {
        ControlMode = WALKMODE;    // cycled back around...
        MSound( 2, 50, 2000, 50, 3000); 
      } 
      else {
        MSound( 1, 50, 2000);  
      }
      if (ControlMode != SINGLELEGMODE)
        g_InControlState.SelectedLeg=255;

    }

    //[Common functions]
    //Switch Balance mode on/off 
    if ((command.buttons & BUT_L4) && !(buttonsPrev & BUT_L4)) {
      g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
      if (g_InControlState.BalanceMode) {
        MSound( 1, 250, 1500); 
      } 
      else {
        MSound( 2, 100, 2000, 50, 4000);
      }
    }

    //Stand up, sit down  
    if ((command.buttons & BUT_L5) && !(buttonsPrev & BUT_L5)) {
      if (g_BodyYOffset>0) 
        g_BodyYOffset = 0;
      else
        g_BodyYOffset = 35;
    }

    // We will use L6 with the Right joystick to control both body offset as well as Speed...
    // We move each pass through this by a percentage of how far we are from center in each direction
    // We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
    // which give an idea, but only for those whoes robot has a speaker
    if (command.buttons & BUT_L6 ) {
      // raise or lower the robot on the joystick up /down
      // Maybe should have Min/Max
      g_BodyYOffset += command.lookV/25;

      // Likewise for Speed control
      int dspeed = command.lookH / 16;   // 
      if ((dspeed < 0) && g_InControlState.SpeedControl) {
        if ((word)(-dspeed) <  g_InControlState.SpeedControl)
          g_InControlState.SpeedControl += dspeed;
        else 
          g_InControlState.SpeedControl = 0;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl);  
      }
      if ((dspeed > 0) && (g_InControlState.SpeedControl < 2000)) {
        g_InControlState.SpeedControl += dspeed;
        if (g_InControlState.SpeedControl > 2000)
          g_InControlState.SpeedControl = 2000;
        MSound( 1, 50, 1000+g_InControlState.SpeedControl); 
      }

      command.lookH = 0; // don't walk when adjusting the speed here...
    }

    //[Walk functions]
    if (ControlMode == WALKMODE) {
      //Switch gates
      if (((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1))
        && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
      && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
        && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
        g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
        if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
          MSound( 1, 50, 2000);  
        } 
        else {
          MSound (2, 50, 2000, 50, 2250); 
          g_InControlState.GaitType = 0;
        }
        GaitSelect();
      }

      //Double leg lift height
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound( 1, 50, 2000);  
        HeightSpeedMode = (HeightSpeedMode + 1) & 0x3; // wrap around mode
        DoubleTravelOn = HeightSpeedMode & 0x1;
        if ( HeightSpeedMode & 0x2)
          g_InControlState.LegLiftHeight = 80;
        else
          g_InControlState.LegLiftHeight = 50;
      }

      // Switch between Walk method 1 && Walk method 2
      if ((command.buttons & BUT_R2) && !(buttonsPrev & BUT_R2)) {
        MSound (1, 50, 2000);
        WalkMethod = !WalkMethod;
      }

      //Walking
      if (WalkMethod)  //(Walk Methode) 
        g_InControlState.TravelLength.z = (command.lookV); //Right Stick Up/Down  

      else
      {
        g_InControlState.TravelLength.x = -command.walkH;
        g_InControlState.TravelLength.z = command.walkV;
      }

      if (!DoubleTravelOn) {  //(Double travel length)
        g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
        g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
      }

      g_InControlState.TravelLength.y = -(command.lookH)/4; //Right Stick Left/Right 
    }

    //[Translate functions]
    g_BodyYShift = 0;
    if (ControlMode == TRANSLATEMODE) {
      g_InControlState.BodyPos.x =  SmoothControl(((command.walkH)*2/3), g_InControlState.BodyPos.x, SmDiv);
      g_InControlState.BodyPos.z =  SmoothControl(((command.walkV)*2/3), g_InControlState.BodyPos.z, SmDiv);
      g_InControlState.BodyRot1.y = SmoothControl(((command.lookH)*2), g_InControlState.BodyRot1.y, SmDiv);

      //      g_InControlState.BodyPos.x = (command.walkH)/2;
      //      g_InControlState.BodyPos.z = -(command.walkV)/3;
      //      g_InControlState.BodyRot1.y = (command.lookH)*2;
      g_BodyYShift = (-(command.lookV)/2);
    }

    //[Rotate functions]
    if (ControlMode == ROTATEMODE) {
      g_InControlState.BodyRot1.x = (command.walkV);
      g_InControlState.BodyRot1.y = (command.lookH)*2;
      g_InControlState.BodyRot1.z = (command.walkH);
      g_BodyYShift = (-(command.lookV)/2);
    }
#ifdef OPT_GPPLAYER
    //[GPPlayer functions]
    if (ControlMode == GPPLAYERMODE) {
      // Lets try some speed control... Map all values if we have mapped some before
      // or start mapping if we exceed some minimum delta from center
      // Have to keep reminding myself that commander library already subtracted 128...
      if (g_ServoDriver.FIsGPSeqActive() ) {
        if ((g_sGPSMController != 32767)  
          || (command.lookV > 16) || (command.lookV < -16))
        {
          // We are in speed modify mode...
          if (command.lookV >= 0)
            g_sGPSMController = map(command.lookV, 0, 127, 0, 200);
          else  
            g_sGPSMController = map(command.lookV, -127, 0, -200, 0);
          g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
        }
      }

      //Switch between sequences
      if ((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          if (GPSeq < 5) {  //Max sequence
            MSound (1, 50, 1500);  
            GPSeq = GPSeq+1;
          } 
          else {
            MSound (2, 50, 2000, 50, 2250);
            GPSeq=0;
          }
        }
      }
      //Start Sequence
      if ((command.buttons & BUT_R2) && !(buttonsPrev & BUT_R2)) {
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          g_ServoDriver.GPStartSeq(GPSeq);
          g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
        }
        else {
          g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
            MSound (2, 50, 2000, 50, 2000);
        }
      }

    }
#endif // OPT_GPPLAYER

    //[Single leg functions]
    if (ControlMode == SINGLELEGMODE) {
      //Switch leg for single leg control
      if ((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
        MSound (1, 50, 2000);  
        if (g_InControlState.SelectedLeg<5)
          g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
        else
          g_InControlState.SelectedLeg=0;
      }

      g_InControlState.SLLeg.x= (byte)((int)command.walkH+128)/2; //Left Stick Right/Left
      g_InControlState.SLLeg.y= (byte)((int)command.lookV+128)/10; //Right Stick Up/Down
      g_InControlState.SLLeg.z = (byte)((int)command.walkV+128)/2; //Left Stick Up/Down

      // Hold single leg in place
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound (1, 50, 2000);  
        g_InControlState.fSLHold = !g_InControlState.fSLHold;
      }
    }


    //Calculate walking time delay
    g_InControlState.InputTimeDelay = 128 - max(max(abs(command.walkH), abs(command.walkV)), abs(command.lookH));

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = max(g_BodyYOffset + g_BodyYShift,  0);

    // Save away the buttons state as to not process the same press twice.
    buttonsPrev = command.buttons;
    extPrev = command.ext;
    g_ulLastMsgTime = millis();
  } 
  else {
    // We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
    if (g_InControlState.fHexOn) {
      if ((millis() - g_ulLastMsgTime) > ARBOTIX_TO)
        CommanderTurnRobotOff();
    }
  }
}

//==============================================================================
// CommanderTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void CommanderTurnRobotOff(void)
{
  //Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  g_InControlState.SelectedLeg = 255;
  g_InControlState.fHexOn = 0;
}


#endif //USEPS2














