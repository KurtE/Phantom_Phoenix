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
//PS2 CONTROLS:
//[Common Controls]
//- Right Joystick Button - Turn on/off the bot
//- Left Joystick Button - Act like Select button
//- Left (S8) - Switch modes - Walk, Shift, Rotate...
//- Right (S7) - ...
//- 
//- L2Toggle Rotate mode
//- CircleToggle Single leg mode
//   - Square        Toggle Balance mode
//- TriangleMove body to 35 mm from the ground (walk pos) 
//and back to the ground
//- D-Pad upBody up 10 mm
//- D-Pad downBody down 10 mm
//- D-Pad leftdecrease speed with 50mS
//- D-Pad rightincrease speed with 50mS
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
//[Single leg Controls]
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls]
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
enum {
  WALKMODE=0, TRANSLATEMODE, ROTATEMODE, SINGLELEGMODE, MODECNT};

enum {
  NORM_NORM=0, NORM_LONG, HIGH_NORM, HIGH_LONG};


#define cTravelDeadZone 4      //The deadzone for the analog input from the remote

#define ARBOTIX_TO  1000        // if we don't get a valid message in this number of mills turn off



//=============================================================================
// Global - Local to this file only...
//=============================================================================
Commander command = Commander();
unsigned long g_ulLastMsgTime;

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
extern void MSound(uint8_t _pin, byte cNotes, ...);
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
        MSound(SOUND_PIN, 2, 50, 2000, 50, 3000);  //sound SOUND_PIN, 
      } 
      else {
        MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, 
      }
    }

    //[Common functions]
    //Switch Balance mode on/off 
    if ((command.buttons & BUT_L4) && !(buttonsPrev & BUT_L4)) {
      g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
      if (g_InControlState.BalanceMode) {
        MSound(SOUND_PIN, 1, 250, 1500);  //sound SOUND_PIN, [250\3000]
      } 
      else {
        MSound(SOUND_PIN, 2, 100, 2000, 50, 4000);
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
    // Will start off using this as one shot, but then will probably use timer or like to move up or down
    // Need to remember that the joystick values have been converted to signed values with center at 0
    if ((command.buttons & BUT_L6) && !(buttonsPrev & BUT_L6)) {
      if (command.lookV > 32)
        g_BodyYOffset += 10;
      else if (command.lookV < -32)
        g_BodyYOffset -= 10;

      if (command.lookH > 32) {
        if (g_InControlState.SpeedControl>0) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
          MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
        }
      } 
      else if (command.lookH < -32) {
        if (g_InControlState.SpeedControl<2000 ) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
          MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
        }
      }
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
          MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
        } 
        else {
          MSound (SOUND_PIN, 2, 50, 2000, 50, 2250); 
          g_InControlState.GaitType = 0;
        }
        GaitSelect();
      }

      //Double leg lift height
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
        HeightSpeedMode = (HeightSpeedMode + 1) & 0x3; // wrap around mode
        DoubleTravelOn = HeightSpeedMode & 0x1;
        if ( HeightSpeedMode & 0x2)
          g_InControlState.LegLiftHeight = 80;
        else
          g_InControlState.LegLiftHeight = 50;
      }

#ifdef LATER 
      // Switch between Walk method 1 && Walk method 2
      if (command.buttons(PSB_R3)) { // R3 Button Test
        MSound (SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
        WalkMethod = !WalkMethod;
      }
#endif  

#ifdef LATER
      //Walking
      if (WalkMethod)  //(Walk Methode) 
        g_InControlState.TravelLength.z = (command.lookV-128); //Right Stick Up/Down  

      else
#endif                
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

    //[Single leg functions]
    if (ControlMode == SINGLELEGMODE) {
      //Switch leg for single leg control
      if ((command.buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
        MSound (SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
        if (g_InControlState.SelectedLeg<5)
          g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
        else
          g_InControlState.SelectedLeg=0;
      }

      g_InControlState.SLLeg.x= (byte)(command.walkH+128)/2; //Left Stick Right/Left
      g_InControlState.SLLeg.y= (byte)(command.lookV+128)/10; //Right Stick Up/Down
      g_InControlState.SLLeg.z = (byte)(command.walkV+128)/2; //Left Stick Up/Down

      // Hold single leg in place
      if ((command.buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
        MSound (SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
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






