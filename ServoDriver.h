
//==============================================================================
// ServoDriver.h  This header file defines the ServoDriver class.
// 
// This class is used by the main Phoenix code to talk to the servos, without having
// to know details about how the servos are connected.  There may be several implementations
// of this class, such as one will use the SSC-32 to control the servos.  There may be 
// additional versions of this class that allows the main processor to control the servos.
//==============================================================================
#ifndef _Servo_Driver_h_
#define _Servo_Driver_h_

#include "Hex_Cfg.h"  // make sure we know what options are enabled...
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

class ServoDriver {
  public:
    void Init(void);
    
    word GetBatteryVoltage(void);

#ifdef OPT_GPPLAYER    
    inline boolean  FIsGPEnabled(void) {return _fGPEnabled;};
    boolean         FIsGPSeqDefined(uint8_t iSeq);
    inline boolean  FIsGPSeqActive(void) {return _fGPActive;};
    void            GPStartSeq(uint8_t iSeq);  // 0xff - says to abort...
    void            GPPlayer(void);
    uint8_t         GPNumSteps(void);          // How many steps does the current sequence have
    uint8_t         GPCurStep(void);           // Return which step currently on... 
    void            GPSetSpeedMultiplyer(short sm) ;      // Set the Speed multiplier (100 is default)
#endif
    void BeginServoUpdate(void);    // Start the update 
#ifdef c4DOF
    void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1);
#else
    void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1);
#endif    
    void CommitServoDriver(word wMoveTime);
    void FreeServos(void);
    
    // Allow for background process to happen...
#ifdef OPT_BACKGROUND_PROCESS
    void BackgroundProcess(void);
#endif
    
#ifdef OPT_TERMINAL_MONITOR  
    void ShowTerminalCommandList(void);
    boolean ProcessTerminalCommand(byte *psz, byte bLen);
#endif    

  private:
  
#ifdef OPT_GPPLAYER    
    boolean _fGPEnabled;     // IS GP defined for this servo driver?
    boolean _fGPActive;      // Is a sequence currently active - May change later when we integrate in sequence timing adjustment code
    uint8_t  _iSeq;          // current sequence we are running
    short    _sGPSM;        // Speed multiplier +-200 
#endif

} ;   

#endif //_Servo_Driver_h_
