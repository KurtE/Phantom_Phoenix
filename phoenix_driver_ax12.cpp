//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver - This version is setup to use the SSC-32 to control
// the servos.
//====================================================================
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#include <avr\pgmspace.h>
#endif
#include "Hex_Globals.h"
#include "ServoDriver.h"
#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif

#define NUMSERVOS (NUMSERVOSPERLEG*6)

#ifdef USE_AX12_DRIVER

#include <ax12.h>

#define USE_BIOLOIDEX            // Use the Bioloid code to control the AX12 servos...
#define USE_AX12_SPEED_CONTROL   // Experiment to see if the speed control works well enough...
boolean g_fAXSpeedControl;      // flag to know which way we are doing output...
#include "BioloidEx.h"


#ifdef USE_AX12_SPEED_CONTROL
// Current positions in AX coordinates
word      g_awCurAXPos[NUMSERVOS];
word      g_awGoalAXPos[NUMSERVOS];
#endif

#ifdef DBGSerial
// Only allow debug stuff to be turned on if we have a debug serial port to output to...
//#define DEBUG_SERVOS
#endif

#ifdef DEBUG_SERVOS
#define ServosEnabled   (g_fEnableServos)
#else
#define ServosEnabled  (true)      // always true compiler should remove if...
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
//Servo Pin numbers - May be SSC-32 or actual pins on main controller, depending on configuration.
static const byte cPinTable[] PROGMEM = {
  cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin, 
  cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin,
  cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin
#ifdef c4DOF
    ,cRRTarsPin, cRMTarsPin, cRFTarsPin, cLRTarsPin, cLMTarsPin, cLFTarsPin
#endif
};
#define FIRSTCOXAPIN     0
#define FIRSTFEMURPIN    6
#define FIRSTTIBIAPIN    12
#define FIRSTTARSPIN     18
// Not sure yet if I will use the controller class or not, but...
BioloidControllerEx bioloid = BioloidControllerEx(1000000);
boolean g_fServosFree;    // Are the servos in a free state?


// Some forward references
extern void MakeSureServosAreOn(void);
extern void DoPyPose(byte *psz);

//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
  // First lets get the actual servo positions for all of our servos...
  pinMode(0, OUTPUT);
  g_fServosFree = true;
  bioloid.poseSize = 18;
  bioloid.readPose();
#ifdef cVoltagePin  
  for (byte i=0; i < 8; i++)
    GetBatteryVoltage();  // init the voltage pin
#endif

  g_fAXSpeedControl = false;
}

//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating 
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef cVoltagePin  
word  g_awVoltages[8]={
  0,0,0,0,0,0,0,0};
word  g_wVoltageSum = 0;
byte  g_iVoltages = 0;

word ServoDriver::GetBatteryVoltage(void) {
  g_iVoltages = (++g_iVoltages)&0x7;  // setup index to our array...
  g_wVoltageSum -= g_awVoltages[g_iVoltages];
  g_awVoltages[g_iVoltages] = analogRead(cVoltagePin);
  g_wVoltageSum += g_awVoltages[g_iVoltages];

  return ((long)((long)g_wVoltageSum*125*(CVADR1+CVADR2))/(long)(2048*(long)CVADR2));  

}

#else
#define VOLTAGE_REPEAT_MAX  3
#define VOLTAGE_MAX_TIME_BETWEEN_CALLS 500    // call at least twice a second...

word g_wLastVoltage = 0xffff;    // save the last voltage we retrieved...

unsigned long g_ulTimeLastBatteryVoltage;

word ServoDriver::GetBatteryVoltage(void) {
  if (bioloid.interpolating && (g_wLastVoltage != 0xffff)  && ((millis()-g_ulTimeLastBatteryVoltage) < VOLTAGE_MAX_TIME_BETWEEN_CALLS))
    return g_wLastVoltage;

  register uint8_t bLoopCnt = VOLTAGE_REPEAT_MAX;
  do {
    register word wVoltage = ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1);
    if (wVoltage != 0xffff) {
      g_ulTimeLastBatteryVoltage = millis();
      g_wLastVoltage = wVoltage * 10;
      return g_wLastVoltage;
    }
  } 
  while (--bLoopCnt);

  return 0;

}
#endif

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
#ifdef OPT_GPPLAYER

//--------------------------------------------------------------------
//[FIsGPSeqDefined]
//--------------------------------------------------------------------
boolean ServoDriver::FIsGPSeqDefined(uint8_t iSeq)
{
  return false;  // nope return error
}


//--------------------------------------------------------------------
// Setup to start sequence number...
//--------------------------------------------------------------------
void ServoDriver::GPStartSeq(uint8_t iSeq)
{
  _fGPActive = true;
  _iSeq = iSeq;
}

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
void ServoDriver::GPPlayer(void)
{
}
#endif // OPT_GPPLAYER

//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void ServoDriver::BeginServoUpdate(void)    // Start the update 
{
  MakeSureServosAreOn();
  if (ServosEnabled) {
    DebugToggle(A4);
    if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // If we are trying our own Servo control need to save away the new positions...
      for (byte i=0; i < NUMSERVOS; i++) {
        g_awCurAXPos[i] = g_awGoalAXPos[i];
      }
#endif
    } 
    else       
      bioloid.interpolateStep(true);    // Make sure we call at least once

  }
}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#define cPwmMult      128
#define cPwmDiv       375  
#define cPFConst      512    // half of our 1024 range
#ifdef c4DOF
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1)
#else
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1)
#endif    
{        
  word    wCoxaSSCV;        // Coxa value in SSC units
  word    wFemurSSCV;        //
  word    wTibiaSSCV;        //
#ifdef c4DOF
  word    wTarsSSCV;        //
#endif

  //Update Right Legs
  g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
  if (LegIndex < 3) {
    wCoxaSSCV = (((long)(-sCoxaAngle1))* cPwmMult) / cPwmDiv +cPFConst;
    wFemurSSCV = (((long)(-sFemurAngle1))* cPwmMult) / cPwmDiv +cPFConst;
    wTibiaSSCV = (((long)(-sTibiaAngle1))* cPwmMult) / cPwmDiv +cPFConst;
#ifdef c4DOF
    wTarsSSCV = (((long)(-sTarsAngle1))* cPwmMult) / cPwmDiv +cPFConst;
#endif
  } 
  else {
    wCoxaSSCV = (((long)(sCoxaAngle1))* cPwmMult) / cPwmDiv +cPFConst;
    wFemurSSCV = (((long)((long)(sFemurAngle1))* cPwmMult) / cPwmDiv +cPFConst);
    wTibiaSSCV = (((long)(sTibiaAngle1))* cPwmMult) / cPwmDiv +cPFConst;
#ifdef c4DOF
    wTarsSSCV = (((long)(sTarsAngle1))* cPwmMult) / cPwmDiv +cPFConst;
#endif
  }
  if (ServosEnabled) {
    if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // Save away the new positions... 
      g_awGoalAXPos[FIRSTCOXAPIN+LegIndex] = wCoxaSSCV;    // What order should we store these values?
      g_awGoalAXPos[FIRSTFEMURPIN+LegIndex] = wFemurSSCV;    
      g_awGoalAXPos[FIRSTTIBIAPIN+LegIndex] = wTibiaSSCV;    
#ifdef c4DOF
      g_awGoalAXTarsPos[FIRSTTARSPIN+LegIndex] = wTarsSSCV;    
#endif
#endif
    }
    else {    
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTCOXAPIN+LegIndex]), wCoxaSSCV);
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTFEMURPIN+LegIndex]), wFemurSSCV);
      bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTIBIAPIN+LegIndex]), wTibiaSSCV);
#ifdef c4DOF
      if ((byte)pgm_read_byte(&cTarsLength[LegIndex]))   // We allow mix of 3 and 4 DOF legs...
        bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTARSPIN+LegIndex]), wTarsSSCV);
#endif
    }
  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput) {
    DBGSerial.print(LegIndex, DEC);
    DBGSerial.print("(");
    DBGSerial.print(sCoxaAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wCoxaSSCV, DEC);
    DBGSerial.print("),(");
    DBGSerial.print(sFemurAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wFemurSSCV, DEC);
    DBGSerial.print("),(");
    DBGSerial.print("(");
    DBGSerial.print(sTibiaAngle1, DEC);
    DBGSerial.print("=");
    DBGSerial.print(wTibiaSSCV, DEC);
    DBGSerial.print(") :");
  }
#endif
  g_InputController.AllowControllerInterrupts(true);    // Ok for hserial again...
}


//==============================================================================
// Calculate servo speeds to achieve desired pose timing
// We make the following assumptions:
// AX-12 speed is 59rpm @ 12V which corresponds to 0.170s/60deg
// The AX-12 manual states this as the 'no load speed' at 12V
// The Moving Speed control table entry states that 0x3FF = 114rpm
// and according to Robotis this means 0x212 = 59rpm and anything greater 0x212 is also 59rpm
#ifdef USE_AX12_SPEED_CONTROL
word CalculateAX12MoveSpeed(word wCurPos, word wGoalPos, word wTime)
{
  word wTravel;
  uint32_t factor;
  word wSpeed;
  // find the amount of travel for each servo
  if( wGoalPos > wCurPos) {
    wTravel = wGoalPos - wCurPos;
  } 
  else {
    wTravel = wCurPos - wGoalPos;
  }

  // now we can calculate the desired moving speed
  // for 59pm the factor is 847.46 which we round to 848
  // we need to use a temporary 32bit integer to prevent overflow
  factor = (uint32_t) 848 * wTravel;

  wSpeed = (uint16_t) ( factor / wTime );
  // if the desired speed exceeds the maximum, we need to adjust
  if (wSpeed > 1023) wSpeed = 1023;
  // we also use a minimum speed of 26 (5% of 530 the max value for 59RPM)
  if (wSpeed < 26) wSpeed = 26;

  return wSpeed;
} 
#endif


//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly 
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriver::CommitServoDriver(word wMoveTime)
{
#ifdef cSSC_BINARYMODE
  byte    abOut[3];
#endif

  g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
  if (ServosEnabled) {
    if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
      // Need to first output the header for the Sync Write
      int length = 4 + (NUMSERVOS * 5);   // 5 = id + pos(2byte) + speed(2 bytes)
      int checksum = 254 + length + AX_SYNC_WRITE + 4 + AX_GOAL_POSITION_L;
      word wSpeed;
      setTXall();
      ax12write(0xFF);
      ax12write(0xFF);
      ax12write(0xFE);
      ax12write(length);
      ax12write(AX_SYNC_WRITE);
      ax12write(AX_GOAL_POSITION_L);
      ax12write(4);    // number of bytes per servo (plus the ID...)
      for (int i = 0; i < NUMSERVOS; i++) {
        wSpeed = CalculateAX12MoveSpeed(g_awCurAXPos[i], g_awGoalAXPos[i], wMoveTime);    // What order should we store these values?
        byte id = pgm_read_byte(&cPinTable[i]);
        checksum += id + (g_awGoalAXPos[i]&0xff) + (g_awGoalAXPos[i]>>8) + (wSpeed>>8) + (wSpeed & 0xff);
        ax12write(id);
        ax12write(g_awGoalAXPos[i]&0xff);
        ax12write(g_awGoalAXPos[i]>>8);
        ax12write(wSpeed&0xff);
        ax12write(wSpeed>>8);

      }
      ax12write(0xff - (checksum % 256));
      setRX(0);

#endif
    }
    else {
      bioloid.interpolateSetup(wMoveTime);
    }

  }
#ifdef DEBUG_SERVOS
  if (g_fDebugOutput)
    DBGSerial.println(wMoveTime, DEC);
#endif
  g_InputController.AllowControllerInterrupts(true);    

}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void)
{
  if (ServosEnabled) {
    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    for (byte i = 0; i < NUMSERVOS; i++) {
      Relax(pgm_read_byte(&cPinTable[i]));
    }
    g_InputController.AllowControllerInterrupts(true);    
    g_fServosFree = true;
  }
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void MakeSureServosAreOn(void)
{
  if (ServosEnabled) {
    if (!g_fServosFree)
      return;    // we are not free

    g_InputController.AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
    if (g_fAXSpeedControl) {
      for(int i=0;i<NUMSERVOS;i++){
        g_awGoalAXPos[i] = ax12GetRegister(pgm_read_byte(&cPinTable[i]),AX_PRESENT_POSITION_L,2);
        delay(25);   
      }
    }
    else {
      bioloid.readPose();
    }

    for (byte i = 0; i < NUMSERVOS; i++) {
      TorqueOn(pgm_read_byte(&cPinTable[i]));
    }
    g_InputController.AllowControllerInterrupts(true);    
    g_fServosFree = false;
  }   
}

//==============================================================================
// BackgroundProcess - Allows us to have some background processing for those
//    servo drivers that need us to do things like polling...
//==============================================================================
void  ServoDriver::BackgroundProcess(void) 
{
  if (g_fAXSpeedControl) 
    return;  // nothing to do in this mode...

  if (ServosEnabled) {
    DebugToggle(A3);

    bioloid.interpolateStep(false);    // Do our background stuff...
  }
}


#ifdef OPT_TERMINAL_MONITOR  
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void ServoDriver::ShowTerminalCommandList(void) 
{
  DBGSerial.println(F("M - Toggle Motors on or off"));
  DBGSerial.println(F("F<frame length> - FL in ms"));    // BUGBUG:: 
  DBGSerial.println(F("A - Toggle AX12 speed control"));
#ifdef OPT_PYPOSE
  DBGSerial.println(F("P - Pypose"));
#endif
#ifdef OPT_FIND_SERVO_OFFSETS
  DBGSerial.println(F("O - Enter Servo offset mode"));
#endif        
#ifdef OPT_SSC_FORWARDER
  DBGSerial.println(F("S - SSC Forwarder"));
#endif        
}

//==============================================================================
// ProcessTerminalCommand: The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
boolean ServoDriver::ProcessTerminalCommand(byte *psz, byte bLen)
{
  if ((bLen == 1) && ((*psz == 'm') || (*psz == 'M'))) {
    g_fEnableServos = !g_fEnableServos;
    if (g_fEnableServos) 
      DBGSerial.println(F("Motors are on"));
    else
      DBGSerial.println(F("Motors are off"));

    return true;  
  } 

  if ((bLen == 1) && ((*psz == 'a') || (*psz == 'A'))) {
    g_fAXSpeedControl = !g_fAXSpeedControl;
    if (g_fAXSpeedControl) 
      DBGSerial.println(F("AX12 Speed Control"));
    else
      DBGSerial.println(F("Bioloid Speed"));
  }
  if ((bLen >= 1) && ((*psz == 'f') || (*psz == 'F'))) {
    psz++;  // need to get beyond the first character
    while (*psz == ' ') 
      psz++;  // ignore leading blanks...
    byte bFrame = 0;
    while ((*psz >= '0') && (*psz <= '9')) {  // Get the frame count...
      bFrame = bFrame*10 + *psz++ - '0';
    }
    if (bFrame != 0) {
      DBGSerial.print(F("New Servo Cycles per second: "));
      DBGSerial.println(1000/bFrame, DEC);
      extern BioloidControllerEx bioloid;
      bioloid.frameLength = bFrame;
    }
  } 

#ifdef OPT_FIND_SERVO_OFFSETS
  else if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
    FindServoOffsets();
  }
#endif
#ifdef OPT_SSC_FORWARDER
  else if ((bLen == 1) && ((*psz == 's') || (*psz == 'S'))) {
    SSCForwarder();
  }
#endif
#ifdef OPT_PYPOSE
  else if ((*psz == 'p') || (*psz == 'P')) {
    DoPyPose(++psz);
  }
#endif

}
#endif    


//==============================================================================
// SSC Forwarder - used to allow things like Lynxterm to talk to the SSC-32 
// through the Arduino...  Will see if it is fast enough...
//==============================================================================
#ifdef OPT_SSC_FORWARDER
void  SSCForwarder(void) 
{
}
#endif // OPT_SSC_FORWARDER





//==============================================================================
//	FindServoOffsets - Find the zero points for each of our servos... 
// 		Will use the new servo function to set the actual pwm rate and see
//		how well that works...
//==============================================================================
#ifdef OPT_FIND_SERVO_OFFSETS

void FindServoOffsets()
{

}
#endif  // OPT_FIND_SERVO_OFFSETS

//==============================================================================
// DoPyPose
//==============================================================================
#ifdef OPT_PYPOSE
#define ARB_SIZE_POSE   7  // also initializes
#define ARB_LOAD_POSE   8
#define ARB_LOAD_SEQ    9
#define ARB_PLAY_SEQ    10
#define ARB_LOOP_SEQ    11
#define ARB_TEST        25


void DoPyPose(byte *psz)
{
  int mode = 0;              // where we are in the frame

  unsigned char id = 0;      // id of this frame
  unsigned char length = 0;  // length of this frame
  unsigned char ins = 0;     // instruction of this frame

  unsigned char params[50];  // parameters
  unsigned char index = 0;   // index in param buffer

  int checksum;              // checksum

    typedef struct{
    unsigned char pose;    // index of pose to transition to 
    int time;              // time for transition
  } 
  sp_trans_t;

  //  pose and sequence storage
  int poses[30][30];         // poses [index][servo_id-1]
  sp_trans_t sequence[50];   // sequence
  int seqPos;                // step in current sequence
  word wMY;

  // See if the user gave us a string to process.  If so it should be the XBEE DL to talk to. 
  // BUGBUG:: if using our XBEE stuff could default to debug terminal... 
#ifdef USEXBEE
  if (psz) {
    word wDL;
    for (wDL=0; *psz; psz++) {
      if ((*psz >= '0') && (*psz <= '9'))
        wDL = (wDL << 4) + *psz - '0';
      if ((*psz >= 'a') && (*psz <= 'f'))
        wDL = (wDL << 4) + *psz - 'a' + 10;
      if ((*psz >= 'A') && (*psz <= 'F'))
        wDL = (wDL << 4) + *psz - 'A' + 10;
    }
    if (wDL)
      SetXBeeHexVal('D','L', wDL);
  }

#ifdef DBGSerial
  wMY = GetXBeeHVal('M', 'Y');

  DBGSerial.print(F("My: "));
  DBGSerial.println(wMY, HEX);  
  wMY = GetXBeeHVal('D', 'L');  // I know reused variable...
  DBGSerial.print(F("PC DL: "));
  DBGSerial.println(wMY, HEX);
  DBGSerial.println(F("Exit Terminal and Start Pypose"));
#endif

  // Also lets exit from API Mode back to text mode...
  SetXBeeHexVal('G','T', 1000);  // Set the default Guard time
  SetXBeeHexVal('A','P', 0);    // Exit API mode...
  delay(2000);
  while(Serial.read() != -1)
    ;

#endif    

  // process messages
  for (;;) {
    while(Serial.available() > 0){
      // We need to 0xFF at start of packet
      if(mode == 0){         // start of new packet
        if(Serial.read() == 0xff){
          mode = 2;
          digitalWrite(0,HIGH-digitalRead(0));
        }
        //}else if(mode == 1){   // another start byte
        //    if(Serial.read() == 0xff)
        //        mode = 2;
        //    else
        //        mode = 0;
      }
      else if(mode == 2){   // next byte is index of servo
        id = Serial.read();    
        if(id != 0xff)
          mode = 3;
      }
      else if(mode == 3){   // next byte is length
        length = Serial.read();
        checksum = id + length;
        mode = 4;
      }
      else if(mode == 4){   // next byte is instruction
        ins = Serial.read();
        checksum += ins;
        index = 0;
        mode = 5;
      }
      else if(mode == 5){   // read data in 
        params[index] = Serial.read();
        checksum += (int) params[index];
        index++;
        if(index + 1 == length){  // we've read params & checksum
          mode = 0;
          if((checksum%256) != 255){ 
            // return a packet: FF FF id Len Err params=None check
            Serial.write((byte)0xff);
            Serial.write((byte)0xff);
            Serial.write((byte)id);
            Serial.write((byte)2);
            Serial.write((byte)64);
            Serial.write((byte)(255-((66+id)%256)));
          }
          else{
            if(id == 253){
              // return a packet: FF FF id Len Err params=None check
              Serial.write((byte)0xff);
              Serial.write((byte)0xff);
              Serial.write((byte)id);
              Serial.write((byte)2);
              Serial.write((byte)0);
              Serial.write((byte)(255-((2+id)%256)));
              // special ArbotiX instructions
              // Pose Size = 7, followed by single param: size of pose
              // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size)
              // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size) 
              // Play Seq = A, no params
              if(ins == ARB_SIZE_POSE){
                bioloid.poseSize = params[0];
                bioloid.readPose();    
                //Serial.println(bioloid.poseSize);
              }
              else if(ins == ARB_LOAD_POSE){
                int i;    
                Serial.print("New Pose:");
                for(i=0; i<bioloid.poseSize; i++){
                  poses[params[0]][i] = params[(2*i)+1]+(params[(2*i)+2]<<8); 
                  //Serial.print(poses[params[0]][i]);
                  //Serial.print(",");     
                } 
                Serial.println("");
              }
              else if(ins == ARB_LOAD_SEQ){
                int i;
                for(i=0;i<(length-2)/3;i++){
                  sequence[i].pose = params[(i*3)];
                  sequence[i].time = params[(i*3)+1] + (params[(i*3)+2]<<8);
                  //Serial.print("New Transition:");
                  //Serial.print((int)sequence[i].pose);
                  //Serial.print(" in ");
                  //Serial.println(sequence[i].time);      
                }
              }
              else if(ins == ARB_PLAY_SEQ){
                seqPos = 0;
                while(sequence[seqPos].pose != 0xff){
                  int i;
                  int p = sequence[seqPos].pose;
                  // are we HALT?
                  if(Serial.read() == 'H') return;
                  // load pose
                  for(i=0; i<bioloid.poseSize; i++){
                    bioloid.setNextPose(i+1,poses[p][i]);
                  } 
                  // interpolate
                  bioloid.interpolateSetup(sequence[seqPos].time);
                  while(bioloid.interpolating)
                    bioloid.interpolateStep();
                  // next transition
                  seqPos++;
                }
              }
              else if(ins == ARB_LOOP_SEQ){
                while(1){
                  seqPos = 0;
                  while(sequence[seqPos].pose != 0xff){
                    int i;
                    int p = sequence[seqPos].pose;
                    // are we HALT?
                    if(Serial.read() == 'H') return;
                    // load pose
                    for(i=0; i<bioloid.poseSize; i++){
                      bioloid.setNextPose(i+1,poses[p][i]);
                    } 
                    // interpolate
                    bioloid.interpolateSetup(sequence[seqPos].time);
                    while(bioloid.interpolating)
                      bioloid.interpolateStep();
                    // next transition
                    seqPos++;
                  }
                }
              }
              else if(ins == ARB_TEST){
                int i;
                // Test Digital I/O
                for(i=0;i<8;i++){
                  // test digital
                  pinMode(i,OUTPUT);
                  digitalWrite(i,HIGH);  
                  // test analog
                  pinMode(31-i,OUTPUT);
                  digitalWrite(31-i,HIGH);

                  delay(500);
                  digitalWrite(i,LOW);
                  digitalWrite(31-i,LOW);
                }
                // Test Ax-12
                for(i=452;i<552;i+=20){
                  SetPosition(1,i);
                  delay(200);
                }
                delay(1500);
                // Test Analog I/O
                for(i=0;i<8;i++){
                  // test digital
                  pinMode(i,OUTPUT);
                  digitalWrite(i,HIGH);  
                  // test analog
                  pinMode(31-i,OUTPUT);
                  digitalWrite(31-i,HIGH);

                  delay(500);
                  digitalWrite(i,LOW);
                  digitalWrite(31-i,LOW);
                }
              }   
            }
            else{
              int i;
              // pass thru
              if(ins == AX_READ_DATA){
                int i;
                ax12GetRegister(id, params[0], params[1]);
                // return a packet: FF FF id Len Err params check
                if(ax_rx_buffer[3] > 0){
                  for(i=0;i<ax_rx_buffer[3]+4;i++)
                    Serial.write(ax_rx_buffer[i]);
                }
                ax_rx_buffer[3] = 0;
              }
              else if(ins == AX_WRITE_DATA){
                if(length == 4){
                  ax12SetRegister(id, params[0], params[1]);
                }
                else{
                  int x = params[1] + (params[2]<<8);
                  ax12SetRegister2(id, params[0], x);
                }
                // return a packet: FF FF id Len Err params check
                Serial.write((byte)0xff);
                Serial.write((byte)0xff);
                Serial.write((byte)id);
                Serial.write((byte)2);
                Serial.write((byte)0);
                Serial.write((byte)(255-((2+id)%256)));
              }
            }
          }
        }
      }
    }

    // update joints
    bioloid.interpolateStep();
  }
}

#endif



#endif
















