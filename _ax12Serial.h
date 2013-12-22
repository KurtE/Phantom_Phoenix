/*
  ax12.h - ArbotiX library for AX/RX control.
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h> 

#ifndef ax12_h
#define ax12_h

#define AX12_MAX_SERVOS             30
#define AX12_BUFFER_SIZE            32
//#define ARBOTIX_1280
/** Configuration **/

#define AX12Serial Serial1

#ifndef AX12Serial
#if defined(UBRR3H)
#define AX12Serial Serial3
#else
#define AX12Serial Serial1
#endif
#endif

/** EEPROM AREA **/
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23
/** RAM AREA **/
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49

/** Status Return Levels **/
#define AX_RETURN_NONE              0
#define AX_RETURN_READ              1
#define AX_RETURN_ALL               2

/** Instruction Set **/
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131

/** Error Levels **/
#define ERR_NONE                    0
#define ERR_VOLTAGE                 1
#define ERR_ANGLE_LIMIT             2
#define ERR_OVERHEATING             4
#define ERR_RANGE                   8
#define ERR_CHECKSUM                16
#define ERR_OVERLOAD                32
#define ERR_INSTRUCTION             64

/** AX-S1 **/
#define AX_LEFT_IR_DATA             26
#define AX_CENTER_IR_DATA           27
#define AX_RIGHT_IR_DATA            28
#define AX_LEFT_LUMINOSITY          29
#define AX_CENTER_LUMINOSITY        30
#define AX_RIGHT_LUMINOSITY         31
#define AX_OBSTACLE_DETECTION       32
#define AX_BUZZER_INDEX             40

void ax12Init(long baud);

void setTXall();     // for sync write
void setTX(int id);
void setRX(int id);

void ax12write(unsigned char data);
void ax12writeB(unsigned char data);

int ax12ReadPacket(int length);
int ax12GetRegister(int id, int regstart, int length);
void ax12SetRegister(int id, int regstart, int data);
void ax12SetRegister2(int id, int regstart, int data);
int ax12GetLastError();

extern unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
extern unsigned char ax_tx_buffer[AX12_BUFFER_SIZE];
extern unsigned char ax_rx_int_buffer[AX12_BUFFER_SIZE];
#if defined(AX_RX_SWITCHED)
// Need to stow type of servo (which bus it's on)
extern unsigned char dynamixel_bus_config[AX12_MAX_SERVOS];
#endif

#define SetPosition(id, pos) (ax12SetRegister2(id, AX_GOAL_POSITION_L, pos))
#define GetPosition(id) (ax12GetRegister(id, AX_PRESENT_POSITION_L, 2))
#define TorqueOn(id) (ax12SetRegister(id, AX_TORQUE_ENABLE, 1))
#define Relax(id) (ax12SetRegister(id, AX_TORQUE_ENABLE, 0))

#define GetLeftIRData(id) (ax12GetRegister(id, AX_LEFT_IR_DATA))
#define GetCenterIRData(id) (ax12GetRegister(id, AX_CENTER_IR_DATA))
#define GetRightIRData(id) (ax12GetRegister(id, AX_RIGHT_IR_DATA))
#define GetObstacles(id) (ax12GetRegister(id, OBSTACLE_DETECTION))

#define PlayTone(id, note) (ax12SetRegister(id, AX_BUZZER_INDEX, note))

#endif

