/*
  ax12.cpp - ArbotiX library for AX/RX control.
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
//#define DEBUG_HARDWARE_SERIAL

#include "_ax12Serial.h"
#include <avr/io.h>

#ifdef DEBUG_HARDWARE_SERIAL
#define DebugDigitalWrite(pin, state) digitalWrite(pin, state)
#define DebugDigitalTogle(pin) digitalWrite(pin, !digitalRead(pin))
#else
#define DebugDigitalWrite(pin, state) 
#define DebugDigitalTogle(pin) 
#endif


unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];

#ifndef USE_HARDWARE_SERIAL_WRITE
uint8_t  g_fDataOutput = false;
#endif

/** initializes serial1 transmit at baud, 8-N-1 */
void ax12Init(long baud){
    // Need to enable the PU resistor on the TX pin
#if AX12Serial == Serial1
    pinMode(18, INPUT_PULLUP);
#elif AX12Serial == Serial2
    pinMode(16, INPUT_PULLUP);
#elif AX12Serial == Serial3
    pinMode(14, INPUT_PULLUP);
#endif
    AX12Serial.begin(baud);
  
    // DEBUG
#ifdef DEBUG_HARDWARE_SERIAL    
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
#endif    
    setRX(0);    
}

/** helper functions to switch direction of comms */
void setTX(int id){
    setTXall();
}

void setTXall(){
#if AX12Serial == Serial1
    UCSR1B = (1 << TXEN1);
#elif AX12Serial == Serial2
    UCSR2B = (1 << TXEN2);
#elif AX12Serial == Serial3
    UCSR3B =  (1 << TXEN3);
#endif
}

void flushAX12InputBuffer(void)  {
    // First lets clear out any RX bytes that may be lingering in our queue
    while (AX12Serial.available()) {
        AX12Serial.read();   
    }
}

void setRX(int id){ 
    DebugDigitalWrite(4, HIGH);
    // First clear our input buffer
    flushAX12InputBuffer();
    DebugDigitalWrite(4, LOW);
    
    // Now wait for any pending outputs to fully transmit
    DebugDigitalWrite(5, HIGH);
#ifdef USE_HARDWARE_SERIAL_WRITE
    AX12Serial.flush();
#else
    // Roll our own flush, wait for TX to complete...
    if (g_fDataOutput) {
	uint8_t	bTimeout = 0xff;  // probably not needed as we check for data output before...
#if AX12Serial == Serial1 
	do { } while (bTimeout-- && bit_is_clear(UCSR1A, TXC1));
#elif AX12Serial == Serial2
	do { } while (bTimeout-- && bit_is_clear(UCSR2A, TXC2));
#elif AX12Serial == Serial3
	do { } while (bTimeout-- && bit_is_clear(UCSR3A, TXC3));
#endif

      g_fDataOutput = false;
    }
#endif
    DebugDigitalWrite(5, LOW);
    
    // Now setup to enable the RX and disable the TX
#if AX12Serial == Serial1 
    UCSR1B = ((1 << RXCIE1) | (1 << RXEN1));
#elif AX12Serial == Serial2
    UCSR2B = ((1 << RXCIE2) | (1 << RXEN2);
#elif AX12Serial == Serial3
    UCSR3B = ((1 << RXCIE3) | (1 << RXEN3));
#endif
}


/** Sends a character out the serial port. */
void ax12write(unsigned char data){
#ifdef USE_HARDWARE_SERIAL_WRITE
  AX12Serial.write(data);
#else
// We are rolling our own to handle hang on unmodified hardware serial.
#if AX12Serial == Serial1
    loop_until_bit_is_set(UCSR1A, UDRE1); //wait until the TX data registry can accept new data
    // Load the next byte from the USART transmit buffer into the USART
    UDR1 = data;            // transmit data
#elif AX12Serial == Serial2
    loop_until_bit_is_set(UCSR2A, UDRE2); //wait until the TX data registry can accept new data
    // Load the next byte from the USART transmit buffer into the USART
    UDR2 = data;            // transmit data
#elif AX12Serial == Serial3
    loop_until_bit_is_set(UCSR3A, UDRE3); //wait until the TX data registry can accept new data
    // Load the next byte from the USART transmit buffer into the USART
    UDR3 = data;            // transmit data
#endif
    g_fDataOutput = true;
#endif
}

/** Sends a character out the serial port, and puts it in the tx_buffer */
void ax12writeB(unsigned char data){
    AX12Serial.write(data);
}
/** We have a one-way recieve buffer, which is reset after each packet is receieved.
    A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps. */

/** read back the error code for our latest packet read */
int ax12Error;
int ax12GetLastError(){ return ax12Error; }
/** > 0 = success */

#define COUNTER_TIMEOUT 3000
int ax12ReadPacket(int length){
    unsigned long ulCounter;
    unsigned char offset, checksum;
    unsigned char volatile bcount; 
	unsigned char *psz; 
	unsigned char *pszEnd;
    int ch;
    
    DebugDigitalWrite(2, HIGH);
    offset = 0;
    bcount = 0;
	
	psz = ax_rx_buffer;
	pszEnd = &ax_rx_buffer[length];
#ifdef DEBUG
	pinMode(A4, OUTPUT);
#endif
	
    flushAX12InputBuffer();
	
	// Need to wait for a character or a timeout...
	do {
		ulCounter = COUNTER_TIMEOUT;
        while ((ch = AX12Serial.read()) == -1) {
			if (!--ulCounter) {
#ifdef DEBUG
				DebugDigitalTogle(A4);
#endif
                DebugDigitalTogle(3);
                DebugDigitalWrite(2, LOW);
				return 0;		// Timeout
			}
		}
	} while (ch != 0xff) ;
	*psz++ = 0xff;
#ifdef DEBUG
	DebugDigitalTogle(A4);
#endif	
	while (psz != pszEnd) {
		ulCounter = COUNTER_TIMEOUT;
        while ((ch = AX12Serial.read()) == -1) {
			if (!--ulCounter)  {
                DebugDigitalTogle(3);
                DebugDigitalWrite(2, LOW);
				return 0;		// Timeout
			}
		}
		*psz++ = (unsigned char)ch;
	}
    checksum = 0;
    DebugDigitalWrite(2, LOW);
    for(offset=2;offset<length;offset++)
        checksum += ax_rx_buffer[offset];
    if(checksum != 255){
#ifdef DEBUG
		pinMode(A5, OUTPUT);
		DebugDigitalTogle(A5);
		Serial.println("");
		for(offset=0;offset<length;offset++) {
			Serial.print(ax_rx_buffer[offset], HEX);
			Serial.print(" ");
		}
		Serial.println("");
#endif		
        DebugDigitalTogle(3);
        return 0;
    }else{
        return 1;
    }
}


/******************************************************************************
 * Packet Level
 */

/** Read register value(s) */
int ax12GetRegister(int id, int regstart, int length){  
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 6 + regstart + length)%256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_READ_DATA);
    ax12writeB(regstart);
    ax12writeB(length);
    ax12writeB(checksum);  
	
    setRX(id);    
    if(ax12ReadPacket(length + 6) > 0){
        ax12Error = ax_rx_buffer[4];
        if(length == 1)
            return ax_rx_buffer[5];
        else
            return ax_rx_buffer[5] + (ax_rx_buffer[6]<<8);
    }else{
        return -1;
    }
}

/* Set the value of a single-byte register. */
void ax12SetRegister(int id, int regstart, int data){
    setTX(id);    
    int checksum = ~((id + 4 + AX_WRITE_DATA + regstart + (data&0xff)) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_WRITE_DATA);
    ax12writeB(regstart);
    ax12writeB(data&0xff);
    // checksum = 
    ax12writeB(checksum);
    setRX(id);
    //ax12ReadPacket();
}
/* Set the value of a double-byte register. */
void ax12SetRegister2(int id, int regstart, int data){
    setTX(id);    
    int checksum = ~((id + 5 + AX_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(5);    // length
    ax12writeB(AX_WRITE_DATA);
    ax12writeB(regstart);
    ax12writeB(data&0xff);
    ax12writeB((data&0xff00)>>8);
    // checksum = 
    ax12writeB(checksum);
    setRX(id);
    //ax12ReadPacket();
}


