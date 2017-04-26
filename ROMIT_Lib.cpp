/*
 * ROMIT_Lib.cpp
 *
 *  Created on: Jul 29, 2016
 *      Author: sjian
 */
#include <SoftwareSerial.h>
#include <avr/iom2560.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <Wire.h>
#include <TSYS01.h> //Temperature Sensor
#include <MS5837.h> //Pressure Sensor
#include "Canbus.h"
#include "defaults.h"
#include "global.h"
#include "mcp2515.h"
#include "mcp2515_defs.h"
#include "ROMIT_Lib.h"
#include "mcp_can.h"

//const int SPI_CS_PIN = 53;
//MCP_CAN CAN(SPI_CS_PIN);    // Set CS pin
extern MCP_CAN CAN;    // Set CS pin


// Send a CAN message
void CAN_Send(uint16_t ID, uint8_t dt0, uint8_t dt1, uint8_t dt2, uint8_t dt3,
		uint8_t dt4, uint8_t dt5, uint8_t dt6, uint8_t dt7){
    unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    stmp[0]=dt0;stmp[1]=dt1;stmp[2]=dt2;stmp[3]=dt3;
    stmp[4]=dt4;stmp[5]=dt5;stmp[6]=dt6;stmp[7]=dt7;
    CAN.sendMsgBuf(ID, 0, 8, stmp);
}


// Receive a CAN message
String CAN_Read() {
	unsigned char len = 0;
	unsigned char buf[8];
	String Receive_CAN;
	String Receive;
	unsigned int result_flag=0;
	String Receive_Master_Command;

	if(CAN.readMsgBuf(&len, buf)==CAN_OK)    // CANOK, read data,  len: data length, buf: data buf
	{
		unsigned int canId = CAN.getCanId();
		char str[23];
		unsigned char * pin = buf;
		const char * hex = "0123456789ABCDEF";
		char * pout = str;
		for(; pin < buf+sizeof(buf); pout+=3, pin++){
			pout[0] = hex[(*pin>>4) & 0xF];
			pout[1] = hex[ *pin     & 0xF];
			pout[2] = ':';
		}
		pout[-1] = 0;
		Receive_CAN = String(str);
		if(canId==0x0100){
			Receive="M"+Receive_CAN;
		}
		else if(canId==0x0090){
			Receive="S"+Receive_CAN;
		}
		return Receive;
	}
	else{
		return "";
	}

}


void setMyTimer4(long int frequency_A, long int frequency_B, long int frequency_C){
	cli(); // Disable all interrupts
	TCCR4A = 0;// set entire TCCR4A register to 0
	TCCR4B = 0;// same for TCCR4B
	TCNT4  = 0;//initialize counter value to 0
	// set compare match register for 8khz increments
	OCR4A = (16*10^6) / (frequency_A*1) - 1;// = (16*10^6) / (10000*1) - 1 (must be <65535)
	OCR4B = (16*10^6) / (frequency_B*1) - 1;// = (16*10^6) / (100000*1) - 1 (must be <65535)
	OCR4C = (16*10^6) / (frequency_C*1) - 1;// = (16*10^6) / (1000000*1) - 1 (must be <65535)
	TCCR4B |= (1 << WGM42); // turn on CTC mode
	TCCR4B |= (1 << CS40); // Set CS40 bit for 1 prescaler
	TIMSK4 |= (1 << OCIE4A);// enable timer compare interrupt
	TIMSK4 |= (1 << OCIE4B);// enable timer compare interrupt
	TIMSK4 |= (1 << OCIE4C);// enable timer compare interrupt
	sei();
}

void setMyTimer5(int frequency ){
	cli(); // Enable all interrupts
	TCCR5A = 0;// set entire TCCR5A register to 0
	TCCR5B = 0;// same for TCCR5B
	TCNT5  = 0;//initialize counter value to 0
	// set compare match register for 8khz increments
	OCR5A = (16*10^6) / (frequency*8) - 1;// = (16*10^6) / (10000*8) - 1 (must be <256)
	TCCR5B |= (1 << WGM52); // turn on CTC mode
	TCCR5B |= (1 << CS51); // Set CS51 bit for 8 prescaler
	TIMSK5 |= (1 << OCIE5A);// enable timer compare interrupt
	sei();
}

void setMyTimer3(int frequency ){
	cli(); // Enable all interrupts
	TCCR3A = 0;// set entire TCCR5A register to 0
	TCCR3B = 0;// same for TCCR5B
	TCNT3  = 0;//initialize counter value to 0
	// set compare match register for 8khz increments
	OCR3A = (16*10^6) / (frequency*8) - 1;// = (16*10^6) / (10000*8) - 1 (must be <256)
	TCCR3B |= (1 << WGM32); // turn on CTC mode
	TCCR3B |= (1 << CS31); // Set CS51 bit for 8 prescaler
	TIMSK3 |= (1 << OCIE3A);// enable timer compare interrupt
	sei();
}

void setMyTimer1(int frequency ){
	cli(); // Enable all interrupts
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	// set compare match register for 8khz increments
	OCR1A = (16*10^6) / (frequency*8) - 1;// = (16*10^6) / (10000*8) - 1 (must be <256)
	TCCR1B |= (1 << WGM12); // turn on CTC mode
	TCCR1B |= (1 << CS11); // Set CS11 bit for 8 prescaler
	TIMSK1 |= (1 << OCIE1A);// enable timer compare interrupt
	sei();
}


unsigned long int ConvertString2Int(String Magnet_ps){
	char Digits[7];
	unsigned long int Digits_int[6];
	unsigned long int Result;
	Magnet_ps.toCharArray(Digits,7,0);
	for(int i=0; i<6; i++){
		if((47<(int(Digits[i])))&&((int(Digits[i]))<58)){  // 47< x <58
			Digits_int[i]=int(Digits[i])-48;
		}
		else{
			Digits_int[i]=int(Digits[i])-65+10;
		}
	}
	Result=(Digits_int[0]*(1048576)+Digits_int[1]*65536+Digits_int[2]*4096+
			Digits_int[3]*256+Digits_int[4]*16+Digits_int[5]);
	return Result;
}


// A back up of the previous functions

/*
 *
void CAN_Send(uint16_t ID, uint8_t dt0, uint8_t dt1, uint8_t dt2, uint8_t dt3,
		uint8_t dt4, uint8_t dt5, uint8_t dt6, uint8_t dt7){

	tCAN message;
	message.id = ID; //formatted in HEX
	message.header.rtr = 0;
	message.header.length = 8; //formatted in DEC
	message.data[0] = dt0;
	message.data[1] = dt1;
	message.data[2] = dt2;
	message.data[3] = dt3; //formatted in HEX
	message.data[4] = dt4;
	message.data[5] = dt5;
	message.data[6] = dt6;
	message.data[7] = dt7;
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
	mcp2515_send_message(&message);
}



String CAN_Read() {
	tCAN message;
	unsigned char Receive_buffer[8];
	String Receive;
	String ID;

	if (mcp2515_get_message(&message)) {
		if(message.id==0x0100){
			//Serial.println("Got a Message!");
			for (int i=0; i<8; i++){
				Receive_buffer[i] = message.data[i];
			}
			//Convert int char to Hex String
			//target buffer should be large enough
			char str[23];
			unsigned char * pin = Receive_buffer;
			const char * hex = "0123456789ABCDEF";
			char * pout = str;
			for(; pin < Receive_buffer+sizeof(Receive_buffer); pout+=3, pin++){
				pout[0] = hex[(*pin>>4) & 0xF];
				pout[1] = hex[ *pin     & 0xF];
				pout[2] = ':';
			}
			pout[-1] = 0;
			Receive = String(str);
			Receive="M"+Receive;
		}
		else if(message.id==0x0090){
			//Serial.println("Got a Message!");
			for (int i=0; i<8; i++){
				Receive_buffer[i] = message.data[i];
			}
			//Convert int char to Hex String
			//target buffer should be large enough
			char str[23];
			unsigned char * pin = Receive_buffer;
			const char * hex = "0123456789ABCDEF";
			char * pout = str;
			for(; pin < Receive_buffer+sizeof(Receive_buffer); pout+=3, pin++){
				pout[0] = hex[(*pin>>4) & 0xF];
				pout[1] = hex[ *pin     & 0xF];
				pout[2] = ':';
			}
			pout[-1] = 0;
			Receive = String(str);
			Receive="S"+Receive;
		}
		else{

		}
	}
	else {
	}
	return Receive;
}


String CAN_Read_from_Slave() {
	tCAN message;
	unsigned char Receive_buffer[8];
	String Receive_from_Slave;
	String ID;

	if (mcp2515_get_message(&message)&&(message.id==96)){
		//Serial.println("Got a Message!");
		for (int i=0; i<8; i++){
			Receive_buffer[i] = message.data[i];
		}
		//Convert int char to Hex String
		//target buffer should be large enough
		char str[23];
		unsigned char * pin = Receive_buffer;
		const char * hex = "0123456789ABCDEF";
		char * pout = str;
		for(; pin < Receive_buffer+sizeof(Receive_buffer); pout+=3, pin++){
			pout[0] = hex[(*pin>>4) & 0xF];
			pout[1] = hex[ *pin     & 0xF];
			pout[2] = ':';
		}
		pout[-1] = 0;
		Receive_from_Slave = String(str);
	}
	return Receive_from_Slave;
}
*/
