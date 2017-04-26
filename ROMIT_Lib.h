/*
 * ROMIT_Lib.h
 *
 *  Created on: Jul 29, 2016
 *      Author: sjian
 */

#ifndef ROMIT_LIB_H_
#define ROMIT_LIB_H_
#include <SoftwareSerial.h>
#include <Arduino.h>
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

unsigned long int ConvertString2Int(String Magnet_ps);
void CAN_Send(uint16_t ID, uint8_t dt0, uint8_t dt1, uint8_t dt2, uint8_t dt3,
		uint8_t dt4, uint8_t dt5, uint8_t dt6, uint8_t dt7);
String CAN_Read();
String CAN_Read_from_Slave();
void setMyTimer4(long int frequency_A, long int frequency_B, long int frequency_C);

#endif /* ROMIT_LIB_H_ */
