#include <SoftwareSerial.h>
#include <avr/iom2560.h>
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
#include "ROMIT_Lib.h"
#include <math.h>
#include <Arduino.h>

#include "mcp_can.h"

//Setups Initial the temperature and pressure sensor
TSYS01 Temperature_sensor;
MS5837 Pressure_sensor;

//#define SSerialRX        A14 //Serial Receive pin
//#define SSerialTX        A15 //Serial Transmit pin
#define SSerialTxControl 43
#define RS485Transmit    HIGH
#define RS485Receive     LOW

//#define Pin8LED           8
#define PinFrontLight       23
#define PinFrontLight2      25
#define Driving_Forward     27
#define Driving_Backward    37
#define Ram_Squeeze         35
#define Ram_Release         33
#define Relay7          31
#define Relay8          26

#define Tilt_x_Input    A8 //Blue wire on Inclinometer
#define Tilt_y_Input    A9 //Yellow wire on Inclinometer

#define CP_Probe        A10

String PC_Command="";
char pc_cmd_temp[8]={};

char    character;
int     X_IntValue;    // Holds the integer 10 bit value read in
int     X_degree;     // max 920=180 degree, minimum 100=0 degree
int     X_arc_minute;
int     Y_IntValue;    // Holds the integer 10 bit value read in
int     Y_degree;     // max 920=180 degree, minimum 100=0 degree
int     Y_arc_minute;
int     byteReceived;

int X_IntValue_offset; //Hold the X axis offset
int Y_IntValue_offset; //Hold the Y axis offset

String  stringReceived;

int     case_number;
int     Loop_count=0;
float   CP_output=0;
String CP_output_string;

int position1=0;//, position2=0, position3=0, position4=0;
int Link_Number_Count=0;

String CAN_Read_message_1_new="01:02:00:00:00:00:00:00"; //String CAN_Read_message_2_new="02:02:00:00:00:00:00:00";
String Command_Read_from_Slave="01:02:00:00:00:00:00:00";
String CAN_temp_new="01:02:00:00:00:00:00:00";
String CAN_Read_message_1_old="01:02:00:00:00:00:00:00"; //String CAN_Read_message_2_old="02:02:00:00:00:00:00:00";
String CAN_temp_old="01:02:00:00:00:00:00:00";
String CAN_Read_id;
String Magnets_positions="";


const int SPI_CS_PIN = 53;
MCP_CAN CAN(SPI_CS_PIN);    // Set CS pin

//SoftwareSerial Master_Serial(SSerialRX, SSerialTX);
void Read_and_Print_Sensors(int Loop_count, int Link_number);
String CAN_send2GUI();

void setup()
{
	unsigned char ii=0;

    // Set the default control states for the relays
    pinMode(PinFrontLight, OUTPUT); digitalWrite(PinFrontLight,HIGH); delay(2);
    pinMode(PinFrontLight2, OUTPUT); digitalWrite(PinFrontLight2,HIGH); delay(2);
    pinMode(Driving_Forward, OUTPUT); digitalWrite(Driving_Forward,HIGH); delay(2);
    pinMode(Driving_Backward, OUTPUT); digitalWrite(Driving_Backward,HIGH); delay(2);
    pinMode(Ram_Squeeze, OUTPUT); digitalWrite(Ram_Squeeze,HIGH); delay(2);
    pinMode(Ram_Release, OUTPUT); digitalWrite(Ram_Release,HIGH); delay(2);
    pinMode(Relay7, OUTPUT); digitalWrite(Relay7,HIGH); delay(2);
    pinMode(Relay8, OUTPUT); digitalWrite(Relay8,HIGH); delay(2);


    // Communicate with PC through USB connection
    Serial.begin(115200);delay(5);
    Serial.println("Begin Serial ROMIT CONNECTED");

    // The PIN 43 is used for enabling the RS485 communication using Serial3
    // SSerialTxControl-High, Transmit
    // SSerialTxControl-Low, Receive
    // Transmit a message to PC through RS485 connection
    pinMode(SSerialTxControl, OUTPUT);
    digitalWrite(SSerialTxControl, RS485Transmit); delay(5); // Init Transceiver
    Serial3.end();delay(10);
    Serial3.begin(115200);delay(10);
    Serial3.setTimeout(1000); delay(5);// set the data rate
    Serial3.println("ROMIT <--> PC");delay(10);
    Serial.println("ROMIT <--> PC");delay(10);
    //Master_Serial.println("RMTSTART");delay(5);  // Mst ask Slv to start Beeper
    Serial3.flush(); delay(5);
    digitalWrite(SSerialTxControl, RS485Receive); delay(5); // Enable receive

    //I2C configuration
    Wire.begin();
    Temperature_sensor.init();
    Pressure_sensor.init();
    Pressure_sensor.setFluidDensity(1029);// kg/m^3 (997 freshwater, 1029 for seawater)

    //CAN bus initialization
    while (CAN_OK != CAN.begin(CAN_1000KBPS))
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
    /*
    //We reconfigure the CAN bus rate to 1 Mbps, previously it is 500 kbps
    if(Canbus.init(CANSPEED_500))  // Initialise MCP2515 CAN controller at the specified speed
    {
        Serial.println("YES CAN!");delay(5);
    } else
    {
        Serial.println("NO CAN");delay(5);
    } delay(5);
    */
    //CAN_Send(0x07E5,0x02,0x90,0x76,0x39,0x92,0x01,0x00,0x00); delay(10);
    //CAN_Send(0x07EA,0x01,0x08,0x11,0x00,0x00,0x00,0x00,0x00); delay(10); // Synchronize to 'Node Start' (0x0000,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00)
    //CAN_Send(0x07EA,0x01,0x08,0x01,0x00,0x00,0x00,0x00,0x00); delay(10); // Free Running Mode
    //This is for starting the MTS sensor

    CAN_Send(0x0000,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00);

    Serial.flush();

    Serial3.flush();

    // Add the routine for the calibration of the tilt sensor may need level the vehicle to do the calibration before deployment
    for(ii=0;ii<20;ii++)
    {
    	X_IntValue += analogRead(Tilt_x_Input);delay(1);//Read value
    	Y_IntValue += analogRead(Tilt_y_Input);delay(1);//Read value
    }
    X_IntValue_offset=int(2.5/5*1023-X_IntValue/20);
    Y_IntValue_offset=int(2.5/5*1023-X_IntValue/20);

    //If we can not level the vehicle before testing, we can use the predefined values
    //Serial.println(X_IntValue_offset);
    //Serial.println(Y_IntValue_offset);
    X_IntValue_offset=15;
	Y_IntValue_offset=15;

}

void(* resetFunc) (void) = 0;//declare reset function at address 0

void loop() {
	Serial.println("Start waiting...");

	//Check if there is command from the MTS sensor or slave controller
	//Command_Read_from_Slave=CAN_send2GUI(); // The result will be message_slave or "0" for MTS flag.

	if(Command_Read_from_Slave!="0" && Command_Read_from_Slave!=""){
        Serial.print("CAN Message from slave: ");
        Serial.println(Command_Read_from_Slave);
    }

	// Serial3 Timeout has been set as 1 second, we are waiting for command from the PC GUI
    digitalWrite(SSerialTxControl, RS485Receive);delay(1);
    Serial3.readBytes(pc_cmd_temp,8);
    PC_Command=String(pc_cmd_temp);

    if(PC_Command.length()>0){ //PC_Command!=""
        Serial.print("PC_Command is: ");Serial.println(PC_Command);
    }else{
    	Serial.print("No PC Command yet.");
    	Serial.print("\n\r");
    }

    if ((PC_Command.length()==8)&&(PC_Command.substring(0,3)=="LED")){
        case_number=1;}
    else if((PC_Command.length()==8)&&(PC_Command=="CODESHUT")){
        case_number=2;}
    else if((PC_Command.length()==8)&&(PC_Command=="REBOOTIT")){
        case_number=3;}
    else if((PC_Command.length()==8)&&(PC_Command.substring(0,4)=="CLPT")){
        case_number=4;}
    else if((PC_Command.length()==8)&&(PC_Command.substring(0,4)=="MOVE")){
        case_number=5;}
    else if(Command_Read_from_Slave.substring(0,8)=="01:03:05"){
        case_number=9;}
    else if(PC_Command==""){
        case_number=10;}
    else{
        case_number=0;}

    Serial.print(case_number);

    switch(case_number){
    case 0:
    	break;
    case 1:
        if(PC_Command=="LEDONNOW"){
            digitalWrite(PinFrontLight,LOW);
            //delay(3000);
            digitalWrite(PinFrontLight2,LOW);
            //delay(3000);
            Serial.print("USER Command Received:  "); Serial.println(PC_Command);delay(1);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("LED IS ON!! ");delay(2);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            //Master_Serial.println("SLLEDONN");delay(5); // Give Beep
            //digitalWrite(Pin8LED,HIGH); delay(1);
            //CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
            break;
        }
        else if(PC_Command=="LEDCLOSE"){
            digitalWrite(PinFrontLight,HIGH);
            digitalWrite(PinFrontLight2,HIGH);
            Serial.print("USER Command Received:  "); Serial.println(PC_Command);delay(5);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("LED IS OFF!!"); delay(2);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            //Master_Serial.println("SLLEDOFF");delay(5); // Give Beep
            //digitalWrite(Pin8LED,LOW);delay(1);
            //CAN_Send(0x0080,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x00);
            break;
        }
        else if(PC_Command=="LEDLFTON"){
            digitalWrite(PinFrontLight,LOW);
            digitalWrite(PinFrontLight2,HIGH);
            Serial.print("USER Command Received:  "); Serial.println(PC_Command);delay(1);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("LED LEFT ON ");delay(2);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            //Master_Serial.println("SLLEDONN");delay(5); // Give Beep
            //digitalWrite(Pin8LED,HIGH); delay(1);
            break;
        }
        else if((PC_Command=="LEDRGTON")){
            digitalWrite(PinFrontLight,HIGH);
            digitalWrite(PinFrontLight2,LOW);
            Serial.print("USER Command Received:  "); Serial.println(PC_Command);delay(5);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("LED RIGHT ON"); delay(2);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            //Master_Serial.println("SLLEDOFF");delay(5); // Give Beep
            //digitalWrite(Pin8LED,LOW);delay(1);
            break;
        }
        else{
            break;
        }
    case 2:
        //Master_Serial.println("RMTSHUTD");delay(5);  // Mst ask Slv to start Beeper
        digitalWrite(PinFrontLight,HIGH);
        digitalWrite(PinFrontLight2,HIGH);
        //CAN_Send(0x0000,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00);
        CAN_Send(0x0080,0x02,0x04,0x06,0x00,0x00,0x00,0x00,0x00);// Master2Slave Termination command 2 4 6 0 0 0 0 0

        digitalWrite(SSerialTxControl, RS485Transmit);//delay(5);
        Serial3.println("TERMINATION!");delay(1);
        Serial.println("Master Termination");
        digitalWrite(SSerialTxControl, RS485Receive);//delay(5);
        PC_Command="";

        while(1){
            delay(500);
            while ((Serial3.available())&&PC_Command.length()<8)
            {
                byteReceived = Serial3.read(); // Receive a single character from the software serial port
                character=char(byteReceived);//delay(10);
                PC_Command=PC_Command+(character);//delay(10);
            }
            if((PC_Command.length()==8)&&(PC_Command=="REBOOTIT")){
                CAN_Send(0x0080,0x02,0x04,0x06,0x01,0x01,0x01,0x01,0x01);// Master2Slave Reboot command 2 4 6 1 1 1 1 1
                Serial.println("Master Reboot from Terminaltion");
                Link_Number_Count=0;
                resetFunc(); }//call reset
            else{
                PC_Command="";
                if (PC_Command.length()==0){}else{
                    Serial.println(PC_Command);
                }
                delay(500);
            }
        }
        break;
    case 3:
        CAN_Send(0x0080,0x02,0x04,0x06,0x01,0x01,0x01,0x01,0x01);// Master2Slave Reboot command 2 4 6 1 1 1 1 1
        Serial.println("Master Reboot");
        Link_Number_Count=0;
        resetFunc();
        break;
    case 4:
        if(PC_Command=="CLPTINIT"){
            Serial.print("USER Command Received:  "); Serial.println(PC_Command);//delay(5);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TCLP INITIAL"); delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);

            CAN_Send(0x0080,0x02,0x04,0x06,0x01,0x02,0x03,0x04,0x05);// Master2Slave Initial command 2 4 6 1 2 3 4 5
            //while({ // Slave command 1356789A
                //Command_Read_from_Slave=CAN_Read_from_Slave();
                //Serial.println(Command_Read_from_Slave);
            //}
            break;
        }
        else if(PC_Command=="CLPTPOS1"){
            Serial.print("USER Command Received:  "); Serial.println(PC_Command);//delay(5);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TCLP1-->PS#1"); delay(1);  // Move to position #2
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);

            CAN_Send(0x0080,0x02,0x04,0x06,0x00,0x00,0x01,0x00,0x00);// Master2Slave Position 1 command 2 4 6 0 0 1 0 0
            //while({ // Slave command 1356789A
                //Command_Read_from_Slave=CAN_Read_from_Slave();
                //Serial.println(Command_Read_from_Slave);
            //}
            break;
        }
        else if(PC_Command=="CLPTPOS2"){
            Serial.print("USER Command Received:  "); Serial.println(PC_Command);//delay(5);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TCLP2-->PS#2"); delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);

            CAN_Send(0x0080,0x02,0x04,0x06,0x00,0x00,0x02,0x00,0x00);// Master2Slave Position 2 command 2 4 6 0 0 2 0 0
            //while({ // Slave command 1356789A
                //Command_Read_from_Slave=CAN_Read_from_Slave();
                //Serial.println(Command_Read_from_Slave);
            //}
            break;
        }
        else if(PC_Command=="CLPTPOS3"){
            Serial.print("USER Command Received:  "); Serial.println(PC_Command);//delay(5);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TCLP3-->PS#3"); delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);

            CAN_Send(0x0080,0x02,0x04,0x06,0x00,0x00,0x03,0x00,0x00);// Master2Slave Position 3 command 2 4 6 0 0 3 0 0

            //while({ // Slave command 1356789A
                //Command_Read_from_Slave=CAN_Read_from_Slave();
                //Serial.println(Command_Read_from_Slave);
            //}
            break;
        }
        else if(PC_Command=="CLPTPOS4"){
            Serial.print("USER Command Received:  "); Serial.println(PC_Command);//delay(5);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TCLP4-->PS#4"); delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);

            CAN_Send(0x0080,0x02,0x04,0x06,0x00,0x00,0x04,0x00,0x00);// Master2Slave Position 4 command 2 4 6 0 0 4 0 0
            //while({ // Slave command 1356789A
                //Command_Read_from_Slave=CAN_Read_from_Slave();
                //Serial.println(Command_Read_from_Slave);
            //}
            break;
        }
        else{
            break;
        }
    case 5:
        if(PC_Command=="MOVEFRNT"){
            digitalWrite(Driving_Forward,LOW); // Move Forward
            digitalWrite(Driving_Backward,HIGH);

            Serial.print("USER Command Received:  "); Serial.println(PC_Command);delay(1);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("MOVE FORWARD");delay(2);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);

            CAN_Send(0x0080,0x02,0x04,0x06,0x01,0x00,0x00,0x00,0x01);// Master2Slave Start Proximity Detection command 2 4 6 1 0 0 0 1
            break;
        }
        else if(PC_Command=="MOVEBACK"){
            digitalWrite(Driving_Backward,LOW); // Move Backward
            digitalWrite(Driving_Forward,HIGH);

            Serial.print("USER Command Received:  "); Serial.println(PC_Command);delay(1);

            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("MOVE BACWARD");delay(2);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);

            CAN_Send(0x0080,0x02,0x04,0x06,0x01,0x00,0x00,0x00,0x01);// Master2Slave Start Proximity Detection command 2 4 6 1 0 0 0 1
            break;
        }
        else{
            break;
        }

    case 9:
        if(Command_Read_from_Slave=="01:03:05:06:07:08:09:0A"){ // Slave2Master Initial Finish
            Serial.println("Initial Finish Jumped in");
            for(int i = 0; i<50; i++){
                CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
                delay(1);
                //Serial.println(Command_Read_from_Slave);
                //Serial.println("19910515 Send out");
            }
            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TASK FINISH "); delay(1);
            Serial3.println("TCLP POS #1 "); delay(1);  // Move to position #1
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            Serial.println("TASK FINISH!"); delay(1);
        }
        else if(Command_Read_from_Slave=="01:03:05:0A:09:08:07:01"){ // Slave2Master Proximity Head Forward Detected
            Link_Number_Count++;
            digitalWrite(Driving_Backward,HIGH); // Move Neutral
            digitalWrite(Driving_Forward,HIGH);
            Serial.println("Forward Master Proximity On Chain Jumped in");
            for(int i = 0; i<50; i++){
                CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
                //Serial.println(Command_Read_from_Slave);
                //Serial.println("19910515 Send out");
            }
            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("ON THE CHAIN"); delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            Serial.println("ON THE CHAIN"); delay(1);
            Serial.print("Link_number_string: ");Serial.println(Link_Number_Count);delay(1);
        }
        else if(Command_Read_from_Slave=="01:03:05:0A:09:08:07:00"){ // Slave2Master Proximity Back Head Detected
            Link_Number_Count--;
            digitalWrite(Driving_Backward,HIGH); // Move Neutral
            digitalWrite(Driving_Forward,HIGH);
            Serial.println("Backward Master Proximity On Chain Jumped in");
            for(int i = 0; i<50; i++){
                CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
                //Serial.println(Command_Read_from_Slave);
                //Serial.println("19910515 Send out");
            }
            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("ON THE CHAIN");delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            Serial.println("ON THE CHAIN"); delay(1);
            Serial.print("Link_number_string: ");Serial.println(Link_Number_Count);delay(1);
        }
        else if(Command_Read_from_Slave=="01:03:05:00:00:00:00:01"){ // Slave2Master Position #1 Confirmed
            Serial.println("Position #1 Master Jumped in");
            for(int i = 0; i<50; i++){
                CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
            }
            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TCL1 REACHED"); delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            Serial.println("PS1 TASK FINISH!"); delay(1);
        }
        else if(Command_Read_from_Slave=="01:03:05:00:00:00:00:02"){ // Slave2Master Position #2 Confirmed
            Serial.println("Position #2 Master Jumped in");
            for(int i = 0; i<50; i++){
                CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
                delay(1);
            }
            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TCL2 REACHED"); delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            Serial.println("PS2 TASK FINISH!"); delay(1);
        }
        else if(Command_Read_from_Slave=="01:03:05:00:00:00:00:03"){ // Slave2Master Position #3 Confirmed
            Serial.println("Position #3 Master Jumped in");
            for(int i = 0; i<50; i++){
                CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
                delay(1);
            }
            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TCL3 REACHED"); delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            Serial.println("PS3 TASK FINISH!"); delay(1);
        }
        else if(Command_Read_from_Slave=="01:03:05:00:00:00:00:04"){ // Slave2Master Position #4 Confirmed
            Serial.println("Position #4 Master Jumped in");
            for(int i = 0; i<50; i++){
                CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
                delay(1);
            }
            digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
            Serial3.println("TCL4 REACHED"); delay(1);
            digitalWrite(SSerialTxControl, RS485Receive);delay(1);
            Serial.println("PS4 TASK FINISH!"); delay(1);
        }
        else if(Command_Read_from_Slave=="01:03:05:00:00:00:00:00"){ // Slave2Master Termination Confirmed
            Serial.println("Termination Master Jumped in");
            for(int i = 0; i<50; i++){
                CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
                delay(1);
            }
        }
        else if(Command_Read_from_Slave=="01:03:05:01:01:01:01:01"){ // Slave2Master Reboot Confirmed
            Serial.println("Reboot Master Jumped in");
            for(int i = 0; i<50; i++){
                CAN_Send(0x0080,0x01,0x09,0x09,0x01,0x00,0x05,0x01,0x05);
                delay(1);
            }
        }
        else{
            break;
        }
        break;
    case 10:
        break;
    default:
        PC_Command="";//delay(1);
        Command_Read_from_Slave="";
        digitalWrite(SSerialTxControl, RS485Transmit);delay(1);
        Serial3.println("Bad Command"); delay(1);
        digitalWrite(SSerialTxControl, RS485Receive);delay(1);
        break;
    }
    PC_Command="";//delay(5);
    pc_cmd_temp[0]=0;//Change the first letter so that we will not continuously send the command back to PC


    if(Loop_count==100){
        Loop_count=0;
    }
    //Read_and_Print_Sensors(Loop_count,Link_Number_Count);
    Loop_count++;
    Serial.println("Loop_count:");
    Serial.print(Loop_count);
    Serial.println("\n\r");

    Serial.print(PC_Command);
    Serial3.flush();
}

void Read_and_Print_Sensors(int Loop_count, int Link_number){

    String Temp_string; String Pressure_string; String Depth_string;
    String X_Degree_string; String X_Minute_string; String Y_Degree_string; String Y_Minute_string;
    String Degree_string;
    String Link_number_string;

    //Temperature Sensor
    if(Loop_count%10==0||Loop_count<1){
        Temperature_sensor.read();
        Temp_string=Temperature_sensor.temperature();
        //Temperature Sensor [Temperature] Reading
        Temp_string="TPS"+Temp_string+"T";
        for(int i=Temp_string.length();i<12;i++){
            Temp_string+="T";
        }

        Link_number_string = Link_number;
        Link_number_string="LKN"+Link_number_string+"N";
        for(int i=Link_number_string.length();i<12;i++){
            Link_number_string+="N";
        }
        //while(Serial3.available()>0);
        digitalWrite(SSerialTxControl,RS485Transmit);delay(5);
        Serial3.println(Temp_string);delay(5);
        Serial3.println(Link_number_string);delay(5);
        digitalWrite(SSerialTxControl,RS485Receive);delay(5);
    }

    //Pressure Sensor
    if(Loop_count%10==0||Loop_count<1){
        //Pressure Sensor
        Pressure_sensor.read();//delay(5);
        Pressure_string=String(Pressure_sensor.pressure()); // Unit bar
        Depth_string=String(Pressure_sensor.depth()); //Unit meter

        //Pressure Sensor [Pressure] Reading
        Pressure_string="DSP"+Pressure_string+"P";
        for(int i=Pressure_string.length();i<12;i++){
            Pressure_string+="P";
        }
        //while(Serial3.available()>0);
        digitalWrite(SSerialTxControl,RS485Transmit);delay(5);
        Serial3.println(Pressure_string);delay(5);
        digitalWrite(SSerialTxControl,RS485Receive);delay(5);

        //Pressure Sensor [Depth] Reading
        Depth_string="DSD"+Depth_string+"D";
        for(int i=Depth_string.length();i<12;i++){
            Depth_string+="D";
        }
        //while(Serial3.available()>0);
        digitalWrite(SSerialTxControl,RS485Transmit);delay(5);
        Serial3.println(Depth_string);delay(5);
        digitalWrite(SSerialTxControl,RS485Receive);delay(5);

        //CP Probe Output
        CP_output=analogRead(CP_Probe);
        //Serial.print("AD CP Out: ");
        //Serial.print(CP_output);
        //Serial.print("    String:  ");
        CP_output=(CP_output+3)*4.9;
        CP_output_string=CP_output;
        CP_output_string="OCP"+CP_output_string;
        for(int i=CP_output_string.length();i<12;i++){
            CP_output_string+="C";
        }
        //while(Serial3.available()>0);
        digitalWrite(SSerialTxControl,RS485Transmit);delay(5);
        Serial3.println(CP_output_string);delay(5);
        digitalWrite(SSerialTxControl,RS485Receive);delay(5);

        //Serial.println(CP_output_string);
    }

    //Tilt Sensor
    X_IntValue = analogRead(Tilt_x_Input);//delay(1);//Read value
    X_IntValue = constrain(X_IntValue+X_IntValue_offset,102,921);//delay(1);
    X_degree = map(X_IntValue,102,921,-90,90);//delay(1);
    //X_arc_minute = (int((X_IntValue-100)*13.17))%60;delay(5);
    X_Degree_string=X_degree; X_Minute_string=X_arc_minute;

    Y_IntValue = analogRead(Tilt_y_Input); //delay(1);//Read value
    Y_IntValue = constrain(Y_IntValue+Y_IntValue_offset,102,921);//delay(1);
    Y_degree = map(Y_IntValue,102,921,-90,90);//delay(1);
    //Y_arc_minute = (int((Y_IntValue-100)*13.17))%60;delay(5);
    Y_Degree_string=Y_degree; Y_Minute_string=Y_arc_minute;

    //Tilt Sensor [Tilt_X_Axis] Reading
    Degree_string="TDS"+X_Degree_string+"+"+Y_Degree_string;
    for(int i=Degree_string.length();i<12;i++){
        Degree_string+="D";
    }

    //while(Serial3.available()>0);
    digitalWrite(SSerialTxControl,RS485Transmit);delay(5);
    Serial3.println(Degree_string);delay(5);
    digitalWrite(SSerialTxControl, RS485Receive);delay(5);

    //Serial.println(Degree_string);
}


/*
 * This function is for receiving the CAN messages either from the slave controller or from the
 * MTS sensor. If the message is from the MTS sensor, send the measured data to GUI.
 */

String CAN_send2GUI(){

    String Magnet_ps_1="";
    String Result_ps="";
    String Head_flag;
    String result_message;
    long int Position_1_new=0;
    long int Position_1_old=0;

    CAN_Read_message_1_new=CAN_Read();

    Head_flag=CAN_Read_message_1_new.substring(0,1);
    if(Head_flag=="M"){
        CAN_Read_message_1_new=CAN_Read_message_1_new.substring(1);
    }
    else if (Head_flag=="S"){
        result_message=CAN_Read_message_1_new.substring(1);
        return result_message;
    }
    else{
    	return "";
    }

    Magnet_ps_1=CAN_Read_message_1_new.substring(6,8)+CAN_Read_message_1_new.substring(9,11)+CAN_Read_message_1_new.substring(12,14);

    unsigned int string_length=0;
    String MTS_sensor_out="";
    Position_1_new=ConvertString2Int(Magnet_ps_1)*0.5; //Position_1_int=int(Position_1);
    if(Position_1_new>20){
        Position_1_old=Position_1_new;
    }
    else{
        Position_1_new=Position_1_old;
    }
    Magnet_ps_1=String(Position_1_new);
    string_length=Magnet_ps_1.length();
    for(unsigned int i=0; i<(5-string_length); i++){
        Magnet_ps_1="0"+Magnet_ps_1;
    }

    MTS_sensor_out="C"+Magnet_ps_1+"+"+"00000";//+Magnet_ps_2;
    digitalWrite(SSerialTxControl, RS485Transmit);//delay(1);
    //Serial.println(MTS_sensor_out);delay(2);
    Serial3.println(MTS_sensor_out);delay(2);
    //Serial.println(MTS_sensor_out);delay(2);
    digitalWrite(SSerialTxControl, RS485Receive);//delay(5);

    result_message="0";
    return result_message;
}

/*
Command List:

Master ---> Slave

Initial command         2 4 6 1 2 3 4 5
Position 1 command      2 4 6 0 0 1 0 0
Position 2 command      2 4 6 0 0 2 0 0
Position 3 command      2 4 6 0 0 3 0 0
Position 4 command      2 4 6 0 0 4 0 0
Termination command     2 4 6 0 0 0 0 0
Reboot command          2 4 6 1 1 1 1 1

Slave ---> Master

Initial Finish              1 3 5 6 7 8 9 A
Proximity Head Detected     1 3 5 A 9 8 7 6
Position #1 Confirmed       1 3 5 0 0 0 0 1
Position #2 Confirmed       1 3 5 0 0 0 0 2
Position #3 Confirmed       1 3 5 0 0 0 0 3
Position #4 Confirmed       1 3 5 0 0 0 0 4
Termination Confirmed       1 3 5 0 0 0 0 0
Reboot Confirmed            1 3 5 1 1 1 1 1
*/
