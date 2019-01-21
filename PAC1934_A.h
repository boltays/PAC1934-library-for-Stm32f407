/* Header File */



#ifndef _PAC1934_H_
#define _PAC1934_H_

#include "stdint.h"
#include "i2c.h"
#include "adc.h"

extern uint8_t PAC1934_REFRESH;    
extern uint8_t PAC_CTRL;           
extern uint8_t ACC_COUNT;		
extern uint8_t VPOWER1_ACC;
extern uint8_t VPOWER2_ACC;
extern uint8_t VPOWER3_ACC;
extern uint8_t VPOWER4_ACC;
extern uint8_t VBUS1;
extern uint8_t VBUS2;
extern uint8_t VBUS3;
extern uint8_t VBUS4;
extern uint8_t VSENSE1;					
extern uint8_t VSENSE2;				
extern uint8_t VSENSE3;
extern uint8_t VSENSE4;					
extern uint8_t VBUS1_AVG;				
extern uint8_t VBUS2_AVG;			
extern uint8_t VBUS3_AVG;			
extern uint8_t VBUS4_AVG;		
extern uint8_t VSENSE1_AVG;			
extern uint8_t VSENSE2_AVG;			
extern uint8_t VSENSE3_AVG;
extern uint8_t VSENSE4_AVG;		
extern uint8_t VPOWER1;	
extern uint8_t VPOWER2;			
extern uint8_t VPOWER3;				
extern uint8_t VPOWER4;					
extern uint8_t CHANNEL_DIS;			
extern uint8_t NEG_PWR;	
extern uint8_t REFRESH_G;				
extern uint8_t REFRESH_V;			
extern uint8_t SLOW;			
extern uint8_t CTRL_ACT;					
extern uint8_t CHANNEL_DIS_ACT; 
extern uint8_t NEG_PWR_ACT;
extern uint8_t CTRL_LAT;		
extern uint8_t CHANNEL_DIS_LAT; 	
extern uint8_t NEG_PWR_LAT;
extern uint8_t PID;		
extern uint8_t MID;					
extern uint8_t REV;						

extern int load;
extern int zero;

extern double Voltage_J[10];
extern double Current_J[10];
extern double Power_J[10];
extern double Energy_J[10];
extern double Time_J[10];


#define DEV_ADDR_W1	0x22      //Writing to the first sensor
#define DEV_ADDR_R1 0x23      //Reading from the first sensor
	
#define DEV_ADDR_W2 0x20      //Writing to the second sensor
#define DEV_ADDR_R2 0x21      //Reading from the second sensor

void PAC1934_init(void);
void sendComment(void);
double VoltageMeasurement(int port_number);
double CurrentMeasurement(int port_number);
double PowerMeasurement(int port_number);
double EnergyMeasurement(int port_number);
double TimeMeasurement(int port_number);


#endif

